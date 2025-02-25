import numpy as np

from tqdm import tqdm

import numpy as np
import math

import numpy as np
import trimesh

import math
from scipy.spatial.transform import Rotation as R

from shapely import MultiPoint, convex_hull
import shapely

import trimesh
import fiona
from fiona.crs import from_epsg

import numpy as np
import trimesh

import logging
logger: logging.Logger = logging.getLogger(__name__)


def extract_metadata(images, exifs, reference, camera_pairing_fov):

    # Extract all the metadata for cone matching
    metadata = []
    for i in images:
        image_exif = exifs[i]
        if not 'gps' in image_exif:
            logger.warning(f'Image {i} no GPS -> skip')
            continue
        if not 'ypr' in image_exif:
            logger.warning(f'Image {i} no YPR -> skip')
            continue
    
        annotation = {
            'fn': i,
            'lon': image_exif['gps']['longitude'],
            'lat': image_exif['gps']['latitude'],
            'alt': image_exif['gps']['altitude'],
            'yaw': image_exif['ypr']['yaw'],
            'pitch': image_exif['ypr']['pitch'],
            'roll': image_exif['ypr']['roll'],
            'fov': camera_pairing_fov,
        }
        metadata.append(annotation)

    # Add relative positions
    for i in metadata:
        x, y, z = reference.to_topocentric(i['lat'], i['lon'], i['alt'])
        i['x'] = x
        i['y'] = y
        i['z'] = z

    return metadata


def save_cones_projection_as_gpkg(mesh_list, reference, output_file):
    schema = {
        'geometry': 'Polygon',
        'properties': {'name': 'str'}
    }

    with fiona.open(output_file, 'w', driver='GPKG', crs=from_epsg(4326), schema=schema) as layer:
        for cn, cc in tqdm(mesh_list):
            vertex_array = trimesh.convex.hull_points(cc)
            ll_point_list = []
            for x, y, z in vertex_array:
                lat, lon, _ = reference.to_lla(x, y, z)
                ll_point_list.append((lon, lat))
                extent = convex_hull(MultiPoint(ll_point_list))
        
            layer.write({
                'geometry': shapely.geometry.mapping(extent),
                'properties': {'name': cn}
            })


def create_fov_cone(point, yaw, pitch, roll, fov, cone_height, cone_sections=32):
    """
    Point: x, y, z cone tip
    Yaw 0 degrees -> top direction, increased clockwise
    Pitch 0 degrees -> top to bottom direction
    FOV - cone angle, degrees
    """
    
    # Create a 3D cone
    cone_radius = cone_height * np.tan(math.radians(fov / 2))
    cone = trimesh.creation.cone(radius=cone_radius, height=cone_height, sections=cone_sections)

    # Moving cone tip to 0,0
    cone.apply_translation([0, 0, -cone_height])
    
    # Create transformation matrix
    transform = np.eye(4)
    
    # Rotate cone
    euler_angles_xyz = (pitch, roll, -1 * yaw)
    transform[:3, :3] = R.from_euler('xyz', euler_angles_xyz, degrees=True).as_matrix()  # Rotation
    
    # Shift cone
    transform[:3, 3] = point  # Translation
    cone.apply_transform(transform)

    return cone
    

def match_candidaes_by_fov_cones(metadata, volume_mesh, images_ref=None, images_cand=None):
    all_images = [i['fn'] for i in metadata]
    if images_ref is None:
        images_ref = all_images
    if images_cand is None:
        images_cand = all_images


    # Create cones
    model_diagonal = math.dist(*volume_mesh.bounds)
    cone_list = []
    for i in metadata:
        fn = i['fn']
        x = i['x']
        y = i['y']
        z = i['z']
        yaw = i['yaw']
        pitch = i['pitch']
        roll = i['roll']
        fov = i['fov']
        
        cone = create_fov_cone(point=(x, y, z),
                               yaw=yaw,
                               pitch=pitch,
                               roll=roll,
                               fov=fov,
                               cone_height=model_diagonal)
        cone_list.append((fn, cone))
    
    clipped_cone_list = []
    
    for fn, cone in cone_list:
        clipped_cone_list.append((fn, trimesh.boolean.intersection([cone, volume_mesh])))

    
    # Find overlapping cones
    pairing_map = dict()
    unique_pairs = set()
    for c_fn, c_cone in clipped_cone_list:
        if c_fn not in images_ref:
            continue
        pairing_map[c_fn] = []
        for t_fn, t_cone in clipped_cone_list:
            if t_fn not in images_cand:
                continue
            if c_fn == t_fn:
                continue
            intersection_volume = trimesh.boolean.intersection([c_cone, t_cone]).volume
            if intersection_volume > 0:
                pairing_map[c_fn].append(t_fn)
                unique_pairs.add(tuple(sorted((t_fn, c_fn))))

    return pairing_map, unique_pairs, clipped_cone_list


def pairing_by_cones_from_dataset(ref_images,
                                  cand_images,
                                  exifs,
                                  data,
                                  config_override):
    print('############### Cone matching!!!')
    data.init_reference()
    reference = data.load_reference()

    overriden_config = data.config.copy()
    overriden_config.update(config_override)
    
    c = set()
    dem_name = str(overriden_config["dem_path"])
    dem_hight = float(overriden_config["dem_hight"])
    pairing_fov = float(overriden_config["pairing_fov"])

    metadata = extract_metadata(set(ref_images + cand_images),
                                                        exifs,
                                                        reference,
                                                        pairing_fov)
    volume_mesh, _ = data.load_demmesh(dem_name, dem_hight, reference=reference)

    _, c, cone_list = match_candidaes_by_fov_cones(
        metadata,
        volume_mesh,
        images_ref=ref_images,
        images_cand=cand_images)

    data.save_cones_projection_as_gpkg(cone_list, reference)
    
    return c, {"num_pairs_cones": len(c),}