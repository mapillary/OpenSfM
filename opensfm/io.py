import os
import argparse
import cv2
import json
import numpy as np
import errno


# (TODO): ensure the image order from OpenSfM is the same as Bundler
# (TODO): ensure the coordinate systems are consistent

def export_bundler(image_list, reconstructions, track_graph, bundle_file_path, list_file_path):
    """
    Generate a reconstruction file that is consistent with Bundler's format
    """

    if not os.path.exists(bundle_file_path): os.makedirs(bundle_file_path)
    if not os.path.exists(list_file_path): os.makedirs(list_file_path)

    for j, reconstruction in enumerate(reconstructions):
        lines = []
        lines.append("# Bundle file v0.3")
        points = reconstruction['points']
        shots = reconstruction['shots']
        cameras = reconstruction['cameras']
        num_point = len(points)
        num_shot = len(image_list)
        lines.append(' '.join(map(str, [num_shot, num_point])))
        shots_order = {key: i for i, key in enumerate(image_list)}

        # cameras
        for shot_id in image_list:
            if shot_id in shots:
                shot = shots[shot_id]
                camera = cameras[shot['camera']]
                scale = max(camera['width'], camera['height'])
                focal = camera['focal'] * scale
                k1 = camera['k1']
                k2 = camera['k2']
                R, t = shot['rotation'], shot['translation']
                R = cv2.Rodrigues(np.array(R))[0]
                R[1], R[2] = -R[1], -R[2]  # Reverse y and z
                t[1], t[2] = -t[1], -t[2]
                lines.append(' '.join(map(str, [focal, k1, k2])))
                for i in xrange(3): lines.append(' '.join(list(map(str, R[i]))))
                t = ' '.join(map(str, t))
                lines.append(t)
            else:
                for i in range(5):
                    lines.append("0 0 0")


        # tracks
        for point_id, point in points.iteritems():
            coord = point['coordinates']
            color = map(int, point['color'])
            view_list = track_graph[point_id]
            lines.append(' '.join(map(str, coord)))
            lines.append(' '.join(map(str, color)))
            view_line = []
            for shot_key, view in view_list.iteritems():
                if shot_key in shots.keys():
                    v = view['feature']
                    shot_index = shots_order[shot_key]
                    camera = shots[shot_key]['camera']
                    scale = max(cameras[camera]['width'], cameras[camera]['height'])
                    x = v[0] * scale
                    y = -v[1] * scale
                    view_line.append(' '.join(map(str, [shot_index, view['feature_id'], x, y])))

            lines.append(str(len(view_line)) + ' ' + ' '.join(view_line))

        bundle_file =os.path.join(bundle_file_path, 'bundle_r'+str(j).zfill(3)+'.out')
        with open(bundle_file, 'wb') as fout:
            fout.writelines('\n'.join(lines) + '\n')

        list_file =os.path.join(list_file_path, 'list_r'+str(j).zfill(3)+'.out')
        with open(list_file, 'wb') as fout:
            fout.writelines('\n'.join(map(str, image_list)))

def import_bundler(data_path, bundle_file, list_file, track_file, reconstruction_file=None):
    """
    Return a reconstruction dict and a track graph file (track.csv) compatible with OpenSfM from a Bundler output
    """
    # (TODO): handle cameras with exif info in reconstruction['cameras']
        # camera model
        # image width/height for denormalizing coordinates, principle point

    # Init OpenSfM working folder.
    mkdir_p(data_path)

    # Copy image list.
    list_dir = os.path.dirname(list_file)
    with open(list_file, 'rb') as fin:
        lines = fin.read().splitlines()
    ordered_shots = []
    image_list = []
    for line in lines:
        image_path = os.path.join(list_dir, line.split()[0])
        rel_to_data = os.path.relpath(image_path, data_path)
        image_list.append(rel_to_data)
        ordered_shots.append(os.path.basename(image_path))
    with open(os.path.join(data_path, 'image_list.txt'), 'w') as fout:
        fout.write('\n'.join(image_list) + '\n')

    # Check for bundle_file
    if not bundle_file or not os.path.isfile(bundle_file):
        return None

    with open(bundle_file, 'rb') as fin:
        lines = fin.readlines()
    offset = 1 if '#' in lines[0] else 0

    # header
    num_shot, num_point = map(int, lines[offset].split(' '))
    offset += 1

    # initialization
    reconstruction = {}
    reconstruction['cameras'] = {}
    reconstruction['shots'] = {}
    reconstruction['points'] = {}
    track_graph = {}

    # cameras
    for i in xrange(num_shot):
        # Creating a model for each shot for now.
        # TODO: create mdoel based on exif
        shot_key = ordered_shots[i]
        focal, k1, k2 = map(float, lines[offset].rstrip('\n').split(' '))

        if focal > 0:
            camera_name = 'camera_' + str(i)
            im = cv2.imread(os.path.join(data_path, image_list[i]))
            height, width = im.shape[0:2]
            scale = float(max(height, width))
            focal = focal / scale
            reconstruction['cameras'][camera_name] = {'focal': focal, 'k1': k1, 'k2': k2, 'width': width, 'height': height}

            # Shots
            rline = []
            for k in xrange(3): rline += lines[offset+1+k].rstrip('\n').split(' ')
            R = ' '.join(rline)
            t = lines[offset+4].rstrip('\n').split(' ')
            R = np.array(map(float, R.split())).reshape(3,3)
            t = np.array(map(float, t))

            R[1], R[2] = -R[1], -R[2]  # Reverse y and z
            t[1], t[2] = -t[1], -t[2]

            reconstruction['shots'][shot_key] = {
                                            'camera' : camera_name,
                                            'rotation': list(cv2.Rodrigues(R)[0].flatten(0)),
                                            'translation': list(t)
                                         }
        else:
            print 'ignore failed image', shot_key
        offset += 5

    # tracks
    track_lines = []
    for i in xrange(num_point):
        coordinates = lines[offset].rstrip('\n').split(' ')
        color = lines[offset+1].rstrip('\n').split(' ')
        reconstruction['points'][i] = {
                                        'coordinates': map(float, coordinates),
                                        'color': map(int, color)
                                      }
        view_line = lines[offset+2].rstrip('\n').split(' ')

        # num_view, view_list = int(view_line[0]), view_line[1:].rstrip('\n').split(' ')
        num_view, view_list = int(view_line[0]), view_line[1:]

        for k in xrange(num_view):
            shot_key = ordered_shots[int(view_list[4*k])]
            camera_name = reconstruction['shots'][shot_key]['camera']
            scale = max(reconstruction['cameras'][camera_name]['width'], reconstruction['cameras'][camera_name]['height'])
            v = '\t'.join([ shot_key,
                     str(i),
                  view_list[4*k + 1],
                  str(float(view_list[4*k + 2])/scale),
                  str(-float(view_list[4*k + 3])/scale)
                ])
            track_lines.append(v)
        offset += 3

    # save track file
    with open(track_file, 'wb') as fout:
        fout.writelines('\n'.join(track_lines))

    # save reconstruction
    if reconstruction_file is not None:
        with open(reconstruction_file, 'wb') as fout:
            fout.write(json_dumps([reconstruction]))
    return reconstruction


def mkdir_p(path):
    '''Make a directory including parent directories.
    '''
    try:
        os.makedirs(path)
    except os.error as exc:
        if exc.errno != errno.EEXIST or not os.path.isdir(path):
            raise


def json_dumps(data, indent=4, codec='utf-8'):
    return json.dumps(data, indent=indent, ensure_ascii=False).encode(codec)

def json_loads(text, codec='utf-8'):
    return json.loads(text.decode(codec))
