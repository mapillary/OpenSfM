#!/usr/bin/env python
import json
import itertools
import numpy as np
import scipy.spatial
from opensfm import dataset
from opensfm import reconstruction
import logging

logger = logging.getLogger(__name__)


def triangle_mesh(shot_id, r, graph, data):
    '''
    Create triangle meshes in a list
    '''
    if 'shots' not in r or shot_id not in r['shots'] or shot_id not in graph:
        return [], []

    shot = r['shots'][shot_id]
    cam = r['cameras'][shot['camera']]

    pt = cam.get('projection_type', 'perspective')
    if pt == 'perspective':
        return triangle_mesh_perspective(shot_id, r, graph)
    else:
        return triangle_mesh_equirectangular(shot_id, r, graph)


def triangle_mesh_perspective(shot_id, r, graph):
    shot = r['shots'][shot_id]
    cam = r['cameras'][shot['camera']]

    dx = float(cam['width']) / 2 / max(cam['width'], cam['height'])
    dy = float(cam['height']) / 2 / max(cam['width'], cam['height'])
    pixels = [[-dx, -dy], [-dx, dy], [dx, dy], [dx, -dy]]
    vertices = [None for i in range(4)]
    for track_id, edge in graph[shot_id].items():
        if track_id in r['points']:
            point = r['points'][track_id]
            pixel = reconstruction.reproject(cam, shot, point)
            if -dx <= pixel[0] <= dx and -dy <= pixel[1] <= dy:
                vertices.append(point['coordinates'])
                pixels.append(pixel.tolist())

    try:
        tri = scipy.spatial.Delaunay(pixels)
    except Exception as e:
        logger.error('Delaunay triangulation failed for input: {}'.format(`pixels`))
        raise e

    sums = [0.,0.,0.,0.]
    depths = [0.,0.,0.,0.]
    for t in tri.simplices:
        for i in range(4):
            if i in t:
                for j in t:
                    if j >= 4:
                        depths[i] += reconstruction.camera_coordinates(cam, shot, vertices[j])[2]
                        sums[i] += 1
    for i in range(4):
        if sums[i] > 0:
            d = depths[i] / sums[i]
        else:
            d = 50.0
        vertices[i] = reconstruction.back_project(cam, shot, pixels[i], d).tolist()

    faces = tri.simplices.tolist()
    return vertices, faces


def triangle_mesh_equirectangular(shot_id, r, graph):
    shot = r['shots'][shot_id]
    cam = r['cameras'][shot['camera']]

    bearings = []
    vertices = []

    # Add vertices to ensure that the camera is inside the convex hull of the points
    for point in itertools.product([-1, 1], repeat=3): # vertices of a cube
        bearing = 0.3 * np.array(point) / np.linalg.norm(point)
        bearings.append(bearing)
        point = reconstruction.world_coordinates(cam, shot, bearing)
        vertices.append(point.tolist())

    for track_id, edge in graph[shot_id].items():
        if track_id in r['points']:
            point = r['points'][track_id]['coordinates']
            vertices.append(point)
            direction = reconstruction.camera_coordinates(cam, shot, point)
            pixel = direction / np.linalg.norm(direction)
            bearings.append(pixel.tolist())

    tri = scipy.spatial.ConvexHull(bearings)
    faces = tri.simplices.tolist()
    return vertices, faces
