#!/usr/bin/env python
import json
import numpy as np
import scipy.spatial
from opensfm import dataset
from opensfm import reconstruction

def triangle_mesh(shot_id, r, graph, data):
    '''
    Create triangle meshes in a list
    '''
    if 'shots' not in r or shot_id not in r['shots'] or shot_id not in graph:
        return [], []

    shot = r['shots'][shot_id]
    cam = r['cameras'][shot['camera']]
    dx = float(cam['width']) / 2 / max(cam['width'], cam['height'])
    dy = float(cam['height']) / 2 / max(cam['width'], cam['height'])
    pixels = [[-dx, -dy], [-dx, dy], [dx, dy], [dx, -dy]]
    vertices = [None for i in range(4)]
    for track_id, edge in graph[shot_id].items():
        if track_id in r['points']:
            point = r['points'][track_id]
            vertices.append(point['coordinates'])
            pixel = reconstruction.reproject(cam, shot, point)
            pixels.append(pixel.tolist())

    tri = scipy.spatial.Delaunay(pixels)

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
