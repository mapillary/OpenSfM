#!/usr/bin/env python
import json
import numpy as np
import scipy.spatial
from opensfm import dataset
from opensfm import reconstruction

def triangle_mesh(r, graph, data):
    '''
    Create triangle meshes in a list
    '''
    for shot_id, shot in r['shots'].iteritems():
        if shot_id not in graph:
            continue
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
            w = sums[i] or 50.0
            vertices[i] = reconstruction.back_project(cam, shot, pixels[i], depths[i] / w).tolist()

        faces = tri.simplices.tolist()
        r['shots'][shot_id]['vertices'] = vertices
        r['shots'][shot_id]['faces'] = faces

    return r
