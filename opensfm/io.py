import os
import argparse

import cv2
import json
import numpy as np

import dataset

# (TODO): ensure the image order from OpenSfM is the same as Bundler
# (TODO): ensure the coordinate systems are consistent

def export_bundler(reconstruction, track_graph, bundle_file, list_file, ordered_shots=None, convert_coorindate=True, normalized_coordindate=True):
    """
    Generate a reconstruction file that is consistent with Bundler's format
    """
    lines = []

    with open(bundle_file, 'wb') as fout:
        points = reconstruction['points']
        shots = reconstruction['shots']
        cameras = reconstruction['cameras']
        num_point = len(points)
        num_shot = len(shots)
        shot_ids = list(shots.keys())
        lines.append(' '.join(map(str, [num_shot, num_point])))
        if ordered_shots is None: ordered_shots = shot_ids
        shots_order = {key: i for i, key in enumerate(ordered_shots)}

        # cameras
        for shot_id in ordered_shots:
            shot = shots.get(shot_id, None)
            if shot is not None:
                camera = cameras[shot['camera']]
                scale = max(camera['width'], camera['height'])
                focal = camera['focal'] if normalized_coordindate else camera['focal']*scale
                lines.append(' '.join(map(str, [camera['focal'], camera['k1'], camera['k2']])))
                R, t = shot['rotation'], shot['translation']
                R = cv2.Rodrigues(np.array(R))[0]
                if convert_coorindate:
                    R[3:] = -R[3:]
                    t[1], t[2] = -t[1], -t[2]

                for i in xrange(3): lines.append(' '.join(list(map(str, R[i]))))
                t = ' '.join(map(str, t))
                lines.append(t)
            else:
                # add 5 lines of 3x0
                for i in xrange(5): lines.append(' '.join(map(str,[0, 0, 0])))

        # tracks
        for point_id, point in points.iteritems():
            coord = point['coordinates']
            color = map(int, point['color'])
            view_list = track_graph[point_id]
            lines.append(' '.join(map(str, coord)))
            lines.append(' '.join(map(str, color)))
            view_line = [str(len(view_list))]
            for shot_key, view in view_list.iteritems():
                v = view['feature']
                shot_index = shots_order[shot_key]
                if normalized_coordindate:
                    camera = shots[shot_key]['camera']
                    scale = max(cameras[camera]['width'], cameras[camera]['height'])
                    x = v[0]*scale
                    y = -v[1]*scale
                view_line.append(' '.join(map(str, [shot_index, v[2], x, y])))

            lines.append(' '.join(view_line))

        fout.writelines('\n'.join(lines))

    with open(list_file, 'wb') as fout:
        fout.writelines('\n'.join(map(str, ordered_shots) ))

def import_bundler(bundle_file, list_file, track_file, reconstruction_file=None, convert_coorindate=True):
    """
    Return a reconstruction dict and a track graph file (track.csv) compatible with OpenSfM from a Bundler output
    """
    # (TODO): handle cameras with exif info in reconstruction['cameras']
        # camera model
        # image width/height for denormalizing coordinates, principle point

    # read image list
    with open(list_file, 'rb') as fin:
        image_list = fin.readlines()
        ordered_shots = [im.rstrip('\n').split(' ')[0] for im in image_list]
        image_path = os.path.dirname(image_list[0].rstrip('\n').split(' ')[0])

    reconstruction = {}
    track_graph = {}

    with open(bundle_file, 'rb') as fin:
        lines = fin.readlines()
    offset = 1 if '#' in lines[0] else 0

    # header
    num_shot, num_point = map(int, lines[offset].split(' '))

    # initialization
    reconstruction['cameras'] = {}
    reconstruction['shots'] = {}
    reconstruction['points'] = {}
    offset += 1

    # cameras
    if ordered_shots is None: ordered_shots = np.arange(num_shot)
    for i in xrange(num_shot):
        # Creating a model for each shot for now. TODO: create mdoel based on exif
        shot_key = ordered_shots[i]
        f, k1, k2 = map(float, lines[offset].rstrip('\n').split(' '))
        camera_name = 'camera_' + str(i)
        im = cv2.imread(os.path.join(image_path, shot_key))
        height, width = im.shape[0:2]
        f = float(f)/max(height, width)
        reconstruction['cameras'][camera_name] = {'focal': f, 'k1': k1, 'k2': k2, 'width': width, 'height': height}
        rline = []
        for k in xrange(3): rline += lines[offset+1+k].rstrip('\n').split(' ')
        R = ' '.join(rline)
        t = lines[offset+4].rstrip('\n').split(' ')
        R = np.array(map(float, R.split()))
        t = np.array(map(float, t))

        if convert_coorindate:
            R[3:] = -R[3:]
            t[1:] = -t[1:]

        reconstruction['shots'][shot_key] = {
                                        'camera' : camera_name,
                                        'rotation': list(R),
                                        'translation': list(t)
                                     }

        offset += 5

    # tracks
    track_lines = []
    for i in xrange(num_point):
        coordinates = lines[offset].rstrip('\n').split(' ')
        color = lines[offset+1].rstrip('\n').split(' ')
        reconstruction['points'][i] = {
                                        'coordinates': coordinates,
                                        'color': color
                                      }
        view_line = lines[offset+2]
        num_view, view_list = int(view_line[0]), view_line[2:].rstrip('\n').split(' ')
        for k in xrange(num_view):
            shot_key = ordered_shots[int(view_list[4*k])]
            camera_name = reconstruction['shots'][shot_key]['camera']
            scale = max(reconstruction['cameras'][camera_name]['width'], reconstruction['cameras'][camera_name]['height'])
            v = ' '.join([ view_list[4*k],
                     str(i),
                  view_list[4*k + 1],
                  str(float(view_list[4*k + 2])/scale),
                  str(float(view_list[4*k + 3])/scale)
                ])
            track_lines.append(v)
        offset += 3

    # save track file
    with open(track_file, 'wb') as fout:
        fout.writelines('\n'.join(track_lines))

    # save reconstruction
    if reconstruction_file is not None:
        with open(reconstruction_file, 'wb') as fout:
            fout.write(json.dumps(reconstruction, indent=4))
    return reconstruction

