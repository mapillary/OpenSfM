import os
from subprocess import Popen, PIPE
import numpy as np
import datetime
import dateutil.parser
import cv2

import dataset
import geotag_from_gpx
import geo


def gpx_lerp(alpha, a, b):
    '''Interpolate gpx point as (1 - alpha) * a + alpha * b
    '''
    dt = alpha * (b[0] - a[0]).total_seconds()
    t = a[0] + datetime.timedelta(seconds=dt)
    lat = (1 - alpha) * a[1] + alpha * b[1]
    lon = (1 - alpha) * a[2] + alpha * b[2]
    alt = (1 - alpha) * a[3] + alpha * b[3]
    return t, lat, lon, alt

def segment_sphere_intersection(A, B, C, r):
    '''Intersect the segment AB and the sphere (C,r).

    Assumes A is inside the sphere and B is outside.
    Return the ratio between the length of AI and the length
    of AB, where I is the intersection.
    '''
    AB = np.array(B) - np.array(A)
    CA = np.array(A) - np.array(C)
    a = AB.dot(AB)
    b = 2 * AB.dot(CA)
    c = CA.dot(CA) - r**2
    d = max(0, b**2 - 4 * a * c)
    return (-b + np.sqrt(d)) / (2 * a)

def space_next_point(a, b, last, dx):
    A = geo.ecef_from_lla(a[1], a[2], 0.)
    B = geo.ecef_from_lla(b[1], b[2], 0.)
    C = geo.ecef_from_lla(last[1], last[2], 0.)
    alpha = segment_sphere_intersection(A, B, C, dx)
    return gpx_lerp(alpha, a, b)

def time_next_point(a, b, last, dt):
    da = (a[0] - last[0]).total_seconds()
    db = (b[0] - last[0]).total_seconds()
    alpha = (dt - da) / (db - da)
    return gpx_lerp(alpha, a, b)

def time_distance(a, b):
    return (b[0] - a[0]).total_seconds()

def space_distance(a, b):
    return geo.gps_distance(a[1:3], b[1:3])

def sample_gpx(points, dx, dt=None):
    if dt is not None:
        dx = float(dt)
        print "Sampling GPX file every {0} seconds".format(dx)
        distance = time_distance
        next_point = time_next_point
    else:
        print "Sampling GPX file every {0} meters".format(dx)
        distance = space_distance
        next_point = space_next_point

    key_points = [points[0]]
    a = points[0]
    for i in range(1, len(points)):
        a, b = points[i - 1], points[i]
        dx_b = distance(key_points[-1], b)
        while dx and dx_b >= dx:
            a = next_point(a, b, key_points[-1], dx)
            key_points.append(a)
            assert np.fabs(dx - distance(key_points[-2], key_points[-1])) < 0.1
            dx_b = distance(key_points[-1], b)
    print len(key_points), "points sampled"
    return key_points


def import_video_with_gpx(video_file, gpx_file, output_path, dx, dt=None, start_time=None, visual=False):
    points = geotag_from_gpx.get_lat_lon_time(gpx_file)

    # Rotation
    rotation = Popen(['exiftool', '-Rotation', video_file], stdout=PIPE).stdout.read()

    if rotation:
        rotation = float(rotation.split(':')[1])
        if rotation == 0:
            orientation = 1
        elif rotation == 90:
            orientation = 6
        elif rotation == 180:
            orientation = 3
        elif rotation == 270:
            orientation = 8
    else:
        orientation = 1

    if start_time:
        video_start_time = dateutil.parser.parse(start_time)
    else:
        try:
            exifdate = Popen(['exiftool', '-CreateDate', video_file], stdout=PIPE).stdout.read()
            duration = Popen(['exiftool', '-MediaDuration', video_file], stdout=PIPE).stdout.read()
            datestr = ' '.join(exifdate.split()[-2:])
            video_start_time = datetime.datetime.strptime(datestr,'%Y:%m:%d %H:%M:%S')
            if duration:
                duration = map(int, duration.split()[-1].split(':'))
                video_duration = datetime.timedelta(hours=duration[0],minutes=duration[1],seconds=duration[2])
                video_end_time = video_start_time + video_duration
            else:
                video_end_time = points[-1][0]
        except:
            print 'Video recording timestamp not found. Using first GPS point time.'
            video_start_time = points[0][0]
            video_end_time = points[-1][0]

    print 'Video starts at:', video_start_time

    # Extract video frames.
    dataset.mkdir_p(output_path)
    key_points = sample_gpx(points, dx, dt)

    cap = cv2.VideoCapture(video_file)
    for p in key_points:
        dt = (p[0] - video_start_time).total_seconds()
        if dt > 0:
            cap.set(cv2.cv.CV_CAP_PROP_POS_MSEC, int(dt * 1000))
            ret, frame = cap.read()
            if ret:
                print 'Grabbing frame for time', p[0]
                filepath = os.path.join(output_path, p[0].isoformat() + '.jpg')
                cv2.imwrite(filepath, frame)
                geotag_from_gpx.add_exif_using_timestamp(filepath, points, timestamp=p[0], orientation=orientation)

                # Display the resulting frame
                if visual:
                    # Display the resulting frame
                    max_display_size = 800
                    resize_ratio = float(max_display_size) / max(frame.shape[0], frame.shape[1])
                    frame = cv2.resize(frame, dsize=(0, 0), fx=resize_ratio, fy=resize_ratio)
                    cv2.imshow('frame', frame)
                    if cv2.waitKey(1) & 0xFF == 27:
                        break

    # When everything done, release the capture
    cap.release()
    if visual:
        cv2.destroyAllWindows()

