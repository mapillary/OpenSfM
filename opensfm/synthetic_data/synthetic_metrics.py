import numpy as np
import cv2


def points_errors(reference, candidate):
    common_points = set(reference.points.keys()).\
         intersection(set(candidate.points.keys()))

    return np.array([reference.points[p].coordinates -
                     candidate.points[p].coordinates
                     for p in common_points])


def completeness_errors(reference, candidate):
    return float(len(candidate.shots))/float(len(reference.shots)),\
            float(len(candidate.points))/float(len(reference.points))


def gps_errors(candidate):
    errors = []
    for shot in candidate.shots.values():
        pose1 = shot.metadata.gps_position
        pose2 = shot.pose.get_origin()
        errors.append(pose1-pose2)
    return np.array(errors)


def position_errors(reference, candidate):
    common_shots = set(reference.shots.keys()).\
        intersection(set(candidate.shots.keys()))
    errors = []
    for s in common_shots:
        pose1 = reference.shots[s].pose.get_origin()
        pose2 = candidate.shots[s].pose.get_origin()
        errors.append(pose1-pose2)
    return np.array(errors)


def rotation_errors(reference, candidate):
    common_shots = set(reference.shots.keys()).\
        intersection(set(candidate.shots.keys()))
    errors = []
    for s in common_shots:
        pose1 = reference.shots[s].pose.get_rotation_matrix()
        pose2 = candidate.shots[s].pose.get_rotation_matrix()
        difference = np.transpose(pose1).dot(pose2)
        rodrigues = cv2.Rodrigues(difference)[0].ravel()
        angle = np.linalg.norm(rodrigues)
        errors.append(angle)
    return np.array(errors)
