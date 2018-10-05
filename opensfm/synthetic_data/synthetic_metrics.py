import numpy as np
import cv2
import scipy.spatial as spatial


def points_errors(reference, candidate, max_distance=1):
    ref_points = np.array([p.coordinates for p in reference.points.values()])
    topo_tree = spatial.cKDTree(ref_points)

    cand_points = np.array([p.coordinates for p in candidate.points.values()])
    closest_indexes = topo_tree.query(cand_points)[1]

    return np.array([ref_points[closest]-cand
                     for closest, cand in zip(closest_indexes, cand_points)])


def completeness_errors(reference, candidate):
    return float(len(candidate.shots))/float(len(reference.shots)),\
            float(len(candidate.points))/float(len(reference.points))


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
