import numpy as np
import cv2
import scipy.spatial as spatial


def points_errors(reference, candidate, max_distance=1):
    ref_points = np.array([p.coordinates for p in reference.points.values()])
    topo_tree = spatial.cKDTree(ref_points)

    cand_points = np.array([p.coordinates for p in candidate.points.values()])
    return topo_tree.query(cand_points)[0]


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
        # TODO For now don't consider Z as 
        # there's an issue with XYZ<->WGS84
        pose1[2] = pose2[2]
        error = np.linalg.norm(pose1-pose2)
        errors.append(np.linalg.norm(pose1-pose2))
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