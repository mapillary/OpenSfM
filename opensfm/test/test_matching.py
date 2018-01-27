import numpy as np
from six import iteritems

import opensfm.config
import opensfm.matching
import data_generation


def test_robust_match():
    d = data_generation.CubeDataset(2, 100, 0.1, 0.3)
    p1 = np.array([v['feature'] for k, v in iteritems(d.tracks['shot0'])])
    p2 = np.array([v['feature'] for k, v in iteritems(d.tracks['shot1'])])
    camera1 = d.shots['shot0'].camera
    camera2 = d.shots['shot1'].camera
    num_points = len(p1)
    inlier_matches = np.array([(i, i) for i in range(num_points)])
    outlier_matches = np.random.randint(num_points, size=(num_points // 2, 2))
    matches = np.concatenate((inlier_matches, outlier_matches))
    config = opensfm.config.default_config()
    rmatches = opensfm.matching.robust_match(p1, p2, camera1, camera2, matches,
                                             config)
    assert num_points <= len(rmatches) <= len(matches)


if __name__ == "__main__":
    test_robust_match()
