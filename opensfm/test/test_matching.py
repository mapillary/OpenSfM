import numpy as np

import opensfm.config
import opensfm.matching
import data_generation


def test_robust_match():
    d = data_generation.CubeDataset(2, 100, 0.1, 0.3)
    p1 = np.array([v['feature'] for k, v in d.tracks['shot0'].iteritems()])
    p2 = np.array([v['feature'] for k, v in d.tracks['shot1'].iteritems()])
    camera1 = d.shots['shot0'].camera
    camera2 = d.shots['shot1'].camera
    matches = np.array([(i, i) for i in range(len(p1))])
    config = opensfm.config.default_config()
    opensfm.matching.robust_match(p1, p2, camera1, camera2, matches, config)


if __name__ == "__main__":
    test_robust_match()
