.. Notes and doc on dense matching

Feature Detection and Matching
========================================

Feature Detection
-----------------

-  Feature detection returns 3 values: ''p_unsorted'', ''f_unsorted'', ''c_unsorted''

- ''p_unsorted'' includes ''angle'', ''class_id'', ''octave'', ''pt'', ''response'', ''size'' when using SIFT. You could find it from <https://docs.opencv.org/trunk/d2/d29/classcv_1_1KeyPoint.html> in detail. But OpenSfM only extracts ''pt_x'', ''pt_y'', ''angle'' and ''size''. According to size, OpenSfM inverses sequence of feature points. The bigger the size of blobs are, the weaker the sift features are.

-  ''f_unsorted'' means 128 dimensional vector for each feature point when using SIFT.

-  ''c_unsorted'' stores color (RGB) with the same order with ''f_unsorted'' and ''p_unsorted''

Feature Matching
-----------------

Make sure that the ratio between the nearest distance and the second distance is less than 0.8. For two images, one is for queryid and the other is for trainid and then inverse the two images. If so, it verifies the potentially overlapping image paires.

Since matching is based solely on appearance, it is **not** guaranteed that cooresponding features actually map to the same scene point. Therefore, Geometric verificaion is applied using epipolor constraint.

.. code:: python

    def robust_match_fundamental(p1, p2, matches, config):
        F, mask = cv2.findFundamentalMat(p1, p2, FM_RANSAC, config.get('robust_matching_threshold', 0.006), 0.9999)

''0.006'' means the distance between the feature point and its corresponding epipolar line in normalized coordinate. The real distance is 0.006×2400 pixels.
