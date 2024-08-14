#include <features/hahog.h>

#include <iostream>
#include <vector>

extern "C" {
#include <time.h>
#include <vl/covdet.h>
#include <vl/sift.h>
}

namespace features {

// from VLFeat implementation of _vl_compare_scores
static int vlfeat_compare_scores(const void *a, const void *b) {
  float fa = ((VlCovDetFeature *)a)->peakScore;
  float fb = ((VlCovDetFeature *)b)->peakScore;
  return (fb > fa) - (fb < fa);
}

// select 'target_num_features' for using feature's scores
vl_size select_best_features(VlCovDet *covdet, vl_size num_features,
                             vl_size target_num_features) {
  if (num_features > target_num_features) {
    qsort(vl_covdet_get_features(covdet), num_features, sizeof(VlCovDetFeature),
          vlfeat_compare_scores);
    return target_num_features;
  } else {
    return num_features;
  }
}

// select 'target_num_features' that have a maximum score in their neighbhood.
// The neighborhood is computing using the feature's scale and
// 'non_extrema_suppression' as : neighborhood = non_extrema_suppression * scale
vl_size run_non_maxima_suppression(VlCovDet *covdet, vl_size num_features,
                                   double non_extrema_suppression) {
  vl_index i, j;
  double tol = non_extrema_suppression;
  VlCovDetFeature *features = (VlCovDetFeature *)vl_covdet_get_features(covdet);
  for (i = 0; i < (signed)num_features; ++i) {
    double x = features[i].frame.x;
    double y = features[i].frame.y;
    double sigma = features[i].frame.a11;
    double score = features[i].peakScore;

    for (j = 0; j < (signed)num_features; ++j) {
      double dx_ = features[j].frame.x - x;
      double dy_ = features[j].frame.y - y;
      double sigma_ = features[j].frame.a11;
      double score_ = features[j].peakScore;
      if (score_ == 0) {
        continue;
      }
      if (sigma < (1 + tol) * sigma_ && sigma_ < (1 + tol) * sigma &&
          vl_abs_d(dx_) < tol * sigma && vl_abs_d(dy_) < tol * sigma &&
          vl_abs_d(score) > vl_abs_d(score_)) {
        features[j].peakScore = 0;
      }
    }
  }
  j = 0;
  for (i = 0; i < (signed)num_features; ++i) {
    VlCovDetFeature feature = features[i];
    if (features[i].peakScore != 0) {
      features[j++] = feature;
    }
  }
  return j;
}

vl_size run_features_selection(VlCovDet *covdet, vl_size target_num_features) {
  vl_size numFeaturesKept = vl_covdet_get_num_features(covdet);

  // keep only 1.5 x targetNumFeatures for speeding-up duplicate detection
  if (target_num_features != 0) {
    const int to_keep = 3 * target_num_features / 2;
    numFeaturesKept = select_best_features(covdet, numFeaturesKept, to_keep);
  }

  // Remove non-maxima-in-their-neighborhood features
  const double nonMaximaSuppressionTol =
      vl_covdet_get_non_extrema_suppression_threshold(covdet);
  if (nonMaximaSuppressionTol > 0.) {
    numFeaturesKept = run_non_maxima_suppression(covdet, numFeaturesKept,
                                                 nonMaximaSuppressionTol);
  }

  // Keep the N best
  return select_best_features(covdet, numFeaturesKept, target_num_features);
}

std::vector<VlCovDetFeature> vlfeat_covdet_extract_orientations(
    VlCovDet *covdet, vl_size num_features) {
  VlCovDetFeature *features = (VlCovDetFeature *)vl_covdet_get_features(covdet);
  std::vector<VlCovDetFeature> vecFeatures;
  vecFeatures.reserve(num_features);
  vl_index i, j;
  for (i = 0; i < (signed)num_features; ++i) {
    vl_size numOrientations;
    VlCovDetFeature feature = features[i];
    VlCovDetFeatureOrientation *orientations =
        vl_covdet_extract_orientations_for_frame(covdet, &numOrientations,
                                                 feature.frame);

    for (j = 0; j < (signed)numOrientations; ++j) {
      double A[2 * 2] = {feature.frame.a11, feature.frame.a21,
                         feature.frame.a12, feature.frame.a22};
      double r1 = cos(orientations[j].angle);
      double r2 = sin(orientations[j].angle);

      vecFeatures.emplace_back(features[i]);
      VlCovDetFeature &oriented = vecFeatures.back();

      oriented.orientationScore = orientations[j].score;
      oriented.frame.a11 = +A[0] * r1 + A[2] * r2;
      oriented.frame.a21 = +A[1] * r1 + A[3] * r2;
      oriented.frame.a12 = -A[0] * r2 + A[2] * r1;
      oriented.frame.a22 = -A[1] * r2 + A[3] * r1;
    }
  }
  return vecFeatures;
}

py::tuple hahog(foundation::pyarray_f image, float peak_threshold,
                float edge_threshold, int target_num_features) {
  if (!image.size()) {
    return py::none();
  }

  std::vector<float> points;
  std::vector<float> desc;
  vl_size numFeatures;
  vl_size dimension = 128;

  {
    py::gil_scoped_release release;

    // create a detector object
    VlCovDet *covdet = vl_covdet_new(VL_COVDET_METHOD_HESSIAN);
    // set various parameters (optional)
    vl_covdet_set_first_octave(covdet, 0);
    vl_covdet_set_peak_threshold(covdet, peak_threshold);
    vl_covdet_set_edge_threshold(covdet, edge_threshold);

    // process the image and run the detector
    vl_covdet_put_image(covdet, image.data(), image.shape(1), image.shape(0));
    vl_covdet_set_non_extrema_suppression_threshold(covdet, 0);
    vl_covdet_detect(covdet, std::numeric_limits<vl_size>::max());

    // select the best features to keep
    numFeatures = run_features_selection(covdet, target_num_features);

    // compute the orientation of the features (optional)
    std::vector<VlCovDetFeature> vecFeatures =
        vlfeat_covdet_extract_orientations(covdet, numFeatures);
    numFeatures = vecFeatures.size();

    // get feature descriptors
    VlSiftFilt *sift = vl_sift_new(16, 16, 1, 3, 0);
    vl_index i;
    vl_index patchResolution = 15;
    double patchRelativeExtent = 7.5;
    double patchRelativeSmoothing = 1;
    vl_size patchSide = 2 * patchResolution + 1;
    double patchStep = (double)patchRelativeExtent / patchResolution;
    points.resize(4 * numFeatures);
    desc.resize(dimension * numFeatures);
    std::vector<float> patch(patchSide * patchSide);
    std::vector<float> patchXY(2 * patchSide * patchSide);

    vl_sift_set_magnif(sift, 3.0);
    for (i = 0; i < (signed)numFeatures; ++i) {
      const VlFrameOrientedEllipse &frame = vecFeatures.at(i).frame;
      float det = frame.a11 * frame.a22 - frame.a12 * frame.a21;
      float size = sqrt(fabs(det));
      float angle = atan2(frame.a21, frame.a11) * 180.0f / M_PI;
      points[4 * i + 0] = frame.x;
      points[4 * i + 1] = frame.y;
      points[4 * i + 2] = size;
      points[4 * i + 3] = angle;

      vl_covdet_extract_patch_for_frame(covdet, patch.data(), patchResolution,
                                        patchRelativeExtent,
                                        patchRelativeSmoothing, frame);

      vl_imgradient_polar_f(patchXY.data(), &patchXY[1], 2, 2 * patchSide,
                            patch.data(), patchSide, patchSide, patchSide);

      vl_sift_calc_raw_descriptor(
          sift, patchXY.data(), &desc[dimension * i], (int)patchSide,
          (int)patchSide, (double)(patchSide - 1) / 2,
          (double)(patchSide - 1) / 2,
          (double)patchRelativeExtent / (3.0 * (4 + 1) / 2) / patchStep,
          VL_PI / 2);
    }
    vl_sift_delete(sift);
    vl_covdet_delete(covdet);
  }

  return py::make_tuple(
      foundation::py_array_from_data(points.data(), numFeatures, 4),
      foundation::py_array_from_data(desc.data(), numFeatures, dimension));
}

}  // namespace features
