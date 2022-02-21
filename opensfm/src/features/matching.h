#pragma once

#include <foundation/python_types.h>
#include <foundation/types.h>

#include <set>

namespace features {

py::array_t<int> match_using_words(foundation::pyarray_f features1,
                                   foundation::pyarray_int words1,
                                   foundation::pyarray_f features2,
                                   foundation::pyarray_int words2,
                                   float lowes_ratio, int max_checks);

VecXf compute_vlad_descriptor(const MatXf &features, const MatXf &vlad_centers);

std::pair<std::vector<double>, std::vector<std::string>> compute_vlad_distances(
    const std::map<std::string, VecXf> &vlad_descriptors,
    const std::string &image, std::set<std::string> &other_images);
}  // namespace features
