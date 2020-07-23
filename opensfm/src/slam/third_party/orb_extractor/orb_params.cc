// #include "openvslam/feature/orb_params.h"
#include "orb_params.h"
#include <iostream>

namespace openvslam {
namespace feature {

orb_params::orb_params(const unsigned int max_num_keypts, const float scale_factor, const unsigned int num_levels,
                       const unsigned int ini_fast_thr, const unsigned int min_fast_thr,
                       const std::vector<std::vector<float>>& mask_rects, bool forceMapFeatures)
        : max_num_keypts_(max_num_keypts), scale_factor_(scale_factor), num_levels_(num_levels),
          ini_fast_thr_(ini_fast_thr), min_fast_thr(min_fast_thr),
          mask_rects_(mask_rects), forceMapFeatures_(forceMapFeatures) {
    for (const auto& v : mask_rects_) {
        if (v.size() != 4) {
            throw std::runtime_error("mask rectangle must contain four parameters");
        }
        if (v.at(0) >= v.at(1)) {
            throw std::runtime_error("x_max must be greater than x_min");
        }
        if (v.at(2) >= v.at(3)) {
            throw std::runtime_error("y_max must be greater than x_min");
        }
    }
}

// orb_params::orb_params(const YAML::Node& yaml_node)
//         : orb_params(yaml_node["Feature.max_num_keypoints"].as<unsigned int>(2000),
//                      yaml_node["Feature.scale_factor"].as<float>(1.2),
//                      yaml_node["Feature.num_levels"].as<unsigned int>(8),
//                      yaml_node["Feature.ini_fast_threshold"].as<unsigned int>(20),
//                      yaml_node["Feature.min_fast_threshold"].as<unsigned int>(7),
//                      yaml_node["Feature.mask_rectangles"].as<std::vector<std::vector<float>>>(std::vector<std::vector<float>>()),
//                      yaml_node["Feature.ForceKeypointsOnMapFeatures"].as<bool>(false)) {}

void orb_params::show_parameters() const {
    std::cout << "- number of keypoints: " << max_num_keypts_ << std::endl;
    std::cout << "- scale factor: " << scale_factor_ << std::endl;
    std::cout << "- number of levels: " << num_levels_ << std::endl;
    std::cout << "- initial fast threshold: " << ini_fast_thr_ << std::endl;
    std::cout << "- minimum fast threshold: " << min_fast_thr << std::endl;
    if (!mask_rects_.empty()) {
        std::cout << "- mask rectangles:" << std::endl;
        for (const auto& mask_rect : mask_rects_) {
            std::cout << "  - ["
                      << mask_rect.at(0) << ", "
                      << mask_rect.at(1) << ", "
                      << mask_rect.at(2) << ", "
                      << mask_rect.at(3) << "]" << std::endl;
        }
    }
}

std::vector<float> orb_params::calc_scale_factors(const unsigned int num_scale_levels, const float scale_factor) {
    std::vector<float> scale_factors(num_scale_levels, 1.0);
    for (unsigned int level = 1; level < num_scale_levels; ++level) {
        scale_factors.at(level) = scale_factor * scale_factors.at(level - 1);
    }
    return scale_factors;
}

std::vector<float> orb_params::calc_inv_scale_factors(const unsigned int num_scale_levels, const float scale_factor) {
    std::vector<float> inv_scale_factors(num_scale_levels, 1.0);
    for (unsigned int level = 1; level < num_scale_levels; ++level) {
        inv_scale_factors.at(level) = (1.0f / scale_factor) * inv_scale_factors.at(level - 1);
    }
    return inv_scale_factors;
}

std::vector<float> orb_params::calc_level_sigma_sq(const unsigned int num_scale_levels, const float scale_factor) {
    float scale_factor_at_level = 1.0;
    std::vector<float> level_sigma_sq(num_scale_levels, 1.0);
    for (unsigned int level = 1; level < num_scale_levels; ++level) {
        scale_factor_at_level = scale_factor * scale_factor_at_level;
        level_sigma_sq.at(level) = scale_factor_at_level * scale_factor_at_level;
    }
    return level_sigma_sq;
}

std::vector<float> orb_params::calc_inv_level_sigma_sq(const unsigned int num_scale_levels, const float scale_factor) {
    float scale_factor_at_level = 1.0;
    std::vector<float> inv_level_sigma_sq(num_scale_levels, 1.0);
    for (unsigned int level = 1; level < num_scale_levels; ++level) {
        scale_factor_at_level = scale_factor * scale_factor_at_level;
        inv_level_sigma_sq.at(level) = 1.0f / (scale_factor_at_level * scale_factor_at_level);
    }
    return inv_level_sigma_sq;
}

} // namespace feature
} // namespace openvslam
