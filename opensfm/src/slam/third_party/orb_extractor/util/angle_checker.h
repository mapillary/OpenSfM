#ifndef OPENVSLAM_MATCH_ANGLE_CHECKER_H
#define OPENVSLAM_MATCH_ANGLE_CHECKER_H

#include <cassert>
#include <vector>
#include <numeric>
#include <algorithm>

#include <opencv2/core/fast_math.hpp>

namespace openvslam {
namespace match {

template<typename T>
class angle_checker {
public:
    /**
     * Constructor
     */
    explicit angle_checker(const unsigned int histogram_length = 30,
                           const unsigned int num_bins_thr = 3);

    /**
     * Destructor
     */
    ~angle_checker() = default;

    /**
     * Append a delta angle to the histogram
     */
    void append_delta_angle(float delta_angle, const T& match);

    /**
     * Get the valid matches sampled from the top-N of the histogram
     */
    std::vector<T> get_valid_matches() const;

    /**
     * Get the invalid matches sampled from the remainder of the top-N of the histogram
     */
    std::vector<T> get_invalid_matches() const;

private:
    /**
     * Index sort with the size of each vector
     */
    std::vector<unsigned int> index_sort_by_size(const std::vector<std::vector<T>>& vec) const;

    //! histogram length
    const unsigned int histogram_length_;
    //! inverse of the histogram length
    const float inv_histogram_length_;
    //! histogram object which contain match information at the corresponding bin
    std::vector<std::vector<T>> angle_histogram_;

    //! threshold of the number of VALID bins
    const unsigned int num_bins_thr_;
};

template<typename T>
angle_checker<T>::angle_checker(const unsigned int histogram_length, const unsigned int num_bins_thr)
        : histogram_length_(histogram_length), inv_histogram_length_(1.0f / histogram_length),
          num_bins_thr_(num_bins_thr) {
    assert(num_bins_thr_ <= histogram_length_);
    angle_histogram_.resize(histogram_length_);
    for (auto& bin : angle_histogram_) {
        bin.reserve(300);
    }
}

template<typename T>
inline void angle_checker<T>::append_delta_angle(float delta_angle, const T& match) {
    if (delta_angle < 0.0) {
        delta_angle += 360.0;
    }
    if (360.0 <= delta_angle) {
        delta_angle -= 360.0;
    }
    const auto bin = static_cast<unsigned int>(cvRound(delta_angle * inv_histogram_length_));

    assert(bin < histogram_length_);
    angle_histogram_.at(bin).push_back(match);
}

template<typename T>
std::vector<T> angle_checker<T>::get_valid_matches() const {
    std::vector<T> valid_matches;
    valid_matches.reserve(300 * num_bins_thr_);

    const auto bins = index_sort_by_size(angle_histogram_);
    for (unsigned int bin = 0; bin < histogram_length_; ++bin) {
        const bool is_valid_match = std::any_of(bins.begin(), bins.begin() + num_bins_thr_,
                                                [bin](const unsigned int i) { return bin == i; });
        if (is_valid_match) {
            const auto bin_begin_iter = angle_histogram_.at(bin).cbegin();
            const auto bin_end_iter = angle_histogram_.at(bin).cend();
            valid_matches.reserve(valid_matches.size() + std::distance(bin_begin_iter, bin_end_iter));
            valid_matches.insert(valid_matches.end(), bin_begin_iter, bin_end_iter);
        }
    }

    return valid_matches;
}

template<typename T>
std::vector<T> angle_checker<T>::get_invalid_matches() const {
    std::vector<T> invalid_matches;
    invalid_matches.reserve(300 * (histogram_length_ - num_bins_thr_));

    const auto bins = index_sort_by_size(angle_histogram_);
    for (unsigned int bin = 0; bin < histogram_length_; ++bin) {
        const bool is_valid_match = std::any_of(bins.begin(), bins.begin() + num_bins_thr_,
                                                [bin](const unsigned int i) { return bin == i; });
        if (!is_valid_match) {
            const auto bin_begin_iter = angle_histogram_.at(bin).cbegin();
            const auto bin_end_iter = angle_histogram_.at(bin).cend();
            invalid_matches.reserve(invalid_matches.size() + std::distance(bin_begin_iter, bin_end_iter));
            invalid_matches.insert(invalid_matches.end(), bin_begin_iter, bin_end_iter);
        }
    }

    return invalid_matches;
}

template<typename T>
std::vector<unsigned int> angle_checker<T>::index_sort_by_size(const std::vector<std::vector<T>>& vec) const {
    std::vector<unsigned int> indices(vec.size());
    std::iota(indices.begin(), indices.end(), 0);
    std::sort(indices.begin(), indices.end(),
              [&vec](const unsigned int a, const unsigned int b) {
                  return (vec.at(a).size() > vec.at(b).size());
              });
    return indices;
}

} // namespace match
} // namespace openvslam

#endif // OPENVSLAM_MATCH_ANGLE_CHECKER_H
