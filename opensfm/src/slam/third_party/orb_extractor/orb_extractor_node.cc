#include "orb_extractor_node.h"
#include <iostream>
namespace openvslam {
namespace feature {

std::array<orb_extractor_node, 4> orb_extractor_node::divide_node() {
    // Half width/height of the allocated patch area
    const unsigned int half_x = cvCeil((pt_end_.x - pt_begin_.x) / 2.0);
    const unsigned int half_y = cvCeil((pt_end_.y - pt_begin_.y) / 2.0);
    // std::cout << "dividing pt_begin_: "
    //           << pt_begin_.x << "/" << pt_begin_.y 
    //           << " pt_end: " << pt_end_.x << "/" << pt_end_.y << 
    //           "hal_x: " << half_x << "/" << half_y<< std::endl;
    // Four new child nodes
    std::array<orb_extractor_node, 4> child_nodes;

    // A position of center top, left center, center, right center, and center bottom
    // These positions are used to determine new split areas
    const auto pt_top = cv::Point2i(pt_begin_.x + half_x, pt_begin_.y);
    const auto pt_left = cv::Point2i(pt_begin_.x, pt_begin_.y + half_y);
    const auto pt_center = cv::Point2i(pt_begin_.x + half_x, pt_begin_.y + half_y);
    const auto pt_right = cv::Point2i(pt_end_.x, pt_begin_.y + half_y);
    const auto pt_bottom = cv::Point2i(pt_begin_.x + half_x, pt_end_.y);
    // std::cout << "Creating: "
    //            << "pt_top " << pt_top.x << ", " << pt_top.y
    //            << "pt_left " << pt_left.x << ", " << pt_left.y
    //            << "pt_center " << pt_center.x << ", " << pt_center.y
    //            << "pt_right " << pt_right.x << ", " << pt_right.y
    //            << "pt_bottom " << pt_bottom.x << ", " << pt_bottom.y << std::endl;

    // Assign new patch border for each child nodes
    child_nodes.at(0).pt_begin_ = pt_begin_;
    child_nodes.at(0).pt_end_ = pt_center;
    child_nodes.at(1).pt_begin_ = pt_top;
    child_nodes.at(1).pt_end_ = pt_right;
    child_nodes.at(2).pt_begin_ = pt_left;
    child_nodes.at(2).pt_end_ = pt_bottom;
    child_nodes.at(3).pt_begin_ = pt_center;
    child_nodes.at(3).pt_end_ = pt_end_;

    // Memory reservation for child nodes
    for (auto& node : child_nodes) {
        node.keypts_.reserve(keypts_.size());
    }

    // Distribute keypoints to child nodes
    for (const auto& keypt : keypts_) {
        unsigned int idx = 0;
        if (pt_begin_.x + half_x <= keypt.pt.x) {
            idx += 1;
        }
        if (pt_begin_.y + half_y <= keypt.pt.y) {
            idx += 2;
        }
        // std::cout << "keypt: " << &keypt-&keypts_[0] << "/" << keypts_.size() << "/" << idx << "kpt: " << keypt.pt.x << "/" << keypt.pt.y << std::endl;
        child_nodes.at(idx).keypts_.push_back(keypt);
    }

    return child_nodes;
}

} // namespace feature
} // namespace openvslam
