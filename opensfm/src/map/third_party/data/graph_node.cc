#include <map/shot.h>
#include <map/landmark.h>
#include <map/third_party/data/graph_node.h>
namespace data {

graph_node::graph_node(map::Shot* shot, const bool spanning_parent_is_not_set)
    : owner_shot_(shot), spanning_parent_is_not_set_(spanning_parent_is_not_set) {}

void graph_node::add_connection(map::Shot* shot, const unsigned int weight) {
    bool need_update = false;
    {
        // std::lock_guard<std::mutex> lock(mtx_);
        if (!connected_shots_and_weights_.count(shot)) {
            // if `shot` not exists
            connected_shots_and_weights_[shot] = weight;
            need_update = true;
        }
        else if (connected_shots_and_weights_.at(shot) != weight) {
            // if the weight is updated
            connected_shots_and_weights_.at(shot) = weight;
            need_update = true;
        }
    }

    if (need_update) {
        update_covisibility_orders();
    }
}

void graph_node::erase_connection(map::Shot* shot) {
    bool need_update = false;
    {
        // std::lock_guard<std::mutex> lock(mtx_);
        if (connected_shots_and_weights_.count(shot)) {
            connected_shots_and_weights_.erase(shot);
            need_update = true;
        }
    }

    if (need_update) {
        update_covisibility_orders();
    }
}

void graph_node::erase_all_connections() {
    // remote myself from the connected keyframes
    for (const auto& shot_and_weight : connected_shots_and_weights_) {
        shot_and_weight.first->slam_data_.graph_node_->erase_connection(owner_shot_);
    }
    // remove the buffers
    connected_shots_and_weights_.clear();
    ordered_covisibilities_.clear();
    ordered_weights_.clear();
}

void graph_node::update_connections() {
    // const auto landmarks = owner_shot_->get_landmarks();
    const auto& landmarks = owner_shot_->GetLandmarks();

    std::map<map::Shot*, unsigned int> shot_weights;
    for (const auto lm : landmarks) {
        if (lm != nullptr) {
            // continue;
        // }
        // if (lm->will_be_erased()) {
        //     continue;
        // }

            const auto observations = lm->GetObservations();

            for (const auto& obs : observations) {
                auto shot = obs.first;

                if (*shot == *owner_shot_) {
                    continue;
                }
                // count up weight of `shot`
                shot_weights[shot]++;
            }
        }
    }

    if (shot_weights.empty()) {
        return;
    }

    unsigned int max_weight = 0;
    map::Shot* nearest_covisibility = nullptr;

    // vector for sorting
    std::vector<std::pair<map::ShotUniqueId, map::Shot*>> weight_covisibility_pairs;
    weight_covisibility_pairs.reserve(shot_weights.size());
    for (const auto& shot_weight : shot_weights) {
        auto shot = shot_weight.first;
        const auto weight = shot_weight.second;

        if (max_weight <= weight) {
            max_weight = weight;
            nearest_covisibility = shot;
        }

        if (weight_thr_ < weight) {
            weight_covisibility_pairs.emplace_back(std::make_pair(weight, shot));
        }
    }
    // add ONE node at least
    if (weight_covisibility_pairs.empty()) {
        weight_covisibility_pairs.emplace_back(std::make_pair(max_weight, nearest_covisibility));
    }

    // add connection from the covisibility to myself
    for (const auto& weight_covisibility : weight_covisibility_pairs) {
        auto covisibility = weight_covisibility.second;
        const auto weight = weight_covisibility.first;
        covisibility->slam_data_.graph_node_->add_connection(owner_shot_, weight);
    }
    // for (const auto& wc : weight_covisibility_pairs)
    // {
    //     std::cout << "wc: " << wc.first << " sec: " << wc.second->name_ << "/" << wc.second->id_ << std::endl;
    // }
    // std::pair<map::ShotUniqueId, map::Shot*>
    // sort with weights
    std::sort(weight_covisibility_pairs.rbegin(),
              weight_covisibility_pairs.rend(),
              [](const std::pair<map::ShotUniqueId, map::Shot*>& x,
                 const std::pair<map::ShotUniqueId, map::Shot*>& y) {
                if (x.first < y.first) return true;
                if (x.first == y.first) return x.second->id_ < y.second->id_;
                return false;
              });
    // for (const auto& wc : weight_covisibility_pairs)
    // {
    //     std::cout << "wc after sort: " << wc.first << " sec: " << wc.second->name_ << "/" << wc.second->id_ << std::endl;
    // }
    decltype(ordered_covisibilities_) ordered_covisibilities;
    ordered_covisibilities.reserve(weight_covisibility_pairs.size());
    decltype(ordered_weights_) ordered_weights;
    ordered_weights.reserve(weight_covisibility_pairs.size());
    for (const auto& weight_shot_pair : weight_covisibility_pairs)
    {
        // std::cout << "Push ord. cov: " << weight_shot_pair.second->name_ << std::endl;
        ordered_covisibilities.push_back(weight_shot_pair.second);
        ordered_weights.push_back(weight_shot_pair.first);
    }

    {
        // std::lock_guard<std::mutex> lock(mtx_);

        connected_shots_and_weights_ = shot_weights;
        ordered_covisibilities_ = ordered_covisibilities;
        ordered_weights_ = ordered_weights;

        if (spanning_parent_is_not_set_ && owner_shot_->unique_id_ != 0) {
            // set the parent of spanning tree
            assert(*nearest_covisibility == *ordered_covisibilities.front());
            spanning_parent_ = nearest_covisibility;
            spanning_parent_->slam_data_.graph_node_->add_spanning_child(owner_shot_);
            spanning_parent_is_not_set_ = false;
        }
    }
}

void graph_node::update_covisibility_orders() {
    // std::lock_guard<std::mutex> lock(mtx_);

    std::vector<std::pair<size_t, map::Shot*>> weight_shot_pairs;
    weight_shot_pairs.reserve(connected_shots_and_weights_.size());

    for (const auto& shot_and_weight : connected_shots_and_weights_) {
        weight_shot_pairs.emplace_back(std::make_pair(shot_and_weight.second, shot_and_weight.first));
    }

    // sort with weights
    std::sort(weight_shot_pairs.rbegin(), weight_shot_pairs.rend(),
              [](const std::pair<size_t, map::Shot*>& x,
                 const std::pair<size_t, map::Shot*>& y) {
                return x.first < y.first && x.second->id_ < y.second->id_;
              });

    ordered_covisibilities_.clear();
    ordered_covisibilities_.reserve(weight_shot_pairs.size());
    ordered_weights_.clear();
    ordered_weights_.reserve(weight_shot_pairs.size());
    for (const auto& weight_shot_pair : weight_shot_pairs) {
        // std::cout << ""
        ordered_covisibilities_.push_back(weight_shot_pair.second);
        ordered_weights_.push_back(weight_shot_pair.first);
    }
}

std::set<map::Shot*> graph_node::get_connected_keyframes() const {
    // std::lock_guard<std::mutex> lock(mtx_);
    std::set<map::Shot*> shots;

    for (const auto& shot_and_weight : connected_shots_and_weights_) {
        shots.insert(shot_and_weight.first);
    }

    return shots;
}

std::vector<map::Shot*> graph_node::get_covisibilities() const {
    // std::lock_guard<std::mutex> lock(mtx_);
    return ordered_covisibilities_;
}

std::vector<map::Shot*> graph_node::get_top_n_covisibilities(const unsigned int num_covisibilities) const {
    // std::lock_guard<std::mutex> lock(mtx_);
    if (ordered_covisibilities_.size() < num_covisibilities) {
        return ordered_covisibilities_;
    }
    else {
        return std::vector<map::Shot*>(ordered_covisibilities_.begin(), ordered_covisibilities_.begin() + num_covisibilities);
    }
}

std::vector<map::Shot*> graph_node::get_covisibilities_over_weight(const unsigned int weight) const {
    // std::lock_guard<std::mutex> lock(mtx_);

    if (ordered_covisibilities_.empty()) {
        return std::vector<map::Shot*>();
    }

    auto itr = std::upper_bound(ordered_weights_.begin(), ordered_weights_.end(), weight, std::greater<unsigned int>());
    if (itr == ordered_weights_.end()) {
        return std::vector<map::Shot*>();
    }
    else {
        const auto num = static_cast<unsigned int>(itr - ordered_weights_.begin());
        return std::vector<map::Shot*>(ordered_covisibilities_.begin(), ordered_covisibilities_.begin() + num);
    }
}

unsigned int graph_node::get_weight(map::Shot* shot) const {
    // std::lock_guard<std::mutex> lock(mtx_);
    if (connected_shots_and_weights_.count(shot)) {
        return connected_shots_and_weights_.at(shot);
    }
    else {
        return 0;
    }
}

void graph_node::set_spanning_parent(map::Shot* shot) {
    // std::lock_guard<std::mutex> lock(mtx_);
    assert(!spanning_parent_);
    spanning_parent_ = shot;
}

map::Shot* graph_node::get_spanning_parent() const {
    // std::lock_guard<std::mutex> lock(mtx_);
    return spanning_parent_;
}

void graph_node::change_spanning_parent(map::Shot* shot) {
    // std::lock_guard<std::mutex> lock(mtx_);
    spanning_parent_ = shot;
    shot->slam_data_.graph_node_->add_spanning_child(owner_shot_);
}

void graph_node::add_spanning_child(map::Shot* shot) {
    // std::lock_guard<std::mutex> lock(mtx_);
    spanning_children_.insert(shot);
}

void graph_node::erase_spanning_child(map::Shot* shot) {
    // std::lock_guard<std::mutex> lock(mtx_);
    spanning_children_.erase(shot);
}

void graph_node::recover_spanning_connections() {
    // std::lock_guard<std::mutex> lock(mtx_);

    // 1. find new parents for my children

    std::set<map::Shot*> new_parent_candidates;
    new_parent_candidates.insert(spanning_parent_);

    while (!spanning_children_.empty()) {
        bool max_is_found = false;

        unsigned int max_weight = 0;
        map::Shot* max_weight_parent = nullptr;
        map::Shot* max_weight_child = nullptr;

        for (const auto spanning_child : spanning_children_) {
            // if (spanning_child->will_be_erased()) {
            //     continue;
            // }

            // get intersection between the parent candidates and the spanning-child's covisibilities
            const auto child_covisibilities = spanning_child->slam_data_.graph_node_->get_covisibilities();
            const auto intersection = extract_intersection(new_parent_candidates, child_covisibilities);

            // find the new parent (which has the maximum weight with the spanning child) from the intersection
            for (const auto parent_candidate : intersection) {
                const auto weight = spanning_child->slam_data_.graph_node_->get_weight(parent_candidate);
                if (max_weight < weight) {
                    max_weight = weight;
                    max_weight_parent = parent_candidate;
                    max_weight_child = spanning_child;
                    max_is_found = true;
                }
            }
        }

        if (max_is_found) {
            // update spanning tree
            max_weight_child->slam_data_.graph_node_->change_spanning_parent(max_weight_parent);
            spanning_children_.erase(max_weight_child);
            new_parent_candidates.insert(max_weight_child);
        }
        else {
            // cannot update anymore
            break;
        }
    }

    // if it should be fixed
    if (!spanning_children_.empty()) {
        // set my parent as the new parent
        for (const auto spanning_child : spanning_children_) {
            spanning_child->slam_data_.graph_node_->change_spanning_parent(spanning_parent_);
        }
    }

    spanning_children_.clear();

    // 2. remove myself from my parent's children list

    spanning_parent_->slam_data_.graph_node_->erase_spanning_child(owner_shot_);
}

std::set<map::Shot*> graph_node::get_spanning_children() const {
    // std::lock_guard<std::mutex> lock(mtx_);
    return spanning_children_;
}

bool graph_node::has_spanning_child(map::Shot* shot) const {
    // std::lock_guard<std::mutex> lock(mtx_);
    return static_cast<bool>(spanning_children_.count(shot));
}

// void graph_node::add_loop_edge(map::Shot* shot) {
//     // std::lock_guard<std::mutex> lock(mtx_);
//     loop_edges_.insert(shot);
//     // cannot erase loop edges
//     owner_shot_->set_not_to_be_erased();
// }

// std::set<map::Shot*> graph_node::get_loop_edges() const {
//     // std::lock_guard<std::mutex> lock(mtx_);
//     return loop_edges_;
// }

// bool graph_node::has_loop_edge() const {
//     // std::lock_guard<std::mutex> lock(mtx_);
//     return !loop_edges_.empty();
// }

template<typename T, typename U>
std::vector<map::Shot*> graph_node::extract_intersection(const T& shots_1, const U& shots_2) {
    std::vector<map::Shot*> intersection;
    intersection.reserve(std::min(shots_1.size(), shots_2.size()));
    for (const auto shot_1 : shots_1) {
        for (const auto shot_2 : shots_2) {
            if (*shot_1 == *shot_2) {
                intersection.push_back(shot_1);
            }
        }
    }
    return intersection;
}
} // namespace data
