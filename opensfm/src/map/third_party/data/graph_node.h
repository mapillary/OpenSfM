#ifndef OPENVSLAM_DATA_GRAPH_NODE_H
#define OPENVSLAM_DATA_GRAPH_NODE_H

#include <mutex>
#include <vector>
#include <map>
#include <set>


namespace map
{
class Shot;
}
namespace data
{

class graph_node {
public:
    /**
     * Constructor
     */
    explicit graph_node(map::Shot* keyfrm, const bool spanning_parent_is_not_set = true);

    /**
     * Destructor
     */
    ~graph_node() = default;

    //-----------------------------------------
    // covisibility graph

    /**
     * Add connection between myself and specified keyframes with the weight
     */
    void add_connection(map::Shot* keyfrm, const unsigned int weight);

    /**
     * Erase connection between myself and specified keyframes
     */
    void erase_connection(map::Shot* keyfrm);

    /**
     * Erase all connections
     */
    void erase_all_connections();

    /**
     * Update the connections and the covisibilities by referring landmark observations
     */
    void update_connections();

    /**
     * Update the order of the covisibilities
     * (NOTE: the new keyframe won't inserted)
     */
    void update_covisibility_orders();

    /**
     * Get the connected keyframes
     */
    std::set<map::Shot*> get_connected_keyframes() const;

    /**
     * Get the covisibility keyframes
     */
    std::vector<map::Shot*> get_covisibilities() const;

    /**
     * Get the top-n covisibility keyframes
     */
    std::vector<map::Shot*> get_top_n_covisibilities(const unsigned int num_covisibilities) const;

    /**
     * Get the covisibility keyframes which have weights over the threshold with myself
     */
    std::vector<map::Shot*> get_covisibilities_over_weight(const unsigned int weight) const;

    /**
     * Get the weight between this and specified keyframe
     */
    unsigned int get_weight(map::Shot* keyfrm) const;

    //-----------------------------------------
    // spanning tree

    /**
     * Set the parent node of spanning tree
     * (NOTE: this functions will be only used for map loading)
     */
    void set_spanning_parent(map::Shot* keyfrm);

    /**
     * Get the parent of spanning tree
     */
    map::Shot* get_spanning_parent() const;

    /**
     * Change the parent node of spanning tree
     */
    void change_spanning_parent(map::Shot* keyfrm);

    /**
     * Add the child note of spanning tree
     */
    void add_spanning_child(map::Shot* keyfrm);

    /**
     * Erase the child node of spanning tree
     */
    void erase_spanning_child(map::Shot* keyfrm);

    /**
     * Recover the spanning connections of the connected keyframes
     */
    void recover_spanning_connections();

    /**
     * Get the children of spanning tree
     */
    std::set<map::Shot*> get_spanning_children() const;

    /**
     * Whether this node has the specified child or not
     */
    bool has_spanning_child(map::Shot* keyfrm) const;

    //-----------------------------------------
    // loop edge

    /**
     * Add the loop edge
     */
    // void add_loop_edge(map::Shot* keyfrm);

    /**
     * Get the loop edges
     */
    // std::set<map::Shot*> get_loop_edges() const;

    /**
     * Whether this node has any loop edges or not
     */
    // bool has_loop_edge() const;

private:
    /**
     * Extract intersection from the two lists of keyframes
     */
    template<typename T, typename U>
    static std::vector<map::Shot*> extract_intersection(const T& keyfrms_1, const U& keyfrms_2);

    //! keyframe of this node
    map::Shot* const owner_shot_;

    //! all connected keyframes and their weights
    std::map<map::Shot*, unsigned int> connected_shots_and_weights_;

    //! minimum threshold for covisibility graph connection
    static constexpr unsigned int weight_thr_ = 15;
    //! covisibility keyframe in descending order ot weights
    std::vector<map::Shot*> ordered_covisibilities_;
    //! weights in descending order
    std::vector<unsigned int> ordered_weights_;

    //! parent of spanning tree
    map::Shot* spanning_parent_ = nullptr;
    //! children of spanning tree
    std::set<map::Shot*> spanning_children_;
    //! flag which indicates spanning tree is not set yet or not
    bool spanning_parent_is_not_set_;

    //! loop edges
    // std::set<cslam::KeyFrame*> loop_edges_;

    //! need mutex for access to connections
    // mutable std::mutex mtx_;
};

} // namespace data

#endif // OPENVSLAM_DATA_GRAPH_NODE_H
