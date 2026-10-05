#ifndef MULTI_LAYER_GRAPH_H
#define MULTI_LAYER_GRAPH_H

#include "utility.h"
#include "dynamic_graph.h"

struct MultiLayerGraphParams {
    MultiLayerGraphParams() = default;
    bool enabled = false;
    float floor_height = 2.5f;
    float floor_origin_z = 0.0f;
    bool auto_floor_origin = true;
    float gateway_max_xy_dist = 1.5f;
    float gateway_min_z_delta = 0.05f;
    float gateway_max_slope = 0.6f;
    int gateway_confirmations = 2;
    float layer_hysteresis = 0.15f;
    bool require_trajectory_evidence = true;
};

class MultiLayerGraph {
private:
    rclcpp::Node::SharedPtr nh_;
    MultiLayerGraphParams params_;
    NodePtrStack gateway_nodes_;
    std::unordered_map<NavEdge, int, navedge_hash> pending_gateways_;
    int robot_layer_id_ = 0;
    bool robot_layer_initialized_ = false;

    static NavEdge CanonicalEdge(const NavNodePtr& node1, const NavNodePtr& node2);
    bool IsGatewayCandidate(const NavNodePtr& node1, const NavNodePtr& node2) const;
    bool IsGatewayStillValid(const NavNodePtr& node1, const NavNodePtr& node2) const;
    void ActivateGateway(const NavNodePtr& node1, const NavNodePtr& node2);
    void RemoveGateway(const NavNodePtr& node1, const NavNodePtr& node2);
    void RefreshLayers(const NodePtrStack& graph);

public:
    MultiLayerGraph() = default;
    ~MultiLayerGraph() = default;

    void Init(const rclcpp::Node::SharedPtr nh, const MultiLayerGraphParams& params);
    void Update(NodePtrStack& graph, const NavNodePtr& odom_node);
    void Reset();

    const NodePtrStack& GetGatewayNodes() const { return gateway_nodes_; }
};

#endif
