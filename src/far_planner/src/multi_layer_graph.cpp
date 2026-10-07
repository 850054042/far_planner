#include "far_planner/multi_layer_graph.h"

NavEdge MultiLayerGraph::CanonicalEdge(const NavNodePtr& node1, const NavNodePtr& node2) {
    return node1->id < node2->id ? NavEdge(node1, node2) : NavEdge(node2, node1);
}

void MultiLayerGraph::Init(const rclcpp::Node::SharedPtr nh,
                           const MultiLayerGraphParams& params) {
    nh_ = nh;
    params_ = params;
    Reset();
    FARUtil::kFloorHeight = params_.floor_height;
    FARUtil::kInterLayerCostScale = std::max(1.0f, FARUtil::kInterLayerCostScale);
    if (!params_.auto_floor_origin) {
        FARUtil::kFloorOriginZ = params_.floor_origin_z;
        FARUtil::IsFloorOriginInitialized = true;
    }
}

void MultiLayerGraph::Reset() {
    gateway_nodes_.clear();
    pending_gateways_.clear();
    robot_layer_id_ = 0;
    robot_layer_initialized_ = false;
    if (params_.auto_floor_origin) {
        FARUtil::IsFloorOriginInitialized = false;
    }
}

void MultiLayerGraph::RefreshLayers(const NodePtrStack& graph) {
    for (const auto& node : graph) {
        if (node == nullptr) continue;
        if (!node->is_odom) {
            node->layer_id = FARUtil::LayerId(node->position.z);
            continue;
        }

        const int measured_layer = FARUtil::LayerId(node->position.z);
        if (!robot_layer_initialized_) {
            robot_layer_id_ = measured_layer;
            robot_layer_initialized_ = true;
        } else if (measured_layer != robot_layer_id_) {
            const int direction = measured_layer > robot_layer_id_ ? 1 : -1;
            const float boundary_z = FARUtil::kFloorOriginZ +
                (static_cast<float>(robot_layer_id_) + 0.5f * direction) *
                FARUtil::kFloorHeight;
            const bool crossed = direction > 0
                ? node->position.z > boundary_z + params_.layer_hysteresis
                : node->position.z < boundary_z - params_.layer_hysteresis;
            if (crossed) robot_layer_id_ = measured_layer;
        }
        node->layer_id = robot_layer_id_;
    }
}

bool MultiLayerGraph::IsGatewayCandidate(const NavNodePtr& node1,
                                         const NavNodePtr& node2) const {
    if (node1 == nullptr || node2 == nullptr || node1 == node2) return false;
    if (!node1->is_active || !node2->is_active || node1->is_odom || node2->is_odom) return false;
    return IsGatewayStillValid(node1, node2);
}

bool MultiLayerGraph::IsGatewayStillValid(const NavNodePtr& node1,
                                          const NavNodePtr& node2) const {
    if (node1 == nullptr || node2 == nullptr || node1 == node2) return false;
    if (node1->is_odom || node2->is_odom) return false;
    if (std::abs(node1->layer_id - node2->layer_id) != 1) return false;

    const Point3D delta = node2->position - node1->position;
    const float xy_dist = std::hypot(delta.x, delta.y);
    const float z_dist = std::abs(delta.z);
    if (xy_dist > params_.gateway_max_xy_dist || z_dist < params_.gateway_min_z_delta) return false;
    if (z_dist > params_.floor_height * 0.75f) return false;

    // PCT-style gateway test in graph form: the layer label changes while the
    // sampled ground remains spatially continuous (bounded local slope).
    const float slope = z_dist / std::max(xy_dist, FARUtil::kLeafSize);
    return slope <= params_.gateway_max_slope;
}

void MultiLayerGraph::ActivateGateway(const NavNodePtr& node1,
                                      const NavNodePtr& node2) {
    const bool is_new_gateway = !FARUtil::IsGatewayConnect(node1, node2);
    if (!FARUtil::IsTypeInStack(node2, node1->gateway_connects)) {
        node1->gateway_connects.push_back(node2);
    }
    if (!FARUtil::IsTypeInStack(node1, node2->gateway_connects)) {
        node2->gateway_connects.push_back(node1);
    }
    node1->is_gateway = true;
    node2->is_gateway = true;
    node1->is_covered = true;
    node2->is_covered = true;
    DynamicGraph::AddEdge(node1, node2);
    if (is_new_gateway) {
        RCLCPP_INFO(nh_->get_logger(),
            "Confirmed traversed gateway: node %zu (layer %d, z %.2f) <-> "
            "node %zu (layer %d, z %.2f)",
            node1->id, node1->layer_id, node1->position.z,
            node2->id, node2->layer_id, node2->position.z);
    }
}

void MultiLayerGraph::RemoveGateway(const NavNodePtr& node1,
                                    const NavNodePtr& node2) {
    FARUtil::EraseNodeFromStack(node2, node1->gateway_connects);
    FARUtil::EraseNodeFromStack(node1, node2->gateway_connects);
    node1->is_gateway = !node1->gateway_connects.empty();
    node2->is_gateway = !node2->gateway_connects.empty();
    DynamicGraph::EraseEdge(node1, node2);
}

void MultiLayerGraph::Update(NodePtrStack& graph, const NavNodePtr& odom_node) {
    if (!params_.enabled || odom_node == nullptr || graph.empty()) return;
    if (!FARUtil::IsFloorOriginInitialized) {
        FARUtil::kFloorOriginZ = params_.auto_floor_origin
            ? odom_node->position.z : params_.floor_origin_z;
        FARUtil::IsFloorOriginInitialized = true;
    }
    RefreshLayers(graph);

    // A traversed trajectory is the strongest possible evidence of a gateway.
    // It is therefore accepted immediately and remains available for replanning.
    int trajectory_edges = 0;
    int cross_layer_trajectory_edges = 0;
    int rejected_cross_layer_edges = 0;
    bool traversed_gateway_found = false;
    for (const auto& node : graph) {
        if (node == nullptr || node->is_odom || !node->is_navpoint) continue;
        for (const auto& neighbor : node->trajectory_connects) {
            // Do not require active endpoints here.  A trajectory edge is
            // persistent evidence that the robot physically traversed this
            // segment; one endpoint may leave the local active window before
            // the layer transition is processed.
            if (neighbor != nullptr && neighbor->is_navpoint && node->id < neighbor->id) {
                trajectory_edges++;
                if (!FARUtil::IsAtSameLayer(node, neighbor)) {
                    cross_layer_trajectory_edges++;
                    if (IsGatewayStillValid(node, neighbor)) {
                        ActivateGateway(node, neighbor);
                        traversed_gateway_found = true;
                    } else {
                        rejected_cross_layer_edges++;
                    }
                }
            }
        }
    }

    if (!gateway_nodes_.empty()) traversed_gateway_found = true;

    // The dynamic graph may skip a direct trajectory connection when a stair
    // node temporarily leaves the local active set. If both adjacent floors
    // nevertheless contain navpoints created from the robot's real path, use
    // the closest geometrically valid pair as the traversed transition. This
    // remains stricter than ordinary visibility-edge inference because contour
    // and frontier nodes are never considered.
    if (!traversed_gateway_found && robot_layer_initialized_ && robot_layer_id_ != 0) {
        NavNodePtr best_node1 = nullptr;
        NavNodePtr best_node2 = nullptr;
        float best_distance = FARUtil::kINF;
        for (std::size_t i = 0; i < graph.size(); ++i) {
            const auto& node1 = graph[i];
            if (node1 == nullptr || node1->is_odom || !node1->is_navpoint) continue;
            for (std::size_t j = i + 1; j < graph.size(); ++j) {
                const auto& node2 = graph[j];
                if (node2 == nullptr || node2->is_odom || !node2->is_navpoint ||
                    FARUtil::IsAtSameLayer(node1, node2) ||
                    !IsGatewayStillValid(node1, node2)) continue;
                const float distance = (node2->position - node1->position).norm();
                if (distance < best_distance) {
                    best_distance = distance;
                    best_node1 = node1;
                    best_node2 = node2;
                }
            }
        }
        if (best_node1 != nullptr && best_node2 != nullptr) {
            const bool recovered_is_new =
                !FARUtil::IsGatewayConnect(best_node1, best_node2);
            ActivateGateway(best_node1, best_node2);
            traversed_gateway_found = true;
            if (recovered_is_new) {
                RCLCPP_INFO(nh_->get_logger(),
                    "Recovered traversed gateway from nearest adjacent-layer "
                    "trajectory nodes (distance %.2f m).", best_distance);
            }
        }
    }

    if (robot_layer_initialized_ && robot_layer_id_ != 0 &&
        !traversed_gateway_found &&
        (cross_layer_trajectory_edges == 0 ||
         rejected_cross_layer_edges == cross_layer_trajectory_edges)) {
        RCLCPP_WARN_THROTTLE(
            nh_->get_logger(), *nh_->get_clock(), 5000,
            "No gateway yet on layer %d: trajectory_edges=%d, cross_layer=%d, "
            "geometry_rejected=%d (max_xy=%.2f, max_slope=%.2f)",
            robot_layer_id_, trajectory_edges, cross_layer_trajectory_edges,
            rejected_cross_layer_edges, params_.gateway_max_xy_dist,
            params_.gateway_max_slope);
    }

    // Seed PCT-style candidates from terrain-validated graph edges. Candidates
    // survive removal from the planning graph while gathering confirmations.
    for (const auto& node : graph) {
        if (node == nullptr) continue;
        const NodePtrStack connections = node->connect_nodes;
        for (const auto& neighbor : connections) {
            if (node->id >= neighbor->id || FARUtil::IsAtSameLayer(node, neighbor)) continue;
            // In the robust multi-floor profile, arbitrary visibility edges
            // are not allowed to create gateways. A single bad contour height
            // on an otherwise flat floor would otherwise produce purple nodes.
            if (params_.require_trajectory_evidence) continue;
            const NavEdge edge = CanonicalEdge(node, neighbor);
            if (IsGatewayCandidate(node, neighbor)) {
                if (pending_gateways_.find(edge) == pending_gateways_.end()) {
                    pending_gateways_.insert({edge, 0});
                }
            }
        }
    }

    for (auto it = pending_gateways_.begin(); it != pending_gateways_.end();) {
        const NavNodePtr node1 = it->first.first;
        const NavNodePtr node2 = it->first.second;
        if (!IsGatewayCandidate(node1, node2)) {
            it = pending_gateways_.erase(it);
            continue;
        }
        it->second += 1;
        if (it->second >= params_.gateway_confirmations) {
            ActivateGateway(node1, node2);
            it = pending_gateways_.erase(it);
        } else {
            ++it;
        }
    }

    // Enforce stacked independent 2-D visibility graphs. Cross-layer edges that
    // are not confirmed gateways must never reach the existing Dijkstra search.
    std::vector<NavEdge> invalid_edges;
    for (const auto& node : graph) {
        if (node == nullptr) continue;
        const NodePtrStack connections = node->connect_nodes;
        for (const auto& neighbor : connections) {
            if (node->id < neighbor->id && !FARUtil::IsAtSameLayer(node, neighbor) &&
                !FARUtil::IsGatewayConnect(node, neighbor)) {
                invalid_edges.push_back(CanonicalEdge(node, neighbor));
            }
        }
    }
    for (const auto& edge : invalid_edges) {
        DynamicGraph::EraseEdge(edge.first, edge.second);
        DynamicGraph::ErasePolyEdge(edge.first, edge.second);
    }

    // A confirmed gateway is global graph knowledge.  Do not remove it merely
    // because one endpoint left the local active window: doing so makes a ramp
    // usable on the outward trip but unavailable for the return trip.
    for (const auto& node : graph) {
        if (node == nullptr || !node->is_gateway) continue;
        const NodePtrStack gateways = node->gateway_connects;
        for (const auto& neighbor : gateways) {
            if (node->id >= neighbor->id) continue;
            if (!IsGatewayStillValid(node, neighbor)) {
                RemoveGateway(node, neighbor);
            } else {
                // Dynamic graph maintenance may have cleared the ordinary edge;
                // restore the bidirectional planning edge from persistent gateway
                // metadata on every update.
                DynamicGraph::AddEdge(node, neighbor);
            }
        }
    }

    gateway_nodes_.clear();
    for (const auto& node : graph) {
        if (node != nullptr && node->is_gateway) gateway_nodes_.push_back(node);
    }

    if (FARUtil::IsDebug && !gateway_nodes_.empty()) {
        RCLCPP_INFO_THROTTLE(
            nh_->get_logger(), *nh_->get_clock(), 5000,
            "MultiLayerGraph: %zu gateway nodes connect stacked floor graphs.",
            gateway_nodes_.size());
    }
}
