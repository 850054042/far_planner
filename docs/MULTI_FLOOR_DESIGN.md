# FAR Planner multi-floor extension

## Goal

Keep the original FAR visibility-graph and Dijkstra implementation, but turn the
global graph into stacked per-floor 2-D visibility graphs. The only legal
cross-floor transitions are confirmed gateway edges.

## Data model

Every `NavNode` has:

- `layer_id`: the nearest floor index relative to `floor_origin_z`;
- `is_gateway`: whether the node participates in a cross-floor transition;
- `gateway_connects`: confirmed cross-floor neighbours.

The layer index is:

```text
round((node_z - floor_origin_z) / floor_height)
```

By default, `floor_origin_z` is captured from the first odometry height. This
means the vehicle body height is naturally included and floor 0/1 vehicle
positions differ by exactly the configured floor height.

## Gateway detection

The implementation adapts PCT's gateway concept to FAR's online graph instead
of copying PCT's offline tomogram representation.

PCT identifies a gateway where the discrete layer changes while adjacent ground
elevation stays continuous. FAR already validates candidate edges against its
local terrain map, so `MultiLayerGraph` uses an equivalent graph-domain test:

1. endpoints belong to adjacent layers;
2. their horizontal distance is local;
3. vertical change is non-zero but less than 0.75 floor height;
4. implied ground slope is below `gateway_max_slope`;
5. in the default robust profile, the edge must be an actually traversed
   trajectory connection.

An actually traversed trajectory edge is accepted immediately and remains
valid after either endpoint leaves the local active window. Experimental
terrain/visibility candidates can be re-enabled with
`require_trajectory_evidence: false`; they require `gateway_confirmations`
update cycles, but are more vulnerable to false gateways from noisy heights.

## Graph stitching and search

After every `DynamicGraph::UpdateNavGraph`:

1. relabel all nodes by floor;
2. detect or update gateway candidates;
3. remove all unconfirmed cross-layer visibility and polygon edges;
4. insert confirmed gateway edges into `connect_nodes`;
5. pass the resulting graph to the existing `GraphPlanner`.

The original Dijkstra expansion remains in place. It additionally rejects any
cross-layer neighbour that is not present in both endpoints'
`gateway_connects`. Confirmed cross-layer edges use Euclidean length multiplied
by `inter_layer_cost_scale`, allowing stairs or ramps to be penalized without
changing the search algorithm.

Goal nodes may connect only to nodes on the same layer. Dijkstra reaches another
floor by first entering that floor through a gateway.

## Configuration

Use:

```bash
ros2 launch far_planner far_planner.launch config:=multifloor
```

Important parameters in `config/multifloor.yaml`:

- `map_handler/floor_height`: vertical distance between floor surfaces;
- `multi_layer/auto_floor_origin`: use the first odometry Z as floor 0;
- `multi_layer/floor_origin_z`: explicit floor-0 vehicle Z if auto mode is off;
- `multi_layer/gateway_max_xy_dist`: maximum local endpoint separation;
- `multi_layer/gateway_min_z_delta`: reject numerically flat transitions;
- `multi_layer/gateway_max_slope`: maximum rise/run ratio;
- `multi_layer/gateway_confirmations`: temporal filter for perceived gateways;
- `multi_layer/layer_hysteresis`: extra height beyond a floor boundary before
  the odometry layer label changes;
- `multi_layer/require_trajectory_evidence`: only physically traversed
  trajectory edges may become gateways (recommended);
- `multi_layer/inter_layer_cost_scale`: Dijkstra penalty for changing floor.

### Ramp terrain classification

The development-environment terrain analyser stores local height spread in
the point intensity field.  A continuous slope can therefore oscillate around
the free/obstacle threshold and leave a false obstacle contour in FAR's static
map.  The multi-floor profile uses `util/terrain_free_Z: 0.30` and enables
`clear_obstacle_with_free`: a later free observation removes a stale obstacle
sample only when it overlaps in the same 3-D voxel.  Walls and ramp rails above
the threshold remain obstacles.  Reset the visibility graph after changing
these parameters so contours created under the old classification are removed.

Confirmed gateway nodes are visualized in purple under the
`floor_gateways` marker namespace.

## Current scope

This extension supports continuous traversable connectors such as ramps and
stairs represented by a continuous terrain surface. It does not yet implement
elevator state machines, door calls, moving reference frames, or semantic floor
selection. With the default robust profile, a first physical traversal is
required before an online gateway becomes part of the persistent graph.
