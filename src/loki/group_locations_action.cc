#include <valhalla/loki/worker.h>
#include <valhalla/loki/search.h>
#include <valhalla/baldr/graphreader.h>
#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/directededge.h>
#include <valhalla/baldr/nodeinfo.h>
#include <valhalla/baldr/pathlocation.h>
#include <valhalla/midgard/pointll.h>
#include <valhalla/baldr/tilehierarchy.h>

#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>

#include <queue>
#include <cmath>

namespace valhalla {
namespace loki {

namespace {

uint64_t get_canonical_edge_val(const baldr::GraphId& edge_id,
                                baldr::GraphReader& reader,
                                bool ignore_road_side) {
    if (!ignore_road_side) {
        return edge_id.value;
    }
    baldr::GraphId opp_id = reader.GetOpposingEdgeId(edge_id);
    if (opp_id.Is_Valid() && opp_id.value < edge_id.value) {
        return opp_id.value;
    }
    return edge_id.value;
}

bool is_traversable_edge(const baldr::DirectedEdge* edge) {
    if (!edge) return false;
    if (edge->is_shortcut() || edge->use() == baldr::Use::kTransitConnection ||
        edge->use() == baldr::Use::kEgressConnection || edge->use() == baldr::Use::kPlatformConnection) {
        return false;
    }
    return true;
}

}
}
}

//
// Partitions (Voronoi Region-Adjacency Graph & Bridge Cutter)
//
namespace valhalla {
namespace loki {

namespace {

struct PartitionChokePoint {
    midgard::PointLL point;
    baldr::GraphId edge_id;
    bool is_one_way = false;
};

struct CandidatePartition {
    std::unordered_set<uint32_t> group_indices;
    std::vector<PartitionChokePoint> chokes;
    float score = 0.0f;
};

PartitionChokePoint make_choke_point(const baldr::GraphId& boundary_edge_id,
                                     baldr::GraphReader& reader) {
    PartitionChokePoint cp;
    cp.edge_id = boundary_edge_id;

    auto tile = reader.GetGraphTile(boundary_edge_id);
    if (!tile) return cp;
    const auto* edge = tile->directededge(boundary_edge_id);
    if (!edge) return cp;

    baldr::GraphId opp_id = reader.GetOpposingEdgeId(boundary_edge_id);
    cp.is_one_way = !opp_id.Is_Valid();

    const auto edge_info = tile->edgeinfo(edge);
    const auto& shape = edge_info.shape();

    if (!shape.empty()) {
        // Place at the midpoint of the boundary edge shape
        size_t mid = shape.size() / 2;
        if (shape.size() == 2) {
            float lat = 0.5f * (shape[0].lat() + shape[1].lat());
            float lng = 0.5f * (shape[0].lng() + shape[1].lng());
            cp.point = midgard::PointLL(lng, lat);
        } else {
            cp.point = shape[mid];
        }
    }
    return cp;
}

//Traces the exact shortest path between two adjacent Voronoi owners to find the true bottleneck edge
baldr::GraphId find_bottleneck_edge_between(
    uint64_t border_edge_a,
    uint64_t border_edge_b,
    const std::unordered_map<uint64_t, uint64_t>& parent_canon_edge,
    baldr::GraphReader& reader) {

    // Walk a few steps up the tree from the border collision to pick the cleanest bridge segment
    uint64_t curr = border_edge_a;
    for (int i = 0; i < 2; ++i) {
        auto it = parent_canon_edge.find(curr);
        if (it == parent_canon_edge.end() || it->second == curr) break;
        curr = it->second;
    }
    return baldr::GraphId(curr);
}

} // namespace
} // namespace loki
} // namespace valhalla

namespace valhalla {
namespace loki {

namespace {

struct CollisionBridge {
    uint64_t edge_in_a; // Edge inside Group A reaching toward boundary
    uint64_t edge_in_b; // Edge inside Group B reaching toward boundary
    baldr::GraphId cut_edge;
};

// Safe shape extraction ensuring correct tile/edge resolution
std::vector<midgard::PointLL> get_edge_shape(const baldr::GraphId& edge_id, baldr::GraphReader& reader) {
    if (!edge_id.Is_Valid()) return {};
    auto tile = reader.GetGraphTile(edge_id);
    if (!tile) return {};
    const auto* de = tile->directededge(edge_id);
    if (!de) return {};
    return tile->edgeinfo(de).shape();
}

// Emits an edge and walks backwards up the Dijkstra tree to the root of its Voronoi cell
void trace_back_to_root(
    uint64_t start_canon,
    const std::unordered_set<uint32_t>& partition_groups,
    const std::unordered_map<uint64_t, uint64_t>& parent_canon,
    const std::unordered_map<uint64_t, baldr::GraphId>& canon_rep_edge,
    const std::unordered_map<uint64_t, uint32_t>& canon_owner,
    std::unordered_set<uint64_t>& emitted_canon_edges,
    std::vector<std::vector<midgard::PointLL>>& result_lines,
    baldr::GraphReader& reader) {

    uint64_t curr = start_canon;
    const size_t MAX_STEPS = 256;
    size_t steps = 0;

    while (steps++ < MAX_STEPS) {
        // Emit current edge
        if (emitted_canon_edges.insert(curr).second) {
            auto rep_it = canon_rep_edge.find(curr);
            if (rep_it != canon_rep_edge.end() && rep_it->second.Is_Valid()) {
                auto shape = get_edge_shape(rep_it->second, reader);
                if (shape.size() >= 2) {
                    result_lines.push_back(std::move(shape));
                }
            }
        }

        auto p_it = parent_canon.find(curr);
        if (p_it == parent_canon.end()) break;

        uint64_t parent = p_it->second;
        if (parent == curr) break; // Reached root / origin seed

        // Verify parent is still within this partition's groups
        auto o_it = canon_owner.find(parent);
        if (o_it == canon_owner.end() || !partition_groups.count(o_it->second)) {
            break;
        }

        curr = parent;
    }
}

std::vector<std::vector<midgard::PointLL>> build_partition_connected_subgraph(
    const std::unordered_set<uint32_t>& partition_groups,
    const std::vector<std::vector<baldr::GraphId>>& group_raw_edges,
    const std::vector<CollisionBridge>& partition_bridges,
    const std::unordered_map<uint64_t, uint64_t>& parent_canon,
    const std::unordered_map<uint64_t, baldr::GraphId>& canon_rep_edge,
    const std::unordered_map<uint64_t, uint32_t>& canon_owner,
    baldr::GraphReader& reader) {

    std::vector<std::vector<midgard::PointLL>> result_lines;
    std::unordered_set<uint64_t> emitted_canon_edges;

    auto emit_single_edge = [&](uint64_t c_val, const baldr::GraphId& fallback_id) {
        if (!emitted_canon_edges.insert(c_val).second) return;
        baldr::GraphId eid = fallback_id;
        auto rep_it = canon_rep_edge.find(c_val);
        if (rep_it != canon_rep_edge.end() && rep_it->second.Is_Valid()) {
            eid = rep_it->second;
        }
        auto shape = get_edge_shape(eid, reader);
        if (shape.size() >= 2) {
            result_lines.push_back(std::move(shape));
        }
    };

    // 1. Emit all raw seed edges belonging to every group in the partition
    for (uint32_t gid : partition_groups) {
        if (gid >= group_raw_edges.size()) continue;
        for (const auto& eid : group_raw_edges[gid]) {
            if (!eid.Is_Valid()) continue;
            uint64_t c = get_canonical_edge_val(eid, reader, true);
            emit_single_edge(c, eid);
        }
    }

    // 2. Process all bridges (choke points to outside + connectors between internal groups)
    for (const auto& bridge : partition_bridges) {
        // Emit the physical cut-edge itself
        uint64_t cut_c = get_canonical_edge_val(bridge.cut_edge, reader, true);
        emit_single_edge(cut_c, bridge.cut_edge);

        // Trace backward into Group A's territory
        auto owner_a = canon_owner.find(bridge.edge_in_a);
        if (owner_a != canon_owner.end() && partition_groups.count(owner_a->second)) {
            trace_back_to_root(
                bridge.edge_in_a, partition_groups, parent_canon,
                canon_rep_edge, canon_owner, emitted_canon_edges, result_lines, reader);
        }

        // Trace backward into Group B's territory
        auto owner_b = canon_owner.find(bridge.edge_in_b);
        if (owner_b != canon_owner.end() && partition_groups.count(owner_b->second)) {
            trace_back_to_root(
                bridge.edge_in_b, partition_groups, parent_canon,
                canon_rep_edge, canon_owner, emitted_canon_edges, result_lines, reader);
        }
    }

    return result_lines;
}

} // namespace
} // namespace loki
} // namespace valhalla

//
// Grouping.
//
namespace valhalla {
namespace loki {

namespace {

// Checks if driving forward along `start_edge_id` enters a strict, loop-free dead-end tree.
bool is_closed_dead_end_subtree(const baldr::GraphId& start_edge_id,
                                baldr::GraphReader& reader) {
    if (!start_edge_id.Is_Valid()) return false;

    baldr::GraphId start_opp = reader.GetOpposingEdgeId(start_edge_id);
    if (!start_opp.Is_Valid()) return false;

    auto start_opp_tile = reader.GetGraphTile(start_opp);
    if (!start_opp_tile) return false;
    const auto* start_opp_edge = start_opp_tile->directededge(start_opp);
    if (!start_opp_edge) return false;

    baldr::GraphId entry_node_id = start_opp_edge->endnode();

    std::unordered_set<uint64_t> visited_undirected_edges;
    std::unordered_set<uint64_t> visited_nodes;

    visited_undirected_edges.insert(get_canonical_edge_val(start_edge_id, reader, true));
    visited_nodes.insert(entry_node_id.value);

    std::vector<baldr::GraphId> stack;
    stack.push_back(start_edge_id);

    const size_t MAX_SUBTREE_EDGES = 32;
    const float MAX_SUBTREE_LENGTH_M = 1500.0f;
    size_t explored = 0;
    float total_subtree_length = 0.0f;

    while (!stack.empty()) {
        if (++explored > MAX_SUBTREE_EDGES) {
            return false;
        }

        baldr::GraphId curr_id = stack.back();
        stack.pop_back();

        auto tile = reader.GetGraphTile(curr_id);
        if (!tile) return false;
        const auto* edge = tile->directededge(curr_id);
        if (!is_traversable_edge(edge)) return false;

        total_subtree_length += static_cast<float>(edge->length());
        if (total_subtree_length > MAX_SUBTREE_LENGTH_M) {
            return false;
        }

        baldr::GraphId endnode_id = edge->endnode();

        // Cycle/loop check
        if (visited_nodes.count(endnode_id.value)) {
            return false;
        }
        visited_nodes.insert(endnode_id.value);

        auto end_tile = reader.GetGraphTile(endnode_id);
        if (!end_tile) return false;
        const auto* end_node = end_tile->node(endnode_id);
        if (!end_node) return false;

        // Hierarchy transition means connection to arterial/highway network
        if (end_node->transition_count() > 0) {
            return false;
        }

        // Terminal dead-end tip
        if (end_node->edge_count() <= 1) {
            continue;
        }

        baldr::GraphId opp_curr = reader.GetOpposingEdgeId(curr_id);

        for (uint32_t i = 0; i < end_node->edge_count(); ++i) {
            baldr::GraphId next_id(endnode_id.tileid(), endnode_id.level(), end_node->edge_index() + i);
            if (next_id == opp_curr) {
                continue;
            }

            const auto* next_edge = end_tile->directededge(next_id);
            if (!is_traversable_edge(next_edge)) {
                continue;
            }

            uint64_t canon_next = get_canonical_edge_val(next_id, reader, true);
            if (visited_undirected_edges.count(canon_next)) {
                return false;
            }

            visited_undirected_edges.insert(canon_next);
            stack.push_back(next_id);
        }
    }

    return true;
}

struct SubtreeTrace {
    bool in_dead_end = false;
    baldr::GraphId root_inward_edge;
    float dist_from_root = 0.0f;
    int local_dead_end_side = 0;
};

SubtreeTrace climb_to_dead_end_root(const baldr::GraphId& edge_id,
                                    float percent_along,
                                    baldr::GraphReader& reader) {
    SubtreeTrace trace;

    auto tile = reader.GetGraphTile(edge_id);
    if (!tile) return trace;
    const auto* edge = tile->directededge(edge_id);
    if (!is_traversable_edge(edge)) return trace;

    float edge_len = static_cast<float>(edge->length());
    baldr::GraphId opp_id = reader.GetOpposingEdgeId(edge_id);

    bool fwd_closed = is_closed_dead_end_subtree(edge_id, reader);
    bool bwd_closed = opp_id.Is_Valid() && is_closed_dead_end_subtree(opp_id, reader);

    if (fwd_closed == bwd_closed) {
        return trace;
    }

    trace.in_dead_end = true;

    baldr::GraphId curr_inward;
    float dist_from_curr_start = 0.0f;

    if (fwd_closed) {
        curr_inward = edge_id;
        dist_from_curr_start = percent_along * edge_len;
        trace.local_dead_end_side = 2;
    } else {
        curr_inward = opp_id;
        dist_from_curr_start = (1.0f - percent_along) * edge_len;
        trace.local_dead_end_side = 1;
    }

    std::unordered_set<uint64_t> climbed;
    const size_t MAX_CLIMB = 16;

    for (size_t step = 0; step < MAX_CLIMB; ++step) {
        climbed.insert(get_canonical_edge_val(curr_inward, reader, true));

        baldr::GraphId curr_outward = reader.GetOpposingEdgeId(curr_inward);
        if (!curr_outward.Is_Valid()) break;

        auto out_tile = reader.GetGraphTile(curr_outward);
        if (!out_tile) break;
        const auto* out_edge = out_tile->directededge(curr_outward);
        if (!out_edge) break;

        baldr::GraphId start_node_id = out_edge->endnode();
        auto node_tile = reader.GetGraphTile(start_node_id);
        if (!node_tile) break;
        const auto* node = node_tile->node(start_node_id);
        if (!node || node->edge_count() <= 1) break;

        if (node->transition_count() > 0) {
            break;
        }

        baldr::GraphId open_exit_edge;
        uint32_t open_exit_count = 0;

        for (uint32_t i = 0; i < node->edge_count(); ++i) {
            baldr::GraphId branch_id(start_node_id.tileid(), start_node_id.level(), node->edge_index() + i);
            if (branch_id == curr_inward) {
                continue;
            }

            const auto* branch_edge = node_tile->directededge(branch_id);
            if (!is_traversable_edge(branch_edge)) {
                continue;
            }

            if (!is_closed_dead_end_subtree(branch_id, reader)) {
                open_exit_edge = branch_id;
                open_exit_count++;
            }
        }

        if (open_exit_count != 1 || !open_exit_edge.Is_Valid()) {
            break;
        }

        baldr::GraphId parent_inward = reader.GetOpposingEdgeId(open_exit_edge);
        if (!parent_inward.Is_Valid()) break;

        uint64_t parent_canon = get_canonical_edge_val(parent_inward, reader, true);
        if (climbed.count(parent_canon)) break;

        if (!is_closed_dead_end_subtree(parent_inward, reader)) {
            break;
        }

        auto p_tile = reader.GetGraphTile(parent_inward);
        if (!p_tile) break;
        const auto* p_edge = p_tile->directededge(parent_inward);
        if (!p_edge) break;

        dist_from_curr_start += static_cast<float>(p_edge->length());
        curr_inward = parent_inward;
    }

    trace.root_inward_edge = curr_inward;
    trace.dist_from_root = dist_from_curr_start;
    return trace;
}

} // namespace

//
// Action code.
//
void loki_worker_t::init_group_locations(Api& request) {
    auto& options = *request.mutable_options();
    parse_locations(options.mutable_locations(), request);
    if (options.locations_size() < 1) {
        throw valhalla_exception_t{120};
    }
    parse_costing(request);
}

void loki_worker_t::group_locations(Api& request) {
    auto _ = measure_scope_time(request);
    init_group_locations(request);

    auto& options = *request.mutable_options();
    bool ignore_road_side = options.ignore_road_side();

    try {
        auto locations = baldr::PathLocation::fromPBF(options.locations());
        const auto projections = loki::Search(locations, *reader, costing);

        struct RawPoint {
            uint32_t original_index;
            baldr::GraphId edge_id;
            float percent_along;
            float canonical_percent;
            midgard::PointLL projected;
        };

        // -------------------------------------------------------------
        // STAGE 1: Group strictly by Edge ID first (matches your 155-group baseline).
        // -------------------------------------------------------------
        std::unordered_map<uint64_t, std::vector<RawPoint>> edge_groups;

        for (size_t i = 0; i < locations.size(); ++i) {
            const auto& projection = projections.at(locations[i]);
            if (projection.edges.empty())
                continue;

            baldr::GraphId edge_id = projection.edges.front().id;
            float percent = projection.edges.front().percent_along;
            auto projected = projection.edges.front().projected;

            uint64_t edge_key = edge_id.value;
            float canon_percent = percent;

            if (ignore_road_side) {
                baldr::GraphId opposing_id = reader->GetOpposingEdgeId(edge_id);
                if (opposing_id.Is_Valid() && opposing_id.value < edge_id.value) {
                    edge_key = opposing_id.value;
                    canon_percent = 1.0f - percent;
                }
            }

            edge_groups[edge_key].push_back({
                static_cast<uint32_t>(i),
                edge_id,
                percent,
                canon_percent,
                projected
            });
        }

        // -------------------------------------------------------------
        // STAGE 2: Merge edge groups that belong to the same dead-end subtree.
        // -------------------------------------------------------------
        struct FinalPoint {
            uint32_t original_index;
            float sort_offset;
            midgard::PointLL projected;
        };

        struct MergedGroup {
            int dead_end_side = 0;
            std::vector<FinalPoint> points;
            std::vector<baldr::GraphId> raw_edges;
        };

        std::unordered_map<uint64_t, MergedGroup> final_groups;

        for (auto& [edge_key, raw_points] : edge_groups) {
            baldr::GraphId sample_edge = raw_points.front().edge_id;
            SubtreeTrace sample_trace = climb_to_dead_end_root(sample_edge, 0.0f, *reader);

            if (sample_trace.in_dead_end && sample_trace.root_inward_edge.Is_Valid()) {
                uint64_t root_key = get_canonical_edge_val(sample_trace.root_inward_edge, *reader, true);
                auto& target_group = final_groups[root_key];
                target_group.raw_edges.push_back(sample_trace.root_inward_edge);
                target_group.raw_edges.push_back(sample_edge);

                bool climbed_multiple_edges =
                    get_canonical_edge_val(sample_trace.root_inward_edge, *reader, true) !=
                    get_canonical_edge_val(sample_edge, *reader, true);

                if (climbed_multiple_edges || !target_group.points.empty()) {
                    target_group.dead_end_side = 2;
                } else {
                    target_group.dead_end_side = sample_trace.local_dead_end_side;
                }

                for (const auto& rp : raw_points) {
                    SubtreeTrace pt_trace = climb_to_dead_end_root(rp.edge_id, rp.percent_along, *reader);
                    float offset = pt_trace.in_dead_end ? pt_trace.dist_from_root : rp.canonical_percent;
                    target_group.points.push_back({rp.original_index, offset, rp.projected});
                }
            } else {
                auto& target_group = final_groups[edge_key];
                target_group.raw_edges.push_back(sample_edge);
                for (const auto& rp : raw_points) {
                    target_group.points.push_back({rp.original_index, rp.canonical_percent, rp.projected});
                }
            }
        }

        // -------------------------------------------------------------
        // Serialize Group Results & Build Group Indexing
        // -------------------------------------------------------------
        auto group_locations_res = request.mutable_group_locations();
        group_locations_res->clear_results();

        std::vector<std::vector<baldr::GraphId>> group_raw_edges;
        std::vector<midgard::PointLL> group_centroids;

        uint32_t grp_idx = 0;
        for (auto& [key, group] : final_groups) {
            std::sort(group.points.begin(), group.points.end(), [](const FinalPoint& a, const FinalPoint& b) {
                return a.sort_offset < b.sort_offset;
            });

            auto result = group_locations_res->add_results();
            if (group.dead_end_side != 0) {
                result->set_dead_end_side(group.dead_end_side);
            }

            float lat_sum = 0.0f, lng_sum = 0.0f;
            for (const auto& pt : group.points) {
                result->add_location_index(pt.original_index);
                auto snapped = result->add_locations();
                snapped->set_location_index(pt.original_index);
                snapped->mutable_snapped_point()->set_lat(pt.projected.lat());
                snapped->mutable_snapped_point()->set_lng(pt.projected.lng());
                lat_sum += pt.projected.lat();
                lng_sum += pt.projected.lng();
            }

            float cnt = static_cast<float>(group.points.size());
            group_centroids.push_back(midgard::PointLL(lng_sum / cnt, lat_sum / cnt));
            group_raw_edges.push_back(group.raw_edges);
            ++grp_idx;
        }

        // -------------------------------------------------------------
        // STAGE 3: Multi-Source Road Voronoi Diagram & Graph Partitioning
        // -------------------------------------------------------------
        const uint32_t num_groups = static_cast<uint32_t>(group_raw_edges.size());

        struct VoronoiState {
            baldr::GraphId edge_id;
            uint32_t owner_group;
            float cost_secs;
            bool operator>(const VoronoiState& o) const { return cost_secs > o.cost_secs; }
        };

        std::priority_queue<VoronoiState, std::vector<VoronoiState>, std::greater<VoronoiState>> v_pq;
        std::unordered_map<uint64_t, float> canon_best_cost;
        std::unordered_map<uint64_t, uint32_t> canon_owner;
        std::unordered_map<uint64_t, baldr::GraphId> canon_rep_edge;
        std::unordered_map<uint64_t, uint64_t> parent_canon;

        // Seed all 157 groups simultaneously at cost 0.0
        for (uint32_t g = 0; g < num_groups; ++g) {
            for (const auto& eid : group_raw_edges[g]) {
                if (!eid.Is_Valid()) continue;
                uint64_t c = get_canonical_edge_val(eid, *reader, true);
                canon_best_cost[c] = 0.0f;
                canon_owner[c] = g;
                canon_rep_edge[c] = eid;
                parent_canon[c] = c;

                v_pq.push({eid, g, 0.0f});
                baldr::GraphId opp = reader->GetOpposingEdgeId(eid);
                if (opp.Is_Valid()) {
                    v_pq.push({opp, g, 0.0f});
                }
            }
        }

        // Adjacency between Voronoi cells: adj_bridges[g1][g2] = Set of canonical cut-edges where g1 meets g2
        std::unordered_map<uint32_t, std::unordered_map<uint32_t, std::unordered_set<uint64_t>>> adj_bridges;
        std::unordered_map<uint32_t, std::unordered_map<uint32_t, float>> adj_min_cost;
        // Store rich bridge metadata for collisions
        std::unordered_map<uint32_t, std::unordered_map<uint32_t, std::vector<CollisionBridge>>> all_bridges;

        const float MAX_VORONOI_SECS = 900.0f; // 15-minute ceiling

        while (!v_pq.empty()) {
            auto curr = v_pq.top();
            v_pq.pop();

            uint64_t curr_c = get_canonical_edge_val(curr.edge_id, *reader, true);
            if (curr.cost_secs > canon_best_cost[curr_c]) continue;

            auto tile = reader->GetGraphTile(curr.edge_id);
            if (!tile) continue;
            const auto* de = tile->directededge(curr.edge_id);
            if (!is_traversable_edge(de)) continue;

            baldr::GraphId end_nid = de->endnode();
            auto nt = reader->GetGraphTile(end_nid);
            if (!nt) continue;
            const auto* node = nt->node(end_nid);
            if (!node) continue;

            baldr::GraphId opp_curr = reader->GetOpposingEdgeId(curr.edge_id);

            for (uint32_t i = 0; i < node->edge_count(); ++i) {
                baldr::GraphId next_id(end_nid.tileid(), end_nid.level(), node->edge_index() + i);
                if (next_id == opp_curr) continue;

                const auto* next_de = nt->directededge(next_id);
                if (!is_traversable_edge(next_de)) continue;

                // Ignore dead-end twigs that contain no groups so we NEVER place a choke point in a dead end!
                if (is_closed_dead_end_subtree(next_id, *reader)) {
                    uint64_t next_c_check = get_canonical_edge_val(next_id, *reader, true);
                    if (canon_owner.find(next_c_check) == canon_owner.end()) {
                        continue;
                    }
                }

                uint64_t next_c = get_canonical_edge_val(next_id, *reader, true);
                float step_secs = costing->EdgeCost(next_de, nt).secs;
                float new_cost = curr.cost_secs + step_secs;
                if (new_cost > MAX_VORONOI_SECS) continue;

                auto o_it = canon_owner.find(next_c);
                if (o_it == canon_owner.end()) {
                    // Unclaimed road -> Claim for curr.owner_group
                    canon_best_cost[next_c] = new_cost;
                    canon_owner[next_c] = curr.owner_group;
                    canon_rep_edge[next_c] = next_id;
                    parent_canon[next_c] = curr_c;
                    v_pq.push({next_id, curr.owner_group, new_cost});
                } else if (o_it->second != curr.owner_group) {
                    // COLLISION! Two different groups' Voronoi territories meet right at this edge!
                    // This exact collision edge is the true bridge/choke point between Group A and Group B.
                   uint32_t g_a = curr.owner_group;
                    uint32_t g_b = o_it->second;
                    float total_bridge_cost = curr.cost_secs + canon_best_cost[next_c];

                    adj_bridges[g_a][g_b].insert(next_c);
                    adj_bridges[g_b][g_a].insert(next_c);

                    CollisionBridge br{curr_c, next_c, next_id};
                    all_bridges[g_a][g_b].push_back(br);
                    all_bridges[g_b][g_a].push_back(br);

                    if (adj_min_cost[g_a].find(g_b) == adj_min_cost[g_a].end() ||
                        total_bridge_cost < adj_min_cost[g_a][g_b]) {
                        adj_min_cost[g_a][g_b] = total_bridge_cost;
                        adj_min_cost[g_b][g_a] = total_bridge_cost;
                    }
                }
            }
        }

        // Now every group is a node in the Voronoi Region-Adjacency Graph (`adj_bridges`).
        // Expanding on `adj_bridges` guarantees 100% contiguous, non-overlapping regions!
        std::unordered_set<uint32_t> locked_groups;
        std::vector<CandidatePartition> finalized_partitions;

        const size_t MIN_PART_GROUPS = 8;
        const size_t MAX_PART_GROUPS = 22;

        while (true) {
            CandidatePartition best_cand;
            best_cand.score = -1e9f;
            bool found_any = false;

            for (uint32_t seed = 0; seed < num_groups; ++seed) {
                if (locked_groups.count(seed)) continue;

                // Grow contiguous Voronoi cluster from `seed` picking closest adjacent Voronoi neighbors
                std::unordered_set<uint32_t> cluster = {seed};

                while (cluster.size() < MAX_PART_GROUPS) {
                    // Find unassigned neighbor group with minimum travel time / strongest connection to cluster
                    uint32_t best_next_grp = UINT32_MAX;
                    float best_next_cost = 1e9f;

                    for (uint32_t u : cluster) {
                        for (const auto& [v, cost] : adj_min_cost[u]) {
                            if (cluster.count(v) || locked_groups.count(v)) continue;
                            // Also penalize Euclidean distance from seed so clusters stay round and never snake
                            float geo_dist = group_centroids[seed].Distance(group_centroids[v]);
                            if (geo_dist > 3500.0f) continue; // Hard 3.5 km cluster diameter limit

                            float combined_metric = cost + (geo_dist * 0.05f);
                            if (combined_metric < best_next_cost) {
                                best_next_cost = combined_metric;
                                best_next_grp = v;
                            }
                        }
                    }

                    if (best_next_grp == UINT32_MAX) break;
                    cluster.insert(best_next_grp);

                    if (cluster.size() >= MIN_PART_GROUPS) {
                        // Compute exact choke points: all Voronoi boundary bridges connecting `cluster`
                        // to ANY group OUTSIDE `cluster` (including locked groups!)
                        std::unordered_set<uint64_t> active_cut_edges;
                        bool touches_outside = false;

                        for (uint32_t u : cluster) {
                            for (const auto& [v, bridge_edges] : adj_bridges[u]) {
                                if (!cluster.count(v)) {
                                    touches_outside = true;
                                    for (uint64_t be : bridge_edges) {
                                        active_cut_edges.insert(be);
                                    }
                                }
                            }
                        }

                        if (touches_outside && active_cut_edges.size() >= 1 && active_cut_edges.size() <= 4) {
                            CandidatePartition cand;
                            cand.group_indices = cluster;
                            for (uint64_t ce : active_cut_edges) {
                                baldr::GraphId rep = canon_rep_edge.count(ce) ? canon_rep_edge[ce] : baldr::GraphId(ce);
                                cand.chokes.push_back(make_choke_point(rep, *reader));
                            }

                            float n_g = static_cast<float>(cluster.size());
                            float n_c = static_cast<float>(cand.chokes.size());
                            // Strongly prefer 1 or 2 choke points and ~16-20 groups
                            cand.score = (n_g * n_g * 100.0f) / (n_c * n_c * n_c)
                                         - std::abs(n_g - 18.0f) * 5.0f;

                            if (cand.score > best_cand.score) {
                                best_cand = std::move(cand);
                                found_any = true;
                            }
                        }
                    }
                }
            }

            if (!found_any) break;

            for (uint32_t gid : best_cand.group_indices) {
                locked_groups.insert(gid);
            }
            finalized_partitions.push_back(std::move(best_cand));
        }

        for (const auto& partition : finalized_partitions) {
            auto part_pb = group_locations_res->add_partitions();
            for (uint32_t gid : partition.group_indices) {
                part_pb->add_group_indices(gid);
            }
            for (const auto& choke : partition.chokes) {
                auto cp_pb = part_pb->add_choke_points();
                cp_pb->mutable_point()->set_lat(choke.point.lat());
                cp_pb->mutable_point()->set_lng(choke.point.lng());
                cp_pb->set_edge_id(choke.edge_id.value);
                cp_pb->set_is_one_way(choke.is_one_way);
            }

            // Gather all relevant bridges:
            // 1. Bridges between groups INSIDE this partition (connects all internal groups)
            // 2. Bridges leaving this partition (connects internal groups to external choke points)
            std::vector<CollisionBridge> partition_bridges;
            for (uint32_t u : partition.group_indices) {
                for (const auto& [v, bridges] : all_bridges[u]) {
                    // Internal connections (avoid double counting with u < v)
                    if (partition.group_indices.count(v)) {
                        if (u < v) {
                            partition_bridges.insert(partition_bridges.end(), bridges.begin(), bridges.end());
                        }
                    } else {
                        // External connections to outside choke points
                        partition_bridges.insert(partition_bridges.end(), bridges.begin(), bridges.end());
                    }
                }
            }

            auto lines = build_partition_connected_subgraph(
                partition.group_indices,
                group_raw_edges,
                partition_bridges,
                parent_canon,
                canon_rep_edge,
                canon_owner,
                *reader);

            for (const auto& seg : lines) {
                auto line_pb = part_pb->add_boundary_lines();
                for (const auto& pt : seg) {
                    auto p_pb = line_pb->add_points();
                    p_pb->set_lat(pt.lat());
                    p_pb->set_lng(pt.lng());
                }
            }
        }
    } catch (const std::exception& e) {
        throw valhalla_exception_t{171, std::string(e.what())};
    }
}

} // namespace loki
} // namespace valhalla