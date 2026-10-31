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
#include <map>
#include <set>
#include <algorithm>
#include <queue>
#include <cmath>
#include <tuple>
#include <memory>
#include <numeric>
#include <cstdint>
#include <functional>
#include <iostream>
#include <limits>

namespace valhalla {
namespace loki {

namespace {

// -----------------------------------------------------------------------------
// Basic Graph & Edge Helpers
// -----------------------------------------------------------------------------
baldr::GraphId edge_end_node(const baldr::GraphId& edge_id, baldr::GraphReader& reader) {
    if (!edge_id.Is_Valid()) return {};
    auto tile = reader.GetGraphTile(edge_id);
    if (!tile) return {};
    const auto* de = tile->directededge(edge_id);
    return de ? de->endnode() : baldr::GraphId{};
}

baldr::GraphId edge_start_node(const baldr::GraphId& edge_id, baldr::GraphReader& reader) {
    return edge_end_node(reader.GetOpposingEdgeId(edge_id), reader);
}

uint64_t get_canonical_edge_val(const baldr::GraphId& edge_id,
                                baldr::GraphReader& reader,
                                bool ignore_road_side) {
    if (!ignore_road_side || !edge_id.Is_Valid()) {
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
    if (edge->is_shortcut() || 
        edge->use() == baldr::Use::kTransitConnection ||
        edge->use() == baldr::Use::kEgressConnection || 
        edge->use() == baldr::Use::kPlatformConnection) {
        return false;
    }
    return true;
}

std::vector<midgard::PointLL> get_edge_shape(const baldr::GraphId& edge_id, baldr::GraphReader& reader) {
    if (!edge_id.Is_Valid()) return {};
    auto tile = reader.GetGraphTile(edge_id);
    if (!tile) return {};
    const auto* de = tile->directededge(edge_id);
    if (!de) return {};
    auto shape = tile->edgeinfo(de).shape();
    if (!de->forward()) std::reverse(shape.begin(), shape.end());  // start node first
    return shape;
}

// -----------------------------------------------------------------------------
// Dead-End Detection Helpers
// -----------------------------------------------------------------------------

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
        if (++explored > MAX_SUBTREE_EDGES) return false;

        baldr::GraphId curr_id = stack.back();
        stack.pop_back();

        auto tile = reader.GetGraphTile(curr_id);
        if (!tile) return false;
        const auto* edge = tile->directededge(curr_id);
        if (!is_traversable_edge(edge)) return false;

        total_subtree_length += static_cast<float>(edge->length());
        if (total_subtree_length > MAX_SUBTREE_LENGTH_M) return false;

        baldr::GraphId endnode_id = edge->endnode();
        if (visited_nodes.count(endnode_id.value)) return false;
        visited_nodes.insert(endnode_id.value);

        auto end_tile = reader.GetGraphTile(endnode_id);
        if (!end_tile) return false;
        const auto* end_node = end_tile->node(endnode_id);
        if (!end_node) return false;

        if (end_node->transition_count() > 0) return false;
        if (end_node->edge_count() <= 1) continue;

        baldr::GraphId opp_curr = reader.GetOpposingEdgeId(curr_id);

        for (uint32_t i = 0; i < end_node->edge_count(); ++i) {
            baldr::GraphId next_id(endnode_id.tileid(), endnode_id.level(), end_node->edge_index() + i);
            if (next_id == opp_curr) continue;

            const auto* next_edge = end_tile->directededge(next_id);
            if (!is_traversable_edge(next_edge)) continue;

            uint64_t canon_next = get_canonical_edge_val(next_id, reader, true);
            if (visited_undirected_edges.count(canon_next)) return false;

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

    if (fwd_closed == bwd_closed) return trace;

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
        if (!node || node->edge_count() <= 1 || node->transition_count() > 0) break;

        baldr::GraphId open_exit_edge;
        uint32_t open_exit_count = 0;

        for (uint32_t i = 0; i < node->edge_count(); ++i) {
            baldr::GraphId branch_id(start_node_id.tileid(), start_node_id.level(), node->edge_index() + i);
            if (branch_id == curr_inward) continue;

            const auto* branch_edge = node_tile->directededge(branch_id);
            if (!is_traversable_edge(branch_edge)) continue;

            if (!is_closed_dead_end_subtree(branch_id, reader)) {
                open_exit_edge = branch_id;
                open_exit_count++;
            }
        }

        if (open_exit_count != 1 || !open_exit_edge.Is_Valid()) break;

        baldr::GraphId parent_inward = reader.GetOpposingEdgeId(open_exit_edge);
        if (!parent_inward.Is_Valid()) break;

        uint64_t parent_canon = get_canonical_edge_val(parent_inward, reader, true);
        if (climbed.count(parent_canon) || !is_closed_dead_end_subtree(parent_inward, reader)) break;

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

// -----------------------------------------------------------------------------
// Grouping Functions (2 Separate Functions)
// -----------------------------------------------------------------------------

struct RawPoint {
    uint32_t original_index;
    baldr::GraphId edge_id;
    float percent_along;
    float canonical_percent;
    midgard::PointLL projected;
};

struct FinalPoint {
    uint32_t original_index;
    float sort_offset;
    midgard::PointLL projected;
};

struct FinalGroup {
    uint32_t group_id = 0;
    int dead_end_side = 0;
    std::vector<FinalPoint> points;
    std::vector<baldr::GraphId> raw_edges;
    std::vector<baldr::GraphId> anchor_nodes;
};

// GROUPING FUNCTION 1: Project and group strictly by canonical Edge ID
std::unordered_map<uint64_t, std::vector<RawPoint>> group_points_by_edge(
    const std::vector<baldr::Location>& locations,
    const std::unordered_map<baldr::Location, baldr::PathLocation>& projections,
    bool ignore_road_side,
    baldr::GraphReader& reader) {

    std::unordered_map<uint64_t, std::vector<RawPoint>> edge_groups;

    for (size_t i = 0; i < locations.size(); ++i) {
        auto it = projections.find(locations[i]);
        if (it == projections.end() || it->second.edges.empty()) continue;

        const auto& front_edge = it->second.edges.front();
        baldr::GraphId edge_id = front_edge.id;
        float percent = front_edge.percent_along;
        auto projected = front_edge.projected;

        uint64_t edge_key = edge_id.value;
        float canon_percent = percent;

        if (ignore_road_side) {
            baldr::GraphId opposing_id = reader.GetOpposingEdgeId(edge_id);
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

    return edge_groups;
}

// GROUPING FUNCTION 2: Merge edge groups that sit inside the same dead-end subtree
std::vector<FinalGroup> merge_dead_end_edge_groups(
    const std::unordered_map<uint64_t, std::vector<RawPoint>>& edge_groups,
    baldr::GraphReader& reader) {

    std::unordered_map<uint64_t, FinalGroup> merged_map;

    for (const auto& [edge_key, raw_points] : edge_groups) {
        baldr::GraphId sample_edge = raw_points.front().edge_id;
        SubtreeTrace sample_trace = climb_to_dead_end_root(sample_edge, 0.0f, reader);

        if (sample_trace.in_dead_end && sample_trace.root_inward_edge.Is_Valid()) {
            uint64_t root_key = get_canonical_edge_val(sample_trace.root_inward_edge, reader, true);
            auto& target_group = merged_map[root_key];
            target_group.raw_edges.push_back(sample_trace.root_inward_edge);
            target_group.raw_edges.push_back(sample_edge);

            auto junction = edge_start_node(sample_trace.root_inward_edge, reader);
            if (junction.Is_Valid()) target_group.anchor_nodes.push_back(junction);

            bool climbed_multiple_edges =
                get_canonical_edge_val(sample_trace.root_inward_edge, reader, true) !=
                get_canonical_edge_val(sample_edge, reader, true);

            if (climbed_multiple_edges || !target_group.points.empty()) {
                target_group.dead_end_side = 2;
            } else {
                target_group.dead_end_side = sample_trace.local_dead_end_side;
            }

            for (const auto& rp : raw_points) {
                SubtreeTrace pt_trace = climb_to_dead_end_root(rp.edge_id, rp.percent_along, reader);
                float offset = pt_trace.in_dead_end ? pt_trace.dist_from_root : rp.canonical_percent;
                target_group.points.push_back({rp.original_index, offset, rp.projected});
            }
        } else {
            auto& target_group = merged_map[edge_key];
            target_group.raw_edges.push_back(sample_edge);
            auto a = edge_start_node(sample_edge, reader);
            auto b = edge_end_node(sample_edge, reader);
            if (a.Is_Valid()) target_group.anchor_nodes.push_back(a);
            if (b.Is_Valid()) target_group.anchor_nodes.push_back(b);
            for (const auto& rp : raw_points) {
                target_group.points.push_back({rp.original_index, rp.canonical_percent, rp.projected});
            }
        }
    }

    std::vector<FinalGroup> result;
    result.reserve(merged_map.size());
    uint32_t idx = 0;

    for (auto& [key, grp] : merged_map) {
        std::sort(grp.points.begin(), grp.points.end(), [](const FinalPoint& a, const FinalPoint& b) {
            return a.sort_offset < b.sort_offset;
        });
        grp.group_id = idx++;
        result.push_back(std::move(grp));
    }

    return result;
}

// -----------------------------------------------------------------------------
// Partitioning: region growing with exact boundary (choke) tracking
// -----------------------------------------------------------------------------

struct PartitionChokePoint {
    midgard::PointLL point;
    baldr::GraphId edge_id;   // outgoing edge (inside -> outside)
    bool is_one_way = false;
};

float shape_length(const std::vector<midgard::PointLL>& shape) {
    float total = 0.f;
    for (size_t i = 1; i < shape.size(); ++i) total += shape[i - 1].Distance(shape[i]);
    return total;
}

// shape from its start up to `dist` metres along it (last point interpolated)
std::vector<midgard::PointLL> cut_shape(const std::vector<midgard::PointLL>& shape, float dist) {
    if (shape.size() < 2) return shape;
    std::vector<midgard::PointLL> out{shape.front()};
    float walked = 0.f;
    for (size_t i = 1; i < shape.size(); ++i) {
        float seg = shape[i - 1].Distance(shape[i]);
        if (walked + seg >= dist && seg > 0.f) {
            float t = (dist - walked) / seg;
            out.emplace_back(shape[i - 1].lng() + t * (shape[i].lng() - shape[i - 1].lng()),
                             shape[i - 1].lat() + t * (shape[i].lat() - shape[i - 1].lat()));
            return out;
        }
        walked += seg;
        out.push_back(shape[i]);
    }
    return out;
}

// spot on `shape` closest to p: {metres along the shape, metres away from p}
std::pair<float, float> closest_on_shape(const std::vector<midgard::PointLL>& shape,
                                         const midgard::PointLL& p) {
    float best = std::numeric_limits<float>::max(), along = 0.f, walked = 0.f;
    for (size_t i = 1; i < shape.size(); ++i) {
        const auto& a = shape[i - 1];
        const auto& b = shape[i];
        const double cl = std::cos(a.lat() * 3.14159265358979323846 / 180.0);
        const double dx = (b.lng() - a.lng()) * cl, dy = b.lat() - a.lat();
        const double px = (p.lng() - a.lng()) * cl, py = p.lat() - a.lat();
        const double len2 = dx * dx + dy * dy;
        const double t = len2 > 0.0 ? std::clamp((px * dx + py * dy) / len2, 0.0, 1.0) : 0.0;
        midgard::PointLL q(a.lng() + t * (b.lng() - a.lng()), a.lat() + t * (b.lat() - a.lat()));
        const float seg = a.Distance(b);
        const float d = q.Distance(p);
        if (d < best) { best = d; along = walked + static_cast<float>(t) * seg; }
        walked += seg;
    }
    if (shape.size() == 1) best = shape.front().Distance(p);
    return {along, best};
}

// metres along `shape` to the spot closest to p
float distance_along(const std::vector<midgard::PointLL>& shape, const midgard::PointLL& p) {
    return closest_on_shape(shape, p).first;
}

// shapes of every other road touching either end of the edge (across hierarchy
// levels): these are what a snap of the choke point could wrongly land on
std::vector<std::vector<midgard::PointLL>> neighbour_shapes(const baldr::GraphId& edge_id,
                                                            baldr::GraphReader& reader) {
    const baldr::GraphId opp = reader.GetOpposingEdgeId(edge_id);
    std::vector<baldr::GraphId> nodes{edge_start_node(edge_id, reader), edge_end_node(edge_id, reader)};
    std::unordered_set<uint64_t> seen;
    std::vector<std::vector<midgard::PointLL>> out;
    for (size_t n = 0; n < nodes.size(); ++n) {
        const baldr::GraphId nid = nodes[n];
        if (!nid.Is_Valid() || !seen.insert(nid.value).second) continue;
        auto tile = reader.GetGraphTile(nid);
        const auto* node = tile ? tile->node(nid) : nullptr;
        if (!node) continue;
        for (uint32_t i = 0; i < node->transition_count(); ++i) {
            const auto* tr = tile->transition(node->transition_index() + i);
            if (tr) nodes.push_back(tr->endnode());
        }
        for (uint32_t i = 0; i < node->edge_count(); ++i) {
            baldr::GraphId eid(nid.tileid(), nid.level(), node->edge_index() + i);
            if (eid == edge_id || eid == opp || !is_traversable_edge(tile->directededge(eid))) continue;
            auto shape = get_edge_shape(eid, reader);
            if (!shape.empty()) out.push_back(std::move(shape));
        }
    }
    return out;
}

// Choke on the edge leaving the partition: the spot closest to the partition,
// but at least `offset_m` along the edge, that is `clear_m` away from every
// other road at either end, so snapping to it can only land on this road.
// Short edges with no such spot get the point with the most clearance.
PartitionChokePoint make_choke_point(const baldr::GraphId& edge_id,
                                     baldr::GraphReader& reader,
                                     const std::shared_ptr<sif::DynamicCost>& costing,
                                     float offset_m = 10.f) {
    const float CLEAR_M = 15.f;
    const float STEP_M = 2.f;

    PartitionChokePoint cp;
    cp.edge_id = edge_id;

    // one way = drivable in only one of the two directions for this costing
    auto allowed = [&](const baldr::GraphId& eid) {
        if (!eid.Is_Valid()) return false;
        auto tile = reader.GetGraphTile(eid);
        const auto* de = tile ? tile->directededge(eid) : nullptr;
        return de && costing->Allowed(de, tile);
    };
    cp.is_one_way = allowed(edge_id) != allowed(reader.GetOpposingEdgeId(edge_id));

    auto shape = get_edge_shape(edge_id, reader);
    if (shape.empty()) return cp;
    if (shape.size() == 1) { cp.point = shape.front(); return cp; }

    const float total = shape_length(shape);
    const auto others = neighbour_shapes(edge_id, reader);
    auto clearance = [&](const midgard::PointLL& p) {
        float c = std::numeric_limits<float>::max();
        for (const auto& o : others) c = std::min(c, closest_on_shape(o, p).second);
        return c;
    };

    // an offset pushed out past the partition's own points must stay past them,
    // only the default offset is kept to the first half of a short edge
    const float start = offset_m > 10.f ? std::min(offset_m, std::max(total - 1.f, 0.f))
                                        : std::min(offset_m, total * 0.5f);
    float best_at = start, best_clear = -1.f;
    for (float at = start; at <= total; at += STEP_M) {
        const float c = clearance(cut_shape(shape, at).back());
        if (c >= CLEAR_M) { best_at = at; break; }
        if (c > best_clear) { best_clear = c; best_at = at; }
    }
    cp.point = cut_shape(shape, best_at).back();
    return cp;
}

struct CompletedPartition {
    std::unordered_set<uint32_t> group_indices;
    std::vector<PartitionChokePoint> chokes;
    std::vector<std::vector<midgard::PointLL>> connected_lines;
};

// Node-level undirected graph view. A "node" is a cluster of Valhalla nodes
// linked by hierarchy transitions, so highways (other level) stay connected.
class RegionGraph {
public:
    // public member
    struct Stats { size_t nodes=0, not_trav=0, forbidden=0, dead=0, self=0, kept=0; } stats;

    struct Arc {
        uint64_t canon;        // canonical undirected edge id
        uint64_t to;           // neighbour node key
        baldr::GraphId edge;   // outgoing directed edge
        float secs;
    };

    RegionGraph(baldr::GraphReader& reader, const std::shared_ptr<sif::DynamicCost>& costing)
    : reader_(reader), costing_(costing) {}

    uint64_t key_of(const baldr::GraphId& nid) {
        auto it = node_key_.find(nid.value);
        if (it != node_key_.end()) return it->second;

        std::vector<baldr::GraphId> cluster, stack{nid};
        std::unordered_set<uint64_t> seen{nid.value};
        while (!stack.empty()) {
            baldr::GraphId n = stack.back();
            stack.pop_back();
            cluster.push_back(n);
            auto t = reader_.GetGraphTile(n);
            if (!t) continue;
            const auto* ni = t->node(n);
            if (!ni) continue;
            for (uint32_t i = 0; i < ni->transition_count(); ++i) {
                const auto* tr = t->transition(ni->transition_index() + i);
                if (!tr) continue;
                baldr::GraphId o = tr->endnode();
                if (o.Is_Valid() && seen.insert(o.value).second) stack.push_back(o);
            }
        }
        uint64_t key = nid.value;
        for (const auto& n : cluster) key = std::min(key, n.value);
        for (const auto& n : cluster) node_key_[n.value] = key;
        members_[key] = cluster;
        return key;
    }

    const std::vector<Arc>& arcs(uint64_t key) {
        auto it = arcs_.find(key);
        if (it != arcs_.end()) return it->second;

        std::vector<Arc> out;
        std::vector<baldr::GraphId> dead;
        auto mit = members_.find(key);
        if (mit != members_.end()) {
            ++stats.nodes;

            const auto members = mit->second;  // copy: key_of may touch members_
            for (const auto& m : members) {
                auto tile = reader_.GetGraphTile(m);
                if (!tile) continue;
                const auto* node = tile->node(m);
                if (!node) continue;
                for (uint32_t i = 0; i < node->edge_count(); ++i) {
                    baldr::GraphId eid(m.tileid(), m.level(), node->edge_index() + i);
                    const auto* de = tile->directededge(eid);
                    if (!is_traversable_edge(de)) { ++stats.not_trav; continue; }

                    if (!costing_->Allowed(de, tile)) {
                        baldr::GraphId opp = reader_.GetOpposingEdgeId(eid);
                        bool ok = false;
                        if (opp.Is_Valid()) {
                            auto ot = reader_.GetGraphTile(opp);
                            ok = ot && costing_->Allowed(ot->directededge(opp), ot);
                        }
                        if (!ok) { ++stats.forbidden; continue; }
                    }
                    if (is_dead_end(eid)) { ++stats.dead; dead.push_back(eid); continue; }
                    uint64_t to = key_of(de->endnode());
                    if (to == key) { ++stats.self; continue; }
                    ++stats.kept;
                    float secs = std::max(0.1f, static_cast<float>(costing_->EdgeCost(de, tile).secs));
                    out.push_back({get_canonical_edge_val(eid, reader_, true), to, eid, secs});
                }
            }
        }
        dead_arcs_.emplace(key, std::move(dead));
        return arcs_.emplace(key, std::move(out)).first->second;
    }

    // usable edges leaving `key` into a closed dead-end branch (not in arcs())
    const std::vector<baldr::GraphId>& dead_ends(uint64_t key) {
        arcs(key);
        return dead_arcs_[key];
    }

private:
    bool is_dead_end(const baldr::GraphId& eid) {
        auto it = dead_end_.find(eid.value);
        if (it != dead_end_.end()) return it->second;
        bool r = is_closed_dead_end_subtree(eid, reader_);
        dead_end_[eid.value] = r;
        return r;
    }

    baldr::GraphReader& reader_;
    std::shared_ptr<sif::DynamicCost> costing_;
    std::unordered_map<uint64_t, uint64_t> node_key_;
    std::unordered_map<uint64_t, std::vector<baldr::GraphId>> members_;
    std::unordered_map<uint64_t, std::vector<Arc>> arcs_;
    std::unordered_map<uint64_t, std::vector<baldr::GraphId>> dead_arcs_;
    std::unordered_map<uint64_t, bool> dead_end_;
};

struct BoundaryArc {
    baldr::GraphId edge;  // directed, leaving the region
    uint64_t to;          // outside node
};
using Boundary = std::unordered_map<uint64_t, BoundaryArc>;  // canon edge -> arc

Boundary boundary_of(const std::unordered_set<uint64_t>& nodes, RegionGraph& g) {
    Boundary b;
    for (uint64_t v : nodes)
        for (const auto& a : g.arcs(v))
            if (!nodes.count(a.to)) b[a.canon] = {a.edge, a.to};
    return b;
}

// true if every region neighbour of v still reaches the others once v is gone
bool stays_connected(const std::unordered_set<uint64_t>& region, uint64_t v, RegionGraph& g) {
    std::vector<uint64_t> nbrs;
    for (const auto& a : g.arcs(v))
        if (region.count(a.to)) nbrs.push_back(a.to);
    if (nbrs.size() < 2) return true;

    std::unordered_set<uint64_t> seen{v, nbrs.front()};
    std::vector<uint64_t> stack{nbrs.front()};
    while (!stack.empty()) {
        uint64_t x = stack.back();
        stack.pop_back();
        for (const auto& a : g.arcs(x))
            if (region.count(a.to) && seen.insert(a.to).second) stack.push_back(a.to);
    }
    return std::all_of(nbrs.begin(), nbrs.end(), [&](uint64_t k) { return seen.count(k) > 0; });
}

// Shrinks a region without losing any of its groups (`keep` = their anchors):
//  - drops nodes that open more roads than they close (a split right behind
//    the boundary, e.g. roundabout flares: 2 chokes become 1 before the split)
//  - slides chokes inward along split-free roads, so they end up as close to
//    the groups as possible without crossing a split
void peel_region(std::unordered_set<uint64_t>& region,
                 const std::unordered_set<uint64_t>& keep,
                 RegionGraph& g) {
    std::vector<uint64_t> work(region.begin(), region.end());
    std::unordered_set<uint64_t> queued(region.begin(), region.end());
    while (!work.empty()) {
        uint64_t v = work.back();
        work.pop_back();
        queued.erase(v);
        if (!region.count(v) || keep.count(v)) continue;

        size_t in = 0, out = 0;
        for (const auto& a : g.arcs(v)) ++(region.count(a.to) ? in : out);
        const bool fewer_chokes = in < out;
        const bool slide = in == 1 && out == 1;
        if (!fewer_chokes && !slide) continue;
        if (in >= 2 && !stays_connected(region, v, g)) continue;

        region.erase(v);
        for (const auto& a : g.arcs(v))
            if (region.count(a.to) && queued.insert(a.to).second) work.push_back(a.to);
    }
}

// Dijkstra inside `region` from `sources` until a node with stop(k) is
// settled; the path to it joins `tree`. Returns the path's secs, or -1 when
// no such node is reachable.
template <typename Stop>
double join_path(const std::unordered_set<uint64_t>& region,
                 const std::vector<uint64_t>& sources,
                 Stop stop,
                 std::unordered_set<uint64_t>& tree,
                 RegionGraph& g) {
    using QE = std::pair<double, uint64_t>;
    std::priority_queue<QE, std::vector<QE>, std::greater<QE>> pq;
    std::unordered_map<uint64_t, double> dist;
    std::unordered_map<uint64_t, uint64_t> prev;
    for (uint64_t s : sources)
        if (dist.emplace(s, 0.0).second) pq.push({0.0, s});

    while (!pq.empty()) {
        auto [d, v] = pq.top();
        pq.pop();
        if (d > dist[v]) continue;
        if (stop(v)) {
            for (uint64_t x = v;; x = prev[x]) {
                tree.insert(x);
                if (!prev.count(x)) break;
            }
            return d;
        }
        for (const auto& a : g.arcs(v)) {
            if (!region.count(a.to)) continue;
            const double nd = d + a.secs;
            auto [it, fresh] = dist.try_emplace(a.to, nd);
            if (!fresh && nd >= it->second) continue;
            it->second = nd;
            prev[a.to] = v;
            pq.push({nd, a.to});
        }
    }
    return -1.0;
}

// Drive time linking `groups` to each other inside `region` (greedy Steiner
// tree: grow from the first group, always joining the nearest unlinked one).
// A group's own anchors are linked by its own roads, already in its drive time.
double connection_secs(const std::unordered_set<uint64_t>& region,
                       const std::vector<uint32_t>& groups,
                       const std::vector<std::vector<uint64_t>>& group_keys,
                       RegionGraph& g) {
    if (groups.size() < 2) return 0.0;
    std::unordered_map<uint64_t, std::vector<uint32_t>> at;  // anchor -> unlinked groups
    for (size_t i = 1; i < groups.size(); ++i)
        for (uint64_t k : group_keys[groups[i]]) at[k].push_back(groups[i]);

    std::unordered_set<uint64_t> tree(group_keys[groups[0]].begin(), group_keys[groups[0]].end());
    std::unordered_set<uint32_t> linked{groups[0]};
    auto link_touched = [&]() {
        // every group with an anchor on the tree is linked, its anchors join it
        bool grew = true;
        while (grew) {
            grew = false;
            for (uint32_t grp : groups) {
                if (linked.count(grp)) continue;
                const auto& keys = group_keys[grp];
                if (std::none_of(keys.begin(), keys.end(), [&](uint64_t k) { return tree.count(k) > 0; }))
                    continue;
                linked.insert(grp);
                tree.insert(keys.begin(), keys.end());
                grew = true;
            }
        }
    };
    link_touched();

    double secs = 0.0;
    while (linked.size() < groups.size()) {
        const double d = join_path(region, std::vector<uint64_t>(tree.begin(), tree.end()),
            [&](uint64_t k) {
                auto it = at.find(k);
                return it != at.end() && std::any_of(it->second.begin(), it->second.end(),
                                                     [&](uint32_t grp) { return !linked.count(grp); });
            },
            tree, g);
        if (d < 0.0) break;  // rest is not reachable inside the region
        secs += d;
        link_touched();
    }
    return secs;
}

struct PartitionLimits {
    float  max_secs   = 900.f;  // isochrone radius around the seed group
    size_t max_nodes  = 6000;   // hard cap on region size per growth
    size_t min_groups = 3;      // smaller partitions are not worth the overhead
    size_t max_chokes = 10;     // sanity cap only; the drive time/chokes score decides
    float  delta_secs = 45.f;   // growth bias: cost of opening one extra boundary road
    float  size_exp   = 1.f;    // drive time exponent in the score: < 1 prefers small partitions, > 1 big ones
};

// fewer chokes is always better; size_exp decides how much more drive time is
// worth. a fully closed region (0 chokes) is the best possible shape
double partition_score(double secs, size_t chokes, const PartitionLimits& lim) {
    return std::pow(secs, double(lim.size_exp)) / (chokes == 0 ? 0.5 : double(chokes));
}

// One growth layer: the region after the first `nodes` insertions, its choke
// count (= open roads to the rest of the world), the groups it encloses and
// their drive time (own roads + the roads connecting them).
struct Layer {
    size_t nodes;
    size_t chokes;
    size_t groups;
    double secs;
};

struct Candidate {
    double score = 0.0;
    uint32_t seed = 0;
    size_t epoch = 0;
    std::vector<uint64_t> nodes;  // region of the best layer, empty if none qualified
};

// Grows a region from a seed group, isochrone style, but biased towards nodes
// that close boundary roads instead of opening new ones. Roads without a split
// (degree <= 2 after dead ends are removed) are swallowed whole, so every
// boundary road ends right before a split: that is where the choke sits.
// Every insertion is a layer; the layer with the best drive time/chokes score wins.
class PartitionGrower {
public:
    PartitionGrower(RegionGraph& graph,
                    const std::vector<std::vector<uint64_t>>& group_keys,
                    const std::unordered_map<uint64_t, std::vector<uint32_t>>& groups_at,
                    const std::unordered_map<uint64_t, uint32_t>& owner,
                    const std::vector<int>& part_of,
                    const std::vector<float>& group_secs,
                    const PartitionLimits& lim)
    : graph_(graph), group_keys_(group_keys), groups_at_(groups_at),
      owner_(owner), part_of_(part_of), group_secs_(group_secs), lim_(lim) {}

    Candidate grow(uint32_t seed) {
        reset();
        for (uint64_t k : group_keys_[seed]) absorb(k, 0.f);
        record();

        while (!pq_.empty()) {
            if (order_.size() >= lim_.max_nodes) break;
            QE cur = pq_.top();
            pq_.pop();
            if (!open(cur.node) || ver_[cur.node] != cur.ver) continue;
            absorb(cur.node, dist_[cur.node]);
            record();
        }

        Candidate c;
        c.seed = seed;
        const Layer* best = nullptr;
        for (const auto& l : layers_) {
            if (l.chokes > lim_.max_chokes) continue;
            if (l.groups < lim_.min_groups) continue;
            double s = partition_score(l.secs, l.chokes, lim_);
            if (!best || s > c.score) { best = &l; c.score = s; }
        }
        if (!best) return c;

        // shrink the chosen layer to its groups, then rescore with the real chokes
        std::unordered_set<uint64_t> region(order_.begin(), order_.begin() + best->nodes);
        std::unordered_set<uint64_t> keep;
        std::unordered_set<uint32_t> inside;
        for (uint64_t k : region) {
            auto it = groups_at_.find(k);
            if (it == groups_at_.end()) continue;
            for (uint32_t g : it->second) {
                if (part_of_[g] >= 0) continue;
                const auto& keys = group_keys_[g];
                if (!std::all_of(keys.begin(), keys.end(), [&](uint64_t gk) { return region.count(gk) > 0; }))
                    continue;
                if (inside.insert(g).second) keep.insert(keys.begin(), keys.end());
            }
        }
        peel_region(region, keep, graph_);
        const size_t chokes = boundary_of(region, graph_).size();
        // seed first, the rest in a fixed order so the tree doesn't depend on hashing
        std::vector<uint32_t> ordered(inside.begin(), inside.end());
        std::sort(ordered.begin(), ordered.end());
        auto sit = std::find(ordered.begin(), ordered.end(), seed);
        if (sit != ordered.end()) std::rotate(ordered.begin(), sit, sit + 1);
        double secs = connection_secs(region, ordered, group_keys_, graph_);
        for (uint32_t g : ordered) secs += group_secs_[g];
        c.score = partition_score(secs, chokes, lim_);
        c.nodes.assign(region.begin(), region.end());
        return c;
    }

private:
    struct QE {
        float prio; uint64_t node; uint32_t ver;
        bool operator>(const QE& o) const { return prio > o.prio; }
    };

    void reset() {
        in_.clear(); order_.clear(); dist_.clear(); ver_.clear();
        hits_.clear(); boundary_.clear(); layers_.clear();
        groups_in_ = 0;
        secs_in_ = 0.0;
        tree_.clear();
        pq_ = decltype(pq_)();
    }

    bool open(uint64_t k) const { return !in_.count(k) && !owner_.count(k); }

    // boundary roads gained (+) or closed (-) by inserting k
    int delta(uint64_t k) {
        int d = 0;
        for (const auto& a : graph_.arcs(k)) d += in_.count(a.to) ? -1 : 1;
        return d;
    }

    void push(uint64_t k) {
        float p = dist_[k] + lim_.delta_secs * static_cast<float>(delta(k));
        pq_.push({p, k, ++ver_[k]});
    }

    void insert(uint64_t v) {
        in_.insert(v);
        order_.push_back(v);
        for (const auto& a : graph_.arcs(v)) {
            if (in_.count(a.to)) boundary_.erase(a.canon);  // road became interior
            else boundary_.insert(a.canon);                 // road leads outside
        }
        auto it = groups_at_.find(v);
        if (it == groups_at_.end()) return;
        for (uint32_t g : it->second)
            if (part_of_[g] < 0 && ++hits_[g] == group_keys_[g].size()) {
                ++groups_in_;
                secs_in_ += group_secs_[g];
                link_group(g);
            }
    }

    // join a group that just got inside to the connection tree by its shortest
    // path through the region so far, adding that drive time
    void link_group(uint32_t g) {
        const auto& keys = group_keys_[g];
        const bool on_tree = std::any_of(keys.begin(), keys.end(), [&](uint64_t k) { return tree_.count(k) > 0; });
        if (!tree_.empty() && !on_tree) {
            const double d = join_path(in_, keys, [&](uint64_t k) { return tree_.count(k) > 0; }, tree_, graph_);
            if (d > 0.0) secs_in_ += d;
        }
        tree_.insert(keys.begin(), keys.end());
    }

    // insert v, then follow every split-free road leaving it up to the next split
    void absorb(uint64_t v, float d) {
        std::vector<std::pair<uint64_t, float>> work{{v, d}};
        while (!work.empty()) {
            auto [x, dx] = work.back();
            work.pop_back();
            if (!open(x)) continue;
            auto [xit, xfresh] = dist_.try_emplace(x, dx);
            if (!xfresh) xit->second = std::min(xit->second, dx);
            const float base = xit->second;
            insert(x);

            for (const auto& a : graph_.arcs(x)) {
                if (!open(a.to)) continue;
                float nd = base + a.secs;
                if (nd <= lim_.max_secs) {
                    auto [it, fresh] = dist_.try_emplace(a.to, nd);
                    if (!fresh) it->second = std::min(it->second, nd);
                }
                auto it = dist_.find(a.to);
                if (it == dist_.end()) continue;  // beyond the isochrone
                if (graph_.arcs(a.to).size() <= 2 && order_.size() < lim_.max_nodes)
                    work.push_back({a.to, it->second});
                else
                    push(a.to);  // a split (or cap): its delta changed, re-queue
            }
        }
    }

    void record() {
        Layer l{order_.size(), boundary_.size(), groups_in_, secs_in_};
        // identical counts on a bigger region add nothing, keep the smaller one
        if (!layers_.empty() && layers_.back().chokes == l.chokes && layers_.back().groups == l.groups)
            return;
        layers_.push_back(l);
    }

    RegionGraph& graph_;
    const std::vector<std::vector<uint64_t>>& group_keys_;
    const std::unordered_map<uint64_t, std::vector<uint32_t>>& groups_at_;
    const std::unordered_map<uint64_t, uint32_t>& owner_;
    const std::vector<int>& part_of_;
    const std::vector<float>& group_secs_;
    const PartitionLimits& lim_;

    std::unordered_set<uint64_t> in_;
    std::vector<uint64_t> order_;
    std::unordered_map<uint64_t, float> dist_;
    std::unordered_map<uint64_t, uint32_t> ver_;
    std::unordered_map<uint32_t, size_t> hits_;  // group -> anchors inside
    std::unordered_set<uint64_t> boundary_;      // canon roads leaving the region
    std::vector<Layer> layers_;
    size_t groups_in_ = 0;
    double secs_in_ = 0.0;                    // group roads + connections between them
    std::unordered_set<uint64_t> tree_;       // nodes linking the groups inside
    std::priority_queue<QE, std::vector<QE>, std::greater<QE>> pq_;
};

// Drive time over a group's own roads: a normal group drives its edge once in
// the snapped direction, a dead end group drives its whole closed subtree in
// and back out. Floored at 1 s so a negative size_exp can't blow up the score.
std::vector<float> group_drive_secs(const std::vector<FinalGroup>& groups,
                                    const std::shared_ptr<sif::DynamicCost>& costing,
                                    baldr::GraphReader& reader) {
    const float FALLBACK_MPS = 30.f / 3.6f;  // edge not drivable either way
    std::unordered_map<uint64_t, float> cache;

    auto allowed_secs = [&](const baldr::GraphId& eid, float& secs) {
        if (!eid.Is_Valid()) return false;
        auto tile = reader.GetGraphTile(eid);
        const auto* de = tile ? tile->directededge(eid) : nullptr;
        if (!de || !costing->Allowed(de, tile)) return false;
        secs = static_cast<float>(costing->EdgeCost(de, tile).secs);
        return true;
    };
    // the edge's own direction if allowed, else the opposing one, else its length
    auto edge_secs = [&](const baldr::GraphId& eid) {
        auto it = cache.find(eid.value);
        if (it != cache.end()) return it->second;
        float secs = 0.f;
        if (!allowed_secs(eid, secs) && !allowed_secs(reader.GetOpposingEdgeId(eid), secs)) {
            auto tile = reader.GetGraphTile(eid);
            const auto* de = tile ? tile->directededge(eid) : nullptr;
            secs = de ? static_cast<float>(de->length()) / FALLBACK_MPS : 0.f;
        }
        cache[eid.value] = secs;
        return secs;
    };

    std::vector<float> out(groups.size(), 0.f);
    for (size_t g = 0; g < groups.size(); ++g) {
        const auto& grp = groups[g];
        if (grp.raw_edges.empty()) { out[g] = 1.f; continue; }
        const baldr::GraphId root = grp.raw_edges.front();
        float secs = 0.f;
        if (grp.dead_end_side == 0) {
            secs = edge_secs(root);
        } else {
            // root is the inward edge of the subtree, walk it like draw_branch
            std::unordered_set<uint64_t> seen;
            std::vector<baldr::GraphId> stack{root};
            for (size_t guard = 0; !stack.empty() && guard < 64; ++guard) {
                baldr::GraphId e = stack.back();
                stack.pop_back();
                if (!seen.insert(get_canonical_edge_val(e, reader, true)).second) continue;
                const baldr::GraphId back = reader.GetOpposingEdgeId(e);
                secs += edge_secs(e) + (back.Is_Valid() ? edge_secs(back) : edge_secs(e));
                auto tile = reader.GetGraphTile(e);
                const auto* de = tile ? tile->directededge(e) : nullptr;
                if (!de) continue;
                baldr::GraphId end = de->endnode();
                auto end_tile = reader.GetGraphTile(end);
                const auto* node = end_tile ? end_tile->node(end) : nullptr;
                if (!node) continue;
                for (uint32_t i = 0; i < node->edge_count(); ++i) {
                    baldr::GraphId next(end.tileid(), end.level(), node->edge_index() + i);
                    if (next != back && is_traversable_edge(end_tile->directededge(next)))
                        stack.push_back(next);
                }
            }
        }
        out[g] = std::max(1.f, secs);
    }
    return out;
}

std::vector<CompletedPartition> compute_isochrone_partitions(
    const std::vector<FinalGroup>& groups,
    const std::shared_ptr<sif::DynamicCost>& costing,
    baldr::GraphReader& reader,
    float partition_ratio) {

    const uint32_t n = static_cast<uint32_t>(groups.size());
    if (n == 0) return {};

    // ratio 0..1 -> drive time exponent -1..3, 0.5 is the plain time/chokes ratio
    PartitionLimits lim;
    lim.size_exp = 4.f * partition_ratio - 1.f;
    RegionGraph graph(reader, costing);

    // 1. the open-graph nodes each group hangs off; a group is inside a region
    //    once all of them are
    std::vector<std::vector<uint64_t>> group_keys(n);
    std::unordered_map<uint64_t, std::vector<uint32_t>> groups_at;
    for (uint32_t g = 0; g < n; ++g) {
        auto& keys = group_keys[g];
        for (const auto& a : groups[g].anchor_nodes)
            if (a.Is_Valid()) keys.push_back(graph.key_of(a));
        std::sort(keys.begin(), keys.end());
        keys.erase(std::unique(keys.begin(), keys.end()), keys.end());
        for (uint64_t k : keys) groups_at[k].push_back(g);
    }

    std::unordered_map<uint64_t, uint32_t> owner;  // node -> partition
    std::vector<int> part_of(n, -1);               // group -> partition
    std::vector<std::vector<uint64_t>> part_nodes;
    const std::vector<float> group_secs = group_drive_secs(groups, costing, reader);
    PartitionGrower grower(graph, group_keys, groups_at, owner, part_of, group_secs, lim);

    // 2. one candidate per distinct seed, best ratio first
    std::vector<Candidate> pool;
    std::priority_queue<std::pair<double, size_t>> pq;
    std::set<std::vector<uint64_t>> seen_seeds;
    for (uint32_t g = 0; g < n; ++g) {
        if (group_keys[g].empty() || !seen_seeds.insert(group_keys[g]).second) continue;
        Candidate c = grower.grow(g);
        if (c.nodes.empty()) continue;
        pq.push({c.score, pool.size()});
        pool.push_back(std::move(c));
    }

    // 3. lazy greedy commit; a candidate that overlaps a committed partition is
    //    regrown against the current walls and re-queued
    size_t epoch = 0;
    while (!pq.empty()) {
        size_t idx = pq.top().second;
        pq.pop();
        if (pool[idx].epoch != epoch) {
            const auto& nodes = pool[idx].nodes;
            bool overlap = std::any_of(nodes.begin(), nodes.end(),
                                       [&](uint64_t k) { return owner.count(k) > 0; });
            if (overlap) {
                uint32_t seed = pool[idx].seed;
                if (part_of[seed] >= 0) continue;
                Candidate r = grower.grow(seed);
                r.epoch = epoch;
                if (r.nodes.empty()) continue;
                pq.push({r.score, idx});
                pool[idx] = std::move(r);
                continue;
            }
        }

        const uint32_t pid = static_cast<uint32_t>(part_nodes.size());
        for (uint64_t k : pool[idx].nodes) owner[k] = pid;
        for (uint64_t k : pool[idx].nodes) {
            auto it = groups_at.find(k);
            if (it == groups_at.end()) continue;
            for (uint32_t g : it->second) {
                if (part_of[g] >= 0) continue;
                bool inside = std::all_of(group_keys[g].begin(), group_keys[g].end(), [&](uint64_t gk) {
                    auto o = owner.find(gk);
                    return o != owner.end() && o->second == pid;
                });
                if (inside) part_of[g] = static_cast<int>(pid);
            }
        }
        part_nodes.push_back(std::move(pool[idx].nodes));
        ++epoch;
    }

    // 4. a group sitting on a choke road itself joins the partition that owns
    //    one end of that road
    size_t leftover = 0;
    for (uint32_t g = 0; g < n; ++g) {
        if (part_of[g] >= 0) continue;
        for (uint64_t k : group_keys[g]) {
            auto o = owner.find(k);
            if (o != owner.end()) { part_of[g] = static_cast<int>(o->second); break; }
        }
        if (part_of[g] < 0) ++leftover;
    }

    // 5. emit
    std::vector<CompletedPartition> result(part_nodes.size());
    for (uint32_t g = 0; g < n; ++g)
        if (part_of[g] >= 0) result[part_of[g]].group_indices.insert(g);

    std::vector<std::unordered_set<uint64_t>> emitted(part_nodes.size());
    auto draw = [&](uint32_t p, const baldr::GraphId& eid) {
        if (!emitted[p].insert(get_canonical_edge_val(eid, reader, true)).second) return;
        auto shape = get_edge_shape(eid, reader);
        if (shape.size() >= 2) result[p].connected_lines.push_back(std::move(shape));
    };

    // a closed dead-end branch, entered through `root`
    auto draw_branch = [&](uint32_t p, const baldr::GraphId& root) {
        std::vector<baldr::GraphId> stack{root};
        for (size_t guard = 0; !stack.empty() && guard < 64; ++guard) {
            baldr::GraphId e = stack.back();
            stack.pop_back();
            if (emitted[p].count(get_canonical_edge_val(e, reader, true))) continue;
            draw(p, e);
            auto tile = reader.GetGraphTile(e);
            const auto* de = tile ? tile->directededge(e) : nullptr;
            if (!de) continue;
            baldr::GraphId end = de->endnode();
            auto end_tile = reader.GetGraphTile(end);
            const auto* node = end_tile ? end_tile->node(end) : nullptr;
            if (!node) continue;
            baldr::GraphId back = reader.GetOpposingEdgeId(e);
            for (uint32_t i = 0; i < node->edge_count(); ++i) {
                baldr::GraphId next(end.tileid(), end.level(), node->edge_index() + i);
                if (next != back && is_traversable_edge(end_tile->directededge(next)))
                    stack.push_back(next);
            }
        }
    };

    // groups on an open road, by that road
    std::unordered_map<uint64_t, std::vector<uint32_t>> groups_on_edge;
    for (uint32_t g = 0; g < n; ++g)
        if (part_of[g] >= 0 && groups[g].dead_end_side == 0 && !groups[g].raw_edges.empty())
            groups_on_edge[get_canonical_edge_val(groups[g].raw_edges.front(), reader, true)].push_back(g);

    const float CHOKE_OFFSET_M = 10.f;
    for (uint32_t p = 0; p < part_nodes.size(); ++p) {
        std::unordered_set<uint64_t> nodes(part_nodes[p].begin(), part_nodes[p].end());

        // chokes; a choke road that carries this partition's own points gets its
        // choke just past the farthest of them, and is drawn only up to the choke
        for (const auto& [canon, arc] : boundary_of(nodes, graph)) {
            float offset = CHOKE_OFFSET_M;
            bool carries = false;
            std::vector<midgard::PointLL> shape;
            auto it = groups_on_edge.find(canon);
            if (it != groups_on_edge.end()) {
                for (uint32_t g : it->second) {
                    if (part_of[g] != static_cast<int>(p)) continue;
                    if (!carries) { shape = get_edge_shape(arc.edge, reader); carries = true; }
                    for (const auto& pt : groups[g].points)
                        offset = std::max(offset, distance_along(shape, pt.projected) + CHOKE_OFFSET_M);
                }
            }
            if (carries) {
                const float total = shape_length(shape);
                if (total > 2.f) offset = std::min(offset, total - 1.f);
                emitted[p].insert(canon);
                auto part = cut_shape(shape, offset);
                if (part.size() >= 2) result[p].connected_lines.push_back(std::move(part));
            }
            result[p].chokes.push_back(make_choke_point(arc.edge, reader, costing, offset));        }

        for (uint64_t v : nodes) {
            for (const auto& a : graph.arcs(v))
                if (nodes.count(a.to)) draw(p, a.edge);
            // dead ends hanging off the partition belong to it
            for (const auto& eid : graph.dead_ends(v)) draw_branch(p, eid);
        }
    }

    // a group's own roads (normally already drawn above)
    for (uint32_t g = 0; g < n; ++g) {
        if (part_of[g] < 0) continue;
        const uint32_t p = static_cast<uint32_t>(part_of[g]);
        for (const auto& eid : groups[g].raw_edges) {
            if (is_closed_dead_end_subtree(eid, reader)) { draw_branch(p, eid); continue; }
            baldr::GraphId opp = reader.GetOpposingEdgeId(eid);
            if (opp.Is_Valid() && is_closed_dead_end_subtree(opp, reader)) draw_branch(p, opp);
            else draw(p, eid);
        }
    }

    return result;
}

} // namespace

// -----------------------------------------------------------------------------
// Loki Worker Methods
// -----------------------------------------------------------------------------

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
    float partition_ratio =
        options.has_partition_ratio_case() ? options.partition_ratio() : 0.5f;

    try {
        const auto locations = baldr::PathLocation::fromPBF(options.locations());
        const auto projections = loki::Search(locations, *reader, costing);

        // 1. Group strictly by canonical Edge ID
        auto raw_edge_groups = group_points_by_edge(
            locations, projections, ignore_road_side, *reader);

        // 2. Merge edge groups belonging to the same dead-end subtree
        auto final_groups = merge_dead_end_edge_groups(raw_edge_groups, *reader);

        // 3. Compute partitions sequentially via Isochrone Choke-Barrier Map
        auto partitions = compute_isochrone_partitions(final_groups, costing, *reader, partition_ratio);

        // 4. Serialize results to Protobuf
        auto group_locations_res = request.mutable_group_locations();
        group_locations_res->clear_results();

        for (const auto& group : final_groups) {
            auto result = group_locations_res->add_results();
            if (group.dead_end_side != 0) {
                result->set_dead_end_side(group.dead_end_side);
            }
            for (const auto& pt : group.points) {
                result->add_location_index(pt.original_index);
                auto snapped = result->add_locations();
                snapped->set_location_index(pt.original_index);
                snapped->mutable_snapped_point()->set_lat(pt.projected.lat());
                snapped->mutable_snapped_point()->set_lng(pt.projected.lng());
            }
        }

        for (const auto& partition : partitions) {
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
            for (const auto& seg : partition.connected_lines) {
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