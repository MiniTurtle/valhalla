//#include "loki/search.h"
//#include "loki/worker.h"
//
//#include <algorithm>
//#include <unordered_map>
//#include <vector>
//
//using namespace valhalla;
//using namespace valhalla::baldr;

//namespace valhalla {
//namespace loki {
//
//void loki_worker_t::init_group_locations(Api& request) {
//    auto& options = *request.mutable_options();
//    parse_locations(options.mutable_locations(), request);
//    if (options.locations_size() < 1) {
//	throw valhalla_exception_t{120};
//    }
//    parse_costing(request);
//}
//
//void loki_worker_t::group_locations(Api& request) {
//    auto _ = measure_scope_time(request);
//    init_group_locations(request);
//
//    auto& options = *request.mutable_options();
//    bool ignore_road_side = options.ignore_road_side();
//
//    try {
//		auto locations = PathLocation::fromPBF(options.locations());
//		const auto projections = loki::Search(locations, *reader, costing);
//
//		// Structure to hold projected point info
//		struct PointInfo {
//			uint32_t original_index;
//			GraphId edge_id;
//			float percent_along;
//			valhalla::midgard::PointLL projected;
//		};
//
//		// We will group by an undirected identifier.
//		// If ignore_road_side is false, we strictly use the exact DirectedEdge ID.
//		// If true, we canonicalize it by choosing the smaller GraphId between the edge and its
//		// opposing edge.
//		std::unordered_map<uint64_t, std::vector<PointInfo>> groups;
//
//		for (size_t i = 0; i < locations.size(); ++i) {
//			const auto& projection = projections.at(locations[i]);
//			if (projection.edges.empty())
//			continue; // Skip if it couldn't snap
//
//			GraphId edge_id = projection.edges.front().id;
//			float percent = projection.edges.front().percent_along;
//			auto projected = projection.edges.front().projected;
//
//			uint64_t group_key = edge_id.value;
//			if (ignore_road_side) {
//				GraphId opposing_id = reader->GetOpposingEdgeId(edge_id);
//				if (opposing_id.Is_Valid() && opposing_id.value < edge_id.value) {
//					group_key = opposing_id.value;
//					// Invert percent_along since we are mapping it to the opposing edge's direction
//					percent = 1.0f - percent;
//				}
//			}
//
//			groups[group_key].push_back({static_cast<uint32_t>(i), edge_id, percent, projected});
//		}
//
//	// Prepare response
//	auto group_locations_res = request.mutable_group_locations();
//	group_locations_res->clear_results();
//
//	for (auto& [key, points] : groups) {
//	    // Sort points by percent_along
//	    std::sort(points.begin(), points.end(), [](const PointInfo& a, const PointInfo& b) {
//		return a.percent_along < b.percent_along;
//	    });
//
//	    auto result = group_locations_res->add_results();
//
//	    // Determine if either end of the edge is a dead end
//	    if (!points.empty()) {
//			auto edge_id = points.front().edge_id;
//			auto tile = reader->GetGraphTile(edge_id);
//			if (tile) {
//				const auto* edge = tile->directededge(edge_id);
//				if (edge) {
//					auto endnode_id = edge->endnode();
//					auto end_tile = reader->GetGraphTile(endnode_id);
//					if (end_tile) {
//						const auto* end_node = end_tile->node(endnode_id);
//						if (end_node && end_node->edge_count() == 1) {
//							// If edge_id direction points to a dead end, that's side 2 (end)
//							result->set_dead_end_side(2);
//						}
//					}
//
//					// Check the start node (end node of the opposing edge)
//					auto opp_id = reader->GetOpposingEdgeId(edge_id);
//					if (opp_id.Is_Valid()) {
//						auto opp_tile = reader->GetGraphTile(opp_id);
//						if (opp_tile) {
//							const auto* opp_edge = opp_tile->directededge(opp_id);
//							if (opp_edge) {
//								auto startnode_id = opp_edge->endnode();
//								auto start_tile = reader->GetGraphTile(startnode_id);
//								if (start_tile) {
//									const auto* start_node = start_tile->node(startnode_id);
//									if (start_node && start_node->edge_count() == 1) {
//										result->set_dead_end_side(1);
//									}
//								}
//							}
//						}
//					}
//				}
//			}
//	    }
//
//	    for (const auto& pt : points) {
//			result->add_location_index(pt.original_index);
//
//			auto snapped = result->add_locations();
//			snapped->set_location_index(pt.original_index);
//			snapped->mutable_snapped_point()->set_lat(pt.projected.lat());
//			snapped->mutable_snapped_point()->set_lng(pt.projected.lng());
//	    }
//	}
//
//    } catch (const std::exception& e) { throw valhalla_exception_t{171, std::string(e.what())}; }
//}
//
//} // namespace loki
//} // namespace valhalla

#include <valhalla/loki/worker.h>
#include <valhalla/loki/search.h>
#include <valhalla/baldr/graphreader.h>
#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/directededge.h>
#include <valhalla/baldr/nodeinfo.h>
#include <valhalla/baldr/pathlocation.h>
#include <valhalla/midgard/pointll.h>

#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>

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

        // STAGE 1: Group strictly by Edge ID first (matches your 155-group baseline).
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

        // STAGE 2: Merge edge groups that belong to the same dead-end subtree.
        struct FinalPoint {
            uint32_t original_index;
            float sort_offset;
            midgard::PointLL projected;
        };

        struct MergedGroup {
            int dead_end_side = 0;
            std::vector<FinalPoint> points;
        };

        std::unordered_map<uint64_t, MergedGroup> final_groups;

        for (auto& [edge_key, raw_points] : edge_groups) {
            baldr::GraphId sample_edge = raw_points.front().edge_id;
            SubtreeTrace sample_trace = climb_to_dead_end_root(sample_edge, 0.0f, *reader);

            if (sample_trace.in_dead_end && sample_trace.root_inward_edge.Is_Valid()) {
                // Canonicalize root key so all branches/sides in the dead end merge into 1 group
                uint64_t root_key = get_canonical_edge_val(sample_trace.root_inward_edge, *reader, true);
                auto& target_group = final_groups[root_key];

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
                // Normal through-street: preserve Stage 1 edge_key
                auto& target_group = final_groups[edge_key];
                for (const auto& rp : raw_points) {
                    target_group.points.push_back({rp.original_index, rp.canonical_percent, rp.projected});
                }
            }
        }

        // Prepare response
        auto group_locations_res = request.mutable_group_locations();
        group_locations_res->clear_results();

        for (auto& [key, group] : final_groups) {
            std::sort(group.points.begin(), group.points.end(), [](const FinalPoint& a, const FinalPoint& b) {
                return a.sort_offset < b.sort_offset;
            });

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

    } catch (const std::exception& e) {
        throw valhalla_exception_t{171, std::string(e.what())};
    }
}

} // namespace loki
} // namespace valhalla