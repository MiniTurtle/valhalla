#include "loki/search.h"
#include "loki/worker.h"

#include <unordered_map>
#include <vector>
#include <algorithm>

using namespace valhalla;
using namespace valhalla::baldr;

namespace valhalla {
namespace loki {

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
    auto locations = PathLocation::fromPBF(options.locations());
    const auto projections = loki::Search(locations, *reader, costing);

    // Structure to hold projected point info
    struct PointInfo {
      uint32_t original_index;
      GraphId edge_id;
      float percent_along;
      valhalla::midgard::PointLL projected;
    };

    // We will group by an undirected identifier.
    // If ignore_road_side is false, we strictly use the exact DirectedEdge ID.
    // If true, we canonicalize it by choosing the smaller GraphId between the edge and its opposing edge.
    std::unordered_map<uint64_t, std::vector<PointInfo>> groups;

    for (size_t i = 0; i < locations.size(); ++i) {
      const auto& projection = projections.at(locations[i]);
      if (projection.edges.empty()) continue; // Skip if it couldn't snap

      GraphId edge_id = projection.edges.front().id;
      float percent = projection.edges.front().percent_along;
      auto projected = projection.edges.front().projected;

      uint64_t group_key = edge_id.value;
      if (ignore_road_side) {
        GraphId opposing_id = reader->GetOpposingEdgeId(edge_id);
        if (opposing_id.Is_Valid() && opposing_id.value < edge_id.value) {
          group_key = opposing_id.value;
          // Invert percent_along since we are mapping it to the opposing edge's direction
          percent = 1.0f - percent;
        }
      }

      groups[group_key].push_back({static_cast<uint32_t>(i), edge_id, percent, projected});
    }

    // Prepare response
    auto group_locations_res = request.mutable_group_locations();
    group_locations_res->clear_results();

    for (auto& [key, points] : groups) {
      // Sort points by percent_along
      std::sort(points.begin(), points.end(), [](const PointInfo& a, const PointInfo& b) {
        return a.percent_along < b.percent_along;
      });

      auto result = group_locations_res->add_results();
      
      // Determine if either end of the edge is a dead end
      if (!points.empty()) {
        auto edge_id = points.front().edge_id;
        auto tile = reader->GetGraphTile(edge_id);
        if (tile) {
          const auto* edge = tile->directededge(edge_id);
          if (edge) {
            auto endnode_id = edge->endnode();
            auto end_tile = reader->GetGraphTile(endnode_id);
            if (end_tile) {
              const auto* end_node = end_tile->node(endnode_id);
              if (end_node && end_node->edge_count() == 1) {
                // If edge_id direction points to a dead end, that's side 2 (end)
                result->set_dead_end_side(2);
              }
            }

            // Check the start node (end node of the opposing edge)
            auto opp_id = reader->GetOpposingEdgeId(edge_id);
            if (opp_id.Is_Valid()) {
              auto opp_tile = reader->GetGraphTile(opp_id);
              if (opp_tile) {
                const auto* opp_edge = opp_tile->directededge(opp_id);
                if (opp_edge) {
                  auto startnode_id = opp_edge->endnode();
                  auto start_tile = reader->GetGraphTile(startnode_id);
                  if (start_tile) {
                    const auto* start_node = start_tile->node(startnode_id);
                    if (start_node && start_node->edge_count() == 1) {
                      result->set_dead_end_side(1);
                    }
                  }
                }
              }
            }
          }
        }
      }

      for (const auto& pt : points) {
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
