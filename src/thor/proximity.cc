#include "thor/proximity.h"
#include "midgard/distanceapproximator.h"
#include "midgard/logging.h"
#include "midgard/util.h"

#include <algorithm>

using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::sif;

namespace valhalla {
namespace thor {

Proximity::Proximity(const boost::property_tree::ptree& config) : 
    Dijkstras(config) {
    if (max_search_locations < 1)
        max_search_locations = 1;
}

// Replaces ConstructIsoTile. We only need this to figure out how far to search
// before giving up, so we calculate the physical bounds of the targets.
void Proximity::CalculateMaxDistance(const valhalla::Api& api) {
    PointLL center_ll(api.options().locations(location_index).ll().lng(), 
                      api.options().locations(location_index).ll().lat());
    AABB2<PointLL> bounds(center_ll.lng(), center_ll.lat(), center_ll.lng(), center_ll.lat());
    
    for (const auto& location : api.options().locations()) {
        PointLL ll(location.ll().lng(), location.ll().lat());
        bounds.Expand(ll);
    }

    float width_meters = bounds.Width() * DistanceApproximator<PointLL>::MetersPerLngDegree(bounds.Center().lat());
    float height_meters = bounds.Height() * kMetersPerDegreeLat;

    // The furthest point + 10km of padding to allow for curving roads
    max_meters_ = std::max(width_meters, height_meters) + 10000.0f; 
}

std::vector<Proximity::ProximityResult> Proximity::FindProximity(const ExpansionType& expansion_direction,
                                                      Api& api,
                                                      GraphReader& graphreader,
                                                      const sif::mode_costing_t& mode_costing,
                                                      const travel_mode_t mode,
                                                      uint32_t max_search_locations,
                                                      int32_t location_index) {
    this->max_search_locations = max_search_locations;
    this->location_index = location_index;
    
    CalculateMaxDistance(api);

    mode_ = mode;
    costing_ = mode_costing[static_cast<uint32_t>(mode_)];
    access_mode_ = costing_->access_mode();

    // Map edge ids to target locations
    auto locations = *api.mutable_options()->mutable_locations();
    bool ignore_side = api.options().ignore_road_side();
    
    edge_id_to_location_index.clear();
    for (int32_t index = 0; index < locations.size(); index++) {
        if (index == location_index) continue;

        auto& l = locations[index];
        for (auto& e : l.correlation().edges()) {
            valhalla::baldr::GraphId id(e.graph_id());
            float p = e.percent_along();
            edge_id_to_location_index[id].push_back(std::make_pair(index, p));
            
            if (ignore_side) {
                valhalla::baldr::GraphId opp_id = graphreader.GetOpposingEdgeId(id);
                if (opp_id.Is_Valid()) {
                    edge_id_to_location_index[opp_id].push_back(std::make_pair(index, 1.0f - p));
                }
            }
        }
    }

    Initialize(bdedgelabels_, adjacencylist_, costing_->UnitSize());

    google::protobuf::RepeatedPtrField<Location> location_origin;
    auto* origin_loc = location_origin.Add();
    origin_loc->CopyFrom(locations.Get(location_index));
    
    if (ignore_side) {
        auto* correlation = origin_loc->mutable_correlation();
        int original_size = correlation->edges_size();
        for (int i = 0; i < original_size; ++i) {
            valhalla::baldr::GraphId id(correlation->edges(i).graph_id());
            valhalla::baldr::GraphId opp_id = graphreader.GetOpposingEdgeId(id);
            if (opp_id.Is_Valid()) {
                // Check if opposing edge is already in the list
                bool exists = false;
                for (int j = 0; j < correlation->edges_size(); ++j) {
                    if (valhalla::baldr::GraphId(correlation->edges(j).graph_id()) == opp_id) {
                        exists = true; break;
                    }
                }
                if (!exists) {
                    auto* new_edge = correlation->add_edges();
                    new_edge->CopyFrom(correlation->edges(i)); // Copy attributes like percent_along
                    new_edge->set_graph_id(opp_id);
                    new_edge->set_percent_along(1.0f - new_edge->percent_along()); // Invert percent along
                }
            }
        }
    }
    
    SetOriginLocations(graphreader, location_origin, costing_);

    found_locations.clear();
    found_locations.reserve(max_search_locations);

    auto time_infos = SetTime(location_origin, graphreader);

    auto cb_decision = ExpansionRecommendation::continue_expansion;
    while (cb_decision != ExpansionRecommendation::stop_expansion) {
        uint32_t predindex = adjacencylist_.pop();
        if (predindex == baldr::kInvalidLabel) {
            break;
        }

        sif::BDEdgeLabel pred = bdedgelabels_[predindex];
        edgestatus_.Update(pred.edgeid(), EdgeSet::kPermanent, pred.path_id());

        const baldr::DirectedEdge* opp_pred_edge = nullptr;
        if (expansion_direction == ExpansionType::reverse) {
            opp_pred_edge = graphreader.GetOpposingEdge(pred.edgeid());
            if (opp_pred_edge == nullptr) {
                continue;
            }
        }

        // Did we hit a target?
        auto edge_id = pred.edgeid();
        auto it = edge_id_to_location_index.find(edge_id);
        if (it != edge_id_to_location_index.end()) {
            for (const auto& found_info : it->second) {
                int32_t found_index = found_info.first;
                float dest_p = found_info.second;
                
                // Have we already recorded this location?
                auto loc_it = std::find_if(found_locations.begin(), found_locations.end(),
                    [found_index](const ProximityResult& r) { return r.location_index == found_index; });
                
                if (loc_it == found_locations.end()) {
                    // FIRST HIT: Because Dijkstra pops by lowest cost, the first time 
                    // we see this location is mathematically the lowest cost path to it!
                    
                    const graph_tile_ptr tile = graphreader.GetGraphTile(edge_id);
                    if (!tile) {
                        continue;
                    }
                    const auto* edge = tile->directededge(edge_id);
                    valhalla::sif::Cost edge_cost = costing_->EdgeCost(edge, tile);

                    
                    float unused_fraction = 1.0f - dest_p;
                    float actual_secs = pred.cost().secs - (edge_cost.secs * unused_fraction);
                    float actual_cost = pred.cost().cost - (edge_cost.cost * unused_fraction);
                    
                    float actual_distance = static_cast<float>(pred.path_distance()) - (edge->length() * unused_fraction);
                    
                    // If distance or time is negative, this target is physically behind the origin 
                    // on this directed edge, and cannot be reached going forward. We must skip it
                    // so Dijkstra finds the real, legal route that loops back around to it!
                    if (actual_distance < -1.0f || actual_secs < -1.0f) {
                        continue;
                    }

                    actual_secs = std::max(0.0f, actual_secs);
                    actual_cost = std::max(0.0f, actual_cost);
                    actual_distance = std::max(0.0f, actual_distance);

                    std::vector<uint32_t> path_indices;
                    uint32_t current_idx = predindex;
                    while (current_idx != valhalla::baldr::kInvalidLabel) {
                        path_indices.push_back(current_idx);
                        current_idx = bdedgelabels_[current_idx].predecessor();
                    }
                    std::reverse(path_indices.begin(), path_indices.end());

                    std::vector<int32_t> overlaps;
                    for (size_t i = 0; i < path_indices.size(); i++) {
                        uint32_t trace_current = path_indices[i];
                        auto trace_edge_id = bdedgelabels_[trace_current].edgeid();
                        auto overlap_it = edge_id_to_location_index.find(trace_edge_id);
                        if (overlap_it != edge_id_to_location_index.end()) {
                            
                            float min_p = 0.0f;
                            float max_p = 1.0f;
                            
                            if (i == path_indices.size() - 1) {
                                max_p = dest_p;
                            }
                            
                            if (i == 0) {
                                for (int e_idx = 0; e_idx < origin_loc->correlation().edges_size(); ++e_idx) {
                                    if (valhalla::baldr::GraphId(origin_loc->correlation().edges(e_idx).graph_id()) == trace_edge_id) {
                                        min_p = origin_loc->correlation().edges(e_idx).percent_along();
                                        break;
                                    }
                                }
                            }

                            std::vector<std::pair<int32_t, float>> edge_overlaps;
                            for (const auto& info : overlap_it->second) {
                                float olap_p = info.second;
                                // Add a small epsilon for floating point inaccuracies
                                if (olap_p < min_p - 0.001f || olap_p > max_p + 0.001f) {
                                    continue;
                                }

                                int32_t olap_idx = info.first;
                                // Exclude the current target and the origin from being considered an "overlap"
                                if (olap_idx != found_index && olap_idx != location_index) {
                                    edge_overlaps.push_back(info);
                                }
                            }

                            // Sort overlaps on this edge by percent_along (order of traversal)
                            std::sort(edge_overlaps.begin(), edge_overlaps.end(), [](const auto& a, const auto& b) {
                                return a.second < b.second;
                            });

                            for (const auto& info : edge_overlaps) {
                                if (std::find(overlaps.begin(), overlaps.end(), info.first) == overlaps.end()) {
                                    overlaps.push_back(info.first);
                                }
                            }
                        }
                    }

                    auto route_geometry = TraceShape(predindex, graphreader, dest_p, *origin_loc);

                    found_locations.push_back({
                        found_index,
                        actual_distance,
                        actual_secs,
                        actual_cost,
                        std::move(route_geometry),
                        std::move(overlaps)
                    });
                }
            }
        }

        // Check our stopping conditions
        cb_decision = ShouldExpand(graphreader, pred, expansion_direction);
        if (cb_decision != ExpansionRecommendation::prune_expansion) {
            ExpandInner<ExpansionType::forward>(graphreader, pred.endnode(), pred, predindex,
                                                opp_pred_edge, false, time_infos.front());
        }
    }

    // FINAL SORT: Ensure strictly sorted by cost before returning
    std::sort(found_locations.begin(), found_locations.end(), 
        [](const ProximityResult& a, const ProximityResult& b) {
            return a.cost < b.cost; 
        });

    return found_locations;
}

std::vector<midgard::PointLL> Proximity::TraceShape(uint32_t predindex, baldr::GraphReader& graphreader, float dest_p, const valhalla::Location& origin_loc) {
    std::vector<uint32_t> path_indices;
    uint32_t current = predindex;
    
    // 1. Trace backwards from the destination to the origin
    while (current != valhalla::baldr::kInvalidLabel) {
        path_indices.push_back(current);
        current = bdedgelabels_[current].predecessor();
    }
    
    // 2. Reverse the list so it flows logically from Origin -> Destination
    std::reverse(path_indices.begin(), path_indices.end());
    
    // 3. Build the continuous physical shape
    std::vector<midgard::PointLL> full_shape;
    
    for (size_t i = 0; i < path_indices.size(); i++) {
        uint32_t idx = path_indices[i];
        const auto& label = bdedgelabels_[idx];
        auto edge_id = label.edgeid();
        
        if (!edge_id.Is_Valid()) continue;
        
        auto tile = graphreader.GetGraphTile(edge_id.Tile_Base());
        if (!tile) continue;
        
        const auto* edge = tile->directededge(edge_id);
        auto edge_info = tile->edgeinfo(edge);
        const auto& shape = edge_info.shape();
        
        float s_p = 0.0f;
        float e_p = 1.0f;
        
        if (i == 0) {
            // Find origin percent_along
            for (int e_idx = 0; e_idx < origin_loc.correlation().edges_size(); ++e_idx) {
                const auto& e = origin_loc.correlation().edges(e_idx);
                if (valhalla::baldr::GraphId(e.graph_id()) == edge_id) {
                    s_p = e.percent_along();
                    break;
                }
            }
        }
        
        if (i == path_indices.size() - 1) {
            e_p = dest_p;
        }

        std::vector<midgard::PointLL> edge_shape;
        if (edge->forward()) {
            edge_shape = valhalla::midgard::trim_polyline(shape.begin(), shape.end(), s_p, e_p);
        } else {
            // Reversing the shape because edge is backward
            std::vector<midgard::PointLL> reversed_shape(shape.rbegin(), shape.rend());
            edge_shape = valhalla::midgard::trim_polyline(reversed_shape.begin(), reversed_shape.end(), s_p, e_p);
        }

        auto start_it = (full_shape.empty() || edge_shape.empty()) ? edge_shape.begin() : edge_shape.begin() + 1;
        if (start_it < edge_shape.end()) {
            full_shape.insert(full_shape.end(), start_it, edge_shape.end());
        }
    }
    
    return full_shape;
}

ExpansionRecommendation Proximity::ShouldExpand(baldr::GraphReader& /*graphreader*/,
                                                const sif::EdgeLabel& pred,
                                                const ExpansionType route_type) {
    // 1. Did we hit our target quota? Stop everything.
    if (found_locations.size() >= max_search_locations)
        return ExpansionRecommendation::stop_expansion;

    // 2. Have we traveled too far down this specific path? Prune it.
    uint32_t dist = pred.predecessor() == kInvalidLabel 
                    ? 0 
                    : bdedgelabels_[pred.predecessor()].path_distance();

    if (dist > max_meters_) {
        return ExpansionRecommendation::prune_expansion;
    }

    return ExpansionRecommendation::continue_expansion;
}

// We MUST override this to be empty. The base Dijkstras class calls this expecting 
// us to draw isochrone shapes. We no longer do that, so we intentionally do nothing here.
void Proximity::ExpandingNode(baldr::GraphReader& graphreader,
                              graph_tile_ptr tile,
                              const baldr::NodeInfo* node,
                              const sif::EdgeLabel& current,
                              const sif::EdgeLabel* previous) {
    
}

void Proximity::GetExpansionHints(uint32_t& bucket_count, uint32_t& edge_label_reservation) const {
    bucket_count = 20000;
    edge_label_reservation = kInitialEdgeLabelCountDijkstras;
}

} // namespace thor
} // namespace valhalla