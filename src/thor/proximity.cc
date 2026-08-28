#include "thor/proximity.h"
#include "midgard/distanceapproximator.h"
#include "midgard/logging.h"

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
    for (int32_t index = 0; index < locations.size(); index++) {
        if (index == location_index) continue;

        auto& l = locations[index];
        for (auto& e : l.correlation().edges()) {
            valhalla::baldr::GraphId id(e.graph_id());
            edge_id_to_location_index[id].push_back(index);
        }
    }

    Initialize(bdedgelabels_, adjacencylist_, costing_->UnitSize());

    google::protobuf::RepeatedPtrField<Location> location_origin;
    location_origin.Add()->CopyFrom(locations.Get(location_index));
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
            for (auto found_index : it->second) {
                // Have we already recorded this location?
                auto loc_it = std::find_if(found_locations.begin(), found_locations.end(),
                    [found_index](const ProximityResult& r) { return r.location_index == found_index; });
                
                if (loc_it == found_locations.end()) {
                    // FIRST HIT: Because Dijkstra pops by lowest cost, the first time 
                    // we see this location is mathematically the lowest cost path to it!

                    auto route_geometry = TraceShape(predindex, graphreader);

                    found_locations.push_back({
                        found_index,
                        static_cast<float>(pred.path_distance()),
                        pred.cost().secs,
                        pred.cost().cost,
                        std::move(route_geometry)
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

std::vector<midgard::PointLL> Proximity::TraceShape(uint32_t predindex, baldr::GraphReader& graphreader) {
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
    
    for (uint32_t idx : path_indices) {
        const auto& label = bdedgelabels_[idx];
        auto edge_id = label.edgeid();
        
        if (!edge_id.Is_Valid()) continue;
        
        auto tile = graphreader.GetGraphTile(edge_id.Tile_Base());
        if (!tile) continue;
        
        const auto* edge = tile->directededge(edge_id);
        auto edge_info = tile->edgeinfo(edge);
        const auto& shape = edge_info.shape();
        
        // If the edge was traversed backwards in the graph, we must reverse its shape
        if (edge->forward()) {
            // Prevent coordinate duplication where two edges meet
            auto start_it = (full_shape.empty()) ? shape.begin() : shape.begin() + 1;
            full_shape.insert(full_shape.end(), start_it, shape.end());
        } else {
            auto start_it = (full_shape.empty()) ? shape.rbegin() : shape.rbegin() + 1;
            full_shape.insert(full_shape.end(), start_it, shape.rend());
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