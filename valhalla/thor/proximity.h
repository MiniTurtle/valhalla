#ifndef VALHALLA_THOR_PROXIMITY_H_
#define VALHALLA_THOR_PROXIMITY_H_

#include <valhalla/baldr/double_bucket_queue.h>
#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/graphreader.h>
#include <valhalla/baldr/location.h>
#include <valhalla/midgard/gridded_data.h>
#include <valhalla/proto/common.pb.h>
#include <valhalla/sif/dynamiccost.h>
#include <valhalla/sif/edgelabel.h>
#include <valhalla/thor/dijkstras.h>
#include <valhalla/thor/edgestatus.h>

#include <cstdint>
#include <memory>
#include <unordered_map>
#include <vector>

namespace valhalla {
namespace thor {

/**
 * Algorithm to generate an isochrone as a lat,lon grid with time taken to
 * each each grid point. This gridded data can then be contoured to create
 * isolines or contours.
 */
class Proximity : public Dijkstras {
public:
    struct ProximityResult {
        int32_t location_index;
        float distance_meters;
        float time_seconds;
        float cost;
        std::vector<midgard::PointLL> shape;
        std::vector<int32_t> overlapping_locations;
    };
public:
  /**
   * Constructor.
   * @param config A config object of key, value pairs
   */
  explicit Proximity(const boost::property_tree::ptree& config = {});

  /**
   * Destructor
   */
  virtual ~Proximity() {
  }

  /**
   * Compute an isochrone grid. This creates and populates a lat,lon grid with
   * time taken to reach each grid point. This gridded data is then contoured
   * so it can be output as polygons. Multiple locations are allowed as the
   * origins - within some reasonable distance from each other.
   *
   * @param expansion_type  Which type of expansion to do, forward/reverse/mulitmodal
   * @param api             The request response containing the locations to seed the expansion
   * @param reader          Graph reader to provide access to graph primitives
   * @param costings        Per mode costing objects
   * @param mode            The mode specifying which costing to use
   * @return                The 2d grid each marked with the minimum time to reach it
   */
  std::vector<ProximityResult> FindProximity(const ExpansionType& expansion_type,
                                    valhalla::Api& api,
                                    baldr::GraphReader& reader,
                                    const sif::mode_costing_t& costings,
                                    const sif::TravelMode mode,
                                    uint32_t max_search_locations,
                                    int32_t location_index);

  /**
   * Set the child's expansion callback which will be swapped in and out
   * if the requirements are met.
   *
   * @param callback the functor to call back when the Dijkstra makes progress
   *                             on a given edge
   */
  void SetInnerExpansionCallback(expansion_callback_t&& callback) {
    inner_expansion_callback_ = std::move(callback);
  }

protected:
  // when we expand up to a node we color the cells of the grid that the edge that ends at the
  // node touches
  virtual void ExpandingNode(baldr::GraphReader& graphreader,
                             graph_tile_ptr tile,
                             const baldr::NodeInfo* node,
                             const sif::EdgeLabel& current,
                             const sif::EdgeLabel* previous) override;

  // when the main loop is looking to continue expanding we tell it to terminate here
  virtual ExpansionRecommendation ShouldExpand(baldr::GraphReader& graphreader,
                                               const sif::EdgeLabel& pred,
                                               const ExpansionType route_type) override;

  // tell the expansion how many labels to expect and how many buckets to use
  virtual void GetExpansionHints(uint32_t& bucket_count,
                                 uint32_t& edge_label_reservation) const override;

  void CalculateMaxDistance(const valhalla::Api& api);

  std::vector<midgard::PointLL> TraceShape(uint32_t predindex, baldr::GraphReader& graphreader);

  size_t max_search_locations = 3;
  int32_t location_index = -1;
  std::unordered_map<baldr::GraphId, std::vector<std::pair<int32_t, float>>> edge_id_to_location_index;
  std::vector<ProximityResult> found_locations;
  float max_meters_ = 0;
  expansion_callback_t inner_expansion_callback_;
};

} // namespace thor
} // namespace valhalla

#endif // VALHALLA_THOR_PROXIMITY_H_
