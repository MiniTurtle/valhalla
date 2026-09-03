#include "loki/search.h"
#include "loki/worker.h"

using namespace valhalla;
using namespace valhalla::baldr;

namespace {

void check_distance(const google::protobuf::RepeatedPtrField<valhalla::Location>& locations,
                    float max_iso_distance) {
  // see if any locations pairs are unreachable or too far apart
  for (auto source = locations.begin(); source != locations.end() - 1; ++source) {
    for (auto target = source + 1; target != locations.end(); ++target) {
      // check if distance between latlngs exceed max distance limit
      auto path_distance = to_ll(*source).Distance(to_ll(*target));
      if (path_distance > max_iso_distance) {
        throw valhalla_exception_t{154,
                                   std::to_string(static_cast<size_t>(max_iso_distance)) + " meters"};
      };
    }
  }
}

} // namespace

namespace valhalla {
namespace loki {

void loki_worker_t::init_proximity(Api& request) {
  auto& options = *request.mutable_options();

  // strip off unused information
  parse_locations(options.mutable_locations(), request);
  if (options.locations_size() < 1) {
    throw valhalla_exception_t{120};
  };
  for (auto& l : *options.mutable_locations()) {
    l.clear_heading();
  }

  parse_costing(request);
}

void loki_worker_t::proximity(Api& request) {
  // time this whole method and save that statistic
  auto _ = measure_scope_time(request);

  init_proximity(request);
  auto& options = *request.mutable_options();
  // check that location size does not exceed max.
  // Comes configs(valhalla.json)
  if (options.locations_size() < 2) {
    throw valhalla_exception_t{150, "Requires at least 2 locations"};
  };
  auto max_loc_it = max_locations.find("proximity");
  int max_locs = max_loc_it != max_locations.end() ? max_loc_it->second : 400; // default to 20 if missing
  if (options.locations_size() > max_locs) {
    throw valhalla_exception_t{150, std::to_string(max_locs)};
  };

  if (options.max_search_locations() < 1)
      throw valhalla_exception_t{150, "Requires at least 1 max_search_locations"}; 

  // check the distances
  auto max_dist_it = max_distance.find("proximity");
  float max_dist = max_dist_it != max_distance.end() ? max_dist_it->second : 100000.0f; // default to 100km if missing
  check_distance(options.locations(), max_dist);

  try {
    // correlate the various locations to the underlying graph
    auto locations = PathLocation::fromPBF(options.locations());
    const auto projections = loki::Search(locations, *reader, costing);
    for (size_t i = 0; i < locations.size(); ++i) {
      const auto& projection = projections.at(locations[i]);
      PathLocation::toPBF(projection, options.mutable_locations(i), *reader);
    }
  } catch (const std::exception&) { throw valhalla_exception_t{171}; }
}

} // namespace loki
} // namespace valhalla
