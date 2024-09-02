#include "baldr/json.h"
#include "proto_conversions.h"
//#include "thor/matrix_common.h"
#include "tyr/serializers.h"

#include <cstdint>

using namespace valhalla;
using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::thor;

namespace {

//json::ArrayPtr
//serialize_duration(const valhalla::Matrix& matrix, size_t start_td, const size_t td_count) {
//    auto time = json::array({});
//    for (size_t i = start_td; i < start_td + td_count; ++i) {
//	// check to make sure a route was found; if not, return null for time in matrix result
//	if (matrix.times()[i] != kMaxCost) {
//	    time->emplace_back(static_cast<uint64_t>(matrix.times()[i]));
//	} else {
//	    time->emplace_back(static_cast<std::nullptr_t>(nullptr));
//	}
//    }
//    return time;
//}

//json::ArrayPtr serialize_distance(const valhalla::Matrix& matrix,
//				  size_t start_td,
//				  const size_t td_count,
//				  const size_t /* source_index */,
//				  const size_t /* target_index */,
//				  double distance_scale) {
//    auto distance = json::array({});
//    for (size_t i = start_td; i < start_td + td_count; ++i) {
//	// check to make sure a route was found; if not, return null for distance in matrix result
//	if (matrix.times()[i] != kMaxCost) {
//	    distance->emplace_back(json::fixed_t{matrix.distances()[i] * distance_scale, 3});
//	} else {
//	    distance->emplace_back(static_cast<std::nullptr_t>(nullptr));
//	}
//    }
//    return distance;
//}
} // namespace

namespace osrm_serializers {

// Serialize route response in OSRM compatible format.
//std::string serialize(const Api& request) {
//    auto json = json::map({});
//    auto time = json::array({});
//    auto distance = json::array({});
//    const auto& options = request.options();
//
//    // If here then the matrix succeeded. Set status code to OK and serialize
//    // waypoints (locations).
//    json->emplace("code", std::string("Ok"));
//    json->emplace("sources", osrm::waypoints(options.sources()));
//    json->emplace("destinations", osrm::waypoints(options.targets()));
//
//    for (int source_index = 0; source_index < options.sources_size(); ++source_index) {
//	time->emplace_back(serialize_duration(request.matrix(), source_index * options.targets_size(),
//					      options.targets_size()));
//	distance->emplace_back(serialize_distance(request.matrix(),
//						  source_index * options.targets_size(),
//						  options.targets_size(), source_index, 0, 1.0));
//    }
//    json->emplace("durations", time);
//    json->emplace("distances", distance);
//    json->emplace("algorithm", MatrixAlgoToString(request.matrix().algorithm()));
//
//    std::stringstream ss;
//    ss << *json;
//    return ss.str();
//}
} // namespace osrm_serializers

/*
valhalla output looks like this:

*/
json::ArrayPtr locations_to_array(const google::protobuf::RepeatedPtrField<valhalla::Location>& locs) {
    auto input_locs = json::array({});
    for (const auto& location : locs) {
	if (location.correlation().edges().size() == 0) {
	    input_locs->emplace_back(nullptr);
	} else {
	    auto& corr_ll = location.correlation().edges(0).ll();
	    input_locs->emplace_back(json::map({{"lat", json::fixed_t{corr_ll.lat(), 6}},
						{"lon", json::fixed_t{corr_ll.lng(), 6}}}));
	}
    }
    return input_locs;
}

std::string serialize_all_to_all_json(const Api& request, double distance_scale) {
    auto json = json::map({});
    const auto& options = request.options();

    if (options.verbose()) {
	    json::ArrayPtr locs = locations_to_array(options.locations());
	    json->emplace("locations", locs);
    } // slim it down

    const AllToAll& all_to_all = request.all_to_all();
    auto map = json::map({});

    std::unordered_map<uint32_t, std::unordered_map<uint32_t, int>> data_map;

    for (int i = 0; i < all_to_all.from_indices_size(); ++i) {
        uint32_t from = all_to_all.from_indices(i);
        uint32_t to = all_to_all.to_indices(i);
        data_map[from][to] = i;
        data_map[to][from] = i;
    }

    for (auto& it : data_map) {
        auto map2 = json::map({});
        for (auto& it2 : it.second) {
            
            auto i = it2.second;
            float distance = all_to_all.distances(i);
            float time = all_to_all.times(i);

            auto to = it2.first;

            auto values = json::map({});
            values->emplace("distance", json::fixed_t{distance * distance_scale, 10});
            values->emplace("time", json::fixed_t{ time, 10 });
            map2->emplace(std::to_string(to), values);
        }

        map->emplace(std::to_string(it.first), map2);
    }

    json->emplace("map", map);
  
    json->emplace("units", Options_Units_Enum_Name(options.units()));

    if (options.has_id_case()) {
	    json->emplace("id", options.id());
    }

    // add warnings to json response
    if (request.info().warnings_size() >= 1) {
	    json->emplace("warnings", valhalla::tyr::serializeWarnings(request));
    }

    std::stringstream ss;
    ss << *json;
    return ss.str();
}

namespace valhalla {
namespace tyr {

std::string serialize_all_to_all(Api& request) {
    double distance_scale =
	(request.options().units() == Options::miles) ? kMilePerMeter : kKmPerMeter;
    switch (request.options().format()) {
	case Options_Format_osrm:
        return "";
	    //return osrm_serializers::serialize(request);
	case Options_Format_json:
	    return serialize_all_to_all_json(request, distance_scale);
	case Options_Format_pbf:
	    return serializePbf(request);
	default:
	    throw;
    }
}

} // namespace tyr
} // namespace valhalla
