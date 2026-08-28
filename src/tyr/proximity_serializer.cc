#include "baldr/json.h"
#include "proto_conversions.h"
//#include "thor/matrix_common.h"
#include "tyr/serializers.h"

#include <cstdint>
#include <sstream>

using namespace valhalla;
using namespace valhalla::midgard;
using namespace valhalla::baldr;


/*
valhalla output looks like this:

*/
namespace {
json::ArrayPtr locations_to_array(const google::protobuf::RepeatedPtrField<valhalla::Location>& locs) {
    auto input_locs = json::array({});
    for (const auto& location : locs) {
	if (location.correlation().edges().size() == 0) {
	    input_locs->emplace_back(nullptr);
	} else {
	    auto& corr_ll = location.correlation().edges(0).ll();
	    input_locs->emplace_back(json::map({
            {"lat", json::fixed_t{corr_ll.lat(), 6}},
		    {"lon", json::fixed_t{corr_ll.lng(), 6}}}
        ));
	}
    }
    return input_locs;
}
}

std::string serialize_proximity_json(const Api& request, double distance_scale) {
    auto json = json::map({});
    const auto& options = request.options();

    if (options.verbose()) {
	    json::ArrayPtr locs = locations_to_array(options.locations());
	    json->emplace("locations", locs);
    } // slim it down

    const Proximity& proximity = request.proximity();
    auto array = json::array({});

    auto to = proximity.to();
    for (auto& p : to) {
        auto shape = json::array({});
        for (auto& s : p.shape())
            shape->emplace_back(json::map({
                {"lat", json::fixed_t{s.lat(), 6}},
		        {"lon", json::fixed_t{s.lng(), 6}}}
            ));

        auto obj = json::map({
	        {"index", static_cast<uint64_t>(p.index())},
	        {"cost", json::fixed_t{p.cost(), 6}},
	        {"distance", json::fixed_t{p.distance(), 6}},
	        {"time", json::fixed_t{p.time(), 6}},
	        {"shape", shape}
        });
        array->emplace_back(obj);
    }
    json->emplace("result", array);

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

std::string serialize_proximity(Api& request) {
    double distance_scale = (request.options().units() == Options::miles) ? kMilePerMeter : kKmPerMeter;

    switch (request.options().format()) {
	case Options_Format_osrm:
        return "";
	    //return osrm_serializers::serialize(request);
	case Options_Format_json:
	    return serialize_proximity_json(request, distance_scale);
	case Options_Format_pbf:
	    return serializePbf(request);
	default:
	    throw;
    }
}

} // namespace tyr
} // namespace valhalla
