#include "baldr/datetime.h"
#include "baldr/rapidjson_utils.h"
#include "baldr/tilehierarchy.h"
#include "loki/search.h"
#include "loki/worker.h"
#include "midgard/logging.h"
#include "tyr/actor.h"

#include <unordered_map>

using namespace valhalla;
using namespace valhalla::tyr;
using namespace valhalla::baldr;
using namespace valhalla::loki;

//namespace {
//
//void check_distance(Api& request,
//		    float matrix_max_distance,
//		    float& max_location_distance,
//		    size_t max_timedep_distance) {
//    auto& options = *request.mutable_options();
//    bool added_warning = false;
//    // see if any locations pairs are unreachable or too far apart
//    for (auto& source : *options.mutable_sources()) {
//	for (auto& target : *options.mutable_targets()) {
//	    // check if distance between latlngs exceed max distance limit
//	    auto path_distance = to_ll(source).Distance(to_ll(target));
//
//	    // only want to log the maximum distance between 2 locations for matrix
//	    if (path_distance >= max_location_distance) {
//		max_location_distance = path_distance;
//	    }
//
//	    if (path_distance > matrix_max_distance) {
//		throw valhalla_exception_t{154,
//					   std::to_string(static_cast<size_t>(matrix_max_distance)) +
//					       " meters"};
//	    };
//
//	    // unset the date_time if beyond the limit
//	    if (static_cast<size_t>(path_distance) > max_timedep_distance) {
//		source.set_date_time("");
//		target.set_date_time("");
//		if (max_timedep_distance && !added_warning) {
//		    add_warning(request, 200);
//		    added_warning = true;
//		}
//	    }
//	}
//    }
//}
//
//} // namespace

namespace valhalla {
namespace loki {

void loki_worker_t::init_all_to_all(Api& request) {
    // we require sources and targets
    auto& options = *request.mutable_options();
    parse_locations(options.mutable_locations(), valhalla_exception_t{112});
    if (options.locations_size() < 2) {
	throw valhalla_exception_t{120};
    };

    // need costing
    parse_costing(request);
}

void loki_worker_t::all_to_all(Api& request) {
    // time this whole method and save that statistic
    auto _ = measure_scope_time(request);

    init_all_to_all(request);
    auto& options = *request.mutable_options();
    const auto& costing_name = Costing_Enum_Name(options.costing_type());

    if (costing_name == "multimodal") {
		throw valhalla_exception_t{140, Options_Action_Enum_Name(options.action())};
    }

	auto connectivity_level = TileHierarchy::levels().back();
    uint32_t connectivity_radius = 0;

    // correlate the various locations to the underlying graph
    std::unordered_map<size_t, size_t> color_counts;
    try {
		auto locations = PathLocation::fromPBF(options.locations(), true);
		const auto projections = loki::Search(locations, *reader, costing);
		for (size_t i = 0; i < locations.size(); ++i) {
			const auto& correlated = projections.at(locations[i]);
			PathLocation::toPBF(correlated, options.mutable_locations(i), *reader);

			if (!connectivity_map) {
				continue;
			}

			auto colors = connectivity_map->get_colors(connectivity_level, correlated, connectivity_radius);
			for (auto color : colors) {
				auto itr = color_counts.find(color);
				if (itr == color_counts.cend()) {
					color_counts[color] = 1;
				} else {
					++itr->second;
				}
			}
		}
    } catch (const std::exception&) { throw valhalla_exception_t{171}; }

    // are all the locations in the same color regions
    if (!connectivity_map) {
		return;
    }
    bool connected = false;
    for (const auto& c : color_counts) {
		if ((int)c.second == options.locations_size()) {
			connected = true;
			break;
		}
    }
    if (!connected) {
		throw valhalla_exception_t{170};
    };
}
} // namespace loki
} // namespace valhalla
