#include "config.h"
#include "sif/autocost.h"
#include "sif/bicyclecost.h"
#include "sif/pedestriancost.h"
#include "thor/costmatrix.h"
#include "thor/timedistancebssmatrix.h"
#include "thor/timedistancematrix.h"
#include "thor/unidirectional_astar.h"
#include "thor/worker.h"
#include "tyr/serializers.h"

#include <condition_variable>
#include <thread>

using namespace valhalla;
using namespace valhalla::tyr;
using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::sif;
using namespace valhalla::thor;

namespace valhalla {
namespace thor {

std::string thor_worker_t::proximity(Api& request) {
    // time this whole method and save that statistic
    auto _ = measure_scope_time(request);

    auto& options = *request.mutable_options();
    adjust_scores(options);

    auto max_search_locations = options.max_search_locations();
    if (max_search_locations < 1)
	    max_search_locations = 1;
    else if (max_search_locations > request.mutable_options()->locations_size()-1)
	    max_search_locations = request.mutable_options()->locations_size()-1;

    auto costing = parse_costing(request);

    // If no generalization is requested an optimal factor is computed (based on the isotile grid
    // size).
    if (!options.has_generalize_case()) {
	    options.set_generalize(kOptimalGeneralization);
    }

    // get the raster
    bool reverse = options.reverse() || options.date_time_type() == valhalla::Options::arrive_by;
    auto expansion_type = costing == "multimodal" || costing == "transit"
			      ? ExpansionType::multimodal
			      : (reverse ? ExpansionType::reverse : ExpansionType::forward);
    auto result = proximity_gen.FindProximity(
        expansion_type, 
        request, 
        *reader, 
        mode_costing, 
        mode,
		max_search_locations, 
        0);

    //
    // Fill request data.
    auto proximity = request.mutable_proximity();
    auto to = proximity->mutable_to();
    for (auto& r : result) {
        auto p = to->Add();
        p->set_index(r.location_index);
        p->set_cost(r.cost);
        p->set_time(r.time_seconds);
        p->set_distance(r.distance_meters);
        auto shape = p->mutable_shape();
        for (auto& s : r.shape) {
            auto loc = shape->Add();
            loc->set_lat(s.lat());
            loc->set_lng(s.lng());
        }
    }

    // make the final output (pbf, json or geotiff)
    std::string ret = tyr::serialize_proximity(request);
    return ret;
}
} // namespace thor
} // namespace valhalla
