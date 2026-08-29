#include "thor/worker.h"
#include "tyr/serializers.h"

using namespace valhalla;
using namespace valhalla::thor;

namespace valhalla {
namespace thor {

std::string thor_worker_t::group_locations(Api& request) {
    // does nothing
    
    // just serialize the request that loki populated
    return tyr::serialize_group_locations(request);
}

} // namespace thor
} // namespace valhalla
