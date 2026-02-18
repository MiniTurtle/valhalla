#include "loki/worker.h"
#include "tyr/serializers.h"

namespace valhalla {
namespace loki {

std::string loki_worker_t::query(Api& request) {
  // time this whole method and save that statistic
  auto _ = measure_scope_time(request);

  if (request.options().query_edge_id().size() < 1) {
    throw valhalla_exception_t{116};
  }

  return tyr::serializeQuery(request, *reader);
}

} // namespace loki
} // namespace valhalla
