#include "tyr/serializers.h"
#include "baldr/rapidjson_utils.h"
#include "proto/api.pb.h"

#include <rapidjson/document.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>

namespace valhalla {
namespace tyr {

std::string serialize_group_locations(Api& request) {
  rapidjson::StringBuffer buffer;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);

  writer.StartObject();

  writer.Key("group_locations");
  writer.StartArray();

  if (request.has_group_locations()) {
    const auto& group_locations = request.group_locations();
    for (const auto& result : group_locations.results()) {
      writer.StartArray();
      for (const auto& idx : result.location_index()) {
        writer.Uint(idx);
      }
      writer.EndArray();
    }
  }

  writer.EndArray();

  writer.Key("snapped_locations");
  writer.StartArray();
  if (request.has_group_locations()) {
    const auto& group_locations = request.group_locations();
    for (const auto& result : group_locations.results()) {
      writer.StartArray();
      for (const auto& loc : result.locations()) {
        writer.StartObject();
        writer.Key("index");
        writer.Uint(loc.location_index());
        writer.Key("lat");
        writer.Double(loc.snapped_point().lat());
        writer.Key("lon");
        writer.Double(loc.snapped_point().lng());
        writer.EndObject();
      }
      writer.EndArray();
    }
  }
  writer.EndArray();
  writer.EndObject();

  return buffer.GetString();
}

} // namespace tyr
} // namespace valhalla
