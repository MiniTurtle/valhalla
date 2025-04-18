#include "proto_conversions.h"
#include "midgard/logging.h"

using namespace valhalla;

const std::string empty_str;

namespace valhalla {
const std::string& MatrixAlgoToString(const valhalla::Matrix::Algorithm algo) {
  static const std::unordered_map<valhalla::Matrix::Algorithm, const std::string> algos{
      {valhalla::Matrix::CostMatrix, "costmatrix"},
      {valhalla::Matrix::TimeDistanceMatrix, "timedistancematrix"},
      {valhalla::Matrix::TimeDistanceBSSMatrix, "timedistancbssematrix"},
  };
  auto i = algos.find(algo);
  return i == algos.cend() ? empty_str : i->second;
};

std::string incidentTypeToString(const valhalla::IncidentsTile::Metadata::Type& incident_type) {
  switch (incident_type) {
    case valhalla::IncidentsTile::Metadata::ACCIDENT:
      return "accident";
      break;
    case valhalla::IncidentsTile::Metadata::CONGESTION:
      return "congestion";
      break;
    case valhalla::IncidentsTile::Metadata::CONSTRUCTION:
      return "construction";
      break;
    case valhalla::IncidentsTile::Metadata::DISABLED_VEHICLE:
      return "disabled_vehicle";
      break;
    case valhalla::IncidentsTile::Metadata::LANE_RESTRICTION:
      return "lane_restriction";
      break;
    case valhalla::IncidentsTile::Metadata::MASS_TRANSIT:
      return "mass_transit";
      break;
    case valhalla::IncidentsTile::Metadata::MISCELLANEOUS:
      return "miscellaneous";
      break;
    case valhalla::IncidentsTile::Metadata::OTHER_NEWS:
      return "other_news";
      break;
    case valhalla::IncidentsTile::Metadata::PLANNED_EVENT:
      return "planned_event";
      break;
    case valhalla::IncidentsTile::Metadata::ROAD_CLOSURE:
      return "road_closure";
      break;
    case valhalla::IncidentsTile::Metadata::ROAD_HAZARD:
      return "road_hazard";
      break;
    case valhalla::IncidentsTile::Metadata::WEATHER:
      return "weather";
      break;
    case valhalla::
        IncidentsTile_Metadata_Type_IncidentsTile_Metadata_Type_INT_MAX_SENTINEL_DO_NOT_USE_:
    case valhalla::
        IncidentsTile_Metadata_Type_IncidentsTile_Metadata_Type_INT_MIN_SENTINEL_DO_NOT_USE_:
      // Like the name says, do not use. Simply for ensuring full coverage of switch statement
      break;
  };
  throw std::runtime_error("Unhandled case in incidentTypeToString: " +
                           std::to_string(incident_type));
}

// Get the string representing the incident-Impact
const char* incidentImpactToString(const valhalla::IncidentsTile::Metadata::Impact& impact) {
  switch (impact) {
    case valhalla::IncidentsTile::Metadata::UNKNOWN:
      return "unknown";
      break;
    case valhalla::IncidentsTile::Metadata::CRITICAL:
      return "critical";
      break;
    case valhalla::IncidentsTile::Metadata::MAJOR:
      return "major";
      break;
    case valhalla::IncidentsTile::Metadata::MINOR:
      return "minor";
      break;
    case valhalla::IncidentsTile::Metadata::LOW:
      return "low";
      break;
    case valhalla::
        IncidentsTile_Metadata_Impact_IncidentsTile_Metadata_Impact_INT_MAX_SENTINEL_DO_NOT_USE_:
    case valhalla::
        IncidentsTile_Metadata_Impact_IncidentsTile_Metadata_Impact_INT_MIN_SENTINEL_DO_NOT_USE_:
      // Like the name says, do not use. Simply for ensuring full coverage of switch statement
      break;
  }
  // TODO Throw or warn here? Assert maybe to only crash debug build
  LOG_WARN("Unhandled case in incidentCriticalityToString: " + std::to_string(impact));
  return "UNHANDLED_CASE";
}

const std::string& GuidanceViewTypeToString(const valhalla::DirectionsLeg_GuidanceView_Type type) {
  static const std::unordered_map<int, std::string>
      types{{DirectionsLeg_GuidanceView_Type_kJunction, "jct"},
            {DirectionsLeg_GuidanceView_Type_kSapa, "sapa"},
            {DirectionsLeg_GuidanceView_Type_kTollbranch, "tollbranch"},
            {DirectionsLeg_GuidanceView_Type_kAftertoll, "aftertoll"},
            {DirectionsLeg_GuidanceView_Type_kEnt, "ent"},
            {DirectionsLeg_GuidanceView_Type_kExit, "exit"},
            {DirectionsLeg_GuidanceView_Type_kCityreal, "cityreal"},
            {DirectionsLeg_GuidanceView_Type_kDirectionboard, "directionboard"},
            {DirectionsLeg_GuidanceView_Type_kSignboard, "signboard"}};
  auto i = types.find(type);
  return i == types.cend() ? empty_str : i->second;
}

bool Options_Action_Enum_Parse(const std::string& action, Options::Action* a) {
  static const std::unordered_map<std::string, Options::Action> actions{
      {"route", Options::route},
      {"locate", Options::locate},
      {"sources_to_targets", Options::sources_to_targets},
      {"all_to_all", Options::all_to_all},
      {"optimized_route", Options::optimized_route},
      {"isochrone", Options::isochrone},
      {"trace_route", Options::trace_route},
      {"trace_attributes", Options::trace_attributes},
      {"height", Options::height},
      {"transit_available", Options::transit_available},
      {"expansion", Options::expansion},
      {"centroid", Options::centroid},
      {"status", Options::status},
  };
  auto i = actions.find(action);
  if (i == actions.cend())
    return false;
  *a = i->second;
  return true;
}

bool Options_ExpansionAction_Enum_Parse(const std::string& action, Options::Action* a) {
  static const std::unordered_map<std::string, Options::Action>
      actions{{"route", Options::route},
              {"isochrone", Options::isochrone},
              {"sources_to_targets", Options::sources_to_targets}};
  auto i = actions.find(action);
  if (i == actions.cend())
    return false;
  *a = i->second;
  return true;
}

const std::string& Options_Action_Enum_Name(const Options::Action action) {
  static const std::unordered_map<int, std::string> actions{
      {Options::route, "route"},
      {Options::locate, "locate"},
      {Options::sources_to_targets, "sources_to_targets"},
      {Options::optimized_route, "optimized_route"},
      {Options::isochrone, "isochrone"},
      {Options::trace_route, "trace_route"},
      {Options::trace_attributes, "trace_attributes"},
      {Options::height, "height"},
      {Options::transit_available, "transit_available"},
      {Options::expansion, "expansion"},
      {Options::centroid, "centroid"},
      {Options::status, "status"},
      {Options::all_to_all, "all_to_all"},
  };
  auto i = actions.find(action);
  return i == actions.cend() ? empty_str : i->second;
}

bool Location_Type_Enum_Parse(const std::string& type, Location::Type* t) {
  static const std::unordered_map<std::string, Location::Type> types{
      {"break", Location::kBreak},
      {"through", Location::kThrough},
      {"break_through", Location::kBreakThrough},
      {"via", Location::kVia},
  };
  auto i = types.find(type);
  if (i == types.cend())
    return false;
  *t = i->second;
  return true;
}
const std::string& Location_Type_Enum_Name(const Location::Type type) {
  static const std::unordered_map<int, std::string> types{
      {Location::kBreak, "break"},
      {Location::kThrough, "through"},
      {Location::kBreakThrough, "break_through"},
      {Location::kVia, "via"},
  };
  auto i = types.find(type);
  return i == types.cend() ? empty_str : i->second;
}

const std::string& Location_SideOfStreet_Enum_Name(const Location::SideOfStreet side) {
  static const std::unordered_map<int, std::string> sides{
      {Location::kLeft, "left"},
      {Location::kRight, "right"},
      {Location::kNone, "none"},
  };
  auto i = sides.find(side);
  return i == sides.cend() ? empty_str : i->second;
}

bool Costing_Enum_Parse(const std::string& costing, Costing::Type* c) {
  static const std::unordered_map<std::string, Costing::Type> costings{
      {"auto", Costing::auto_},
      // auto_shorter is deprecated
      {"bicycle", Costing::bicycle},
      {"bus", Costing::bus},
      {"taxi", Costing::taxi},
      {"motor_scooter", Costing::motor_scooter},
      {"multimodal", Costing::multimodal},
      {"pedestrian", Costing::pedestrian},
      {"transit", Costing::transit},
      {"truck", Costing::truck},
      {"motorcycle", Costing::motorcycle},
      // auto_data_fix is deprecated
      {"none", Costing::none_},
      {"", Costing::none_},
      {"bikeshare", Costing::bikeshare},
  };
  auto i = costings.find(costing);
  if (i == costings.cend())
    return false;
  *c = i->second;
  return true;
}

const std::string& Costing_Enum_Name(const Costing::Type costing) {
  static const std::unordered_map<int, std::string> costings{
      {Costing::auto_, "auto"},
      // auto_shorter is deprecated
      {Costing::bicycle, "bicycle"},
      {Costing::bus, "bus"},
      {Costing::taxi, "taxi"},
      {Costing::motor_scooter, "motor_scooter"},
      {Costing::multimodal, "multimodal"},
      {Costing::pedestrian, "pedestrian"},
      {Costing::transit, "transit"},
      {Costing::truck, "truck"},
      {Costing::motorcycle, "motorcycle"},
      // auto_data_fix is deprecated
      {Costing::none_, "none"},
      {Costing::bikeshare, "bikeshare"},
  };
  auto i = costings.find(costing);
  return i == costings.cend() ? empty_str : i->second;
}

bool ShapeMatch_Enum_Parse(const std::string& match, ShapeMatch* s) {
  static const std::unordered_map<std::string, ShapeMatch> matches{
      {"edge_walk", ShapeMatch::edge_walk},
      {"map_snap", ShapeMatch::map_snap},
      {"walk_or_snap", ShapeMatch::walk_or_snap},
  };
  auto i = matches.find(match);
  if (i == matches.cend())
    return false;
  *s = i->second;
  return true;
}

const std::string& ShapeMatch_Enum_Name(const ShapeMatch match) {
  static const std::unordered_map<int, std::string> matches{
      {ShapeMatch::edge_walk, "edge_walk"},
      {ShapeMatch::map_snap, "map_snap"},
      {ShapeMatch::walk_or_snap, "walk_or_snap"},
  };
  auto i = matches.find(match);
  return i == matches.cend() ? empty_str : i->second;
}

bool Options_Format_Enum_Parse(const std::string& format, Options::Format* f) {
  static const std::unordered_map<std::string, Options::Format> formats{
      {"json", Options::json}, {"gpx", Options::gpx},         {"osrm", Options::osrm},
      {"pbf", Options::pbf},   {"geotiff", Options::geotiff},
  };
  auto i = formats.find(format);
  if (i == formats.cend())
    return false;
  *f = i->second;
  return true;
}

const std::string& Options_Format_Enum_Name(const Options::Format match) {
  static const std::unordered_map<int, std::string> formats{
      {Options::json, "json"}, {Options::gpx, "gpx"},         {Options::osrm, "osrm"},
      {Options::pbf, "pbf"},   {Options::geotiff, "geotiff"},
  };
  auto i = formats.find(match);
  return i == formats.cend() ? empty_str : i->second;
}

const std::string& Options_Units_Enum_Name(const Options::Units unit) {
  static const std::unordered_map<int, std::string> units{
      {Options::kilometers, "kilometers"},
      {Options::miles, "miles"},
  };
  auto i = units.find(unit);
  return i == units.cend() ? empty_str : i->second;
}

bool FilterAction_Enum_Parse(const std::string& action, FilterAction* a) {
  static const std::unordered_map<std::string, FilterAction> actions{
      {"exclude", FilterAction::exclude},
      {"include", FilterAction::include},
  };
  auto i = actions.find(action);
  if (i == actions.cend())
    return false;
  *a = i->second;
  return true;
}

const std::string& FilterAction_Enum_Name(const FilterAction action) {
  static const std::unordered_map<int, std::string> actions{
      {FilterAction::exclude, "exclude"},
      {FilterAction::include, "include"},
  };
  auto i = actions.find(action);
  return i == actions.cend() ? empty_str : i->second;
}

bool DirectionsType_Enum_Parse(const std::string& dtype, DirectionsType* t) {
  static const std::unordered_map<std::string, DirectionsType> types{
      {"none", DirectionsType::none},
      {"maneuvers", DirectionsType::maneuvers},
      {"instructions", DirectionsType::instructions},
  };
  auto i = types.find(dtype);
  if (i == types.cend())
    return false;
  *t = i->second;
  return true;
}

bool PreferredSide_Enum_Parse(const std::string& pside, valhalla::Location::PreferredSide* p) {
  static const std::unordered_map<std::string, valhalla::Location::PreferredSide> types{
      {"either", valhalla::Location::either},
      {"same", valhalla::Location::same},
      {"opposite", valhalla::Location::opposite},
  };
  auto i = types.find(pside);
  if (i == types.cend())
    return false;
  *p = i->second;
  return true;
}

bool RoadClass_Enum_Parse(const std::string& rc_name, valhalla::RoadClass* rc) {
  static const std::unordered_map<std::string, valhalla::RoadClass> types{
      {"motorway", valhalla::RoadClass::kMotorway},
      {"trunk", valhalla::RoadClass::kTrunk},
      {"primary", valhalla::RoadClass::kPrimary},
      {"secondary", valhalla::RoadClass::kSecondary},
      {"tertiary", valhalla::RoadClass::kTertiary},
      {"unclassified", valhalla::RoadClass::kUnclassified},
      {"residential", valhalla::RoadClass::kResidential},
      {"service_other", valhalla::RoadClass::kServiceOther},
  };
  auto i = types.find(rc_name);
  if (i == types.cend())
    return false;
  *rc = i->second;
  return true;
}

bool Options_ExpansionProperties_Enum_Parse(const std::string& prop,
                                            Options::ExpansionProperties* a) {
  static const std::unordered_map<std::string, Options::ExpansionProperties>
      actions{{"cost", Options_ExpansionProperties_cost},
              {"duration", Options_ExpansionProperties_duration},
              {"distance", Options_ExpansionProperties_distance},
              {"edge_status", Options_ExpansionProperties_edge_status},
              {"edge_id", Options::ExpansionProperties::Options_ExpansionProperties_edge_id},
              {"pred_edge_id", Options_ExpansionProperties_pred_edge_id},
              {"expansion_type", Options_ExpansionProperties_expansion_type}};
  auto i = actions.find(prop);
  if (i == actions.cend())
    return false;
  *a = i->second;
  return true;
}

const std::unordered_map<int, std::string> vehicle_to_string{
    {static_cast<int>(VehicleType::kCar), "car"},
    {static_cast<int>(VehicleType::kMotorcycle), "motorcycle"},
    {static_cast<int>(VehicleType::kAutoBus), "bus"},
    {static_cast<int>(VehicleType::kTruck), "truck"},
    {static_cast<int>(VehicleType::kMotorScooter), "motor_scooter"},
};

const std::unordered_map<int, std::string> pedestrian_to_string{
    {static_cast<int>(PedestrianType::kFoot), "foot"},
    {static_cast<int>(PedestrianType::kWheelchair), "wheelchair"},
};

const std::unordered_map<int, std::string> bicycle_to_string{
    {static_cast<int>(BicycleType::kRoad), "road"},
    {static_cast<int>(BicycleType::kCross), "cross"},
    {static_cast<int>(BicycleType::kHybrid), "hybrid"},
    {static_cast<int>(BicycleType::kMountain), "mountain"},
};

const std::unordered_map<int, std::string> transit_to_string{
    {static_cast<int>(TransitType::kTram), "tram"},
    {static_cast<int>(TransitType::kMetro), "metro"},
    {static_cast<int>(TransitType::kRail), "rail"},
    {static_cast<int>(TransitType::kBus), "bus"},
    {static_cast<int>(TransitType::kFerry), "ferry"},
    {static_cast<int>(TransitType::kCableCar), "cable_car"},
    {static_cast<int>(TransitType::kGondola), "gondola"},
    {static_cast<int>(TransitType::kFunicular), "funicular"},
};

std::pair<std::string, std::string>
travel_mode_type(const valhalla::DirectionsLeg_Maneuver& maneuver) {
  switch (maneuver.travel_mode()) {
    case TravelMode::kDrive: {
      auto i = vehicle_to_string.find(maneuver.vehicle_type());
      return vehicle_to_string.find(maneuver.vehicle_type()) == vehicle_to_string.cend()
                 ? std::make_pair("drive", "car")
                 : std::make_pair("drive", i->second);
    }
    case TravelMode::kPedestrian: {
      auto i = pedestrian_to_string.find(maneuver.pedestrian_type());
      return i == pedestrian_to_string.cend() ? std::make_pair("pedestrian", "foot")
                                              : std::make_pair("pedestrian", i->second);
    }
    case TravelMode::kBicycle: {
      auto i = bicycle_to_string.find(maneuver.bicycle_type());
      return i == bicycle_to_string.cend() ? std::make_pair("bicycle", "road")
                                           : std::make_pair("bicycle", i->second);
    }
    case TravelMode::kTransit: {
      auto i = transit_to_string.find(maneuver.transit_type());
      return i == transit_to_string.cend() ? std::make_pair("transit", "rail")
                                           : std::make_pair("transit", i->second);
    }
    default:
      throw std::logic_error("Unknown travel mode");
  }
}

const std::string& Expansion_EdgeStatus_Enum_Name(const Expansion_EdgeStatus status) {
  static const std::unordered_map<int, std::string> statuses{
      {Expansion_EdgeStatus_reached, "r"},
      {Expansion_EdgeStatus_settled, "s"},
      {Expansion_EdgeStatus_connected, "c"},
  };
  auto i = statuses.find(status);
  return i == statuses.cend() ? empty_str : i->second;
}
} // namespace valhalla
