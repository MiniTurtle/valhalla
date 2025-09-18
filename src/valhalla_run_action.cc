#include "argparse_utils.h"
#include "baldr/attributes_controller.h"
#include "baldr/graphreader.h"
#include "baldr/pathlocation.h"
#include "loki/search.h"
#include "loki/worker.h"
#include "midgard/encoded.h"
#include "midgard/logging.h"
#include "odin/directionsbuilder.h"
#include "odin/enhancedtrippath.h"
#include "proto/api.pb.h"
#include "proto/directions.pb.h"
#include "proto/options.pb.h"
#include "proto/trip.pb.h"
#include "sif/costfactory.h"
#include "thor/bidirectional_astar.h"
#include "thor/multimodal.h"
#include "thor/route_matcher.h"
#include "thor/triplegbuilder.h"
#include "thor/unidirectional_astar.h"
#include "worker.h"

#include <boost/format.hpp>
#include <cxxopts.hpp>

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

using namespace valhalla;

int main(int argc, char* argv[]) {
    const auto program = std::filesystem::path(__FILE__).stem().string();
    // args
    std::string json_str, json_file;
    boost::property_tree::ptree config;
    Options::Action action;

    try {
	    // clang-format off
        cxxopts::Options options(
          program,
          program + " " + VALHALLA_PRINT_VERSION + "\n\n"
          "a command line tool to call an action.\n"
          "Use -a for the action type & Use the -j option for specifying the json file.");

        options.add_options()
          ("h,help", "Print this help message.")
          ("v,version", "Print the version of this software.")
          ("j,json", "JSON Example: "
            "'{\"locations\":[{\"lat\":40.748174,\"lon\":-73.984984,\"type\":\"break\",\"heading\":200,"
            "\"name\":\"Empire State Building\",\"street\":\"350 5th Avenue\",\"city\":\"New "
            "York\",\"state\":\"NY\",\"postal_code\":\"10118-0110\",\"country\":\"US\"},{\"lat\":40."
            "749231,\"lon\":-73.968703,\"type\":\"break\",\"name\":\"United Nations "
            "Headquarters\",\"street\":\"405 East 42nd Street\",\"city\":\"New "
            "York\",\"state\":\"NY\",\"postal_code\":\"10017-3507\",\"country\":\"US\"}],\"costing\":"
            "\"auto\",\"directions_options\":{\"units\":\"miles\"}}'", cxxopts::value<std::string>())
          ("json-file", "File containing the JSON query", cxxopts::value<std::string>())
          ("c,config", "Valhalla configuration file", cxxopts::value<std::string>())
          ("i,inline-config", "Inline JSON config", cxxopts::value<std::string>())
          ("a,action", "Action type: route, locate, sources_to_targets, all_to_all, optimized_route, isochrone, trace_route, trace_attributes, height, transit_available, expansion, centroid & status", cxxopts::value<std::string>());
	    // clang-format on

	    auto result = options.parse(argc, argv);
	    if (!parse_common_args(program, options, result, &config, "mjolnir.logging"))
	        return EXIT_SUCCESS;

	    if (result.count("json-file") && result.count("json")) {
	        LOG_WARN("json and json-file option are set, using json-file content");
	    } else if (result.count("json-file")) {
	        std::ifstream ifs(result["json-file"].as<std::string>());
	        json_str.assign((std::istreambuf_iterator<char>(ifs)),
			        (std::istreambuf_iterator<char>()));
	    } else if (result.count("json")) {
	        json_str = result["json"].as<std::string>();
	    } else {
	        throw cxxopts::exceptions::exception("Either json or json-file args must be set.");
	    }

        if (!result.count("action")) 
            throw cxxopts::exceptions::exception("Action type is missing.");

        static const std::unordered_map<std::string, Options::Action> str_to_option {
	        { "route", Options::route },
	        { "locate", Options::locate },
	        { "sources_to_targets", Options::sources_to_targets },
	        { "all_to_all", Options::all_to_all },
	        { "optimized_route", Options::optimized_route },
	        { "isochrone", Options::isochrone },
	        { "trace_route", Options::trace_route },
	        { "trace_attributes", Options::trace_attributes },
	        { "height", Options::height },
	        { "transit_available", Options::transit_available },
	        { "expansion", Options::expansion },
	        { "centroid", Options::centroid },
	        { "status", Options::status }
        };

        std::string str_action = result["action"].as<std::string>();
        auto action_it = str_to_option.find(str_action);
        if (action_it == str_to_option.end())
            throw cxxopts::exceptions::exception("Action '" + str_action + "' is not an action");

        action = action_it->second;

    } catch (cxxopts::exceptions::exception& e) {
	    std::cerr << e.what() << std::endl;
	    return EXIT_FAILURE;
    } catch (std::exception& e) {
	    std::cerr << "Unable to parse command line options because: " << e.what() << "\n"
		      << "This is a bug, please report it at " PACKAGE_BUGREPORT << std::endl;
	    return EXIT_FAILURE;
    }

    try {
        auto actor = tyr::actor_t::actor_t(config, true);

        valhalla::Api request;
        auto& options = *request.mutable_options();
        options.set_action(action);
        //valhalla::ParseApi(json_str, action, request);

        std::string result = actor.act(request, nullptr, json_str);
        std::cout << "[result_begin]" << result << "[result_end]" << std::endl;
    } catch (std::exception& e) {
	    std::cerr << "Something went wrong with the execution: " << e.what() << std::endl;
	    return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
