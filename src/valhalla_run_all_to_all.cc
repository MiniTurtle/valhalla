#include "argparse_utils.h"
#include "baldr/graphreader.h"
#include "baldr/pathlocation.h"
#include "baldr/rapidjson_utils.h"
#include "config.h"
#include "loki/worker.h"
#include "midgard/logging.h"
#include "odin/directionsbuilder.h"
#include "odin/util.h"
#include "sif/costfactory.h"
#include "thor/bidirectional_astar.h"
#include "thor/worker.h"
#include "worker.h"

#include <boost/format.hpp>
#include <boost/property_tree/ptree.hpp>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cxxopts.hpp>
#include <iostream>
#include <string>
#include <vector>

using namespace valhalla;
using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::loki;
using namespace valhalla::sif;
using namespace valhalla::thor;

// Main method for testing time and distance methods
int main(int argc, char* argv[]) {
    const auto program = filesystem::path(__FILE__).stem().string();
    // args
    std::string json_str;

    try {
	    // clang-format off
        cxxopts::Options options(
          program,
          program + " " + VALHALLA_VERSION + "\n\n"
          "a command line test tool for distance and time routing.\n"
          "Use the -j option for specifying locations.");

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
          ("c,config", "Valhalla configuration file", cxxopts::value<std::string>())
          ("i,inline-config", "Inline JSON config", cxxopts::value<std::string>());
	    // clang-format on

	    auto result = options.parse(argc, argv);
	    if (!parse_common_args(program, options, result, "mjolnir.logging"))
	        return EXIT_SUCCESS;

	    if (!result.count("json")) {
	        throw cxxopts::OptionException("A JSON format request must be present.\n\n" +
					       options.help());
	    }
	    json_str = result["json"].as<std::string>();

        //
        // Try to read json from file
        if (!json_str.empty() && json_str.length() < 400) {
            std::ifstream json_from_file(json_str, std::ifstream::in);
            if (json_from_file.is_open()) {
                json_from_file.seekg(0, std::ios::end);
                size_t size = json_from_file.tellg();
                json_from_file.seekg(std::ios::beg);
                json_str.resize(size, 0);
                json_from_file.read(json_str.data(), size);
                json_from_file.close();
            }
         }
    } catch (cxxopts::OptionException& e) {
	    std::cerr << e.what() << std::endl;
	    return EXIT_FAILURE;
    } catch (std::exception& e) {
	    std::cerr << "Unable to parse command line options because: " << e.what() << "\n"
		      << "This is a bug, please report it at " PACKAGE_BUGREPORT << "\n";
	    return EXIT_FAILURE;
    }

    Api request;
    ParseApi(json_str, valhalla::Options::all_to_all, request);
    auto& options = *request.mutable_options();

    // Find path locations (loki) for sources and targets
    auto t0 = std::chrono::high_resolution_clock::now();
    loki_worker_t lw(config());
    lw.all_to_all(request);
    auto t1 = std::chrono::high_resolution_clock::now();
    uint32_t ms = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
    LOG_INFO("Location Processing took " + std::to_string(ms) + " ms");

    t0 = std::chrono::high_resolution_clock::now();
    thor_worker_t tw(config());
    tw.all_to_all(request);

    t1 = std::chrono::high_resolution_clock::now();
    ms = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
    LOG_INFO("route_processing_time (ms)::" + std::to_string(ms));

    // Shutdown protocol buffer library
    google::protobuf::ShutdownProtobufLibrary();

    return EXIT_SUCCESS;
}
