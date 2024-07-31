#include "sif/autocost.h"
#include "sif/bicyclecost.h"
#include "sif/pedestriancost.h"
#include "thor/costmatrix.h"
#include "thor/timedistancebssmatrix.h"
#include "thor/timedistancematrix.h"
#include "thor/worker.h"
#include "tyr/serializers.h"
#include "config.h"

using namespace valhalla;
using namespace valhalla::tyr;
using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::sif;
using namespace valhalla::thor;

namespace valhalla {
namespace thor {

std::string thor_worker_t::all_to_all(Api& request) {
    // time this whole method and save that statistic
    auto _ = measure_scope_time(request);

    auto& options = *request.mutable_options();
    //adjust_scores(options);
    auto costing = parse_costing(request);


    // Get type of route - this provides the costing method to use.
    const std::string& routetype = valhalla::Costing_Enum_Name(options.costing_type());
    LOG_INFO("routetype: " + routetype);

    // Get the costing method - pass the JSON configuration
    sif::TravelMode mode;

    // 
    // Construct costing
    CostFactory factory;
    auto mode_costing = factory.CreateModeCosting(options, mode);

    BidirectionalAStar& bd = bidir_astar;

    cost_ptr_t cost = mode_costing[static_cast<uint32_t>(mode)];
    cost->set_allow_destination_only(false);

    auto locations = options.locations();
	/*for (int i = 0; i < locations.size(); ++i) 
        for (uint32_t j = i + 1; j < locations.size(); ++j) {
	        auto loc_i = locations[i].ll();
	        auto loc_j = locations[j].ll();
		    if (loc_i.lat() != loc_j.lat())
                continue; 
            if (loc_i.lng() != loc_j.lng())
                continue;
		    locations.erase(locations.begin()+j);
            i--;
	        break;
        }*/

    std::vector<uint64_t> destionation_edges;

    struct pair_hash {
    std::size_t operator()(const std::pair<uint64_t, uint64_t>& p) const {
            auto hash1 = std::hash<uint64_t>{}(p.first);
            auto hash2 = std::hash<uint64_t>{}(p.second);
            return hash1 ^ (hash2 << 1); // Combine the two hash values
        }
    };
    struct pair_equal {
        bool operator()(const std::pair<uint64_t, uint64_t>& p1, const std::pair<uint64_t, uint64_t>& p2) const {
            return p1.first == p2.first && p1.second == p2.second;
        }
    };
    struct PathData {
        float path_distance = 0;
        sif::Cost elapsed_cost;
    };
    std::unordered_map<std::pair<uint64_t, uint64_t>, PathData, pair_hash, pair_equal> map;
    bool was_early_exit = false;

    uint64_t early_exit_dest_edge = 0;
    auto callback = [&](const uint64_t& begin, const uint64_t& end) {
        was_early_exit = true;
        if (!map.empty()) {
            //for (auto dest_edge : destionation_edges) {
            std::pair<uint64_t, uint64_t> pair(begin, end);
            auto it = map.find(pair);
            if (it != map.end()) {
                early_exit_dest_edge = end;
                return true;
            }
        }

        /*    std::pair<baldr::GraphId, baldr::GraphId> pair_reverse(dest_edge, end.edgeid);
            it = map.find(pair_reverse);
            if (it != map.end())
                return true;*/
            
            /*std::pair<baldr::GraphId, baldr::GraphId> pair_op(end.opp_edgeid, dest_edge);
            it = map.find(pair_op);
            if (it != map.end())
                return true;*/

            /*   std::pair<baldr::GraphId, baldr::GraphId> pair_op_reverse(dest_edge, end.opp_edgeid);
            it = map.find(pair_op_reverse);
            if (it != map.end())
                return true;*/
            //}

        was_early_exit = false;
        return false;
    };
    bd.set_early_exit(callback);

    auto all_to_all = request.mutable_all_to_all();

    for (uint32_t i = 0; i < locations.size()-1; i++) {
	    valhalla::Location origin = locations[i];
        for (uint32_t j = i + 1; j < locations.size(); ++j) {
		    valhalla::Location dest = locations[j];

            destionation_edges.clear();
            for (const auto& edge : dest.correlation().edges()) {
                destionation_edges.push_back(edge.graph_id());
            }

            bd.Clear();
            auto paths = bd.GetBestPath(origin, dest, *reader, mode_costing, mode, options);

            std::vector<PathData> path_datas;
            path_datas.reserve(paths.size());
            for (auto& path : paths) {
		        {
                    PathData data;
                    float& distance = data.path_distance;
                    sif::Cost& cost = data.elapsed_cost;
                    for (auto it = path.begin(); it != path.end(); it++) {
                        distance += it->path_distance;
                        cost += it->elapsed_cost;
                    }
                    path_datas.push_back(data);
                }

		        for (size_t path_i = 0; path_i < path.size()-1; path_i++)
		            for (size_t path_j = path_i+1; path_j < path.size(); path_j++) {
                        auto& p_i = path[path_i];
                        auto& p_j = path[path_j];
  
                        PathData data;
                        float& distance = data.path_distance;
                        sif::Cost& cost = data.elapsed_cost;
                        for (auto it = path.begin()+path_i; it != path.begin()+path_j+1; it++) {
                            distance += it->path_distance;
                            cost += it->elapsed_cost;
                        }
                        
                        map[{p_i.edgeid.value, p_j.edgeid.value}] = data;
                        map[{p_j.edgeid.value, p_i.edgeid.value}] = data;
                    }
                }

            if (was_early_exit) {
                //
                // add the end of the map path to path
                for (size_t i = 0; i < paths.size(); i++) {
                    auto& path = paths[i];
                    auto& path_data = path_datas[i];

                    PathInfo& path_it = path.back();

                    auto it = map.find({path_it.edgeid.value, early_exit_dest_edge});
                    assert(it != map.end());

                    auto& insert_path_data = it->second;
                    //path.insert(path.end(), insert_path.begin(), insert_path.end());
                    path_data.path_distance += insert_path_data.path_distance;
                    path_data.elapsed_cost += insert_path_data.elapsed_cost;
                }
            }

            for (auto& path_data : path_datas) {
                all_to_all->add_from_indices(i);
                all_to_all->add_to_indices(j);
                all_to_all->add_distances(path_data.path_distance);
                all_to_all->add_times(path_data.elapsed_cost.secs);
            }
        }
    }

    return tyr::serialize_all_to_all(request);
}
} // namespace thor
} // namespace valhalla
