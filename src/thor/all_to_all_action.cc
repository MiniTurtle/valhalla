#include "sif/autocost.h"
#include "sif/bicyclecost.h"
#include "sif/pedestriancost.h"
#include "thor/costmatrix.h"
#include "thor/timedistancebssmatrix.h"
#include "thor/timedistancematrix.h"
#include "thor/worker.h"
#include "thor/unidirectional_astar.h"
#include "tyr/serializers.h"
#include "config.h"
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

std::string thor_worker_t::all_to_all(Api& request) {
    const boost::property_tree::ptree& pt = thor_worker_t::config;

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
    
    //bd.set_early_exit(callback);

    auto all_to_all = request.mutable_all_to_all();

    class SingleThread {
    public:
        struct SolvedData {
            uint32_t i;
            uint32_t j;
            float time;
            float distance;

            SolvedData(){}
            SolvedData(
                uint32_t _i,
                uint32_t _j,
                float _time,
                float _distance) : i(_i), j(_j), time(_time), distance(_distance){}
        };

        std::vector<SingleThread::SolvedData> data;

        bool should_close = false;

    private:
        //BidirectionalAStar bd;
        UnidirectionalAStar<ExpansionType::forward> bd;
        std::thread thread;
        const google::protobuf::RepeatedPtrField<valhalla::Location>& locations;
        baldr::GraphReader* reader;
        const sif::mode_costing_t& mode_costing;
        const sif::TravelMode& mode;
        valhalla::Options* options;

        std::mutex mutex_signal;
        int solve_batch = -1; 
        std::function<void()> on_done;
        std::condition_variable cv;

    public:
        SingleThread(
            const boost::property_tree::ptree& config, 
            valhalla::Options* _options, 
            baldr::GraphReader* _reader,
            const sif::mode_costing_t& _mode_costing,
            const sif::TravelMode& _mode) 
            : 
            bd(config.get_child("thor")), 
            locations(_options->locations()),
            mode_costing(_mode_costing),
            mode(_mode),
            options(_options) {

            reader = new baldr::GraphReader(config.get_child("mjolnir"));
            thread = std::thread(&SingleThread::main, this);
        }

        ~SingleThread() {
            delete reader;
        }

        void join() {
            thread.join();
        }

        void close() {
            should_close = true;
            std::lock_guard<std::mutex> lock(mutex_signal);
            cv.notify_one();
        }

        void set_on_done(std::function<void()>& callback) {
            on_done = std::move(callback);
        }

        void solve_locations(int i) {  
            solve_batch = i;
            std::lock_guard<std::mutex> lock(mutex_signal);
            cv.notify_one();
        }

        void notify_on_done(std::vector<SingleThread::SolvedData>& _data) {
            data = std::move(_data);
            on_done();
        }

    private:
        static void main(SingleThread* st) {
            baldr::GraphReader& reader = *st->reader;
            const sif::mode_costing_t& mode_costing = st->mode_costing;
            const sif::TravelMode& mode = st->mode;
            const Options& options = *st->options;
            auto locations = st->locations;
            auto& bd = st->bd;

            //std::unordered_map<std::pair<uint64_t, uint64_t>, PathData, pair_hash, pair_equal> map;
            //bool was_early_exit = false;

            //uint64_t early_exit_dest_edge = 0;
            //auto callback = [&](const uint64_t& begin, const uint64_t& end) {
            //    was_early_exit = true;
            //    if (!map.empty()) {
            //        //for (auto dest_edge : destionation_edges) {
            //        std::pair<uint64_t, uint64_t> pair(begin, end);
            //        auto it = map.find(pair);
            //        if (it != map.end()) {
            //            early_exit_dest_edge = end;
            //            return true;
            //        }
            //    }

            //    /*    std::pair<baldr::GraphId, baldr::GraphId> pair_reverse(dest_edge, end.edgeid);
            //        it = map.find(pair_reverse);
            //        if (it != map.end())
            //            return true;*/
            //
            //        /*std::pair<baldr::GraphId, baldr::GraphId> pair_op(end.opp_edgeid, dest_edge);
            //        it = map.find(pair_op);
            //        if (it != map.end())
            //            return true;*/

            //        /*   std::pair<baldr::GraphId, baldr::GraphId> pair_op_reverse(dest_edge, end.opp_edgeid);
            //        it = map.find(pair_op_reverse);
            //        if (it != map.end())
            //            return true;*/
            //        //}

            //    was_early_exit = false;
            //    return false;
            //};

            //bd.set_early_exit(callback);

            while(true) {
		        {
                    std::unique_lock<std::mutex> lck(st->mutex_signal);
                    st->cv.wait(lck, [st] { return st->solve_batch >= 0 || st->should_close; });
                }

                if (st->should_close)
                    return;

                uint32_t i = st->solve_batch;
                st->solve_batch = -1;

                std::vector<SingleThread::SolvedData> data;
                data.reserve(locations.size());

	            valhalla::Location origin = locations[i];
                for (uint32_t j = 0; j < locations.size(); ++j) {
                    if (i == j) {
                        data.emplace_back(i, j, 0, 0);
                        continue;
                    }

		            valhalla::Location dest = locations[j];

                    /* destionation_edges.clear();
                    for (const auto& edge : dest.correlation().edges()) {
                        destionation_edges.push_back(edge.graph_id());
                    }*/

                    bd.Clear();
             
                    std::vector<std::vector<PathInfo>> paths;
                    try {
                        paths = bd.GetBestPath(origin, dest, reader, mode_costing, mode, options);
                    } catch(const std::exception& e) {
                        std::cerr << "GetBeshPath: " << e.what() << std::endl;
                        data.emplace_back(i, j, -1, -1);
                        continue;
                    } catch(...) {
                        std::cerr << "GetBeshPath error" << std::endl;
                        data.emplace_back(i, j, -1, -1);
                        continue;
                    }

                    if (paths.empty()) {
                        data.emplace_back(i, j, -1, -1);
                        continue;
                    }

                    float distance = 0;
                    sif::Cost cost;
                    for (auto& path : paths) {
                        for (auto it = path.begin(); it != path.end(); it++) {
                            distance += it->path_distance;
                            cost += it->elapsed_cost;
                        }
                    }
                    data.emplace_back(i, j, distance, cost.secs);

                   /* std::vector<PathData> path_datas;
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
                        }*/

		                /*for (size_t path_i = 0; path_i < path.size()-1; path_i++)
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
                            }*/
                    //}

                    //if (was_early_exit) {
                    //    //
                    //    // add the end of the map path to path
                    //    for (size_t i = 0; i < paths.size(); i++) {
                    //        auto& path = paths[i];
                    //        auto& path_data = path_datas[i];

                    //        PathInfo& path_it = path.back();

                    //        auto it = map.find({path_it.edgeid.value, early_exit_dest_edge});
                    //        assert(it != map.end());

                    //        auto& insert_path_data = it->second;
                    //        //path.insert(path.end(), insert_path.begin(), insert_path.end());
                    //        path_data.path_distance += insert_path_data.path_distance;
                    //        path_data.elapsed_cost += insert_path_data.elapsed_cost;
                    //    }
                    //}

                    /*for (auto& path_data : path_datas) {
                        data.emplace_back(i, j, path_data.path_distance, path_data.elapsed_cost.secs);
                    }*/
                }

                st->notify_on_done(data);
            }
        }
    };

    //
    // Setup threads
    //
    const int num_threads = 6;
    //const int num_threads = 1;
    std::vector<SingleThread*> thread_data;
    thread_data.reserve(num_threads);

    int next_location_batch = 0;

    std::vector<SingleThread*> taskQueue;
    std::mutex queueMutex;
    std::mutex notifyMutex;
    std::condition_variable queueCv;

    for (int i = 0; i < num_threads; i++) {
        SingleThread* st = new SingleThread(pt, &options, reader.get(), mode_costing, mode);
        thread_data.push_back(st);

        //
        // on_done is called from Thread.
        std::function<void()> on_done = [&, st]() {
		    {
                std::lock_guard<std::mutex> lock(queueMutex);
                taskQueue.push_back(st);
            }
            std::lock_guard<std::mutex> lock(notifyMutex);
            queueCv.notify_one();
        };

        if (next_location_batch < locations.size()) {
            st->set_on_done(on_done);
            st->solve_locations(next_location_batch);

            std::cout << "[progress_begin]" << (next_location_batch+1) << "/" << locations.size() << "[progress_end]" << std::endl; 
            next_location_batch++;
            continue;
        }

        st->close();
    }

    //
    // Manager threads
    //
    while(true) {
        std::vector<std::vector<SingleThread::SolvedData>> solved_data;

        bool is_done = true;
        for (auto& st : thread_data)
            if (!st->should_close) {
                is_done = false;
                break;
            }

        if (is_done)
            break;

	    {
	        {
                std::unique_lock<std::mutex> lock_wait(notifyMutex);
                queueCv.wait(lock_wait, [&] { return !taskQueue.empty(); });
            }
            std::lock_guard<std::mutex> lock(queueMutex);

            solved_data.reserve(taskQueue.size());

            for (auto mgr : taskQueue) {
                solved_data.push_back(std::move(mgr->data));

                if (next_location_batch >= locations.size()) {
                    mgr->close();
                    continue;
                }

                std::cout << "[progress_begin]" << (next_location_batch+1) << "/" << locations.size() << "[progress_end]" << std::endl; 

                mgr->solve_locations(next_location_batch);
                next_location_batch++;
            }

            taskQueue.clear();
        }
        
        for (auto data : solved_data) {
            for (auto& path_data : data) {
                all_to_all->add_from_indices(path_data.i);
                all_to_all->add_to_indices(path_data.j);
                all_to_all->add_distances(path_data.distance);
                all_to_all->add_times(path_data.time);
            }
        }
    }

    for (int i = 0; i < num_threads; i++) {
        thread_data[i]->close();
        thread_data[i]->join();
        delete thread_data[i];
    }

    return tyr::serialize_all_to_all(request);
}
} // namespace thor
} // namespace valhalla
