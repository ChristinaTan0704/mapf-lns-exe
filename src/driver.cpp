#include <boost/program_options.hpp>
#include <boost/tokenizer.hpp>
#include "LNS.h"
#include "AnytimeBCBS.h"
#include "AnytimeEECBS.h"
#include "PIBT/pibt.h"

// --destoryStrategy Random --uniform_neighbor 0 --neighborSize 25  -m pre_work/baseline/MAPF-LNS/map/empty-32-32.map -a pre_work/baseline/MAPF-LNS/scene/empty-32-32-random-25.scen -k 350 -t 1510 --initAlgo PP --maxIterations=2000 --state data/initial_state_json_10s/LACAM2/map-empty-32-32-scene-25-agent-350.json --log_step 100
// --destoryStrategy Random --uniform_neighbor 0 --neighborSize 8 -m pre_work/baseline/MAPF-LNS/map/ost003d.map -a data/nss_random_scene/gen_scene/ost003d-random-59_randomGen.scen -k 400 -t 60 --seed 94480 --initAlgo PP --maxIterations=1
//--map pre_work/baseline/MAPF-LNS/map/random-32-32-20.map --agentNum 150 --state data/initial_state_json_10s/LNS2/map-random-32-32-20-scene-1-agent-150.json --adaptive_weight 1 1 --pprun 1 --num_subset 20 --uniform_neighbor 2 --replanTime 0.6
//  pre_work/baseline/MAPF-LNS/lns --destoryStrategy Adaptive --uniform_neighbor 0 --neighborSize 16 -m pre_work/baseline/MAPF-LNS/map/ost003d.map -a pre_work/baseline/MAPF-LNS/scene/ost003d-random-3.scen -k 600 -t 610 --initAlgo PP --maxIterations=2000 --state data/initial_state_json_10s/LNS2/map-ost003d-scene-3-agent-600.json --log_step 50
/* Main function */ 
int main(int argc, char** argv)
{
	namespace po = boost::program_options;
	// Declare the supported options.
	po::options_description desc("Allowed options");
	desc.add_options()
		("help", "produce help message")
        ("state", po::value<string>()->default_value(""), "json file that stores the state")
        // params for the input instance and experiment settings
        ("collect_data", po::value<int>()->default_value(0), "doing data collection or not")
        ("uniform_neighbor", po::value<int>()->default_value(0), "uniformly genreate the neighbor_size or not")
		("map,m", po::value<string>()->required(), "input file for map")
		("agents,a", po::value<string>()->required(), "input file for agents")
		("agentNum,k", po::value<int>()->default_value(0), "number of agents")
        ("log_step", po::value<int>()->default_value(1), "number of agents")
        ("output,o", po::value<string>(), "output file")
        ("tabu_discount", po::value<double>()->default_value(1), "cutoff time (seconds)")
		("cutoffTime,t", po::value<double>()->default_value(7200), "cutoff time (seconds)")
		("replanTime", po::value<double>()->default_value(0.6), "cutoff time for one replan (seconds)")
		("screen,s", po::value<int>()->default_value(0),
		        "screen option (0: none; 1: LNS results; 2:LNS detailed results; 3: MAPF detailed results)")
		("stats", po::value<string>(), "output stats file")

		// solver
		("solver", po::value<string>()->default_value("LNS"), "solver (LNS, A-BCBS, A-EECBS)")

        // params for LNS
        ("neighborSize", po::value<int>()->default_value(5), "Size of the neighborhood")
        ("seed", po::value<int>()->default_value(0), "Size of the neighborhood")
        ("maxIterations", po::value<int>()->default_value(1000000), "maximum number of iterations")
        ("initAlgo", po::value<string>()->default_value("EECBS"),
                "MAPF algorithm for finding the initial solution (EECBS, PP, PPS, CBS, PIBT, winPIBT)")
        ("replanAlgo", po::value<string>()->default_value("PP"),
                "MAPF algorithm for replanning (EECBS, CBS, PP)")
        ("destoryStrategy", po::value<string>()->default_value("Adaptive"),
                "Heuristics for finding subgroups (Random, RandomWalk, Intersection, Adaptive, RandomWalkAdv, RandomWalkOnce, RandomWalkAdvOnce, RandomWalkProb, RandomWalkMostDelayed, RandomWalkOri)")
        ("pibtWindow", po::value<int>()->default_value(5),
             "window size for winPIBT")
        ("winPibtSoftmode", po::value<bool>()->default_value(true),
             "winPIBT soft mode")
		;
	po::variables_map vm;
	po::store(po::parse_command_line(argc, argv, desc), vm);

	if (vm.count("help")) {
		cout << desc << endl;
		return 1;
	}

    PIBTPPS_option pipp_option;
    pipp_option.windowSize = vm["pibtWindow"].as<int>();
    pipp_option.winPIBTSoft = vm["winPibtSoftmode"].as<bool>();

    po::notify(vm);

	srand(vm["seed"].as<int>());  // TODO uncomment this line
    // srand(time(NULL)); // TODO del this line

	Instance instance(vm["map"].as<string>(), vm["agents"].as<string>(), vm["state"].as<string>(),
		vm["agentNum"].as<int>());
    // return 0; // TODO del this line
    double time_limit = vm["cutoffTime"].as<double>();
    int screen = vm["screen"].as<int>();
	if (vm["solver"].as<string>() == "LNS")
    {
        LNS lns(instance, time_limit,
                vm["initAlgo"].as<string>(),
                vm["replanAlgo"].as<string>(),
                vm["destoryStrategy"].as<string>(),
                vm["neighborSize"].as<int>(),
                vm["maxIterations"].as<int>(), screen, pipp_option);
        lns.uniform_neighbor = vm["uniform_neighbor"].as<int>();
        lns.tabu_discount = vm["tabu_discount"].as<double>();
        lns.collect_data = vm["collect_data"].as<int>();
        lns.state_json = vm["state"].as<string>();
        lns.log_step = vm["log_step"].as<int>();
        lns.replan_time_limit = vm["replanTime"].as<double>();
        bool succ = lns.run();
        if (succ)
            lns.validateSolution();
        if (vm.count("output"))
            lns.writeResultToFile(vm["output"].as<string>());
        if (vm.count("stats"))
            lns.writeIterStatsToFile(vm["stats"].as<string>());
        // lns.writePathsToFile("path.txt");
    }
    else if (vm["solver"].as<string>() == "A-BCBS") // anytime BCBS(w, 1)
    {
        AnytimeBCBS bcbs(instance, time_limit, screen);
        bcbs.run();
        bcbs.validateSolution();
        if (vm.count("output"))
            bcbs.writeResultToFile(vm["output"].as<string>());
        if (vm.count("stats"))
            bcbs.writeIterStatsToFile(vm["stats"].as<string>());
    }
    else if (vm["solver"].as<string>() == "A-EECBS") // anytime EECBS
    {
        AnytimeEECBS eecbs(instance, time_limit, screen);
        eecbs.run();
        eecbs.validateSolution();
        if (vm.count("output"))
            eecbs.writeResultToFile(vm["output"].as<string>());
        if (vm.count("stats"))
            eecbs.writeIterStatsToFile(vm["stats"].as<string>());
    }
	else
    {
	    cerr << "Solver " << vm["solver"].as<string>() << " does not exist!" << endl;
	    exit(-1);
    }
	return 0;

}