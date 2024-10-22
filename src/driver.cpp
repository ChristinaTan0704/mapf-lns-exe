﻿#include <boost/program_options.hpp>
#include <boost/tokenizer.hpp>
#include "LNS.h"
#include "AnytimeBCBS.h"
#include "AnytimeEECBS.h"
#include "PIBT/pibt.h"
// --uniform_neighbor 4 --nbAlgo TS --neighborSize 4 --destroyStrategy RandomWalkProb --map /local-scratchb/jiaqit/exp/mapf-lns-benchmark/data/map/empty-32-32.map                 --state /local-scratchb/jiaqit/exp/mapf-lns-benchmark/data/lns2_init_states/map-empty-32-32-scene-24-agent-300.json                 --agentNum 300  --maxIterations 100  --cutoffTime 300
// --uniform_neighbor 4 --nbAlgo TS --neighborSize 4 --destroyStrategy RandomWalkProb --map /local-scratchb/jiaqit/exp/mapf-lns-benchmark/data/map/den520d.map                 --state /local-scratchb/jiaqit/exp/mapf-lns-benchmark/data/lns2_init_states/map-den520d-scene-24-agent-900.json                 --agentNum 900  --maxIterations 100  --cutoffTime 300
// --destroyStrategy RandomWalkProb --uniform_neighbor 3 --neighborSize 4                 --map /local-scratchb/jiaqit/exp/mapf-lns-benchmark/data/map/empty-32-32.map                 --state /local-scratchb/jiaqit/exp/mapf-lns-benchmark/data/lns2_init_states/map-empty-32-32-scene-24-agent-300.json                 --agentNum 300  --maxIterations 100  --cutoffTime 300
/* Main function */
int main(int argc, char** argv)
{
	namespace po = boost::program_options;
	// Declare the supported options.
	po::options_description desc("Allowed options");
	desc.add_options()
		("help", "produce help message")
        ("state", po::value<string>()->required(), "json file that stores the state")
        ("uniform_neighbor", po::value<int>()->default_value(0), "(0) fixed nb_size specified by --neighborSize (1) nb_size sample from {4,8,16,32} (2) nb_size sample from 5~16 (3) simple adaptive (4) bandit based algorithm")
		("map,m", po::value<string>()->required(), "input file for map")
		// ("agents,a", po::value<string>()->required(), "input file for agents")
		("agentNum,k", po::value<int>()->required(), "number of agents")
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
        ("effi_factor", po::value<double>()->default_value(0.1), "weight for nb efficiency")
        ("nbAlgo", po::value<string>()->default_value("RLE"),
                "nb selection algorithm (RLE, UCB, TS)")
        ("neighborSize", po::value<int>()->default_value(5), "Size of the neighborhood")
        ("seed", po::value<int>()->default_value(0), "Size of the neighborhood")
        ("maxIterations", po::value<int>()->default_value(1000000), "maximum number of iterations")
        ("initAlgo", po::value<string>()->default_value("EECBS"),
                "MAPF algorithm for finding the initial solution (EECBS, PP, PPS, CBS, PIBT, winPIBT)")
        ("replanAlgo", po::value<string>()->default_value("PP"),
                "MAPF algorithm for replanning (EECBS, CBS, PP)")
        ("destroyStrategy", po::value<string>()->default_value("Adaptive"),
                "Heuristics for finding subgroups (Random, RandomWalk, Intersection, Adaptive, RandomWalkProb)")
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

	srand(vm["seed"].as<int>()); 

	Instance instance(vm["map"].as<string>(), vm["state"].as<string>(),
		vm["agentNum"].as<int>());
    double time_limit = vm["cutoffTime"].as<double>();
    int screen = vm["screen"].as<int>();
	if (vm["solver"].as<string>() == "LNS")
    {
        LNS lns(instance, time_limit,
                vm["initAlgo"].as<string>(),
                vm["replanAlgo"].as<string>(),
                vm["destroyStrategy"].as<string>(),
                vm["neighborSize"].as<int>(),
                vm["maxIterations"].as<int>(), screen, pipp_option);
        lns.uniform_neighbor = vm["uniform_neighbor"].as<int>();
        lns.nb_algo_name = vm["nbAlgo"].as<string>();
        lns.tabu_discount = vm["tabu_discount"].as<double>();
        lns.state_json = vm["state"].as<string>();
        lns.log_step = vm["log_step"].as<int>();
        lns.replan_time_limit = vm["replanTime"].as<double>();
        lns.effi_factor = vm["effi_factor"].as<double>();
        bool succ = lns.run();
        if (succ)
            lns.validateSolution();
        if (vm.count("output"))
            lns.writeResultToFile(vm["output"].as<string>());
        if (vm.count("stats"))
            lns.writeIterStatsToFile(vm["stats"].as<string>());
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