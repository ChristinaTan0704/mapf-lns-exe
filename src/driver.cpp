﻿#include <boost/program_options.hpp>
#include <boost/tokenizer.hpp>
#include "LNS.h"
#include "AnytimeBCBS.h"
#include "AnytimeEECBS.h"
#include "PIBT/pibt.h"

/* Main function */
int main(int argc, char** argv)
{
	namespace po = boost::program_options;
	// Declare the supported options.
	po::options_description desc("Allowed options");
    std::vector<int> inputIntegers;
	desc.add_options()
		("help", "produce help message")
        ("state", po::value<string>()->required(), "json files that store the state")
        ("map,m", po::value<string>(), "input file for map")
        ("agentNum,k", po::value<int>(), "number of agents")
        ("adaptive_weight", po::value<std::vector<double>>()->multitoken()->default_value(std::vector<double>{1.0, 1.0, 0.0}, "1.0 1.0"), "weight for the adaptive strategy")
        ("pprun", po::value<int>()->default_value(1), "number of time to run PP replan")
		("replanTime", po::value<double>()->default_value(0.6), "cutoff time for one replan (seconds)")
        ("num_subset", po::value<int>()->default_value(20), "number of subset to generate")
        ("uniform_neighbor", po::value<int>()->default_value(0), "(0) fixed nb_size specified by --neighborSize (1) nb_size sample from {2,4,8,16,32} (2) nb_size sample from 5~16")
        ("replan", po::value<bool>()->default_value(true),"use pp to replan or not")
        // params for the input instance and experiment settings
        ("agents,a", po::value<string>()->default_value(""), "input file for agents")
        ("output,o", po::value<string>(), "output file")
        ("tabu_discount", po::value<double>()->default_value(1), "cutoff time (seconds)")
		("cutoffTime,t", po::value<double>()->default_value(7200), "cutoff time (seconds)")
		("screen,s", po::value<int>()->default_value(0),
		        "screen option (0: none; 1: LNS results; 2:LNS detailed results; 3: MAPF detailed results)")
		("stats", po::value<string>(), "output stats file")
		// solver
		("solver", po::value<string>()->default_value("LNS"), "solver (LNS, A-BCBS, A-EECBS)")
        // params for LNS
        ("neighborSize", po::value<int>()->default_value(8), "Size of the neighborhood")
        ("seed", po::value<int>()->default_value(0), "Size of the neighborhood")
        ("replanAlgo", po::value<string>()->default_value("PP"),
                "MAPF algorithm for replanning (EECBS, CBS, PP)")
        ("destroyStrategy", po::value<string>()->default_value("Adaptive"),
                "Heuristics for finding subgroups (Random, RandomWalk, Intersection, Adaptive, RandomWalkProb, RadomWalkLarge)")
        ("pibtWindow", po::value<int>()->default_value(5),
             "window size for winPIBT")
        ("winPibtSoftmode", po::value<bool>()->default_value(true),
             "winPIBT soft mode")
		;
	po::variables_map vm;
	po::store(po::parse_command_line(argc, argv, desc), vm);

    srand(vm["seed"].as<int>()); 

	if (vm.count("help")) {
		cout << desc << endl;
		return 1;
	}

    PIBTPPS_option pipp_option;
    pipp_option.windowSize = vm["pibtWindow"].as<int>();
    pipp_option.winPIBTSoft = vm["winPibtSoftmode"].as<bool>();

    po::notify(vm);

    srand(time(NULL));
    string state = vm["state"].as<std::string>();

    std::cout << "Input State :  " << state << endl;
    Instance instance(vm["map"].as<string>(), vm["agents"].as<string>(), state,
                      vm["agentNum"].as<int>());
    double time_limit = vm["cutoffTime"].as<double>();
    int screen = vm["screen"].as<int>();
    LNS lns(instance, time_limit, "JSON",
            vm["replanAlgo"].as<string>(),
            vm["destroyStrategy"].as<string>(),
            vm["neighborSize"].as<int>(),
            vm["num_subset"].as<int>(), screen, pipp_option);
    lns.uniform_neighbor = vm["uniform_neighbor"].as<int>();
    lns.tabu_discount = vm["tabu_discount"].as<double>();
    lns.state_json = state;
    lns.replan_time_limit = vm["replanTime"].as<double>();
    lns.destroy_weights = vm["adaptive_weight"].as<vector<double>>();
    lns.pprun = vm["pprun"].as<int>();
    lns.replan = vm["replan"].as<bool>();

    std::string  input;

    while (true) {
        std::cout << "CPP program start" << endl;
        std::cout << "Enter command: ";
        std::getline(std::cin, input);
        if (input.empty()) {
            continue;  // Skip empty input
        }

        // Split the input into argc and argv-like structure
        std::vector<std::string> argv;
        std::istringstream iss(input);
        std::string arg;
        while (iss >> std::quoted(arg)) {  // Handle potentially quoted arguments
            argv.push_back(arg);
        }

        // Add the program name at the start of argv
        argv.insert(argv.begin(), "program");

        // Parse the command line arguments
        po::variables_map vm;
        try {
            po::store(po::command_line_parser(argv).options(desc).run(), vm);

            if (vm.count("help")) {
                std::cout << desc << "\n";
                continue;
            }

            po::notify(vm);  // This throws on error, be sure to catch it
        } catch (const po::error& e) {
            std::cerr << "ERROR: " << e.what() << std::endl;
            std::cerr << desc << std::endl;
            continue;
        }

        if (vm.count("state")) {
            lns.state_json  = vm["state"].as<std::string>();
            lns.destroy_weights  = vm["adaptive_weight"].as<std::vector<double>>();
            lns.replan_time_limit = vm["replanTime"].as<double>();

            bool succ = lns.run();  // You might want to pass state and weights
            cout << "tabu_list: ";
            for (auto i : lns.tabu_list)
                cout << i << " ";
            cout << endl;
            std::cout << "Finish Generating for State: " << state << std::endl;
        }
        

    }



    if (vm.count("output"))
        lns.writeResultToFile(vm["output"].as<string>());
    if (vm.count("stats"))
        lns.writeIterStatsToFile(vm["stats"].as<string>());
	return 0;

}