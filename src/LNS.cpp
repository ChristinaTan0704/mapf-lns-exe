#include "LNS.h"
#include <queue>
#include <random>
#include <array>
#include <nlohmann/json.hpp>


LNS::LNS(const Instance& instance, double time_limit, string init_algo_name, string replan_algo_name, string destory_name,
         int neighbor_size, int num_of_subset, int screen, PIBTPPS_option pipp_option) :
         instance(instance), time_limit(time_limit), init_algo_name(std::move(init_algo_name)),
         replan_algo_name(replan_algo_name), neighbor_size(neighbor_size), num_of_subset(num_of_subset),
         screen(screen), path_table(instance.map_size),pipp_option(pipp_option)
{
    start_time = Time::now();
    if (destory_name == "Adaptive")
    {
        ALNS = true;
        destroy_weights.assign(3, 1);
    }
    else if (destory_name == "RandomWalk")
        destroy_strategy = RANDOMWALK;
    else if (destory_name == "Intersection")
        destroy_strategy = INTERSECTION;
    else if (destory_name == "Random")
        destroy_strategy = RANDOMAGENTS;
    else if (destory_name == "RandomWalkLarge")
        destroy_strategy = RANDOMWALKLARGE;
    else if (destory_name == "RandomWalkProb")
        destroy_strategy = RANDOMWALKPROB;
    int N = instance.getDefaultNumberOfAgents();
    agents.reserve(N);
    for (int i = 0; i < N; i++){
        agents.emplace_back(instance, i);
    }

    preprocessing_time = ((fsec)(Time::now() - start_time)).count();
    if (screen >= 2)
        cout << "Pre-processing time = " << preprocessing_time << " seconds." << endl;
    int debug;
}

bool LNS::run()
{

    // only for statistic analysis, and thus is not included in runtime
    initial_solution_runtime = 0;
    bool succ = true;
    int count = 0;
    // load initial state from json file
    using json = nlohmann::json;
    std::ifstream f(state_json);
    json data = json::parse(f);
    neighbor.sum_of_costs = 0;
    for (auto & [key, value] : data.items()){
        Path path;
        int id = std::stoi(key);
        for (const auto& pair : value) {
            int row = static_cast<int>(pair[0]);
            int col = static_cast<int>(pair[1]);
            auto loc = instance.num_of_cols * row + col;
            path.emplace_back(PathEntry(loc));
        }
        agents[id].path = path;
        neighbor.sum_of_costs += path.size() - 1;
        path_table.insertPath(agents[id].id, agents[id].path);
    }
    initial_sum_of_costs = neighbor.sum_of_costs;
    sum_of_costs = neighbor.sum_of_costs;
    cout << "load initial state from " << state_json << endl;
    cout << "Initial solution cost = " << initial_sum_of_costs << endl;
    neighbor_size = replan_agents.size();
    // set the replanend agents
    neighbor.agents.resize(neighbor_size);
    for (int i = 0; i < neighbor_size; i++){
        neighbor.agents[i] = agents[replan_agents[i]].id;
    }

    succ = true;
    // print the replanned agents 
    cout << "replaned agents : ";
    for (auto id : neighbor.agents)
        cout << agents[id].id << " ";
    cout << endl;

    neighbor.old_paths.resize(neighbor.agents.size());
    neighbor.old_sum_of_costs = 0;
    for (int i = 0; i < (int)neighbor.agents.size(); i++)
    {
        if (replan_algo_name == "PP")
            neighbor.old_paths[i] = agents[neighbor.agents[i]].path;
        path_table.deletePath(neighbor.agents[i], agents[neighbor.agents[i]].path);
        neighbor.old_sum_of_costs += agents[neighbor.agents[i]].path.size() - 1;
    }
    cout << "one start end" << endl;

    runPP();

    cout << "one replan end" << endl;
    path_table.reset();


    return true;
}

bool LNS::runPP()
{   
    double pp_sum_of_costs = 0; // sum of costs of the new paths of all pp runs
    double pp_sum_of_time = 0; // sum of time of all pp runs
    auto shuffled_agents = neighbor.agents;
    int replan_iter = 0;
    bool first_iter = true;


    while (replan_iter < pprun){
        replan_iter += 1;
        auto replan_start_time = Time::now();
        std::random_shuffle(shuffled_agents.begin(), shuffled_agents.end());
        int remaining_agents = (int)shuffled_agents.size();
        auto p = shuffled_agents.begin();
        neighbor.sum_of_costs = 0;

        while (p != shuffled_agents.end() && ((fsec)(Time::now() - replan_start_time)).count() < replan_time_limit)
        {
            int id = *p;
            agents[id].path = agents[id].path_planner.findOptimalPath(path_table, replan_time_limit);
            if (agents[id].path.empty())
            {
                break;
            }
            neighbor.sum_of_costs += (int)agents[id].path.size() - 1;
            path_table.insertPath(agents[id].id, agents[id].path);
            remaining_agents--;
            ++p;


        }
        pp_sum_of_time +=  ((fsec)(Time::now() - replan_start_time)).count() ;
        if (remaining_agents == 0){
            pp_sum_of_costs += neighbor.sum_of_costs;
        }
        else{
            pp_sum_of_costs += neighbor.old_sum_of_costs;
        }

        if (first_iter == true){
            if (neighbor.sum_of_costs < neighbor.old_sum_of_costs and remaining_agents == 0){
                cout << "Improved : " << neighbor.old_sum_of_costs - neighbor.sum_of_costs << endl;
                cout << "new paths start " << endl;
                for (auto id : shuffled_agents) {
                    cout << "agent " << agents[id].id << " ";
                    int t = 0;
                    for (auto loc: agents[id].path) {
                        // cout << "(" << t << "," << loc.location << ")" << " --> ";
                        auto coord = instance.getCoordinate(loc.location);
                        cout  << "(" << coord.first << "," << coord.second << ") --> ";
                        t += 1;
                    }
                    cout << endl;
                }
                cout << "new paths end " << endl;

            }
            else if (neighbor.sum_of_costs >= neighbor.old_sum_of_costs and remaining_agents == 0){
                cout << "No improvement" << endl;
            }
            else if (remaining_agents!=0){
                cout << "No path" << endl;
            }
        }
        else{
            if (neighbor.sum_of_costs < neighbor.old_sum_of_costs and remaining_agents == 0){
                cout << "Improved : " << neighbor.old_sum_of_costs - neighbor.sum_of_costs << endl;
            }else if (neighbor.sum_of_costs >= neighbor.old_sum_of_costs and remaining_agents == 0){
                cout << "No improvement" << endl;
            }
            else if (remaining_agents!=0){
                cout << "No path" << endl;
            }

        }
        first_iter = false; 



        // set back to old paths
        auto p2 = shuffled_agents.begin();
        while (p2 != p) // remove the planned paths
        {
            int a = *p2;
            path_table.deletePath(agents[a].id, agents[a].path);
            ++p2;
        }
        neighbor.sum_of_costs = neighbor.old_sum_of_costs;
    }

    for (int i = 0; i < (int)neighbor.agents.size(); i++)
    {
        auto a = neighbor.agents[i];
        agents[a].path = neighbor.old_paths[i];
        path_table.insertPath(agents[a].id, agents[a].path);

    }
    neighbor.sum_of_costs = neighbor.old_sum_of_costs;

    cout << "average_replan_time: " << pp_sum_of_time / pprun << endl;
    cout << "average_improvement: " << (neighbor.old_sum_of_costs * pprun - pp_sum_of_costs) / pprun << endl;



    return true;
}

void LNS::validateSolution() const
{
    int sum = 0;
    for (const auto& a1_ : agents)
    {
        if (a1_.path.empty())
        {
            cerr << "No solution for agent " << a1_.id << endl;
            exit(-1);
        }
        else if (a1_.path_planner.start_location != a1_.path.front().location)
        {
            cerr << "The path of agent " << a1_.id << " starts from location " << a1_.path.front().location
                << ", which is different from its start location " << a1_.path_planner.start_location << endl;
            exit(-1);
        }
        else if (a1_.path_planner.goal_location != a1_.path.back().location)
        {
            cerr << "The path of agent " << a1_.id << " ends at location " << a1_.path.back().location
                 << ", which is different from its goal location " << a1_.path_planner.goal_location << endl;
            exit(-1);
        }
        for (int t = 1; t < (int) a1_.path.size(); t++ )
        {
            if (!instance.validMove(a1_.path[t - 1].location, a1_.path[t].location))
            {
                cerr << "The path of agent " << a1_.id << " jump from "
                     << a1_.path[t - 1].location << " to " << a1_.path[t].location
                     << " between timesteps " << t - 1 << " and " << t << endl;
                exit(-1);
            }
        }
        sum += (int) a1_.path.size() - 1;
        for (const auto& a2_: agents)
        {
            if (a1_.id >= a2_.id || a1_.path.empty())
                continue;
            const auto a1 = a1_.path.size() <= a2_.path.size()? a1_ : a2_;
            const auto a2 = a1_.path.size() <= a2_.path.size()? a2_ : a1_;
            int t = 1;
            for (; t < (int) a1.path.size(); t++)
            {
                if (a1.path[t].location == a2.path[t].location) // vertex conflict
                {
                    cerr << "Find a vertex conflict between agents " << a1.id << " and " << a2.id <<
                            " at location " << a1.path[t].location << " at timestep " << t << endl;
                    exit(-1);
                }
                else if (a1.path[t].location == a2.path[t - 1].location &&
                        a1.path[t - 1].location == a2.path[t].location) // edge conflict
                {
                    cerr << "Find an edge conflict between agents " << a1.id << " and " << a2.id <<
                         " at edge (" << a1.path[t - 1].location << "," << a1.path[t].location <<
                         ") at timestep " << t << endl;
                    exit(-1);
                }
            }
            int target = a1.path.back().location;
            for (; t < (int) a2.path.size(); t++)
            {
                if (a2.path[t].location == target)  // target conflict
                {
                    cerr << "Find a target conflict where agent " << a2.id << " traverses agent " << a1.id <<
                         "'s target location " << target << " at timestep " << t << endl;
                    exit(-1);
                }
            }
        }
    }
    if (sum_of_costs != sum)
    {
        cerr << "The computed sum of costs " << sum_of_costs <<
             " is different from the sum of the paths in the solution " << sum << endl;
        exit(-1);
    }
}
