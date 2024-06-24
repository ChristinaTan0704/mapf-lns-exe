#include "LNS.h"
#include <queue>
#include <random>
#include <array>
#include <nlohmann/json.hpp>

int getRandomFromSetExp() {
    std::array<int, 5> values = {2, 4, 8, 16, 32};
    // Random number generator
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> distrib(0, values.size() - 1);

    // Generate random index and return the value at that index
    int randomIndex = distrib(gen);
    return values[randomIndex];
}

int getRandomFromRange() {
    // Random number generator
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> distrib(5, 16); // Adjusted to cover the range [0, 31]

    // Generate and return a random number from the range [0, 31]
    return distrib(gen);
}

LNS::LNS(const Instance& instance, double time_limit, string init_algo_name, string replan_algo_name, string destory_name,
         int neighbor_size, int num_of_iterations, int screen, PIBTPPS_option pipp_option) :
         instance(instance), time_limit(time_limit), init_algo_name(std::move(init_algo_name)),
         replan_algo_name(replan_algo_name), neighbor_size(neighbor_size), num_of_iterations(num_of_iterations),
         screen(screen), path_table(instance.map_size),pipp_option(pipp_option), replan_time_limit(time_limit / 100)
{
    start_time = Time::now();
    destroy_weights.assign(3 * num_neighbor_sizes, 1);

    if (destory_name == "Adaptive")
    {
        ALNS = true;
        destroy_weights.assign(3 * num_neighbor_sizes, 1);
    }
    else if (destory_name == "RandomWalk")
        destroy_strategy = RANDOMWALK;
    else if (destory_name == "Intersection")
        destroy_strategy = INTERSECTION;
    else if (destory_name == "Random")
        destroy_strategy = RANDOMAGENTS;
    else if (destory_name == "RandomWalkProb")
        destroy_strategy = RANDOMWALKPROB;
    else
    {
        cerr << "Destroy heuristic " << destory_name << " does not exists. " << endl;
        exit(-1);
    }

    int N = instance.getDefaultNumberOfAgents();
    agents.reserve(N);
    for (int i = 0; i < N; i++)
        agents.emplace_back(instance, i);
    preprocessing_time = ((fsec)(Time::now() - start_time)).count();
    if (screen >= 2)
        cout << "Pre-processing time = " << preprocessing_time << " seconds." << endl;
}

bool LNS::run()
{
    // only for statistic analysis, and thus is not included in runtime
    sum_of_distances = 0;
    for (const auto & agent : agents)
    {
        sum_of_distances += agent.path_planner.my_heuristic[agent.path_planner.start_location];
    }

    initial_solution_runtime = 0;
    bool succ = false;
    int count = 0;
    start_time = Time::now();
    if (state_json == ""){
        while (!succ && initial_solution_runtime < time_limit)
        {

            succ = getInitialSolution();
            initial_solution_runtime = ((fsec)(Time::now() - start_time)).count();
            count++;
        }
    }
    else{
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
        succ = true;
        initial_sum_of_costs = neighbor.sum_of_costs;
        sum_of_costs = neighbor.sum_of_costs;
        initial_solution_runtime = ((fsec)(Time::now() - start_time)).count();
        cout << "load initial state from " << state_json << endl;
    }


    iteration_stats.emplace_back(neighbor.agents.size(),
                                 initial_sum_of_costs, initial_solution_runtime, init_algo_name);
    runtime = initial_solution_runtime;
    if (succ)
    {

        cout << "Initial solution cost = " << initial_sum_of_costs << ", "
             << "runtime = " << initial_solution_runtime << endl;
    }
    else
    {
        cout << "Failed to find an initial solution in "
             << runtime << " seconds and  " << count << " iterations" << endl;
        return false; // terminate because no initial solution is found
    }
    cout << "col " << instance.num_of_cols << " row " << instance.num_of_rows << endl;
    cout << "Initial State Start" << endl;
    for (int i = 0; i < agents.size(); i++){
        cout << "agent " << agents[i].id << " ";
        int t = 0;
        for (auto loc : agents[i].path){
            cout << "(" << t << "," << loc.location << ")" << " --> ";
            t += 1;
        }
        cout << endl;
    }
    cout << "Initial State End" << endl;

    bool improved = false;
    double lns_runtime = 0;
    double clipped_time = 0;
    auto removal_start = Time::now();
    double removal_time = 0;

    while (lns_runtime < time_limit or iteration_stats.size() <= num_of_iterations)
    {
        removal_time = 0;
        if (uniform_neighbor==1){ // sample from {2,4,8,16,32}
            neighbor_size =getRandomFromSetExp();
        }
        else if (uniform_neighbor==2){ // sample a random int from range 5 ~ 16
            neighbor_size =getRandomFromRange();
        }

        runtime =((fsec)(Time::now() - start_time)).count();
        if(screen >= 1)
            validateSolution();
        if (ALNS){
            removal_start = Time::now();
            chooseDestroyHeuristicbyALNS();
            removal_time +=  ((fsec)(Time::now() - removal_start)).count() ;
        }



        removal_start = Time::now();
        switch (destroy_strategy)
        {
            case RANDOMWALK:
                succ = generateNeighborByRandomWalk();
                break;
            case INTERSECTION:
                succ = generateNeighborByIntersection();
                break;
            case RANDOMWALKPROB:
                succ = generateNeighborByRandomWalkProbSelect();
                break;
            case RANDOMAGENTS:
                neighbor.agents.resize(agents.size());
                for (int i = 0; i < (int)agents.size(); i++)
                    neighbor.agents[i] = i;
                if (neighbor.agents.size() > neighbor_size)
                {
                    std::random_shuffle(neighbor.agents.begin(), neighbor.agents.end());
                    neighbor.agents.resize(neighbor_size);
                }
                succ = true;
                break;
            default:
                cerr << "Wrong neighbor generation strategy" << endl;
                exit(-1);
        }
        removal_time +=  ((fsec)(Time::now() - removal_start)).count() ;
        if(!succ)
            continue;

        // store the neighbor information
        neighbor.old_paths.resize(neighbor.agents.size());
        neighbor.old_sum_of_costs = 0;
        for (int i = 0; i < (int)neighbor.agents.size(); i++)
        {
            if (replan_algo_name == "PP")
                neighbor.old_paths[i] = agents[neighbor.agents[i]].path;
            path_table.deletePath(neighbor.agents[i], agents[neighbor.agents[i]].path);
            neighbor.old_sum_of_costs += agents[neighbor.agents[i]].path.size() - 1;
        }
        num_of_low_level = 0;
        auto replan_start_time = Time::now();
        if (replan_algo_name == "EECBS")
            succ = runEECBS();
        else if (replan_algo_name == "CBS")
            succ = runCBS();
        else if (replan_algo_name == "PP")
            succ = runPP();
        else
        {
            cerr << "Wrong replanning strategy" << endl;
            exit(-1);
        }
        if (succ) improved = true;

        auto replan_time = ((fsec)(Time::now() - replan_start_time)).count();
        if (replan_time > replan_time_limit){
            replan_time = replan_time_limit;
        }

        if (ALNS) // update destroy heuristics
        {
            removal_start = Time::now();
            if (neighbor.old_sum_of_costs > neighbor.sum_of_costs )
                destroy_weights[selected_neighbor] =
                        reaction_factor * (neighbor.old_sum_of_costs - neighbor.sum_of_costs) / neighbor.agents.size()
                        + (1 - reaction_factor) * destroy_weights[selected_neighbor];
            else
                destroy_weights[selected_neighbor] =
                        (1 - decay_factor) * destroy_weights[selected_neighbor];
            removal_time +=  ((fsec)(Time::now() - removal_start)).count() ;
        }

        if (iteration_stats.size() <= 2000 or  iteration_stats.size() % log_step == 0 or replan_time > replan_time_limit){
            cout << "num_of_low_level : " << num_of_low_level << " lns_runtime : " << lns_runtime << " replan_time : " <<  replan_time << " neighbor_size  : " << neighbor_size << " group_size : " << neighbor.agents.size() << " removal_time : " << removal_time ;
            if (ALNS){
                cout << " destroy_weights : " << destroy_weights[0] << " " << destroy_weights[1] << " " << destroy_weights[2];
            }
            cout << endl;

        }
        lns_runtime = lns_runtime + replan_time + removal_time;
        
        runtime = ((fsec)(Time::now() - start_time)).count();
        sum_of_costs += neighbor.sum_of_costs - neighbor.old_sum_of_costs;

        int sum_of_delay = 0;
        for (int i = 0; i < agents.size(); i++){
            sum_of_delay += agents[i].getNumOfDelays();
        }
        if (iteration_stats.size() <= 2000 or  iteration_stats.size() % log_step == 0 or replan_time > replan_time_limit){
            cout << "Iteration " << iteration_stats.size() << ", "
                 << "group size = " << neighbor.agents.size() << ", "
                 << "solution cost = " << sum_of_costs << ", sum_of_delay = " << sum_of_delay << endl;
        }
        iteration_stats.emplace_back(neighbor.agents.size(), sum_of_costs, runtime, replan_algo_name);
    }


    average_group_size = - iteration_stats.front().num_of_agents;
    for (const auto& data : iteration_stats)
        average_group_size += data.num_of_agents;
    if (average_group_size > 0)
        average_group_size /= (double)(iteration_stats.size() - 1);

    int sum_of_delay = 0;
    int num_no_delay = 0;
    for (int i = 0; i < agents.size(); i++){
        sum_of_delay += agents[i].getNumOfDelays();
        if (agents[i].getNumOfDelays() == 0){num_no_delay +=1;}
    }

    cout << getSolverName() << ": Iterations = " << iteration_stats.size() << ", "
         << "solution cost = " << sum_of_costs << ", "
         << "initial solution cost = " << initial_sum_of_costs << ", "
         << "lns_runtime = " << lns_runtime << ", "
         << "group size = " << average_group_size << ", "
         << "clipped_time = " << clipped_time << ", "
         << "failed iterations = " << num_of_failures << ", sum_of_delay = " << sum_of_delay << ", num_agents  = " << agents.size() << ", num_no_delay = " << num_no_delay << endl;
    return true;
}

bool LNS::getInitialSolution()
{
    neighbor.agents.resize(agents.size());
    for (int i = 0; i < (int)agents.size(); i++)
        neighbor.agents[i] = i;
    neighbor.old_sum_of_costs = MAX_COST;
    neighbor.sum_of_costs = 0;
    bool succ = false;
    if (init_algo_name == "EECBS")
        succ = runEECBS();
    else if (init_algo_name == "PP")
        succ = runPP();
    else if (init_algo_name == "PIBT")
        succ = runPIBT();
    else if (init_algo_name == "PPS")
        succ = runPPS();
    else if (init_algo_name == "winPIBT")
        succ = runWinPIBT();
    else if (init_algo_name == "CBS")
        succ = runCBS();
    else
    {
        cerr <<  "Initial MAPF solver " << init_algo_name << " does not exist!" << endl;
        exit(-1);
    }
    if (succ)
    {
        initial_sum_of_costs = neighbor.sum_of_costs;
        sum_of_costs = neighbor.sum_of_costs;
        return true;
    }
    else
    {
        return false;
    }

}

bool LNS::runEECBS()
{
    vector<SingleAgentSolver*> search_engines;
    search_engines.reserve(neighbor.agents.size());
    for (int i : neighbor.agents)
    {
        search_engines.push_back(&agents[i].path_planner);
    }

    ECBS ecbs(search_engines, path_table, screen - 1);
    ecbs.setPrioritizeConflicts(true);
    ecbs.setDisjointSplitting(false);
    ecbs.setBypass(true);
    ecbs.setRectangleReasoning(true);
    ecbs.setCorridorReasoning(true);
    ecbs.setHeuristicType(heuristics_type::WDG, heuristics_type::GLOBAL);
    ecbs.setTargetReasoning(true);
    ecbs.setMutexReasoning(false);
    ecbs.setConflictSelectionRule(conflict_selection::EARLIEST);
    ecbs.setNodeSelectionRule(node_selection::NODE_CONFLICTPAIRS);
    ecbs.setSavingStats(false);
    double w;
    if (iteration_stats.empty())
        w = 2; // initial run
    else
        w = 1.1; // replan
    ecbs.setHighLevelSolver(high_level_solver_type::EES, w);
    runtime = ((fsec)(Time::now() - start_time)).count();
    double T = time_limit - runtime;
    if (!iteration_stats.empty()) // replan
        T = min(T, replan_time_limit);
    bool succ = ecbs.solve(T, 0);
    num_of_low_level = ecbs.num_LL_expanded;
    if (succ && ecbs.solution_cost < neighbor.old_sum_of_costs) // accept new paths
    {
        auto id = neighbor.agents.begin();
        for (size_t i = 0; i < neighbor.agents.size(); i++)
        {
            agents[*id].path = *ecbs.paths[i];
            path_table.insertPath(agents[*id].id, agents[*id].path);
            ++id;
        }
        neighbor.sum_of_costs = ecbs.solution_cost;
        if (sum_of_costs_lowerbound < 0)
            sum_of_costs_lowerbound = ecbs.getLowerBound();
    }
    else // stick to old paths
    {
        if (!neighbor.old_paths.empty())
        {
            for (int id : neighbor.agents)
            {
                path_table.insertPath(agents[id].id, agents[id].path);
            }
            neighbor.sum_of_costs = neighbor.old_sum_of_costs;
        }
        if (!succ)
            num_of_failures++;
    }
    return succ;
}

bool LNS::runCBS()
{
    if (screen >= 2)
        cout << "old sum of costs = " << neighbor.old_sum_of_costs << endl;
    vector<SingleAgentSolver*> search_engines;
    search_engines.reserve(neighbor.agents.size());
    for (int i : neighbor.agents)
    {
        search_engines.push_back(&agents[i].path_planner);
    }

    CBS cbs(search_engines, path_table, screen - 1);
    cbs.setPrioritizeConflicts(true);
    cbs.setDisjointSplitting(false);
    cbs.setBypass(true);
    cbs.setRectangleReasoning(true);
    cbs.setCorridorReasoning(true);
    cbs.setHeuristicType(heuristics_type::WDG, heuristics_type::ZERO);
    cbs.setTargetReasoning(true);
    cbs.setMutexReasoning(false);
    cbs.setConflictSelectionRule(conflict_selection::EARLIEST);
    cbs.setNodeSelectionRule(node_selection::NODE_CONFLICTPAIRS);
    cbs.setSavingStats(false);
    cbs.setHighLevelSolver(high_level_solver_type::ASTAR, 1);
    runtime = ((fsec)(Time::now() - start_time)).count();
    double T = time_limit - runtime; // time limit
    if (!iteration_stats.empty()) // replan
        T = min(T, replan_time_limit);
    bool succ = cbs.solve(T, 0);
    num_of_low_level = cbs.num_LL_expanded;
    if (succ && cbs.solution_cost < neighbor.old_sum_of_costs) // accept new paths
    {
        auto id = neighbor.agents.begin();
        for (size_t i = 0; i < neighbor.agents.size(); i++)
        {
            agents[*id].path = *cbs.paths[i];
            path_table.insertPath(agents[*id].id, agents[*id].path);
            ++id;
        }
        neighbor.sum_of_costs = cbs.solution_cost;
        if (sum_of_costs_lowerbound < 0)
            sum_of_costs_lowerbound = cbs.getLowerBound();
    }
    else // stick to old paths
    {
        if (!neighbor.old_paths.empty())
        {
            for (int id : neighbor.agents)
            {
                path_table.insertPath(agents[id].id, agents[id].path);
            }
            neighbor.sum_of_costs = neighbor.old_sum_of_costs;

        }
        if (!succ)
            num_of_failures++;
    }
    return succ;
}

bool LNS::runPP()
{
    auto shuffled_agents = neighbor.agents;
    std::random_shuffle(shuffled_agents.begin(), shuffled_agents.end());
    if (screen >= 2) {
        for (auto id : shuffled_agents)
            cout << id << "(" << agents[id].path_planner.my_heuristic[agents[id].path_planner.start_location] <<
                "->" << agents[id].path.size() - 1 << "), ";
        cout << endl;
    }
    int remaining_agents = (int)shuffled_agents.size();
    auto p = shuffled_agents.begin();
    neighbor.sum_of_costs = 0;
    auto pp_start_time = Time::now();
    while (p != shuffled_agents.end() && ((fsec)(Time::now() - pp_start_time)).count() < replan_time_limit)
    {
        int id = *p;
        if (screen >= 3)

            cout << "Remaining agents = " << remaining_agents <<
                 ", remaining time = " << time_limit - runtime << " seconds. " << endl
                 << "Agent " << agents[id].id << endl;
        agents[id].path = agents[id].path_planner.findOptimalPath(path_table);
        num_of_low_level += agents[id].path_planner.num_expanded;
        if (agents[id].path.empty())
        {
            break;
        }
        neighbor.sum_of_costs += (int)agents[id].path.size() - 1;
        if (neighbor.sum_of_costs >= neighbor.old_sum_of_costs){
            break;
        }

        path_table.insertPath(agents[id].id, agents[id].path);
        remaining_agents--;
        ++p;

    }
    if (p == shuffled_agents.end() && neighbor.sum_of_costs < neighbor.old_sum_of_costs && ((fsec)(Time::now() - pp_start_time)).count() < replan_time_limit ) // accept new paths
    {
        return true;
    }
    else // stick to old paths
    {
        if (p != shuffled_agents.end())
            num_of_failures++;
        auto p2 = shuffled_agents.begin();
        while (p2 != p) // remove the planned paths
        {
            int a = *p2;
            path_table.deletePath(agents[a].id, agents[a].path);
            ++p2;
        }
        if (!neighbor.old_paths.empty())
        {
            p2 = neighbor.agents.begin();
            for (int i = 0; i < (int)neighbor.agents.size(); i++)
            {
                int a = *p2;
                agents[a].path = neighbor.old_paths[i];
                path_table.insertPath(agents[a].id, agents[a].path);
                ++p2;
            }
            neighbor.sum_of_costs = neighbor.old_sum_of_costs;
        }
        return false;
    }
}

bool LNS::runPPS(){
    auto shuffled_agents = neighbor.agents;
    std::random_shuffle(shuffled_agents.begin(), shuffled_agents.end());

    MAPF P = preparePIBTProblem(shuffled_agents);
    P.setTimestepLimit(pipp_option.timestepLimit);

    // seed for solver
    std::mt19937* MT_S = new std::mt19937(0);
    PPS solver(&P,MT_S);
    solver.setTimeLimit(time_limit);
    bool result = solver.solve();
    if (result)
        updatePIBTResult(P.getA(),shuffled_agents);
    return result;
}
bool LNS::runPIBT(){
    auto shuffled_agents = neighbor.agents;
     std::random_shuffle(shuffled_agents.begin(), shuffled_agents.end());

    MAPF P = preparePIBTProblem(shuffled_agents);

    // seed for solver
    std::mt19937* MT_S = new std::mt19937(0);
    PIBT solver(&P,MT_S);
    solver.setTimeLimit(time_limit);
    bool result = solver.solve();
    if (result)
        updatePIBTResult(P.getA(),shuffled_agents);
    return result;
}

bool LNS::runWinPIBT(){
    auto shuffled_agents = neighbor.agents;
    std::random_shuffle(shuffled_agents.begin(), shuffled_agents.end());

    MAPF P = preparePIBTProblem(shuffled_agents);
    P.setTimestepLimit(pipp_option.timestepLimit);

    // seed for solver
    std::mt19937* MT_S = new std::mt19937(0);
    winPIBT solver(&P,pipp_option.windowSize,pipp_option.winPIBTSoft,MT_S);
    solver.setTimeLimit(time_limit);
    bool result = solver.solve();
    if (result)
        updatePIBTResult(P.getA(),shuffled_agents);
    return result;
}

MAPF LNS::preparePIBTProblem(vector<int> shuffled_agents){

    // seed for problem and graph
    std::mt19937* MT_PG = new std::mt19937(0);

    Graph* G = new SimpleGrid(instance.getMapFile());

    std::vector<Task*> T;
    PIBT_Agents A;

    for (int i : shuffled_agents){
        assert(G->existNode(agents[i].path_planner.start_location));
        assert(G->existNode(agents[i].path_planner.goal_location));
        PIBT_Agent* a = new PIBT_Agent(G->getNode( agents[i].path_planner.start_location));

        A.push_back(a);
        Task* tau = new Task(G->getNode( agents[i].path_planner.goal_location));


        T.push_back(tau);
        if(screen>=5){
            cout<<"Agent "<<i<<" start: " <<a->getNode()->getPos()<<" goal: "<<tau->getG().front()->getPos()<<endl;
        }
    }

    return MAPF(G, A, T, MT_PG);

}

void LNS::updatePIBTResult(const PIBT_Agents& A,vector<int> shuffled_agents){
    int soc = 0;
    for (int i=0; i<A.size();i++){
        int a_id = shuffled_agents[i];

        agents[a_id].path.resize(A[i]->getHist().size());
        int last_goal_visit = 0;
        if(screen>=2)
            std::cout<<A[i]->logStr()<<std::endl;
        for (int n_index = 0; n_index < A[i]->getHist().size(); n_index++){
            auto n = A[i]->getHist()[n_index];
            agents[a_id].path[n_index] = PathEntry(n->v->getId());

            //record the last time agent reach the goal from a non-goal vertex.
            if(agents[a_id].path_planner.goal_location == n->v->getId() 
                && n_index - 1>=0
                && agents[a_id].path_planner.goal_location !=  agents[a_id].path[n_index - 1].location)
                last_goal_visit = n_index;

        }
        //resize to last goal visit time
        agents[a_id].path.resize(last_goal_visit + 1);
        if(screen>=2)
            std::cout<<" Length: "<< agents[a_id].path.size() <<std::endl;
        if(screen>=5){
            cout <<"Agent "<<a_id<<":";
            for (auto loc : agents[a_id].path){
                cout <<loc.location<<",";
            }
            cout<<endl;
        }
        path_table.insertPath(agents[a_id].id, agents[a_id].path);
        soc += agents[a_id].path.size()-1;
    }

    neighbor.sum_of_costs =soc;
}

void LNS::chooseDestroyHeuristicbyALNS()
{
    double sum = 0;
    for (const auto& h : destroy_weights)
        sum += h;
    if (screen >= 2)
    {
        cout << "destroy weights = ";
        for (const auto& h : destroy_weights)
            cout << h / sum << ",";
    }
    double r = (double) rand() / RAND_MAX;
    double threshold = destroy_weights[0];
    selected_neighbor = 0;
    while (threshold < r * sum)
    {
        selected_neighbor++;
        threshold += destroy_weights[selected_neighbor];
    }
    switch (selected_neighbor / num_neighbor_sizes)
    {
        case 0 : destroy_strategy = RANDOMWALK; break;
        case 1 : destroy_strategy = INTERSECTION; break;
        case 2 : destroy_strategy = RANDOMAGENTS; break;
        default : cerr << "ERROR" << endl; exit(-1);
    }
}

bool LNS::generateNeighborByIntersection(bool temporal)
{
    if (intersections.empty())
    {
        for (int i = 0; i < instance.map_size; i++)
        {
            if (!instance.isObstacle(i) && instance.getDegree(i) > 2)
                intersections.push_back(i);
        }
    }

    set<int> neighbors_set;
    auto pt = intersections.begin();
    std::advance(pt, rand() % intersections.size());
    int location = *pt;
    path_table.get_agents(neighbors_set, neighbor_size, location);
    if (neighbors_set.size() < neighbor_size)
    {
        set<int> closed;
        closed.insert(location);
        std::queue<int> open;
        open.push(location);
        while (!open.empty() && (int) neighbors_set.size() < neighbor_size)
        {
            int curr = open.front();
            open.pop();
            for (auto next : instance.getNeighbors(curr))
            {
                if (closed.count(next) > 0)
                    continue;
                open.push(next);
                closed.insert(next);
                if (instance.getDegree(next) >= 3)
                {
                    path_table.get_agents(neighbors_set, neighbor_size, next);
                    if ((int) neighbors_set.size() == neighbor_size)
                        break;
                }
            }
        }
    }
    neighbor.agents.assign(neighbors_set.begin(), neighbors_set.end());
    if (neighbor.agents.size() > neighbor_size)
    {
        std::random_shuffle(neighbor.agents.begin(), neighbor.agents.end());
        neighbor.agents.resize(neighbor_size);
    }
    if (screen >= 2)
        cout << "Generate " << neighbor.agents.size() << " neighbors by intersection " << location << endl;
    return true;
}

bool LNS::generateNeighborByRandomWalk()
{

    if (neighbor_size >= (int)agents.size())
    {
        neighbor.agents.resize(agents.size());
        for (int i = 0; i < (int)agents.size(); i++)
            neighbor.agents[i] = i;
        return true;
    }

    int a = findMostDelayedAgent();
    if (a < 0)
        return false;

    set<int> neighbors_set;
    neighbors_set.insert(a);
    int count = 0;
    while (neighbors_set.size() < neighbor_size && count < 10)
    {
        int t = rand() % agents[a].path.size();
        randomWalk(a, agents[a].path[t].location, t, neighbors_set, neighbor_size, (int) agents[a].path.size() - 1);
        count++;
        // select the next agent randomly
        int idx = rand() % neighbors_set.size();
        int i = 0;
        for (auto n : neighbors_set)
        {
            if (i == idx)
            {
                a = n; 
                break;
            }
            i++;
        }
    }
    if (neighbors_set.size() < 2)
        return false;
    neighbor.agents.assign(neighbors_set.begin(), neighbors_set.end());
    if (screen >= 2)
        cout << "Generate " << neighbor.agents.size() << " neighbors by random walks of agent " << a
             << "(" << agents[a].path_planner.my_heuristic[agents[a].path_planner.start_location]
             << "->" << agents[a].path.size() - 1 << ")" << endl;

    return true;
}

bool LNS::generateNeighborByRandomWalkProbSelect()
{
    if (neighbor_size >= (int)agents.size())
    {
        neighbor.agents.resize(agents.size());
        for (int i = 0; i < (int)agents.size(); i++)
            neighbor.agents[i] = i;
        return true;
    }

    // clear and init delayed_agents and delay_list before each lns iteration
    delayed_agents.clear();
    delay_list.clear();
    int max_delay = 0;
    for (int i = 0; i < agents.size(); i++)
    {
        int agent_delay = agents[i].getNumOfDelays();
        if (agent_delay > max_delay) max_delay = agent_delay;
        if (agent_delay > 0){
            delayed_agents.push_back(i);
            // if the agent is selected in previous rounds, discount it by tabu_discount
            if (tabu_list.find(i) != tabu_list.end()){
                delay_list.push_back(agent_delay*tabu_discount);
            }else{
                delay_list.push_back(agent_delay);
            }
        }
    }


    set<int> neighbors_set;
    int count = 0;
    while (neighbors_set.size() < neighbor_size && count < 10)
    {
        int a = findAgentBasedOnDelay();
        if (a < 0)
            return false;

        int t = rand() % agents[a].path.size();
        randomWalk(a, agents[a].path[t].location, t, neighbors_set, neighbor_size, (int) agents[a].path.size() - 1);
        count++;

    }
    if (neighbors_set.size() < 2)
        return false;
    neighbor.agents.assign(neighbors_set.begin(), neighbors_set.end());


    return true;
}

int LNS::findMostDelayedAgent()
{
    int a = -1;
    int max_delays = -1;
    for (int i = 0; i < agents.size(); i++)
    {
        if (tabu_list.find(i) != tabu_list.end())
            continue;
        int delays = agents[i].getNumOfDelays();
        if (max_delays < delays)
        {
            a = i;
            max_delays = delays;
        }
    }
    if (max_delays == 0)
    {
        tabu_list.clear();
        return -1;
    }
    tabu_list.insert(a);
    if (tabu_list.size() == agents.size())
        tabu_list.clear();
    return a;
}


int LNS::findAgentBasedOnDelay() {
    // Ensure both vectors are of the same size
    if (delayed_agents.size() != delay_list.size()) {
        throw std::invalid_argument("Vectors must be of the same size.");
    }

    // Compute the cumulative sum of delay_list
    std::vector<int> cumulative_weights(delay_list.size());
    std::partial_sum(delay_list.begin(), delay_list.end(), cumulative_weights.begin());

    // Generate a random number in the range [0, total weight)
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> distrib(0, cumulative_weights.back() - 1);

    int random_number = distrib(gen);

    // Find the index where random_number fits in cumulative_weights
    auto it = std::lower_bound(cumulative_weights.begin(), cumulative_weights.end(), random_number);

    // Calculate the index based on the iterator
    int index = std::distance(cumulative_weights.begin(), it);

    // Return the corresponding element from delayed_agents
    tabu_list.insert(delayed_agents[index]);
    delay_list[index] *= tabu_discount; // if selected in the current LNS round, discount it again
    return delayed_agents[index];
}

int LNS::findRandomAgent() const
{
    int a = 0;
    int pt = rand() % (sum_of_costs - sum_of_distances) + 1;
    int sum = 0;
    for (; a < (int) agents.size(); a++)
    {
        sum += agents[a].getNumOfDelays();
        if (sum >= pt)
            break;
    }
    assert(sum >= pt);
    assert(sum >= pt);
    return a;
}

// a random walk with path that is shorter than upperbound and has conflicting with neighbor_size agents
void LNS::randomWalk(int agent_id, int start_location, int start_timestep,
                     set<int>& conflicting_agents, int neighbor_size, int upperbound)
{


    int loc = start_location;
    for (int t = start_timestep; t < upperbound; t++)
    {
        auto next_locs = instance.getNeighbors(loc);
        next_locs.push_back(loc);
        while (!next_locs.empty())
        {
            int step = rand() % next_locs.size();
            auto it = next_locs.begin();
            advance(it, step);
            int next_h_val = agents[agent_id].path_planner.my_heuristic[*it];
            if (t + 1 + next_h_val < upperbound) // move to this location
            {
                path_table.getConflictingAgents(agent_id, conflicting_agents, loc, *it, t + 1);
                loc = *it;
                break;
            }
            next_locs.erase(it);
        }
        if (next_locs.empty() || conflicting_agents.size() >= neighbor_size)
            break;
    }
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

void LNS::writeIterStatsToFile(string file_name) const
{
    std::ofstream output;
    output.open(file_name);
    // header
    output << "num of agents," <<
           "sum of costs," <<
           "runtime," <<
           "cost lowerbound," <<
           "sum of distances," <<
           "MAPF algorithm" << endl;

    for (const auto &data : iteration_stats)
    {
        output << data.num_of_agents << "," <<
               data.sum_of_costs << "," <<
               data.runtime << "," <<
               max(sum_of_costs_lowerbound, sum_of_distances) << "," <<
               sum_of_distances << "," <<
               data.algorithm << endl;
    }
    output.close();
}

void LNS::writeResultToFile(string file_name) const
{
    std::ifstream infile(file_name);
    bool exist = infile.good();
    infile.close();
    if (!exist)
    {
        ofstream addHeads(file_name);
        addHeads << "runtime,solution cost,initial solution cost,min f value,root g value," <<
                 "iterations," <<
                 "group size," <<
                 "runtime of initial solution,area under curve," <<
                 "preprocessing runtime,solver name,instance name" << endl;
        addHeads.close();
    }
    ofstream stats(file_name, std::ios::app);
    double auc = 0;
    if (!iteration_stats.empty())
    {
        auto prev = iteration_stats.begin();
        auto curr = prev;
        ++curr;
        while (curr != iteration_stats.end() && curr->runtime < time_limit)
        {
            auc += (prev->sum_of_costs - sum_of_distances) * (curr->runtime - prev->runtime);
            prev = curr;
            ++curr;
        }
        auc += (prev->sum_of_costs - sum_of_distances) * (time_limit - prev->runtime);
    }
    stats << runtime << "," << sum_of_costs << "," << initial_sum_of_costs << "," <<
            max(sum_of_distances, sum_of_costs_lowerbound) << "," << sum_of_distances << "," <<
            iteration_stats.size() << "," << average_group_size << "," <<
            initial_solution_runtime << "," << auc << "," <<
            preprocessing_time << "," << getSolverName() << "," << instance.getInstanceName() << endl;
    stats.close();
}

void LNS::writePathsToFile(string file_name) const
{
    std::ofstream output;
    output.open(file_name);
    // header
    output << agents.size() << endl;

    for (const auto &agent : agents)
    {
        for (const auto &state : agent.path)
            output << state.location << ",";
        output << endl;
    }
    output.close();
}