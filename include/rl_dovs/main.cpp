/*
 * main.cpp
 * Author: Andrew
 */

#include <iostream>
#include <fstream>
#include "Simulation.h"
#include <boost/program_options.hpp>
#include <random>
#include "State.h"


namespace po = boost::program_options;

int main (int argc, char *argv[]) {

	std::string library = "", nom_fich = "", scenario = "", fileQTable;
	int lookAhead, rca, algorithm, timeHorizon, num_scen = 1, numEpisodes, actives, passives;
    double epsilon, alpha, fixed, epsilonDiscount, gamma;
    bool accConst, interactive, colision = false, unreachable = false, debug, graph, readQTable, manual, test, 
         video, learning, random;

    // Program command line options
    po::options_description desc("Options");
    desc.add_options()("help,h", "Display help")
    ("graph,g", po::bool_switch(&graph)->default_value(false), "Display visualizations")
    ("library,l", po::value<std::string>(&library)->default_value("dislin"), "Change graphics library")
    ("file,f", po::value<std::string>(&nom_fich)->default_value(""), "Load scenario in file")
    ("lookahead,o", po::value<int>(&lookAhead)->default_value(0),"Define lookahead")
    ("rca,r", po::value<int>(&rca)->default_value(0), 
            "Determine if the running algorithm should be cooperative or not")
    ("algorithm,a", po::value<int>(&algorithm)->default_value(1), 
            "Algorithm used in simulation: 0=Greedy, 1=Strategy, 2=A*")
    ("timeHorizon,t", po::value<int>(&timeHorizon)->default_value(0), "Specify time horizon")
    ("accConst,c", po::value<bool>(&accConst)->default_value(0), "Consider dynamic constraints")
    ("interactive,i", po::bool_switch(&interactive)->default_value(false), 
            "Interactive mode, simulation runs frame by frame")
    ("debug,d", po::bool_switch(&debug)->default_value(false), 
            "Debug mode, display additional information on screen")
    ("readQTable", po::bool_switch(&readQTable)->default_value(false), 
            "Read Q-Table from file instead of train")
    ("qTableFile", po::value<std::string>(&fileQTable)->default_value("qTable.txt"), 
            "File where the Q-Table is read/saved")
    ("episodes,e", po::value<int>(&numEpisodes)->default_value(100), "Number of episodes to train")
    ("epsilon", po::value<double>(&epsilon)->default_value(1.0), "Initial epsilon (exploration rate) value")
    ("manual,m", po::bool_switch(&manual)->default_value(false), "Manual learning")
    ("test", po::bool_switch(&test)->default_value(false), "Test the learnt QTable")
    ("video,v", po::bool_switch(&video)->default_value(false), "Save episode to video")
    ("learning", po::bool_switch(&learning)->default_value(false), "Use reinforcement learning")
    ("random", po::bool_switch(&random)->default_value(false), "Random scenarios")
    ("createRandomScenario", po::value<std::string>(&scenario)->default_value(""), 
            "Create random scenario and save to file nom_fich.json")
    ("actives", po::value<int>(&actives)->default_value(1), 
            "Number of active agents in random scenario")
    ("passives", po::value<int>(&passives)->default_value(20), 
            "Number of passive agents in random scenario")
    ("fixed", po::value<double>(&fixed)->default_value(0.0),
            "Porcentage of random scenarios with fixed obstacles")
    ("alpha", po::value<double>(&alpha)->default_value(0.01),
            "Value of learning rate")
    ("epsilonDiscount", po::value<double>(&epsilonDiscount)->default_value(0.0025),
            "Value discounted from exploration rate in each episode")
    ("gamma", po::value<double>(&gamma)->default_value(0.5),
            "Value of discount rate");


    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    if (vm.count("help")) {
        std::cout << desc << "\n";
        return 1;
    }

	Simulation sim;

    double totalTime = 0.0;
    std::ofstream f("logs/totalTime.dat", std::ios::trunc);

    std::ofstream uValues("logs/uValues.txt", std::ios::trunc);
    std::ofstream metrics("logs/metrics.csv", std::ios::trunc);
    std::ofstream learningValues("logs/learning.csv", std::ios::trunc);

    clock_t tstart;
    int fails = 0, badScenarios = 0, tookToLongs = 0, totalIterations = 0, succeses = 0;
    bool success;

    int numNewStates = 0;

    bool chosenDirToGoal;

    if (learning) {     // Q-Learning algorithm

        // Variable initialization
        // Number of possible actions
        const int numActions = 8;

        // Number of episodes
        int episodes = 0;

        // State frequency
        std::map<State, int> stateFrequencies;
        std::map<State, int>::iterator stateFrequency;

        // Q-Table
        std::map<State, std::array<double, numActions>> qTable;
        std::map<State, std::array<double, numActions>>::iterator qValue;
        std::map<State, std::array<double, numActions>>::iterator qValueNew;
        
        // Load Q-Table from file if necessary
        if (readQTable) {
            epsilon = 0;
            sim.readQTable(qTable, fileQTable);
        }

        // Epsilon and exploration random number generator
        std::random_device rd{};
        std::mt19937 generator{rd()};
        std::uniform_real_distribution<double> distributionEpsilon;
        std::uniform_real_distribution<double>::param_type parEpsilon(0, 1);
        distributionEpsilon.param(parEpsilon);
        std::uniform_real_distribution<double> distributionExplore;

        // Actions
        std::vector<std::vector<std::pair<int, Velocidad>>> validActions;
        std::vector<int> numValidActions;
        std::vector<Velocidad> actions;
        std::vector<Velocidad> oldActions;
        int numAction, zone;
        std::vector<int> numChosenActions = std::vector<int>();
        double max;

        // State and new state
        std::vector<State>  states, newStates;

        // Reward
        std::vector<double> rewards = std::vector<double>();
        double rewardModifier = 0;

        tstart = clock();

        // Repeat Q-Learning algorithm for the selected amount of episodes
        while (episodes < numEpisodes) {

            // Environment set up
            std::cout << std::endl << "---> Episode: " << episodes << "\tEpsilon: " << epsilon 
                      << "\tAlpha: " << alpha << std::endl;
            learningValues << "---> Episode " << episodes << std::endl;

            clock_t tEpisodeStart = clock();

            std::cout << "Setting the environment..." << std::endl;
            sim.SetEnvironmentRL(graph, library, debug, video, nom_fich, actives, passives, fixed);

            std::cout << std::endl << "Starting simulation" << std::endl << std::endl;
            
            int iteration = 0, nextIteration = -1 ;
            bool finished = false;

            // Initialize variables
            actions.clear();
            std::vector<bool> chosenDirToGoal = std::vector<bool>();
            states.clear();
            newStates.clear();
            rewards.clear();
            rewardModifier = 0;

            // Model initial environment and get valid actions
            sim.ModelEnvironment(lookAhead, timeHorizon, accConst);
            validActions = sim.getActions(lookAhead, zone);
            

            // Get current state
            states = sim.getStates(zone, true);
            bool col = true;

            stateFrequencies.clear();
            success = false;

            while (!finished) {

                numChosenActions.clear();
                chosenDirToGoal.clear();
                oldActions = actions;
                actions.clear();
                numAction = 0;
                
                // For each state of active agents, explore or exploit
                for (int k=0; k < states.size(); k++) {
                    
                    // Update state frequency
                    stateFrequency = stateFrequencies.find(states[k]);
                    if (stateFrequency == stateFrequencies.end()) {
                        stateFrequencies.insert(std::pair<State, int> (states[k], 1));
                    } else {
                        stateFrequency->second++;
                    }

                    // Find the Q-Values of the state, if they exist
                    qValue = qTable.find(states[k]);
                    if (qValue == qTable.end()) { // The q value does not exist
                        numNewStates++;
                        std::array<double, numActions> newValues;
                        qTable.insert(std::pair<State, std::array<double, numActions>>(states[k], newValues));
                        qValue = qTable.find(states[k]);
                    }

                    if (!manual) {      // Non-manual mode, explore or exploit

                        // Explore or exploit to choose action
                        if (distributionEpsilon(generator) < epsilon) { //Explore
                            std::uniform_real_distribution<double>::param_type 
                                    parExplore(0, validActions[k].size()-0.000000001);
                            distributionExplore.param(parExplore);
                            numAction = floor(distributionExplore(generator));
                        } else { // Exploit
                            max = -INF;
                            for (int i=0; i<validActions[k].size(); i++) {
                                if (qValue->second[validActions[k][i].first] > max ) {
                                    max = qValue->second[validActions[k][i].first];
                                    numAction = i;
                                }
                            }
                        }
                    } else {            // Manual mode, user chooses action
                        numAction = -1;
                        char c;
                        int  a = -1;
                        while (numAction == -1) {
                            std::cout << "Choose action: ";
                            std::cin >> c;
                            if (c == 'a') {
                                a = 2;
                            } else if (c == 's') {
                                a = 1;
                            } else if (c == 'w') {
                                a = 3;
                            } else if (c == 'd') {
                                a = 4;
                            } else if (c == 'e') {
                                a = 0;
                            } else if (c == '1') {
                                a = 5;
                            } else if (c == '2') {
                                a = 6;
                            } else if (c == '3') {
                                a = 7;
                            }

                            for (int l=0; l < validActions[k].size(); l++) {
                                if (a == validActions[k][l].first) {
                                    numAction = l;
                                }
                            }
                        }
                    }

                    // Double check limits on chosen action
                    if (validActions[k][numAction].second.v >= 0 
                        && validActions[k][numAction].second.v <= 1.5 
                        && validActions[k][numAction].second.w >= -1 
                        && validActions[k][numAction].second.w <= 1)
                        
                        actions.push_back(validActions[k][numAction].second);
                    else
                        actions.push_back(oldActions[k]);

                    // Check if the chosen action leads to goal
                    if (validActions[k][numAction].first > 4) {
                        chosenDirToGoal.push_back(true);
                    } else {
                        chosenDirToGoal.push_back(false);
                    }

                    // Store chosen action
                    numChosenActions.push_back(validActions[k][numAction].first);
                }

                // If in interactive mode, display menu
                if (interactive && iteration >= nextIteration) {
                    while (1) {
                        std::cout << std::endl 
                                  << "Press n to continue to next iteration or a to advance to an iteration" 
                                  << std::endl;
                        std::string option;
                        std::cin >> option;
                        if (option == "n")
                            break;

                        if (option == "a") {
                            std::cout << "Enter iteration number: ";
                            std::cin >> option;
                            nextIteration = atoi(option.c_str());
                            break;
                        }
                    }
                }

                // Apply action
                sim.executeActions(actions);

                unreachable = false; colision = false;

                // Execute action
                if (!sim.UpdateCycle(unreachable)) {
                    finished = true;
                    if (!unreachable) {         // There has been a collision
                            colision = true;
                            rewardModifier = -10;
                            if (iteration >= 50)
                                fails++;
                            else badScenarios++;
                            std::cout << "FAIL: COLLISION" << std::endl;
                    } else {                    // Maximum number of iterations exceeded
                        rewardModifier = -10;
                        tookToLongs++;
                        std::cout << "FAIL: TOOK TO LONG" << std::endl;
                    }
                }

                if (sim.IsFinished() && !colision && !unreachable) {    // Simulation has finished
                    finished = true;
                    if (iteration < 500) {          // Agent reached goal
                        std::cout << "OK: " << iteration << " iterations" << std::endl;
                        totalIterations += iteration;
                        succeses++;
                        success = true;
                        rewardModifier = 10;
                    } else {                        // Maximum number of iterations exceeded
                        tookToLongs++;
                        std::cout << "FAIL: TOOK TO LONG" << std::endl;
                        rewardModifier = -10;
                    }
                }


                // Model environment and get valid actions
                sim.ModelEnvironment(lookAhead, timeHorizon, accConst);
                
    
                for (auto validAc: validActions) {
                    if (validAc.size() == 0 && !success) {          // Agent reached goal
                        totalIterations += iteration;
                        succeses++;
                        std::cout << "OK: " << iteration << " iterations" << std::endl;
                        finished = true;
                    }
                }
                
                // Get new state
                newStates = sim.getStates(zone, true);

                // Obtain reward based on current state
                rewards = sim.getRewards(states, chosenDirToGoal);
                
                // If agent has finished, swap reward with the termination reward
                if (rewardModifier != 0) {
                    for (int k=0; k<rewards.size(); k++){
                        rewards[k] = rewardModifier;
                    }
                }
                
                // If the agent has gone too far, end simulation
                for (int k=0; k<newStates.size(); k++) {
                    if (newStates[k].getDistanceToGoal() == 4) {
                        std::cout << "FAIL: TOO FAR" << std::endl;
                        finished = true;
                        rewards[k] = -10;
                    }
                }
                
                // Update Q-Table for each state-action pair
                for (int k=0; k<newStates.size(); k++) {

                    learningValues << rewards[k] << ", ";

                    if (!test) {        // When testing, don't update Q-Table
                        // Update Q-Table
                        qValueNew = qTable.find(newStates[k]);
                        if (qValueNew != qTable.end()) {    // The new state exists
                            max = -INF;
                            for (int i=0; i<numActions; i++) {      // Obtain maximum Q-Value of new state
                                if (qValueNew->second[i] > max) {
                                    max = qValueNew->second[i];
                                }
                            }
                            qValue->second[numChosenActions[k]] = qValue->second[numChosenActions[k]] 
                                    + alpha*(rewards[k] + gamma*max - qValue->second[numChosenActions[k]]);
                        } else {    // The new state doesn't exist
                            qValue->second[numChosenActions[k]] = qValue->second[numChosenActions[k]] 
                                    + alpha*(rewards[k] - qValue->second[numChosenActions[k]]);
                        }
                    }

                    // Update state
                    states[k] = newStates[k];
                }

                learningValues << std::endl;

                validActions.clear();
                validActions = sim.getActions(lookAhead, zone);

                clock_t tfinish = clock();
                totalTime += ((double)(tfinish-tstart))/CLOCKS_PER_SEC;
                f << ((double)(tfinish-tstart))/CLOCKS_PER_SEC << std::endl;
                iteration++;
            }
            sim.ClearAgents();

            // Store utilty of each state in log
            for (qValue = qTable.begin(); qValue != qTable.end(); qValue++) {
                max = -INF;
                for (int i=0; i < numActions; i++) {
                    if (qValue->second[i] > max) {
                        max = qValue->second[i];
                    }
                }
                uValues << qValue->first.getCode() << "\t" << max << "\t" << episodes << std::endl;
            }

            metrics << episodes << ", " << qTable.size() << ", " << numNewStates << ", " 
                    << success << ", " << iteration << std::endl;

            clock_t tEpisodeEnd = clock();
            std::cout << "Training time for episode: " << (tEpisodeEnd-tEpisodeStart)/CLOCKS_PER_SEC 
                      << " seconds" << std::endl;
            std::cout << "Total training time at this moment: " << (tEpisodeEnd-tstart)/CLOCKS_PER_SEC/60 
                      << " minutes" << std::endl;
            std::cout << "Number of new states: " << numNewStates << std::endl;
            std::cout << "Number of states at the moment: " << qTable.size() << std::endl;
            numNewStates = 0;
            episodes++;

            if (manual) {
                std::string op;
                std::cout << "Finish [y/N]? ";
                std::cin >> op;
                if (op == "Y") {
                    episodes = numEpisodes + 1;
                } else {
                    numEpisodes = episodes + 1;
                }
            }

            // Reduce exploration rate
            epsilon = epsilon-epsilonDiscount > 0 ? epsilon-epsilonDiscount: 0;
        }

        // Write Q-Table to file when finished training
        sim.writeQTable(qTable, "outQ.txt");

    } else {    // DOVS model

        int episodes = 0;
        bool finished = false;
        int nextIteration = 0;
        int iteration = 0;
        tstart = clock();

        while (episodes < numEpisodes) {

            unreachable = false;
            std::cout << "---> Episode " << episodes << std::endl;

            // Set environment
            if (random)
                sim.SetEnvironmentRL(graph, library, debug, video, nom_fich, actives, passives, fixed);
            else 
                sim.SetEnvironment(graph, library, debug);
            
            while (!finished) {

                // Model environment and compute motion to apply
                sim.ModelEnvironment(lookAhead, timeHorizon, accConst);
                sim.ComputeMotion(lookAhead, algorithm);

                // Apply velocities
                if (!sim.UpdateCycle(unreachable)) {
                    finished = true;
                    if (!unreachable) {
                        std::cout << "Iterations: " << iteration << std::endl;
                        colision = true;
                        if (iteration >= 50) {
                            fails++;
                        } else {
                            badScenarios++;
                        }
                    } else {
                        tookToLongs++;
                    }
                }

                if (sim.IsFinished() && !finished) {
                    if (iteration < 500) {  // Agent reached goal
                        totalIterations += iteration;
                        succeses++;
                    } else {    // Maximum number of iterations exceeded
                        tookToLongs++;
                    }

                    finished = true;
                }

                // If interactive, displya menu
                if (interactive && iteration >= nextIteration) {
                    while (1) {
                        std::cout << std::endl 
                            << "Press n to continue to next iteration or a to advance to an iteration" << std::endl;
                        std::string option;
                        std::cin >> option;
                        if (option == "n")
                            break;

                        if (option == "a") {
                            std::cout << "Enter iteration number: ";
                            std::cin >> option;
                            nextIteration = atoi(option.c_str());
                            break;
                        }
                    }
                }
                iteration++;
            }

            sim.ClearAgents();
            iteration = 0;
            finished = false;
            episodes++;
        }
    }

    // Save simulation to video if chosen
    if (video)
        sim.createVideo();

    f.close();
    uValues.close();
    metrics.close();
    std::cout << "--->RESULTS:" << std::endl;
    std::cout << "* Number of episodes: " << numEpisodes << std::endl;
    std::cout << "* Number of successes: " << succeses << std::endl;
    std::cout << "* Number of colisions: " << fails << std::endl;
    std::cout << "* Number of took to longs: " << tookToLongs << std::endl;
    std::cout << "* Number of bad scenarios: " << badScenarios << std::endl;
    std::cout << "* Success rate: " << (float) ((float)succeses/(numEpisodes-badScenarios-tookToLongs))*100 
              << "%" << std::endl;
    std::cout << "* Average iterations in successes: " << (float) totalIterations/succeses << std::endl;
    clock_t tend = clock();
    std::cout << "Training time: " << (tend-tstart)/CLOCKS_PER_SEC << " seconds (" 
              << (tend-tstart)/CLOCKS_PER_SEC/60 << " minutes)" << std::endl;

    std::ofstream flog;
    flog.open("logs/logReciprocal.dat", std::ios::app);
    if (succeses == 1) {
        flog << std::endl <<"SUCCESS" << std::endl;
        std::cout << "SUCESS" << std::endl;
    } else {
        flog << std::endl << "FAILURE" << std::endl;
        std::cout << std::endl << "FAILURE" << std::endl;
    }
    flog.close();
    learningValues.close();

    if (colision) return 1;
	return 0;
}
