/*
 * simulacion.h
 *
 *  Created on: 14/04/2015
 *      Author: maite
 */

#ifndef SIMULATION_H_
#define SIMULATION_H_

#include <random>
#include <cstdlib>
#include <eigen3/Eigen/LU>

// andrew RL
#include "State.h"

#include "ActiveAgent.h";
#include "PassiveAgent.h"

//#include "Planner3D.h"

//#include <eigen3/Eigen/Eigen>


class Simulation{
private:

	//std::vector<std::shared_ptr<Agent>> agents;
	static std::vector<std::unique_ptr<Agent>> agents;
	//boundsVS bounds;

	double time_step;
	static double min_dist;	//minimum distance to goal

	//Graphs
	bool graph;
	const char *library;

	// Variables for scenario creation.
	static bool insertActive;
	static bool quit;
	static bool finished;
	static bool start;
	static bool insertRandom;
	static int passives;
	static int display_xmax;
	static int display_xmin;
	static int display_ymax;
	static int display_ymin;
	bool video;

	// Debug mode
	bool debug;

	// andrew RL
	std::vector<double> distLastState;
	std::vector<double> distCurrentState;
	std::vector<double> thetaLastState;
	std::vector<double> thetaCurrentState;
	std::vector<double> distObsLastState;
	std::vector<double> distObsCurrentState;
	std::vector<double> vLastState;
	std::vector<double> vCurrentState;
	std::vector<double> wLastState;
	std::vector<double> wCurrentState;
	std::vector<int> currentClosestAgent;
	std::vector<int> lastClosestAgent;
	std::vector<double> thetaObsCurrentState;
	std::vector<double> thetaObsLastState;
	std::vector<double> relThetaCurrentState;
	std::vector<double> relThetaLastState;
	std::vector<int> lastCollision;
	std::vector<int> currentCollision;
	std::vector<int> lastCollisionAgent;
	std::vector<int> currentCollisionAgent;
	std::vector<double> thetaObsOriginal;
	std::vector<bool> freeActions;

	static std::mt19937 generator;
	static std::uniform_real_distribution<double> distributionX;
	static std::uniform_real_distribution<double> distributionY;
	static std::uniform_real_distribution<double> distributionTita;
	static std::uniform_real_distribution<double> distributionV;
	static std::uniform_real_distribution<double> distributionW;
	static std::uniform_real_distribution<double> distributionFixed;

    void GenerateStaticObstacles(int id_number);
    void GenerateDynamicObstacles();

    bool CheckCollision();

public:

	Simulation();
	~Simulation();

	void SetEnvironment(bool g, const char *library, const char *filename, int &num_scen, int itScen);
	void SetEnvironment(bool graph, std::string library, bool debug);
	void createRandomScenario(int actives, int passives, double fixed);
	void SetEnvironmentRL(bool graph, std::string library, bool debug, bool video, std::string nomFich, 
						  int actives, int passives, double fixed);

	static void calibrate();
	static void createScenario(bool graph);
	static void menuCreateScenario();
	static void translateCoordinates(int *x_in, int *y_in, double *x_out, double *y_out);
	void writeScenarioJSON(std::string scenario);
	void readScenarioJSON(std::string name);
	static bool isNumber(char c);
	void createVideo();
	static void PlotWorldEnvironmentStatic();


	//Functions to modify the location and velocity of the agents
	void AddAgent(std::unique_ptr<Agent> ag);
	void ClearAgents(){ agents.clear(); };
	Agent& GetAgent(const int id);

    void WriteMetricsLog(double cost, const char* name);

    //CLASS SIMULATION: ¿unificar con la función SetEnvironment?
    void GenerateNewScenario(bool new_scen);
    void SetEnvironmentSimulation(int g, const char* lib, const char* filename, int& num_scen, const char* fileDOVTS);

    //CLASS SIMULATION
    bool IsFinished();
    bool UpdateCycle(bool &unreachable);
    void ModelEnvironment(const int i, const int th, const bool accConst);    //void ModelEnvironment_RS(); void ModelEnvironment_VS();
    void ComputeMotion(int lookAhead, const int algorithm);
	void MotionShared(const bool rca, const int algorithm);
	Velocidad ComputeBestMotion(Velocidad current, constraints c, Eigen::Matrix<double,2,2> values);
    //Plots
    void PlotWorldEnvironment(bool video=false);
    void PlotRobocentricEnvironment();
    void PlotVSEnvironment();
    void PlotAnalysisVS();

    //CLASS AGENT
    //Functions for the 3D planner: DOVTS + specific planner: RRT*, A*...
    //void SetPlanner3d(const char* filename);
    //void ModelDOVTS();
    //void PlanDOVTS();
    //bool ConsiderObstacleVS(Obstaculo* obs);

	// andrew RL
	std::vector<std::vector<std::pair<int, Velocidad>>> getActions(int lookAhead, int &zone);
	std::vector<State> getStates(int zone, bool freeDirToGoal);
	std::vector<double> getRewards(std::vector<State> lastState, std::vector<bool> chosenDirToGoal);
	void executeActions(std::vector<Velocidad> action);
	void writeQTable(std::map<State, std::array<double, 8>> qTable, std::string fileName);
	void readQTable(std::map<State, std::array<double, 8>> &qTable, std::string fileName);

};
#endif /* SIMULATION_H_ */
