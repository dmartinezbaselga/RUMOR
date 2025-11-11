#ifndef AGENT_H
#define AGENT_H

#include "TData.h"
#include <iostream>
#include <fstream>
#include <vector>

#include "dovts.h"

class Root;

class Agent{
    private:
        int id;     //agent identifier

        double real_radius;
        double apparent_radius;

    protected:
        Tsc loc;    //initial localization (world coordinates)
        double v,w;
        double av, aw;
        double v0, w0;
        double distance_segment;
        bool graph, segment;
        Tpf first_point, last_point;

	public:
        bool active;

        Agent(){};
        Agent(int i, double x, double y, double tita, double v, double w, double r, bool active, bool graph=true,
        bool segment = false, double distance_segment = 0, Tpf first_point = Tpf(), Tpf last_point = Tpf());
        virtual ~Agent();

        //2D planning with greedy algorithm
        virtual Velocidad MotionGreedy(double time_step, std::vector<std::unique_ptr<Agent>> *agents, bool video=false){ 
            std::cout << "Motion for passive agent " << id << ", returning v=" << v << " and w=" << w << std::endl;
            return {v,w}; 
        }

        //3D planning
        virtual Velocidad
        MotionPlan(double time_step, double time_horizon, unsigned ndata, unsigned mdata, unsigned long numC,
                           int lookAhead) = 0;
        //2D planning with heuristic strategies
        virtual Velocidad MotionStrategy(double time_step, std::vector<std::unique_ptr<Agent>> *agents, bool video=false, int iteracion = 0, bool graph=true, bool debug=false) {
            return {v,w};
        };

        // andrew RL
        virtual std::vector<std::pair<int, Velocidad>> MotionStrategyRL(double time_step, std::vector<std::unique_ptr<Agent>> *agents, bool &freeActions,  bool video=false, int iteracion = 0, bool graph=true, bool debug=false) {
            return std::vector<std::pair<int, Velocidad>>();
        };
        //2D planning with shared responsibility for avoiding a collision
        virtual Velocidad MotionShared(std::vector<std::unique_ptr<Agent>> *agents, double time_step, boundsVS bounds, int lookAhead){
            return {v,w};
        };

        virtual Velocidad MotionReciprocal(Velocidad velTarget, double time_step, boundsVS bounds, std::string role, double d) {
            return {v,w};
        };

        virtual Velocidad
        MotionReciprocal2(Velocidad velTarget, double time_step, boundsVS bounds, std::string role, double d, const int action,
                                  const double prevUt) {
            return {v,w};
        };

        virtual Velocidad MotionReciprocalDOVS(Velocidad velTarget, double time_step, boundsVS bounds, std::string role, double d,
                                               std::vector<std::unique_ptr<Agent>> *agents){
            return {v,w};
        }

        virtual Velocidad
        MotionReciprocalDOVTS(std::list<std::pair<Comando, int>> goalTarget, double time_step, boundsVS bounds, std::string role) {
            return {v,w};
        }

        virtual Velocidad
        SelectVelocity(boundsVS bounds, double time_step) { return {v, w};};

        virtual  void PlotVSData(boundsVS bounds, double time_step){};

        virtual bool IsFinished(double min_dist){  //returns true or false if the agent has reached all its goals
            return true;   //default behaviour for passive agents (they do not reach any goal)
        };
        virtual void Update(double t);
        virtual bool GetCurrentGoal(Tpf& goal){ return false;}
        virtual void
        ModelAgents(std::vector<std::unique_ptr<Agent>> *agents, const double stept, int lookAhead,
                            const double th, const bool accConst) {};
        virtual void ModelAgentsFusion(std::vector<std::unique_ptr<Agent>> *agents, const double stept, int lookAhead,
                           const double th, const bool accConst){};

        virtual Command GetLastCommand(const int id){ return Command();}
        virtual Command GetFirstCommand(const int id){ return Command();}
        virtual Command GetStraightLineCommand(const int id){ return Command();};
        virtual Command GetMinVelocityCommand(const int id){ return Command();}

        virtual void WriteMetricsLog(std::vector<std::unique_ptr<Agent>> *agents){};

        virtual void SetBoundsVS(const double stept){};

        void SetVelocity(Velocidad vel);
        void SetAcceleration(double av, double aw);
        void SetLocalization(Tsc l);

        int GetId();
        double GetV();
        double GetW();
        Tsc GetLocalization();
        double GetRealRadius();

        double GetAV();
        double GetAW();
        double getV0();
        double getW0();
        double getDistanceSegment();
        Tpf getFirstPoint();
        Tpf getLastPoint();
        bool isSegment();

        void printAgent();
};

#endif

//NOTES:
//- A virtual function must be defined for the class in which it is first declared (unless it is declared to be a pure virtual function (=0))