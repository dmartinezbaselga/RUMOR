//
// Created by maite14 on 27/06/17.
//

#ifndef VS_ACTIVEAGENT_H
#define VS_ACTIVEAGENT_H

#include "Agent.h"

class ActiveAgent: public Agent {

    std::vector<Tpf> goals;	//vector of goals where the agent should reach at (world coordinates)
    int current_goal;

    boundsVS bounds;

    //velocity-time information
    //std::vector<Comando> commands3d;    //support for exact dovt: exact times at which collisions are produced
    //std::vector<std::vector<Command>> commands2d;   //support for bounded dovt: minimum and maximum times for escape or enter the collision band
    std::vector<dovt> commands2d;   //support for bounded dovt: minimum and maximum times for escape or enter the collision band
    std::vector<dovtAhead> commands2dAhead; //support for bounded dovt within look ahead
    std::vector<Command> fusion2d;
    std::vector<int> fusion2dAgents;
    bool originInside;

    Gnuplot gp;
    Gnuplot gp1;
    // Create a script which can be manually fed into gnuplot later:
    //    Gnuplot gp(">script.gp");
    boost::iostreams::file_descriptor_sink file_descriptor_sink;

    bool log = false;
    std::string nameLog;
    Velocidad dir_goal;

    //Look Ahead computation of data: roots and DOVT
    dovtAhead ComputeDOVT_LA(dovt dovt, int ahead, std::vector<std::vector<std::pair<Tsc, Velocidad>>> vector,
                             Tsc locCAg_ini, std::vector<std::vector<std::pair<Tsc, Velocidad>>> lookAheadStatesCAg,
                             double stept, boundsVS vs);


    void PlotDOVT(dovt dovt, int ahead, dovtAhead dovtAhead);
    void PlotStatesAhead(const std::vector<std::vector<std::pair<Tsc,Velocidad>>>& states, const std::vector<std::vector<std::pair<Tsc,Velocidad>>>& statesCAg);
public:

    ActiveAgent(){};
    ActiveAgent(int i, double x, double y, double tita, double v0, double w0, double av0, double aw0, double r, bool graph=true) :
                Agent(i, x, y, tita, v0, w0, r, true, graph){
        av = av0; aw = aw0;
        //planner.SetPlanner(th, step, ndata, mdata, numC, boundsVS);
        //space3d = dovts((unsigned) std::ceil(th/step), step, ndata, mdata, boundsVS);

        current_goal = 0;

        commands2d.reserve(obst_max*comm_max);

        originInside = false;

        file_descriptor_sink = gp.operator*();

        log = false;
    };
    ~ActiveAgent(){};

    void AddGoal(double xGoal, double yGoal);
    bool GetCurrentGoal(Tpf &goal);
    Tpf GetFirstGoal();
    bool GetOriginInside();

    void SetBoundsVS(const double stept);

    void SetNameLog(std::string name);
    void UpdateLog();
    void WriteMetricsLog(std::vector<std::unique_ptr<Agent>> *agents);

    bool IsFinished(double min_dist);
    void Update(double t);
    void ModelAgents(std::vector<std::unique_ptr<Agent>> *agents, const double stept, int i, const double th,
                         const bool accConst);

    void ModelAgentsFusion(std::vector<std::unique_ptr<Agent>> *agents, const double stept, int lookAhead,
                                        const double th, const bool accConst);

    Command GetLastCommand(const int id);
    Command GetFirstCommand(const int id);
    Command GetStraightLineCommand(const int id);
    Command GetMinVelocityCommand(const int id);

    Velocidad MotionGreedy(double time_step, std::vector<std::unique_ptr<Agent>> *agents, bool video=false);

    Velocidad
    MotionPlan(double time_step, double time_horizon, unsigned ndata, unsigned mdata, unsigned long numC,
                   int lookAhead);
    Velocidad MotionStrategy(double time_step, std::vector<std::unique_ptr<Agent>> *agents, bool video=false, int iteracion = 0, bool graph=true, bool debug=false);
    // andrew RL
    std::vector<std::pair<int, Velocidad>> MotionStrategyRL(double time_step, std::vector<std::unique_ptr<Agent>> *agents, bool &freeActions, bool video=false, int iteracion = 0, bool graph=true, bool debug=false);
    PosibleVelocidad findClosestSafeVelocity();
    
    Velocidad MotionShared(std::vector<std::unique_ptr<Agent>> *agents, double time_step, boundsVS bounds, int lookAhead);

    Velocidad MotionReciprocal(Velocidad gV, double time_step, boundsVS bounds, std::string role, double d);
    Velocidad MotionReciprocal2(Velocidad gV, double time_step, boundsVS bounds, std::string role, double d, const int action,
                                    const double prevUt);
    PosibleVelocidad FindClosestSafeRec(boundsVS bounds);
    PosibleVelocidad FindClosestReachableRec(Velocidad gV, double time_step, boundsVS bounds, std::string role,
                                             double d,
                                             bool safe);

    Velocidad MotionReciprocalDOVS(Velocidad velTarget, double time_step, boundsVS bounds, std::string role, double d,
                                                std::vector<std::unique_ptr<Agent>> *agents);
    Velocidad
    MotionReciprocalDOVTS(std::list<std::pair<Comando, int>> velTarget, double time_step, boundsVS bounds, std::string role);

    Velocidad SelectVelocity(boundsVS bounds, double time_step);
    void PlotVSData(boundsVS bounds, double time_step);

    //Functions to plot VTS
    void PlotVSEnvironment(const std::vector<Command> &commands, boundsVS bounds, goalVS goal, double t, bool video=false, int iteracion = 0, bool debug=false);

    const std::vector<dovt>& GetCommands(){ return commands2d;}
    void PlotRelativeInfo(ActiveAgent &ag);

    void getFreeVelocitiesVector(std::vector<float>& safeVelocities);

    std::vector<Velocidad> isGoalInDW(double time_step, std::vector<std::unique_ptr<Agent>> *agents, bool video, int iteracion, bool graph, bool debug,
                                        std::vector<int>& goalVel, bool dqn = false);

    template <typename T>
    void CalculaValoresVarios(const std::vector<Command> &comandos, std::vector< std::vector< T >> &mxray,
                              std::vector<std::vector<T>> &myray,
                              std::vector<std::vector<T>> &mzray,
                              std::vector<int> &num_eltos, int &obj_din, std::vector<int> fusion2dAgents, std::vector<std::vector<int>>& positions){


        //std::cout << "Inside CalculaValoresVarios, size... " << comandos.size() << std::endl;
        std::vector<Command>::const_iterator cIt = comandos.begin ();
        int k = 0; int n = 0; int i = 0;
        int nuevo = true;
        //std::array<T, comm_size> datax = mxray.at(k);
        //std::array<T, comm_size> datay = myray.at(k);
        //std::array<T, comm_size> dataz = mzray.at(k);
        std::vector<T> xdata, ydata, zdata;
        xdata.reserve(comm_max); ydata.reserve(comm_max); zdata.reserve(comm_max);
        int id = 0;
        std::vector<int> pos;
        while (cIt != comandos.end()){
            //std::cout << "Hay valores" << std::endl;
            Command c = (*cIt);
            //Tiempos t = (*tIt).t;
            Velocidad sup = c.sup.vel;
            if (c.objeto == 1) {
                xdata.push_back(sup.w);
                ydata.push_back(sup.v);
                zdata.push_back(c.sup.t);
                pos.push_back(fusion2dAgents[id]);
                //mxray[ k ][ n ] = sup.w; myray[ k ][ n ] = sup.v; mzray[ k ][ n ] = c.sup.t;
                n++;
                if (nuevo) nuevo = false;
            } else if (!nuevo) {    //hemos almacenado los limites superiores, ahora hay que meter los inferiores
                int eltos = n;
                //cout << "CalculaValores2" << std::endl;
                for (int j = 0; j < eltos; j++) {
                    //cout << "CalculaValores2 j" << std::endl;
                    Command c = (*(cIt - 1 - j));
                    //Tiempos t = (*(tIt-1-j)).t;
                    Velocidad inf = c.inf.vel;
                    xdata.push_back(inf.w);
                    ydata.push_back(inf.v);
                    zdata.push_back(c.inf.t);
                    pos.push_back(fusion2dAgents[id-1-j]);
                    //mxray[ k ][ n ] = inf.w; myray[ k ][ n ] = inf.v; mzray[ k ][ n ] = c.inf.t;
                    n++;
                }
                mxray.push_back(xdata);
                myray.push_back(ydata);
                mzray.push_back(zdata);
                positions.push_back(pos);
                num_eltos.push_back(n);
                //k++;
                nuevo = true;
                n = 0;
                xdata.clear(); ydata.clear(); zdata.clear(); pos.clear();
            }
            id++;
            ++cIt; //++tIt;
        }
        if (!nuevo){    //hemos almacenado los limites superiores, ahora hay que meter los inferiores
            int eltos = n;
            for (int j = 0; j < eltos; j++) {
                //cout << "CalculaValores2 j" << std::endl;
                Command c = (*(cIt - 1 - j));
                //Tiempos t = (*(tIt-1-j)).t;
                Velocidad inf = c.inf.vel;
                xdata.push_back(inf.w);
                ydata.push_back(inf.v);
                zdata.push_back(c.inf.t);
                pos.push_back(fusion2dAgents[id-1-j]);
                //mxray[ k ][ n ] = inf.w; myray[ k ][ n ] = inf.v; mzray[ k ][ n ] = c.inf.t;
                n++;
            }
            mxray.push_back(xdata);
            myray.push_back(ydata);
            mzray.push_back(zdata);
            positions.push_back(pos);
            num_eltos.push_back(n);
            //num_eltos[ k ] = n;
            k++;
        }
        obj_din = k;
    }


};


#endif //VS_ACTIVEAGENT_H
