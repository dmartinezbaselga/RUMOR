//
// Created by maite14 on 8/02/17.
//

#ifndef VS_DOVTS_H
#define VS_DOVTS_H

#include "gnuplot-iostream.h"

#include <dislin.h>

#include "dataDOVTS.h"
#include "VS.h"

//Typedef for forbidden commands
//typedef std::vector<Command> dovt;
typedef struct RootCommand{
    Root inf, sup;  //inf = rootMax, sup = rootMin
    RootCommand(){};
    RootCommand(const Root& i, const Root& s):inf(i), sup(s) {};
};

class dovt {
    //The following vectors store the collision points and corresponding velocities/times at current instant
    int id; //id of the agent for which the dovt has been computed
    std::vector<Command> commands;
    std::vector<RootCommand> roots;

    Command straight_traj;  //command for the straight line trajectory
    Command min_velocity;   //command which contains the minimum value of velocity (v,w) which allows to pass before the obstacle (valley)

    //boundsVS bounds;
    double tminDOVT, dmin;

public:
    //dovt(const int i, const boundsVS b, const double size, const double vobs){
    dovt(const int i, const double size, const double vobs, const double vlim_max){
        id = i; //bounds = b;
        dmin = 1e5; tminDOVT = 2*size/vobs; min_velocity = Command(Comando(Velocidad(vlim_max, 0), 0), Comando(Velocidad(0,0),0), 1, i);
    }
    ~dovt(){};

    const int GetId(){ return id; }
    void Add(Command c, RootCommand r){
        if (commands.empty() || !(c == commands.back())){
            commands.push_back(c);
            roots.push_back(r);
            if (std::atan2(c.sup.vel.v, c.sup.vel.w) == M_PI_2) straight_traj = c;
        }
    }
    const std::vector<Command>& GetCommands(){
        return commands;
    }
    const std::vector<RootCommand>& GetRoots(){
        return roots;
    }
    const Command GetCommandAt(const int ind){
        if (!commands.empty() && ind < (int)commands.size())
            return commands[ind];
        return Command();
    }
    const std::pair<Command, RootCommand> GetValueAt(const int ind){
        if (!commands.empty() && ind < (int)commands.size())
            return std::make_pair(commands[ind], roots[ind]);
        return std::make_pair(Command(), RootCommand());
    }
    const Command GetRightmostCommand(){
        //Returns the command wich leads to the right tangent trajectory
        if (!commands.empty())
            return commands.back();
            //return (commands.size() > 1 && commands.back().objeto != 0) ? commands.back() : commands[commands.size()-1];
        else
            return Command();
    }
    const Command GetLeftmostCommand(){
        //Returns the command wich leads to the left tangent trajectory
        if (!commands.empty())
            return commands.front();
        else
            return Command();
    }
    const Command GetStraightCommand(){
        return straight_traj;
    }
    const Command GetMinVelocityCommand(){
        //Look for the minimum command

        int ind_i = 0; int valle = 0; int valle_piso;
        for (int i = 0; i<(int)commands.size(); i++){
            //comprobamos si hay valles
            if (i>=2){
                Command ci = commands[i];
                Command ci_1 = commands[i-1];
                Command ci_2 = commands[i-2];

                if (((ci.sup.vel.v < ci_1.sup.vel.v && ci_1.sup.vel.v >= ci_2.sup.vel.v) ||
                     (ci.sup.vel.v < ci_1.sup.vel.v && ci_1.sup.vel.v < ci_2.sup.vel.v && i-2 == ind_i)) &&
                    //comandos[i-2].sup.w >= comandos[i-1].sup.w &&
                    //comandosIdFusion[i-1].sup.w > comandosIdFusion[i].sup.w &&
                    valle==0 && atan2(ci_1.sup.vel.v, ci_1.sup.vel.w) !=
                                atan2(ci_2.sup.vel.v, ci_2.sup.vel.w)){
                    //std::abs(comandos[i-2].sup.w) >= std::abs(comandos[i-1].sup.w) &&
                    //std::abs(comandos[i-1].sup.w) > std::abs(comandos[i].sup.w)
                    valle = 1;
                }else if(ci.sup.vel.v > ci_1.sup.vel.v &&
                         ci_1.sup.vel.v < ci_2.sup.vel.v && valle==1){
                    valle = 2;
                    valle_piso = i-1;
                }else if (((ci.sup.vel.v <= ci_1.sup.vel.v && ci_1.sup.vel.v > ci_2.sup.vel.v) ||
                           (i == (int)commands.size()-2 && (ci.sup.vel.v >= ci_1.sup.vel.v && ci_1.sup.vel.v > ci_2.sup.vel.v)))
                          &&
                          //(std::abs(ci_2.sup.w) < wmax_left && atan2(ci.sup.v, ci.sup.w) != atan2(ci_1.sup.v, ci_1.sup.w) && valle == 2))
                          (atan2(ci.sup.vel.v, ci.sup.vel.w) != atan2(ci_1.sup.vel.v, ci_1.sup.vel.w) && valle == 2))
                {
                    min_velocity = commands[valle_piso];
                    break;
                    //if (atan2(commands[valle_termina].sup.vel.v, commands[valle_termina].sup.vel.w) - atan2(commands[valle_inicia].sup.vel.v, commands[valle_inicia].sup.vel.w) < 5*M_PI/180){
                        //la anchura del valle es muy pequeña: no lo consideramos valle
                        //hay_valle = false;
                    //}

                }
            }
        }

/*
        auto it = commands.begin();
        if ((int) commands.size() > 1)
            for (std::advance(it,1); it != std::prev(commands.end(),1); ++it){
                Command c = (*it);

                double acandidate;
                if (c.sup.vel.w != 0)
                    acandidate = std::atan2(1, 1 / (c.sup.vel.v/c.sup.vel.w));
                else
                    acandidate = M_PI_2;
                double vmax, wmax;
                ObtenerComandoMaximo(acandidate, bounds.vlim_max, bounds.wmax_left, bounds.wmax_right, vmax, wmax);
                //if (c.sup.vel < Velocidad(vmax,wmax) && c.sup.vel.v < min_velocity.sup.vel.v) min_velocity = c;

                double d = std::sqrt(std::pow(c.sup.vel.w - c.inf.vel.w, 2) + std::pow(c.sup.vel.v - c.inf.vel.v, 2)); // + std::pow(c.sup.t - c.inf.t, 2));
                //if (std::abs(c.sup.t - c.inf.t) > tminDOVT)
                if (c.sup.vel < Velocidad(vmax,wmax) && c.sup.vel.v < min_velocity.sup.vel.v && d < dmin){
                    dmin = d;
                    min_velocity = c;
                }
            }
*/
        return min_velocity;
    }
};

typedef struct dovtAhead{
    //The following data store collision points, velocities and times for future times of lookahead
    //For every collision point, for every lookAhead, the new transformed points and commands are stored
    std::vector<std::vector<std::vector<std::pair<Root,Comando>>>> supAhead;
    std::vector<std::vector<std::vector<std::pair<Root,Comando>>>> infAhead;
};

//Definition of the specific types for the velocity-time space
typedef cell2d<double, double, std::vector<Command>> cellVW;    //???? CÓMO ALMACENO LOS COMANDOS DE VELOCIDAD PARA NO DUPLICAR LA INFORMACIÓN (SÓLO COMANDOS, CON ID...)
typedef std::pair<unsigned , unsigned> keyWV;
typedef struct layerVW{
    unsigned size_w, size_v;    //number of cells in each dimension (discretization of the velocity map (v,w))
    map2d< keyWV, cellVW > mapVW;

    layerVW(){ size_v = 0; size_w = 0; };
    layerVW(unsigned s_1d, unsigned s_2d):size_w(s_1d), size_v(s_2d){
        mapVW = map2d< keyWV, cellVW>(s_1d*s_2d);
    }
};

class DOVTS: public VS{

    //Entre otras cosas, tendrá un map3d.....
    //vts es el espacio de velocidades-tiempo: t_horizon/t_step capas de datos de tipo mapa de velocidades (v,w)
    map3d< layerVW > vts; //map3d< map2d< std::pair<unsigned , unsigned>, cell2d<double, double, std::vector<Command>> >> vts;
    //double stept; // double stepw, stepv; //step between cells (width and height of each cell): stepw = std::abs(vs->GetWMaxRight() - vs->GetWMaxLeft())/sizew; stepv = vs->GetVMax()/sizev;
    //the time horizon is computed as: stept*vts.data3d.size()

    bool InsertKey(keyWV k_wv, unsigned k_t, cellVW cell, unsigned long nCells);

    void UpdateCell(const keyWV keyVel, const unsigned k, cellVW &cell);

    void GenerateCellLine(const Command command, unsigned long nCells);
    void FillCells(const Command c, unsigned long nCells, Gnuplot& g);

public:
    unsigned sizew, sizev;
    double stept;
    int delta;

    DOVTS(){};
    DOVTS(unsigned n, double time_step, unsigned sw, unsigned sv, boundsVS dataVS):stept(time_step), sizew(sw), sizev(sv), VS(dataVS){
        vts = map3d<layerVW>(n);
        //Create space for the n layers of size (sw,sv)
        delta = 1;
        layerVW layer = layerVW(sw+delta,sv+delta);
        for (unsigned i = 0; i < n; i++) {
            vts.data3d.push_back(layer);
        }
    }
    ~DOVTS(){
        ClearVTSpace();
    }

    //DOVTS& operator= (const DOVTS& stateIn);    //assignment operator

    void InsertVTS(const map3d<layerVW>& space);   //function which inserts the elements of a map into the velocity-time space

    Velocidad GetVelocity(keyWV key);
    double GetTime(unsigned k);

    bool IsValidWV(Velocidad vel);
    bool IsValidT(double t);

    keyWV GetKeyWV(Velocidad vel);
    unsigned GetKeyT(double t);

    bool GetKey(const Velocidad vel, const double t, std::pair<keyWV, unsigned>& key);

    bool IsCellFree(std::pair<keyWV, unsigned> key);
    bool IsCellFree(Velocidad vel, double t);

    void InsertDOVT(const std::vector<Command> &commands, unsigned long numC);
    void FillMap2(const std::vector<Command>& dov, unsigned long n);  //function which inserts the maximum/minimum commands into the velocity-time space
    void FillMap(const std::vector<Command>& dov, unsigned long n);  //function which inserts the maximum/minimum commands into the velocity-time space
    void FillCosts();

    const map3d< layerVW >& GetVTSpace();
    int GetSizeVTS(){ return vts.data3d.size(); }
    const layerVW & GetLayer(unsigned k);

    unsigned GetSizeV();
    unsigned GetSizeW();
    unsigned long GetSizeT();
    double GetTh();

    void ClearVTSpace();

    void GenerateCellLine(const Command command);

    void Plot(Gnuplot& gp, const Velocidad followGoal, const double t);
    void PlotDislin();

    void InsertDOVTAhead(const std::vector<std::vector<std::vector<std::pair<Root, Comando>>>>& dovtAhead, const std::vector<Comando>& dovt, int lookAhead, unsigned long nCells);

    void CompleteLines(Command currentC, Command previousC, unsigned long nCells, Gnuplot &g);

    void BresenLine(const Command c, unsigned long nCells, Gnuplot &g);

    std::vector<std::pair<keyWV, unsigned>> BresenLine3d(const std::pair<keyWV, unsigned> keyIni, const std::pair<keyWV, unsigned> keyEnd, unsigned long nCells, Gnuplot& g);
};

#endif //VS_DOVTS_H
