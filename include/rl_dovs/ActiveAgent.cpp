//
// Created by maite14 on 27/06/17.
//

#include "ActiveAgent.h"
//#include "TrajectoryAgent.h"
#include "StaticAgent.h"
#include "LinearAgent.h"
#include "CircularAgent.h"
#include "config.h"
#include <string>

#include "graficas.h"

#include "astar.h"

#include "Strategies.h"

#include <eigen3/Eigen/LU>

#include "GeometryCGAL.h"

#include <CGAL/Kernel/global_functions.h>
#include <ros/package.h>
#include <ros/ros.h>

#include <iostream>
using namespace std;

bool fusionCommands = true;


void ActiveAgent::AddGoal(double xGoal, double yGoal) {
    goals.clear();
    goals.push_back(Tpf(xGoal, yGoal));
    // cout << goals.size() << endl;
}

bool ActiveAgent::GetCurrentGoal(Tpf& goal) {

    assert(current_goal>=0);
    goal = goals[current_goal];

    return true;
}

Tpf ActiveAgent::GetFirstGoal() {
    return goals[0];
}

void ActiveAgent::SetNameLog(std::string name){
    nameLog = ros::package::getPath("rl_dovs") + "/" + name;
    log = true;
}

void ActiveAgent::UpdateLog(){
    std::ofstream flog;
    if (nameLog.empty())
        throw std::runtime_error("No valid name for log file");

    //Goal with respect to agent's position
    Tpf goal;
    transfor_inversa_p(goals[current_goal].x, goals[current_goal].y, &loc, &goal);
    flog.open(nameLog, std::ios::app);
    flog << goal.x << "\t" << goal.y << "\t" << loc.x << "\t" << loc.y <<
         "\t" << loc.tita << "\t" << v << "\t" << w << "\t";
}

bool ActiveAgent::IsFinished(double min_dist) {
//The agent finishes navigation when it has reached all its goals
    std::ofstream flog;
    if (log) flog.open(nameLog, std::ios::app);

    bool finished = false;

    //Goal with respect to agent's position
    if (goals.size() > current_goal){
        Tpf goal;
        transfor_inversa_p(goals[current_goal].x, goals[current_goal].y, &loc, &goal);

        // andrew 
        // min_dist = 1;
        // if (goal.x*goal.x + goal.y*goal.y < min_dist && this->v < 0.2){
        if (goal.x*goal.x + goal.y*goal.y < 0.15){
        // if (goal.x*goal.x + goal.y*goal.y < min_dist){

            // if (current_goal + 1 >= (int)goals.size()){ //last goal
                //if (log) flog << "GOAL REACHED" << std::endl;
                finished = true;
        //  }//else{
        //         //if (log) flog << "GOAL REACHED" << std::endl;
        //         current_goal++;
        //     }
        }
    }
    else{
        finished = true;
    }
    // cout << "IS FINISHED " << finished << endl;
    if (log) flog.close();
    return finished;
}

void ActiveAgent::Update(double t) {

    Agent::Update(t);

    //Update log with goal, location and velocity information
    if (log) UpdateLog();
}

Tsc SimulateAhead(Tsc loc, Velocidad mov, double t){

    Tsc locNew;
    double x = 0; double y = 0; double tita = 0;
    if (std::abs(mov.w) < 1e-5){
        locNew.x = loc.x + mov.v*std::cos(loc.tita)*t;
        locNew.y = loc.y + mov.v*std::sin(loc.tita)*t;
        locNew.tita = loc.tita;
    }
    else{	//w != 0
        locNew.x = loc.x - (mov.v/mov.w)*std::sin(loc.tita) + (mov.v/mov.w)*std::sin(loc.tita+mov.w*t);
        locNew.y = loc.y + (mov.v/mov.w)*std::cos(loc.tita) - (mov.v/mov.w)*std::cos(loc.tita+mov.w*t);
        locNew.tita = NormalisePI(loc.tita + mov.w*t);
    }
    return locNew;
}

std::vector<std::vector<std::pair<Tsc,Velocidad>>> ComputeStatesAhead(Tsc iniLoc, Velocidad iniVel, constraints c, boundsVS bounds, double stept, int lookAhead){
//Compute positions and velocities of the agents considering the future possible movements at time t0 + (lookAhead)*stept

    std::vector<std::vector<std::pair<Tsc,Velocidad>>> lookAheadStates; //for each lookAhead considered we store the positions and velocities
    std::vector<std::pair<Tsc,Velocidad>> states;
    std::list<std::pair<Tsc,Velocidad>> toExplore;
    //std::list<std::pair<Tsc,Velocidad>> states;

    states.push_back(std::make_pair(iniLoc, iniVel));
    lookAheadStates.push_back(states);
    for (int k = 0; k < lookAhead; k++){

        toExplore.assign(states.begin(), states.end());
        states.clear();

        while (!toExplore.empty()){
            Tsc location = toExplore.front().first;
            Velocidad velocity = toExplore.front().second;
            toExplore.pop_front();
            DW dw(velocity, c, bounds, stept);
            for (int eti = 0; eti < 5; eti++){
                movement m = static_cast<movement>(eti);
                Velocidad vel = dw.GetVelocity(m);
                states.push_back(std::make_pair(SimulateAhead(location, vel, stept), vel));
            }
        }
        //TODO: Delete repeated states
        //states.remove_if([](const std::pair<Tsc, Velocidad>& d){return d.second.operator==() < 2;});
        //std::unique(states.begin(), states.end(), ([](const std::pair<Tsc, Velocidad>& l, const std::pair<Tsc, Velocidad>& r){return  l.second == r.second;}));
        lookAheadStates.push_back(states);
    }
    return lookAheadStates;
}

Velocidad ComputeMaximumCommand(const double ang, const boundsVS bounds);
Velocidad ComputeVelocity(const double distance, const double rcandidate, const double acandidate, const double time, const Velocidad max);
double AngDisplacement(const Root point, const double rcandidate, const unsigned k);
dovtAhead ActiveAgent::ComputeDOVT_LA(dovt dovtAg, int lookAhead, std::vector<std::vector<std::pair<Tsc, Velocidad>>> lookAheadStates,
                                      Tsc locCAg_ini, std::vector<std::vector<std::pair<Tsc, Velocidad>>> lookAheadStatesCAg,
                                      double stept, boundsVS bounds) {

    dovtAhead dataAhead;

    if (lookAheadStates.empty()){
        //Complete the vector for the current Agent assuming it will not change its velocity
        for (int k = 0; k<lookAhead; k++) {
            std::vector<std::pair<Tsc, Velocidad>> states;
            states.push_back(std::make_pair(SimulateAhead(Tsc(), Velocidad(this->GetV(), this->GetW()), stept), Velocidad(this->GetV(), this->GetW())));
            lookAheadStates.push_back(states);
        }
    }

    for (int n = 0; n<dovtAg.GetCommands().size(); n++){
        std::pair<Command, RootCommand> dovtValue = dovtAg.GetValueAt(n);
        Command c = dovtValue.first;
        //assert (!(c == Command()));   //TODO: Check when the command is empty
        if (c.objeto == 0) continue;
        RootCommand rC = dovtValue.second;
        Eigen::Vector3d infPoint, supPoint;
        infPoint(0) = rC.inf.x; infPoint(1) = rC.inf.y; infPoint(2) = 1.0;
        supPoint(0) = rC.sup.x; supPoint(1) = rC.sup.y; supPoint(2) = 1.0;

        std::vector<std::vector<std::pair<Root, Comando>>> dataInfAheadK;
        std::vector<std::vector<std::pair<Root, Comando>>> dataSupAheadK;
        for (int k = 0; k<lookAhead; k++){
            //Compute the motions performed by the colliding agent and store them for later transforming the CB points
            std::vector<Eigen::Matrix3d> movs;
            if (!lookAheadStatesCAg.empty()){
                std::vector<std::pair<Tsc,Velocidad>> statesCAg = lookAheadStatesCAg[k];
                for (int sCAg = 0; sCAg < (int) statesCAg.size(); sCAg++) {
                    Tsc stateCAg = statesCAg[sCAg].first;
                    Eigen::Matrix3d TCAg_ini, TCAg_LA, Tmov;

                    //Transformation matrix for current location of the Colliding Agent
                    TCAg_ini(0, 0) = std::cos(locCAg_ini.tita);
                    TCAg_ini(0, 1) = -std::sin(locCAg_ini.tita);
                    TCAg_ini(0, 2) = locCAg_ini.x;
                    TCAg_ini(1, 0) = std::sin(locCAg_ini.tita);
                    TCAg_ini(1, 1) = std::cos(locCAg_ini.tita);
                    TCAg_ini(1, 2) = locCAg_ini.y;
                    TCAg_ini(2, 0) = 0.0;
                    TCAg_ini(2, 1) = 0.0;
                    TCAg_ini(2, 2) = 1.0;

                    //Transformation matrix for look ahead location of the Colliding Agent
                    TCAg_LA(0, 0) = std::cos(stateCAg.tita);
                    TCAg_LA(0, 1) = -std::sin(stateCAg.tita);
                    TCAg_LA(0, 2) = stateCAg.x;
                    TCAg_LA(1, 0) = std::sin(stateCAg.tita);
                    TCAg_LA(1, 1) = std::cos(stateCAg.tita);
                    TCAg_LA(1, 2) = stateCAg.y;
                    TCAg_LA(2, 0) = 0.0;
                    TCAg_LA(2, 1) = 0.0;
                    TCAg_LA(2, 2) = 1.0;

                    Tmov = TCAg_ini.inverse() * TCAg_LA;

                    movs.push_back(Tmov);
                }
            }else{
                movs.push_back(Eigen::Matrix3d::Identity());
            }

            std::vector<std::pair<Tsc,Velocidad>> states = lookAheadStates[k];
            std::vector<std::pair<Root, Comando>> infAheadK;
            std::vector<std::pair<Root, Comando>> supAheadK;
            for (int s = 0; s<states.size(); s++) {
                Tsc location = states[s].first;
                Eigen::Matrix3d T;
                Eigen::Vector3d infPointNew, supPointNew;
                //Transformation matrix for current location of the Agent
                T(0, 0) = std::cos(location.tita);
                T(0, 1) = -std::sin(location.tita);
                T(0, 2) = location.x;
                T(1, 0) = std::sin(location.tita);
                T(1, 1) = std::cos(location.tita);
                T(1, 2) = location.y;
                T(2, 0) = 0.0;
                T(2, 1) = 0.0;
                T(2, 2) = 1.0;

                infPointNew = T.inverse() * infPoint;
                supPointNew = T.inverse() * supPoint;

                for (int m = 0; m<(int)movs.size(); m++) {
                    Eigen::Matrix3d Tmov = movs[m];
                    Eigen::Vector3d infTfPoint, supTfPoint;

                    infTfPoint = infPointNew.transpose()*Tmov;
                    //Compute intersecting points of the CB wrt new positions ahead for the agent
                    //root min.
                    //tfPoint = ((T.inverse() * infPoint).transpose()* Tmov).transposeInPlace();
                    Root pointR = Root(infTfPoint.x(), infTfPoint.y(), rC.inf.i);
                    double radiusAhead = (std::pow(infTfPoint.x(), 2) + std::pow(infTfPoint.y(), 2)) / (2 * infTfPoint.y());
                    double acandidate = std::atan2(1, 1 / radiusAhead);
                    //double distance = (acandidate == M_PI_2 || std::abs(pointR.y) < 1e-5) ? copysign(pointR.x, radiusAhead) : AngDisplacement(pointR, radiusAhead, 0);
                    double distance = (acandidate == M_PI_2) ? pointR.x : (std::abs(pointR.y) < 1e-5 ? copysign(pointR.x,radiusAhead) : AngDisplacement(pointR, radiusAhead, 0));
                    if (acandidate != M_PI_2 || distance >= 0){
                    //if ((c.inf.t - (k + 1) * stept) > 1e-5) {
                        Velocidad max;
                        max = bounds.ComputeMaximumCommand(acandidate);     //ObtenerComandoMaximo(acandidate, bounds.vlim_max, bounds.wmax_left, bounds.wmax_right, max.v, max.w);
                        Comando newC;
                        //newC.t = c.inf.t - (k + 1) * stept;
                        newC.t = c.inf.t;
                        newC.vel = ComputeVelocity(distance, radiusAhead, acandidate, newC.t, max);
                        if (newC.vel.v < 0 ) {
                            std::cout << "MAL" << std::endl;
                        }
                        //newC.vel.w = titaAhead/newC.t;
                        //newC.vel.v = newC.vel.w * radiusAhead;
                        if (max < newC.vel) newC.vel = max;
                        infAheadK.push_back(std::make_pair(pointR, newC));
                    //}
                    }

                    //root max.
                    supTfPoint = supPointNew.transpose() * Tmov;
                    pointR = Root(supTfPoint.x(), supTfPoint.y(), rC.sup.i);

                    radiusAhead = (std::pow(supTfPoint.x(), 2) + std::pow(supTfPoint.y(), 2)) / (2 * supTfPoint.y());
                    acandidate = std::atan2(1, 1 / radiusAhead);
                    distance = (acandidate == M_PI_2) ? pointR.x : (std::abs(pointR.y) < 1e-5 ? copysign(pointR.x,radiusAhead): AngDisplacement(pointR,radiusAhead,0));
                    if (acandidate != M_PI_2 || distance >= 0) {
                        //if ((c.sup.t - (k + 1) * stept) > 1e-5) { //0){
                        Velocidad max;
                        max = bounds.ComputeMaximumCommand(acandidate);    //ObtenerComandoMaximo(acandidate, bounds.vlim_max, bounds.wmax_left, bounds.wmax_right, max.v, max.w);
                        Comando newC;
                        //newC.t = c.sup.t - (k + 1) * stept;
                        newC.t = c.sup.t;
                        newC.vel = ComputeVelocity(distance, radiusAhead, acandidate, newC.t, max);
                        if (newC.vel.v < 0) {
                            std::cout << "MAL" << std::endl;
                        }
                        //newC.vel.w = titaAhead/newC.t;
                        //newC.vel.v = newC.vel.w * radiusAhead;
                        if (max < newC.vel) newC.vel = max;
                        supAheadK.push_back(std::make_pair(pointR, newC));
                        //}
                    }
                }
            }
            dataInfAheadK.push_back(infAheadK);
            dataSupAheadK.push_back(supAheadK);
        }
        dataAhead.infAhead.push_back(dataInfAheadK);
        dataAhead.supAhead.push_back(dataSupAheadK);
    }

    return dataAhead;
}

void ComputeRange(Range& r1, int& i, Range& r2, int& j, std::vector<Range>& out) {

    bool check = false;

    if (r1.first >= 0 && r2.first < 0){
        out.push_back(r1);
        i++;
    }else if (r1.first < 0 && r2.first >= 0){
        out.push_back(r2);
        j++;
    }else{
        //both ranges have same sign
        if (r1.second < r2.first){
            out.push_back(r1);
            i++;
        }else{
            if ((std::abs(r1.second) == INF && std::abs(r2.first) == INF) || std::abs(r1.second - r2.first) < 1e-3) {
                out.push_back(r1);
                i++;
            }else{
                if (r1.first < r2.first){
                    if ((std::abs(r1.first) != INF || std::abs(r2.first) != INF) && std::abs(r1.first - r2.first) > 1e-3) {
                        out.push_back({r1.first, r2.first});
                        r1 = {r2.first, r1.second};
                    }else{
                        //check r2.second limit
                        //out.push_back({r1.first, r2.first});
                        check = true;
                    }
                }else{
                    check = true;
                    if ((std::abs(r1.first) == INF && std::abs(r2.first) == INF) || std::abs(r1.first - r2.first) < 1e-3) {
                        //check r2.second limit
                    }else{
                        //check r2.second limit
                        //out.push_back({r2.first, r1.first});
                        //r2 = {r1.first, r2.second};
                    }
                }

                //Check second
                if (r2.second < r1.first){
                    out.push_back({r2.first, r2.second});
                    j++;
                    check = false;
                }else{
                    if ((std::abs(r2.second) == INF && std::abs(r1.first) == INF) || std::abs(r2.second - r1.first) < 1e-3){
                        out.push_back({r2.first, r2.second});
                        r1 = {r2.second, r1.second};
                        j++;
                        check = false;
                    }else{
                        if ((std::abs(r2.first) != INF || std::abs(r1.first) != INF) && std::abs(r2.first - r1.first) > 1e-3){
                            out.push_back({r2.first, r1.first});
                            r2 = {r1.first, r2.second};
                        }
                        check = true;
                    }
                }

                if (check){
                    if (r1.second < r2.second){
                        if ((std::abs(r1.second) == INF && std::abs(r2.second) == INF) || std::abs(r1.second - r2.second) < 1e-3){
                            out.push_back({r2.first, r2.second});
                            i++; j++;
                        }else{
                            out.push_back({r2.first, r1.second});
                            r2 = {r1.second, r2.second};
                            i++;
                        }
                    }else{
                        if ((std::abs(r1.second) == INF && std::abs(r2.second) == INF) || std::abs(r1.second - r2.second) < 1e-3){
                            out.push_back({r2.first, r2.second});
                            i++; j++;
                        }else{
                            out.push_back({r2.first, r2.second});
                            r1 = {r2.second, r1.second};
                            j++;
                        }
                    }
                }
            }
        }
    }

}

std::vector<Range> UnionRanges(const std::vector<Range>& trajectories){

    std::vector<Range> result;

    double max_width = 0.5; double factor = 1.75;
    Range rCompare;
    int positionCompare;
    int i = 0; bool check = false;
    while (i<(int)trajectories.size()){
        Range r = trajectories[i];
        if (check){
            if (r.second - rCompare.first > factor*max_width){
                result.push_back(rCompare);
                result.push_back(r);
                check = false;
            }else{
                rCompare = {rCompare.first, r.second};
            }
        }else{
            if (r.second - r.first > max_width){
                result.push_back(r);
            }else{
                rCompare = r;
                check = true;
            }
        }
        i++;
    }
    if (check) {
        result.push_back(rCompare);
    }

    return result;
}

// andrew
std::vector<Range> FusionCollidingTrajectories(const std::vector<std::vector<Range>>& totalCollidingTraj){
//std::vector<Range> FusionCollidingTrajectories(const std::vector<std::vector<Range>>& totalCollidingTraj){
    std::vector<Range> result;
    std::vector<Range> traj1, traj2;
    auto it = totalCollidingTraj.begin();
    if (it != totalCollidingTraj.end()) {
        traj1 = (*it);

    }
    ++it;
    
    if (it == totalCollidingTraj.end())
        return traj1;

    while (it != totalCollidingTraj.end()) {
        traj2 = (*it);
        int i = 0; int j = 0;
        bool finished = false;
        Range r1 = traj1[i]; Range r2 = traj2[j];
        while (!finished){
            std::vector<Range> out;
            int prev_i = i; int prev_j = j;
            ComputeRange(r1, i, r2, j, out);

            //Update result with out
            for (auto itR = out.begin(); itR != out.end(); ++itR){
                result.push_back((*itR));
            }

            if (i < (int)traj1.size() && i != prev_i){
                r1 = traj1[i];
            }

            if (j < (int)traj2.size() && j != prev_j){
                r2 = traj2[j];
            }

            if (i >= (int)traj1.size()){
                //Fill the gap between ranges
                if (std::abs(result[result.size()-1].second - r2.first) > 1e-3){
                    if (result[result.size()-1].second > 0 && r2.first < 0){
                        result.push_back({result[result.size()-1].second, INF});
                        result.push_back({-INF, r2.first});
                    }
                }

                while (j < (int)traj2.size()){
                    result.push_back(r2);
                    j++;
                    if (j < (int)traj2.size()) r2 = traj2[j];
                }
            }

            if (j >= (int)traj2.size()){
                //Fill the gap between ranges
                if (std::abs(result[result.size()-1].second - r1.first) > 1e-3){
                    if (result[result.size()-1].second > 0 && r1.first < 0){
                        result.push_back({result[result.size()-1].second, INF});
                        result.push_back({-INF, r1.first});
                    }
                }

                while (i < (int)traj1.size()){
                    result.push_back(r1);
                    i++;
                    if (i < (int)traj1.size()) r1 = traj1[i];
                }
            }

            finished = (i >= (int)traj1.size() && j >= (int)traj2.size());
        }
        traj1 = UnionRanges(result);
        result.clear();

        ++it;
    }

    return traj1;
}

std::vector<Command> CompareDOVT(const std::vector<Command>& dovt1, const std::vector<Command>& dovt2, const std::vector<int> fusion2dAgents, const std::vector<int> fusion2dAgentsNew, std::vector<int>& fusion2dAgentsOut ){

    std::vector<Command> result;

    if (dovt1.empty()) {
        fusion2dAgentsOut = fusion2dAgentsNew;
        return dovt2;
    }
    if (dovt2.empty()) {
        fusion2dAgentsOut = fusion2dAgents;
        return dovt1;
    }
   
    double tita1Last = std::atan2(dovt1[(int)dovt1.size()-1].sup.vel.v, dovt1[(int)dovt1.size()-1].sup.vel.w);
    double tita2Last = std::atan2(dovt2[(int)dovt2.size()-1].sup.vel.v, dovt2[(int)dovt2.size()-1].sup.vel.w);
    double tita1First = std::atan2(dovt1[0].sup.vel.v, dovt1[0].sup.vel.w);
    double tita2First = std::atan2(dovt2[0].sup.vel.v, dovt2[0].sup.vel.w);
    if (tita1Last < tita2First){
        result.insert(result.end(), dovt1.begin(), dovt1.end());
        fusion2dAgentsOut.insert(fusion2dAgentsOut.end(), fusion2dAgents.begin(), fusion2dAgents.end());
        result.push_back(Command());
        fusion2dAgentsOut.push_back(-1);
        result.insert(result.end(), dovt2.begin(), dovt2.end());
        fusion2dAgentsOut.insert(fusion2dAgentsOut.end(), fusion2dAgentsNew.begin(), fusion2dAgentsNew.end());;
        return result;
    }else if (tita2Last < tita1First){
        result.insert(result.end(), dovt2.begin(), dovt2.end());
        fusion2dAgentsOut.insert(fusion2dAgentsOut.end(), fusion2dAgentsNew.begin(), fusion2dAgentsNew.end());
        result.push_back(Command());
        fusion2dAgentsOut.push_back(-1);
        result.insert(result.end(), dovt1.begin(), dovt1.end());
        fusion2dAgentsOut.insert(fusion2dAgentsOut.end(), fusion2dAgents.begin(), fusion2dAgents.end());
        return result;
    }
    //We know at this point that both vectors contain elements
    int i = 0; double tita1; Command lastCommand; bool primer = true;
    while(i<(int)dovt1.size()){
        tita1 = std::atan2(dovt1[i].sup.vel.v, dovt1[i].sup.vel.w);
        if (dovt1[i].objeto == 0 || tita2First - tita1 > 1e-5){ //tita1 < tita2First
            Command cmd1 = dovt1[i];
            if (primer || !(cmd1 == lastCommand)){
                result.push_back(cmd1);
                fusion2dAgentsOut.push_back(fusion2dAgents.front());
                lastCommand = cmd1;
            }
            if (primer) primer = false;
            i++;
        }else break;
    }

    int j = 0; double tita2;
    while(j<(int)dovt2.size()){
        tita2 = std::atan2(dovt2[j].sup.vel.v, dovt2[j].sup.vel.w);
        if (dovt2[j].objeto == 0 || tita1First - tita2 > 1e-5){ //tita2 < tita1First)
            Command cmd2 = dovt2[j];
            if (primer || !(cmd2 == lastCommand)) {
                result.push_back(cmd2);
                fusion2dAgentsOut.push_back(fusion2dAgentsNew.front());
                lastCommand = cmd2;
            }
            if (primer) primer = false;
            j++;
        }else break;
    }

    while (i < (int)dovt1.size() && j < (int)dovt2.size()){
        Command cmd1 = dovt1[i]; Command cmd2 = dovt2[j];
        Command cmd;
        int cmdAgent;
        tita1 = std::atan2(cmd1.sup.vel.v, cmd1.sup.vel.w);
        tita2 = std::atan2(cmd2.sup.vel.v, cmd2.sup.vel.w);
        int obj1 = cmd1.objeto; int obj2 = cmd2.objeto;
        if (cmd1.objeto != 0 && cmd2.objeto != 0){
            if (std::abs(tita1 - tita2) < 1e-8) {
                Comando sup, inf;
                if (cmd1.sup.vel.v >= cmd2.sup.vel.v) {
                    sup = cmd1.sup;
                    cmdAgent = fusion2dAgents.front();
                } else {
                    sup = cmd2.sup;
                    cmdAgent = fusion2dAgentsNew.front();
                }
                if (cmd1.inf.vel.v <= cmd2.inf.vel.v) {
                    inf = cmd1.inf;
                } else {
                    inf = cmd2.inf;
                }
                cmd = Command(sup, inf, 1);
                i++; j++;
            }else{
                if (tita1 < tita2){
                    cmd = Command(cmd1);
                    cmdAgent = fusion2dAgents.front();
                    i++;
                }else{
                    cmd = Command(cmd2);
                    cmdAgent = fusion2dAgentsNew.front();
                    j++;
                }
            }
        }else{
            if (cmd1.objeto != 0) {
                cmdAgent = fusion2dAgents.front();
                cmd = cmd1;
            } else {
                cmdAgent = fusion2dAgentsNew.front();
                cmd = cmd2;
            }
            i++; j++;
        }

        if (primer || !(cmd == lastCommand)){
            result.push_back(cmd);
            fusion2dAgentsOut.push_back(cmdAgent);
            lastCommand = cmd;
        }
        if (primer) primer = false;
    }

    while (i < (int)dovt1.size()){
        result.push_back(dovt1[i]);
        fusion2dAgentsOut.push_back(fusion2dAgents.front());
        i++;
    }

    while (j < (int)dovt2.size()){
        result.push_back(dovt2[j]);
        fusion2dAgentsOut.push_back(fusion2dAgentsNew.front());
        j++;
    }

    return result;
}

void ActiveAgent::ModelAgentsFusion(std::vector<std::unique_ptr<Agent>> *agents, const double stept, int lookAhead,
                              const double th, const bool accConst) {
//Computation of DOVT (velocities and times of collision) for each agent

    //clear command vector
    Tpf goalAg;
    commands2d.clear();
    fusion2d.clear();
    //commands2d.reserve(obst_max*comm_max);

    fusionCommands = true;
    // if (this->graph) {
    //     //Clean the plot
    //     int numWindow = 0;
    //     for (int i=0; i < (int) agents->size(); i++) {
    //         if (this->GetId() == agents->at(i)->GetId()) {
    //             SeleccionaVentana(numWindow + 2);
    //             numWindow++;
    //         }
    //     }
    //     LimpiaVentana();
    // }

    bool insideCB = false;
    originInside = false;

    std::vector<std::vector<std::pair<Tsc, Velocidad>>> lookAheadStates;
    std::vector<std::vector<std::vector<std::pair<Tsc, Velocidad>>>> totalStatesCAg;
    //andrew
    std::vector<int> ids;
    std::vector<bool> is_segment;
    std::vector<double> radio;
    std::vector<Tpf> first_points_vec, last_points_vec;
    //std::vector<std::unique_ptr<Agent>>::iterator it;
    for (auto it = agents->begin(); it != agents->end(); ++it) {
        // printf("%d, %d\n", (*it)->GetId(), this->GetId());
        if ((*it)->GetId() != this->GetId()) {   //For all the agents except itself
            // andrew
            float theta = this->GetLocalization().tita;
            std::vector<double> currentDirection {cos(theta), sin(theta)};
            std::vector<double> obsDirection {(*it)->GetLocalization().x - this->GetLocalization().x,(*it)->GetLocalization().y - this->GetLocalization().y};
            float dot = currentDirection[0] * obsDirection[0] + currentDirection[1]*obsDirection[1];   
            float det = currentDirection[0]*obsDirection[1] - currentDirection[1]*obsDirection[0];    
            float angle = atan2(det, dot); 

            float modulus = sqrt(pow(obsDirection[0], 2) + pow(obsDirection[1], 2));

            if (modulus < 4) {

                //INI: LookAhead positions and velocities for the agent
                ////for each lookAhead, for each available velocity from the current velocity, we store the resultant positions and the velocities applied
                lookAheadStates = ComputeStatesAhead(this->loc, Velocidad(this->GetV(), this->GetW()), constraints(av, aw),
                                                    bounds, stept, lookAhead);
                
                std::vector<std::vector<std::pair<Tsc, Velocidad>>> lookAheadStatesCAg = ComputeStatesAhead(
                        (*it)->GetLocalization(), Velocidad((*it)->GetV(), (*it)->GetW()),
                        constraints((*it)->GetAV(), (*it)->GetAW()),
                        bounds, stept, lookAhead);

                totalStatesCAg.push_back(lookAheadStatesCAg);
                ids.push_back((*it)->GetId());
                is_segment.push_back((*it)->isSegment());
                radio.push_back((*it)->GetRealRadius() + this->GetRealRadius() * config::safety_factor);
                first_points_vec.push_back((*it)->getFirstPoint());
                last_points_vec.push_back((*it)->getLastPoint());
            }
        }
    }
    int id = 0;
    std::vector<int> ids2;
    if (!lookAheadStates.empty()){
        std::vector<std::vector<Range>> totalCollidingTraj;
        for (int k = 0; k <= lookAhead; k++) {
            std::vector<std::pair<Tsc, Velocidad>> states = lookAheadStates[k];
            for (auto itS = states.begin(); itS != states.end(); ++itS) {
                Tsc agLoc = (*itS).first;
                for (auto itTotalCag = totalStatesCAg.begin(); itTotalCag != totalStatesCAg.end(); ++itTotalCag) {
                    std::vector<std::vector<std::pair<Tsc, Velocidad>>> lookAheadStatesCAg = (*itTotalCag);
                    std::vector<std::pair<Tsc, Velocidad>> statesCAg = lookAheadStatesCAg[k];
                    for (auto itSCAg = statesCAg.begin(); itSCAg != statesCAg.end(); ++itSCAg) {

                        //Compute Collision Band (CB) of the obstacle: instance of a linear or circular agent
                        std::unique_ptr<TrajectoryAgent> trajectory;
                        Tsc cagLoc = (*itSCAg).first;
                        Velocidad cagVel = (*itSCAg).second;
                        ///*
                        if (is_segment[id]){
                            cagLoc.x = first_points_vec[id].x;
                            cagLoc.y = first_points_vec[id].y;
                            Tsc sist2 = Tsc(last_points_vec[id].x, last_points_vec[id].y, cagLoc.tita);
                            trajectory = std::unique_ptr<TrajectoryAgent>{new StaticAgent(agLoc, cagLoc, radio[id], cagVel.v, ids[id], true, this->GetRealRadius()*config::safety_factor, sist2)};
                        }
                        else if (cagVel.v < 0.1) {
                            //if (!(*it)->IsFinished(0.7))
                            trajectory = std::unique_ptr<TrajectoryAgent>{new StaticAgent(agLoc, cagLoc, radio[id], cagVel.v, ids[id])};
                        }else{
                            if (std::abs(cagVel.w) > 1e-3){
                                trajectory = std::unique_ptr<TrajectoryAgent> {new CircularAgent(agLoc, cagLoc, radio[id], cagVel.v, cagVel.w, stept, ids[id])};
                            }
                            else{
                                trajectory = std::unique_ptr<TrajectoryAgent> {new LinearAgent(agLoc, cagLoc, radio[id], cagVel.v, ids[id])};
                            }
                            //trajectory = std::unique_ptr<TrajectoryAgent> {new LinearAgent(agLoc, cagLoc, radio[id], cagVel.v, ids[id])};
                        }
                        //*/

                        //if (cagVel.v < 1e-5) cagVel.v = 1e-10;
                        //trajectory = std::unique_ptr<TrajectoryAgent>{new LinearAgent(agLoc, cagLoc, radio[id], cagVel.v, id)};
                        std::vector<Range> collidingTraj;
                        trajectory->IntersectingTrajectories(collidingTraj);
                        originInside = originInside || trajectory->GetOriginInside();
                        totalCollidingTraj.push_back(collidingTraj);
                        id++;
                    }
                }
            }
        }
        //Fusion of the whole set of different trajectories computed
        // andrew
        std::vector<Range> finalCollidingTraj = FusionCollidingTrajectories(totalCollidingTraj);
        int nradios = 10;
        bool stretches = false;
        unsigned traverse = 1;
        double timeHorizon = th;
        id = 0;
        for (int k = 0; k <= lookAhead; k++) {
            std::vector<std::pair<Tsc, Velocidad>> states = lookAheadStates[k];
            for (auto itS = states.begin(); itS != states.end(); ++itS) {
                Tsc agLoc = (*itS).first;

                for (auto itTotalCag = totalStatesCAg.begin(); itTotalCag != totalStatesCAg.end(); ++itTotalCag) {
                    std::vector<std::vector<std::pair<Tsc, Velocidad>>> lookAheadStatesCAg = (*itTotalCag);
                    std::vector<std::pair<Tsc, Velocidad>> statesCAg = lookAheadStatesCAg[k];
                    for (auto itSCAg = statesCAg.begin(); itSCAg != statesCAg.end(); ++itSCAg) {

                        //Compute Collision Band (CB) of the obstacle: instance of a linear or circular agent
                        std::unique_ptr<TrajectoryAgent> trajectory;
                        Tsc cagLoc = (*itSCAg).first;
                        Velocidad cagVel = (*itSCAg).second;
                        ///*
                        if (is_segment[id]){
                            cagLoc.x = first_points_vec[id].x;
                            cagLoc.y = first_points_vec[id].y;
                            Tsc sist2 = Tsc(last_points_vec[id].x, last_points_vec[id].y, cagLoc.tita);
                            trajectory = std::unique_ptr<TrajectoryAgent>{new StaticAgent(agLoc, cagLoc, radio[id], cagVel.v, ids[id], true, this->GetRealRadius()*config::safety_factor, sist2)};
                        }
                        else if (cagVel.v < 0.1) {
                            //if (!(*it)->IsFinished(0.7))
                            trajectory = std::unique_ptr<TrajectoryAgent>{new StaticAgent(agLoc, cagLoc, radio[id], cagVel.v, ids[id])};
                        } else {
                            if (std::abs(cagVel.w) > 1e-3){
                                trajectory = std::unique_ptr<TrajectoryAgent>{
                                        new CircularAgent(agLoc, cagLoc, radio[id], cagVel.v, cagVel.w, stept, ids[id])};
                            }
                            else{
                                trajectory = std::unique_ptr<TrajectoryAgent>{
                                        new LinearAgent(agLoc, cagLoc, radio[id], cagVel.v, ids[id])};
                            }
                            // trajectory = std::unique_ptr<TrajectoryAgent> {new LinearAgent(agLoc, cagLoc, radio[id], cagVel.v, ids[id])};
                        }
                        //*/

                        //if (cagVel.v < 1e-5) cagVel.v = 1e-10;
                        //trajectory = std::unique_ptr<TrajectoryAgent>{new LinearAgent(agLoc, cagLoc, radio[id], cagVel.v, id)};

                        dovt dovtAg(id, radio[id], cagVel.v, bounds.vlim_max);
                        // andrew -> agLoc
                        trajectory->ComputeDOVT(finalCollidingTraj, (unsigned) nradios, stretches, traverse, radio[id],
                                                bounds, timeHorizon, accConst, Velocidad(this->GetV(), this->GetW()), 
                                                constraints(this->GetAV(), this->GetAW()), dovtAg, agLoc);
                        if (!dovtAg.GetCommands().empty()) {
                            ids2.push_back(ids[id]);
                            commands2d.push_back(dovtAg);
                        }
                        id++;

                        //Plot the VS environment
                        /*
                        VS space(bounds); Tpf goal; Tpf goalAg;
                        this->GetCurrentGoal(goal); Tsc loc = this->GetLocalization();
                        transfor_inversa_p(goal.x, goal.y, &loc, &goalAg);
                        space.InsertGoal(goalAg, stept);
                        PlotVSEnvironment(dovtAg.GetCommands(), bounds, space.GetGoal(), stept);
                        //*/
                    }
                }
            }
        }
    }

    //Compare each command in each dovt computed, they are the same radius
    auto itC = commands2d.begin();
    int idd = 0;
    std::vector<int> fusion2dAgentsNew, fusion2dAgentsOut;
    fusion2dAgents.clear();
    if (itC != commands2d.end()){
        fusion2d = (*itC).GetCommands();
        for (int h = 0; h < (*itC).GetCommands().size(); h++) {
            fusion2dAgents.push_back(ids2[idd]);
        }
        ++itC;
        idd++;
    }     
    while (itC != commands2d.end()) {
        for (int h = 0; h < (*itC).GetCommands().size(); h++) fusion2dAgentsNew.push_back(ids2[idd]);
        fusion2d = CompareDOVT(fusion2d, (*itC).GetCommands(), fusion2dAgents, fusion2dAgentsNew, fusion2dAgentsOut);
        fusion2dAgents.clear();
        for (int h=0; h < fusion2dAgentsOut.size(); h++) 
            fusion2dAgents.push_back(fusion2dAgentsOut[h]);

        fusion2dAgentsOut.clear();
        fusion2dAgentsNew.clear();
        ++itC;
        idd++;

        VS space(bounds); Tpf goal; Tpf goalAg;
        this->GetCurrentGoal(goal); Tsc loc = this->GetLocalization();
        transfor_inversa_p(goal.x, goal.y, &loc, &goalAg);
        space.InsertGoal(goalAg, stept);
//        space.SetDirGoal(dir_goal);
        // PlotVSEnvironment(fusion2d, bounds, space.GetGoal(), stept);
    }
    if (!fusion2d.empty()) {
        fusion2d.push_back(Command());
        fusion2dAgents.push_back(-1);
    }
}


void ActiveAgent::ModelAgents(std::vector<std::unique_ptr<Agent>> *agents, const double stept, int lookAhead,
                              const double th, const bool accConst) {
//Computation of DOVT (velocities and times of collision) for each agent

    //clear command vector
    commands2d.clear();
    fusion2d.clear();
    commands2dAhead.clear();

    fusionCommands = false;

    //Clean the plot
    SeleccionaVentana(this->GetId()+1);
    LimpiaVentana();

    bool insideCB = false;
    originInside = false;

    std::string path = ros::package::getPath("rl_dovs");
    std::string nameFile = path + "/timeComputeDOVT2_ag"+std::to_string(this->GetId())+".dat";
    std::ofstream fdata(nameFile, std::ios::app);
    clock_t start = clock();

    std::vector<std::unique_ptr<Agent>>::iterator it;
    for (it = agents->begin(); it != agents->end(); ++it){
        if ((*it)->GetId() != this->GetId()){   //For all the agents except itself
            //if Consider() ...

            double timeHorizon = th;
            double radio = (this->GetRealRadius() + (*it)->GetRealRadius())*config::safety_factor;
            //Compute Collision Band (CB) of the obstacle: instance of a linear or circular agent
            std::unique_ptr<TrajectoryAgent> trajectory;
            /*
            if ((*it)->GetV() < 1e-5) {
                //if (!(*it)->IsFinished(0.7))
                    trajectory = std::unique_ptr<TrajectoryAgent>{new StaticAgent(this->loc, (*it)->GetLocalization(), radio, (*it)->GetV(), (*it)->GetId())};
                    double disBreaking = (this->GetV()*this->GetV())/(2*this->GetAV());
                    double disCollision = Distancia(this->GetLocalization().x - (*it)->GetLocalization().x, this->GetLocalization().y - (*it)->GetLocalization().y) - (this->GetRealRadius() + (*it)->GetRealRadius());
                    //if (std::abs(disCollision - disBreaking) > (this->GetRealRadius() + (*it)->GetRealRadius())) timeHorizon = this->GetV()/this->GetAV();   //time needed for the agent to stop
                    //std::cout << "Distancia: " << disCollision << ", d_breaking: " << disBreaking << std::endl;
            }else{
                if (std::abs((*it)->GetW()) > 1e-3)
                    trajectory = std::unique_ptr<TrajectoryAgent> {new CircularAgent(this->loc, (*it)->GetLocalization(), radio, (*it)->GetV(), (*it)->GetW(), stept, (*it)->GetId())};
                else
                    trajectory = std::unique_ptr<TrajectoryAgent> {new LinearAgent(this->loc, (*it)->GetLocalization(), radio, (*it)->GetV(), (*it)->GetId())};

                //trajectory = std::unique_ptr<TrajectoryAgent> {new LinearAgent(this->loc, (*it)->GetLocalization(), this->GetRealRadius() + (*it)->GetRealRadius(), (*it)->GetV(), (*it)->GetId())};
            }
            //*/

            if ((*it)->GetV() < 1e-5 && !(*it)->IsFinished(0.7))
                (*it)->SetVelocity({1e-3, (*it)->GetW()});
            trajectory = std::unique_ptr<TrajectoryAgent> {new LinearAgent(this->loc, (*it)->GetLocalization(), this->GetRealRadius() + (*it)->GetRealRadius(), (*it)->GetV(), (*it)->GetId())};

            std::vector<Range> collidingTraj;
            trajectory->IntersectingTrajectories(collidingTraj);

            int nradios = 10; bool stretches = false; unsigned traverse = 1;
            //std::vector<Command> dovtAg; //dovtAg.reserve(2*comm_max);  //REVISAR SI SE PUEDE HACER MÁS EFICIENTE (NO RESERVAR TANTO ESPACIO FUERA Y DENTRO DE LA FUNCIÓN)
            //dovtAg = trajectory->ComputeDOVT(collidingTraj, (unsigned) nradios, stretches, traverse, radio, bounds);
            dovt dovtAg((*it)->GetId(), this->GetRealRadius() + (*it)->GetRealRadius(), (*it)->GetV(), bounds.vlim_max);
            trajectory->ComputeDOVT(collidingTraj, (unsigned) nradios, stretches, traverse, radio, bounds, timeHorizon,
                                    accConst, Velocidad(this->GetV(), this->GetW()), {this->GetAV(), this->GetAW()}, dovtAg, this->GetLocalization());
            if (!dovtAg.GetCommands().empty()) commands2d.push_back(dovtAg);

            //Plot the VS environment
            VS space(bounds); Tpf goal; Tpf goalAg;
            this->GetCurrentGoal(goal); Tsc loc = this->GetLocalization();
            transfor_inversa_p(goal.x, goal.y, &loc, &goalAg);
            space.InsertGoal(goalAg, stept);
            //PlotVSEnvironment(dovtAg.GetCommands(), bounds, space.GetGoal(), stept);    //PlotVSEnvironment(relativeAg->GetCommands(), boundsVS);

            //INI: LookAhead positions and velocities for the agent
            ////for each lookAhead, for each available velocity from the current velocity, we store the resultant positions and the velocities applied
            std::vector<std::vector<std::pair<Tsc,Velocidad>>> lookAheadStates = ComputeStatesAhead(Tsc(), Velocidad(this->GetV(), this->GetW()), constraints(av, aw), bounds, stept, lookAhead);
            std::vector<std::vector<std::pair<Tsc,Velocidad>>> lookAheadStatesCAg = ComputeStatesAhead(trajectory->GetLocalization(), Velocidad((*it)->GetV(), (*it)->GetW()), constraints((*it)->GetAV(), (*it)->GetAW()),
                                                                                                       bounds, stept, lookAhead);

            //PlotStatesAhead(lookAheadStates, lookAheadStatesCAg);
            /*
            if (lookAhead > 0) {
                for (int k = 0; k<lookAhead; k++) {
                    std::vector<std::unique_ptr<Agent>> agentsAhead;
                    std::vector<std::pair<Tsc, Velocidad>> states = lookAheadStates[k];
                    for (int s = 0; s < states.size(); s++) {
                        Tsc locAhead = states[s].first;
                        Velocidad velAhead = states[s].second;

                        std::unique_ptr<ActiveAgent> ag {new ActiveAgent(this->GetId(), locAhead.x, locAhead.y, locAhead.tita, velAhead.v, velAhead.w, av, aw, this->GetRealRadius())};
                        //ag->AddGoal(x2, y2);
                        Tpf goal; this->GetCurrentGoal(goal);
                        ag->AddGoal(goal.x, goal.y);
                        ag->SetBoundsVS(stept);
                        agentsAhead.push_back(std::move(ag));

                        std::vector<std::pair<Tsc, Velocidad>> statesCAg = lookAheadStatesCAg[k];
                        for (int sCAg = 0; sCAg < (int) statesCAg.size(); sCAg++) {
                            Tsc locCAgAhead = statesCAg[sCAg].first;
                            Velocidad velCAgAhead = statesCAg[sCAg].second;

                            std::unique_ptr<ActiveAgent> agCAg {new ActiveAgent((*it)->GetId(), locCAgAhead.x, locCAgAhead.y, locCAgAhead.tita, velCAgAhead.v, velCAgAhead.w, av, aw, (*it)->GetRealRadius())};
                            //ag->AddGoal(x2, y2);
                            Tpf goal; (*it)->GetCurrentGoal(goal);
                            agCAg->AddGoal(goal.x, goal.y);
                            agCAg->SetBoundsVS(stept);
                            agentsAhead.push_back(std::move(agCAg));

                            std::unique_ptr<TrajectoryAgent> trajectoryAhead;
                            if (velCAgAhead.v < 1e-5) {
                                //if (!(*it)->IsFinished(0.7))
                                trajectoryAhead = std::unique_ptr<TrajectoryAgent>{new StaticAgent(locAhead, locCAgAhead, radio, velCAgAhead.v, (*it)->GetId())};
                                double disBreaking = (this->GetV()*this->GetV())/(2*this->GetAV());
                                double disCollision = Distancia(this->GetLocalization().x - (*it)->GetLocalization().x, this->GetLocalization().y - (*it)->GetLocalization().y) - (this->GetRealRadius() + (*it)->GetRealRadius());
                                //if (std::abs(disCollision - disBreaking) > (this->GetRealRadius() + (*it)->GetRealRadius())) timeHorizon = this->GetV()/this->GetAV();   //time needed for the agent to stop
                                //std::cout << "Distancia: " << disCollision << ", d_breaking: " << disBreaking << std::endl;
                            }else{
                                if (std::abs(velCAgAhead.w) > 1e-3)
                                    trajectoryAhead = std::unique_ptr<TrajectoryAgent> {new CircularAgent(locAhead, locCAgAhead, radio, velCAgAhead.v, velCAgAhead.w, stept, (*it)->GetId())};
                                else
                                    trajectoryAhead = std::unique_ptr<TrajectoryAgent> {new LinearAgent(locAhead, locCAgAhead, radio, velCAgAhead.v, (*it)->GetId())};

                                //trajectory = std::unique_ptr<TrajectoryAgent> {new LinearAgent(this->loc, (*it)->GetLocalization(), this->GetRealRadius() + (*it)->GetRealRadius(), (*it)->GetV(), (*it)->GetId())};
                            }

                            std::vector<Range> collidingTraj;
                            trajectoryAhead->IntersectingTrajectories(collidingTraj);

                            int nradios = 10; bool stretches = false; unsigned traverse = 1;
                            dovt dovtAgAhead((*it)->GetId(), radio, velCAgAhead.v, bounds.vlim_max);
                            trajectoryAhead->ComputeDOVT(collidingTraj, (unsigned) nradios, stretches, traverse, radio,
                                                         bounds, timeHorizon, 0, Velocidad(), constraints(),
                                                         dovtAgAhead);
                            if (!dovtAgAhead.GetCommands().empty()) commands2d.push_back(dovtAgAhead);

                            PlotVSEnvironment(dovtAgAhead.GetCommands(), bounds, space.GetGoal(), stept);    //PlotVSEnvironment(relativeAg->GetCommands(), boundsVS);
                        }

                        //Goal with respect to the agent
                        Tpf goalAg;
                        Tpf goalCurrent = goals[current_goal];
                        transfor_inversa_p(goalCurrent.x, goalCurrent.y, &locAhead, &goalAg);

                        Strategies planner(stept, bounds);
                        planner.space.InsertAgent(velAhead, av, aw); //Add agent's dynamic window information
                        planner.space.InsertGoal(goalAg, stept);    //Add goal information

                        std::vector<std::vector<Command>> commandsMerged;   //commandsMerged.reserve(comm_max*obst_max);
                        if (!commands2d.empty())
                            commandsMerged = planner.space.InsertDOV(commands2d, &agentsAhead, stept, *ag);

                    }
                }
            }
            //*/

            //DOVT computation for the lookAhead information
            dovtAhead dataAheadDOVT;
            if (lookAhead > 0){
                dataAheadDOVT = ComputeDOVT_LA(dovtAg, lookAhead, lookAheadStates, trajectory->GetLocalization(), lookAheadStatesCAg, stept, bounds);
                commands2dAhead.push_back(dataAheadDOVT);
            }

            //GeometryCGAL geom(commands2d);
            //Velocidad vScape = geom.MinDistance(Velocidad(this->GetV(), this->GetW()));
            //geom.ConvexHull(commands2d, commands2dAhead);

            //PlotDOVT(dovtAg, lookAhead, dataAheadDOVT);

            //FIN: LookAhead positions and velocities for the agent

            //Store if the agent lies inside the collision band of any DOV
            originInside = originInside || trajectory->GetOriginInside();

            //PlotVSEnvironment(dovtAg, bounds);
        }
    }
    clock_t finish = clock();
    fdata << ((double)(finish-start))/CLOCKS_PER_SEC << std::endl;

    fdata << std::endl;
    fdata.close();
}

Velocidad ActiveAgent::MotionGreedy(double time_step, std::vector<std::unique_ptr<Agent>> *agents, bool video){
    //Goal with respect to the agent
    Tpf goalAg;
    Tpf goal = goals[current_goal];
    transfor_inversa_p(goal.x, goal.y, &loc, &goalAg);
    Strategies planner(time_step, bounds);
    planner.space.InsertAgent(Velocidad(v, w), av, aw); //Add agent's dynamic window information

    planner.space.InsertGoal(goalAg, time_step);    //Add goal information


    //SeleccionaVentana(this->GetId()+1);
    //LimpiaVentana();
    if (fusionCommands){
        planner.space.SetCommands(fusion2d);

        PlotVSEnvironment(fusion2d, bounds, planner.space.GetGoal(), time_step, video);

    }else{
        std::vector<std::vector<Command>> commandsMerged;   //commandsMerged.reserve(comm_max*obst_max);
        if (!commands2d.empty())
            commandsMerged = planner.space.InsertDOV(commands2d, agents, time_step, *this);

        //if graph
        if (commandsMerged.empty()) PlotVSEnvironment(std::vector<Command>(), bounds, planner.space.GetGoal(), time_step, video);
        else {
            for (auto it = commandsMerged.begin(); it != commandsMerged.end(); ++it)
                PlotVSEnvironment((*it), bounds, planner.space.GetGoal(), time_step, video);   //PlotVSEnvironment(relativeAg->GetCommands(), boundsVS);
            commandsMerged.clear();
        }
    }
    return planner.Greedy();

}

// add graph for greedy etc
Velocidad ActiveAgent::MotionStrategy(double time_step, std::vector<std::unique_ptr<Agent>> *agents, bool video, int iteracion, bool graph, bool debug) {

    // Goal with respect to the agent
    Tpf goalAg;
    Tpf goal = goals[current_goal];
    transfor_inversa_p(goal.x, goal.y, &loc, &goalAg);

    Strategies planner(time_step, bounds);
    planner.space.InsertAgent(Velocidad(v, w), av, aw); //Add agent's dynamic window information

    planner.space.InsertGoal(goalAg, time_step);    //Add goal information
    // cout << "planner insert goal " << goalAg.x << ", " << goalAg.y <<": " << planner.space.GetGoal().commandGoal.v << ", " << planner.space.GetGoal().commandGoal.w << endl;
    planner.space.SetDirGoal(dir_goal);
    if (graph || video) {
        SeleccionaVentana(this->GetId()+1);
        LimpiaVentana();
    }
    if (fusionCommands){
        planner.space.SetCommands(fusion2d);
        // if (graph || video)
        //     PlotVSEnvironment(fusion2d, bounds, planner.space.GetGoal(), time_step, video, iteracion, debug);

    }else{
        std::vector<std::vector<Command>> commandsMerged;   //commandsMerged.reserve(comm_max*obst_max);
        if (!commands2d.empty())
            commandsMerged = planner.space.InsertDOV(commands2d, agents, time_step, *this);

        //if graph
        if (commandsMerged.empty()) PlotVSEnvironment(std::vector<Command>(), bounds, planner.space.GetGoal(), time_step);
        else {
            for (auto it = commandsMerged.begin(); it != commandsMerged.end(); ++it)
                PlotVSEnvironment((*it), bounds, planner.space.GetGoal(), time_step);   //PlotVSEnvironment(relativeAg->GetCommands(), boundsVS);
            commandsMerged.clear();
        }
    }

    //PARAMETERS FOR STRATEGIES-PLANNING
    double w_max_steering, ang_left, ang_right, wz2, walign, valley, goal_valley, vorientacion, worientacion;
    std::string path = ros::package::getPath("rl_dovs");
    std::string filename = path + "/data/heuristics.txt";
    std::ifstream f;
    f.open(filename);
    f >> w_max_steering; f >> ang_left; f >> ang_right; f >> wz2; f >> walign;
    f >> valley; f >> goal_valley; f >> vorientacion; f >> worientacion;
    f.close();

    planner.SetHeuristicValues(w_max_steering, ang_left, ang_right, wz2, walign, valley, goal_valley, vorientacion, worientacion);

    Velocidad motion;
    // cout << "2: planner insert goal " << goalAg.x << ", " << goalAg.y <<": " << planner.space.GetGoal().commandGoal.v << ", " << planner.space.GetGoal().commandGoal.w << endl;
    planner.ComputeMotion(this->GetId(), this->GetLocalization(),  goals[current_goal], agents, originInside, motion);
    // cout << "3: planner insert goal " << goalAg.x << ", " << goalAg.y <<": " << planner.space.GetGoal().commandGoal.v << ", " << planner.space.GetGoal().commandGoal.w << endl;

    if (graph || video) {
    //Plot the dir_goal finally computed to be reached
        //if (graph || video)

        SeleccionaVentana(this->GetId()+1);
        goalVS goal_plot = planner.space.GetGoal();
        goal_plot.dirGoal.v = planner.GetDirGoal().v;
        goal_plot.dirGoal.w = planner.GetDirGoal().w;
        PlotVSEnvironment(fusion2d, bounds, goal_plot, time_step, video, iteracion, debug);
        // PlotVSEnvironment(fusion2d, bounds, planner.space.GetGoal(), time_step, video, iteracion, debug);
        color("white");
        graf(bounds.wmax_right, bounds.wmax_left, bounds.wmax_right, 0.5, bounds.vlim_min, bounds.vlim_max, bounds.vlim_min, 0.5);
        float x[2], y[2];
        x[0] = 0.0; y[0] = 0.0;
        x[1] = planner.GetDirGoal().w; y[1] = planner.GetDirGoal().v;
        color("blue");
        linwid(1);
        rline(x[0], y[0], x[1], y[1]);
        linwid(1);
        TerminaEjes();
    }
    dir_goal = planner.GetDirGoal();

    return motion;
}

Velocidad ActiveAgent::MotionPlan(double time_step, double time_horizon, unsigned ndata, unsigned mdata, unsigned long numC,
                                  int lookAhead) {

    //Goal with respect to the agent
    Tpf goalAg;
    Tpf goal = goals[current_goal];
    transfor_inversa_p(goal.x, goal.y, &loc, &goalAg);

    //Define cell sizes to the maximum linear and angular accelerations of the Agent
    //if (stepw > aw*time_step)
    unsigned n = std::ceil(std::abs(bounds.wmax_right - bounds.wmax_left) / (aw * time_step));
    //if (stepv > av*time_step)
    unsigned m = std::ceil(bounds.vlim_max / (av * time_step));

    //Planner: A*
    Astar astar(time_step, time_horizon, n, m, bounds);

    //Built of DOVTS with the obstacles, agent and goal mapped

    //Translate dynamic information of agents into DOVTS for planner
    std::string path = ros::package::getPath("rl_dovs");
    std::string nameFile = path + "/timeFillMap2_ag"+std::to_string(this->GetId())+".dat";
    std::ofstream fdata(nameFile, std::ios::app);
    clock_t start = clock();
    for (auto it = commands2d.begin(); it != commands2d.end(); ++it) {
        std::vector<Command> commandsDOVT = (*it).GetCommands();
        astar.space.InsertDOVT(commandsDOVT, numC);
    }
    clock_t finish = clock();
    fdata << ((double)(finish-start))/CLOCKS_PER_SEC << std::endl;
    fdata << std::endl;
    fdata.close();

    /*
    auto itDOVT = commands2d.begin();

    for (auto it = commands2dAhead.begin(); it != commands2dAhead.end(); ++it) {
        auto dovtInf = it->infAhead;
        auto dovtSup = it->supAhead;
        std::vector<Command> dovt = (itDOVT != commands2d.end() ? (*itDOVT).GetCommands() : std::vector<Command>());
        std::vector<Comando> infCommandDOVT;
        std::vector<Comando> supCommandDOVT;
        for (int i = 0; i<(int)dovt.size();i++){
            if (dovt[i].objeto != 0){
                infCommandDOVT.push_back(dovt[i].inf);
                supCommandDOVT.push_back(dovt[i].sup);
            }
        }
        astar.space.InsertDOVTAhead(dovtInf, infCommandDOVT, lookAhead, numC);
        astar.space.InsertDOVTAhead(dovtSup, supCommandDOVT, lookAhead, numC);
        ++itDOVT;
    }
    */

    nameFile = "timeFillCosts2_ag"+std::to_string(this->GetId())+".dat";
    fdata.open(nameFile, std::ios::app);
    start = clock();
    astar.space.FillCosts();    //Add cost to the cells
    finish = clock();
    fdata << ((double)(finish-start))/CLOCKS_PER_SEC << std::endl;
    fdata << std::endl;
    fdata.close();

    astar.space.InsertAgent(Velocidad(v, w), av, aw);   //Add agent's dynamic window information

    astar.space.InsertGoal(goalAg, time_step);  //Add goal information

    //Compute the goal to be followed: any of the approximations or the exact one
    astar.ComputeFollowGoal(goalAg);

    //std::list<std::pair<Comando, int>> velTarget;
    //velTarget.push_back(std::make_pair(Comando(astar.GetFollowGoal(), astar.GetFollowTime()), 0));

    //Make the plan
    std::list<Velocidad> plan;
    double cost = 0;
    nameFile = "timeAstar2_ag"+std::to_string(this->GetId())+".dat";
    fdata.open(nameFile, std::ios::app);
    start = clock();
    //bool computed = astar.MakePlan(this->GetLocalization(), goals[current_goal], velTarget, plan, cost);
    bool computed = astar.ComputePlan(plan);
    finish = clock();
    fdata << ((double)(finish-start))/CLOCKS_PER_SEC << std::endl;
    fdata << std::endl;
    fdata.close();

    std::string buffer;
    std::ostringstream buff;
    buff << std::fixed << std::setprecision(2) << cost;
    buffer = buff.str();
    //gp << "set label at graph 1,1,1 \"Cost: " << buffer << "\" lc 004\n";
    gp << "set label at graph 1,1,1 \"Id: " << this->GetId() << "\" lc 004\n";
    //Gnuplot gp1;
    //astar.space.Plot(gp, astar.GetFollowGoal(), astar.GetFollowTime());

    /*//Considering the DOVT ahead
    system("cp dovt_forbidden.dat dovt_forbiddenDOVT.dat");

    astar.space.ClearVTSpace();
    for (auto it = commands2d.begin(); it != commands2d.end(); ++it) {
        std::vector<Command> dovt = (*it).commands;
        astar.space.InsertDOVT(dovt, numC);
    }
    auto itDOVT = commands2d.begin();

    for (auto it = commands2dAhead.begin(); it != commands2dAhead.end(); ++it) {
        auto dovtInf = it->infAhead;
        auto dovtSup = it->supAhead;
        std::vector<Command> dovt = (itDOVT != commands2d.end() ? (*itDOVT).commands : std::vector<Command>());
        std::vector<Comando> infCommandDOVT;
        std::vector<Comando> supCommandDOVT;
        for (int i = 0; i<(int)dovt.size();i++){
            if (dovt[i].objeto != 0){
                infCommandDOVT.push_back(dovt[i].inf);
                supCommandDOVT.push_back(dovt[i].sup);
            }
        }
        astar.space.InsertDOVTAhead(dovtInf, infCommandDOVT, lookAhead, numC);
        astar.space.InsertDOVTAhead(dovtSup, supCommandDOVT, lookAhead, numC);
        ++itDOVT;
    }
    astar.space.FillCosts();

    astar.space.InsertAgent(Velocidad(v, w), av, aw); //Add agent's dynamic window information
    astar.space.InsertGoal(goalAg, time_step); //Add goal information
    //Compute the goal to be followed: any of the approximations or the exact one
    astar.ComputeFollowGoal(goalAg);

    plan.clear(); cost = 0;
    astar.MakePlan(this->GetLocalization(), goals[current_goal], plan, cost);
    //*/

    /*
    //Plot VS environment
    for (auto it = commands2d.begin(); it != commands2d.end(); ++it)
        PlotVSEnvironment((*it).GetCommands(), bounds, astar.space.GetGoal(), time_step);    //PlotVSEnvironment(relativeAg->GetCommands(), boundsVS);

    /*
    //Plot results
    //    if (graph) {
    std::string buffer;
    std::ostringstream buff;
    buff << std::fixed << std::setprecision(2) << cost;
    buffer = buff.str();
    gp << "set label at graph 1,1,1 \"Cost: " << buffer << "\" lc 004\n";
    Gnuplot gp2;
    astar.space.Plot(gp2, astar.GetFollowGoal(), astar.GetFollowTime());
    //gp2 << "splot 'dovt_forbiddenDOVT.dat' w l lc 001, 'dovt_forbidden.dat' w l lc 002\n";
    //gp2.flush();
    //    }
    //*/

    //The algorithm always returns a plan, even though it is a bad plan
    //plannedPath = plan;

    //Velocidad vel = plan.front();
    //plan.pop_front();

    /*//In case the algorithm computes a plan or it returns an empty plan
    //Apply the computed plan, or as many velocities so as a new plan is computed
    if (!currentPath.empty()){
        Velocidad vel = plan.front();
        plan.pop_front();

        robot.SetV(vel.v);
        robot.SetW(vel.w);
        plannedPath = plan;

    }else if (!plannedPath.empty()){

        Velocidad vel = plannedPath.front();
        plannedPath.pop_front();

        robot.SetV(vel.v);
        robot.SetW(vel.w);

    } //else: the robot mantains the same velocity
    */

    //WriteMetricsLog(cost);

    if (computed)
        return plan.front();
    else{
        // std::cout << "NO PLAN COMPUTED!" << std::endl;
        //Slow down
        double v = GetV() - av*time_step >= 0 ? GetV() - av*time_step : 0;
        double w = 0;
        if (GetW() > 0)
            w = GetW() - aw*time_step >= 0 ? GetW() - aw*time_step : 0;
        else if (GetW() < 0)
            w = GetW() + aw*time_step <= 0 ? GetW() + aw*time_step : 0;

        return {v,w};
        //return {GetV(), GetW()};
    }
}

void ActiveAgent::PlotVSEnvironment(const std::vector<Command> &commands, boundsVS bounds, goalVS goal, double t, bool video, int iteracion, bool debug){
    //Plot the robot, goal and obstacles on the Velocity Space; i.e., on terms of velocity
    //if (strcmp(library, dis_lib.c_str()) == 0) {    //dislin library
    std::vector<std::vector<double>> mxray, myray, mzray;
    std::vector<int> mnum_eltos;
    int obj_din = 0;
    std::vector<std::vector<double>> mxray1, myray1, mzray1;
    std::vector<int> mnum_eltos1;
    mxray1.reserve(obst_max * comm_max);
    myray1.reserve(obst_max * comm_max);
    mzray1.reserve(obst_max * comm_max);
    mnum_eltos1.reserve(obst_max);
    int obj_din1 = 0;

    double dinDcha = w + aw*t; // ((w + aw*t > bounds.wmax_left) ? bounds.wmax_left : w+aw*t);
    double dinIzda = w - aw*t; //((w - aw*t < bounds.wmax_right) ? bounds.wmax_right : w-aw*t);
    double dinArr = v + av*t; //((v + av*t > bounds.vlim_max) ? bounds.vlim_max : v+av*t);
    double dinAbjo = v - av*t; //((v - av*t < bounds.vlim_min) ? bounds.vlim_min : v-av*t);

    if (!video) SeleccionaVentana(this->GetId()+1);   //selwin(4);
    //LimpiaVentana();    //erase();
    std::vector<std::vector<int>> positions;

    if (!commands.empty()) CalculaValoresVarios(commands, mxray1, myray1, mzray1, mnum_eltos1, obj_din1, fusion2dAgents, positions);
    DibujaVS(this->GetId()+1, mxray1, myray1, mnum_eltos1, obj_din1, v, w,
             dinArr, dinAbjo, dinIzda, dinDcha,
             goal.commandGoal, goal.dirGoal, goal.velGoal,
             bounds, av, aw, t, video, iteracion, positions, debug);
    TerminaEjes();
}

void ActiveAgent::PlotDOVT(dovt dovtAg, int lookAhead, dovtAhead dovtAhead) {

    std::vector<std::string> color;
    for (int k = 0; k<(lookAhead+1)*2; k++){
        std::string cs = std::to_string(k+1);
        color.push_back(cs);
    }

    std::stringstream gpPlot, gpPlotC;
    gpPlot << "splot "; //gp << "plot ";
    gpPlotC << "splot ";

    //std::vector<std::vector<std::pair<double, double>>> rootGP;
    //std::vector<std::vector<std::pair<double, double>>> commandGP;
    std::vector<std::vector<std::tuple<double, double, double>>> rootGP;
    std::vector<std::vector<std::tuple<double, double, double>>> commandGP;

    //Data at current time
    //std::vector<std::pair<double, double>> rootsInf, rootsSup;
    //std::vector<std::pair<double, double>> commandsInf, commandsSup;
    std::vector<std::tuple<double, double, double>> rootsInf, rootsSup;
    std::vector<std::tuple<double, double, double>> commandsInf, commandsSup;
    std::vector<std::tuple<double, double, double>> radios;

    for (int n = 0; n<dovtAg.GetRoots().size(); n++){
        Command commandValue = dovtAg.GetValueAt(n).first;
        RootCommand rootValue = dovtAg.GetValueAt(n).second;
        //std::pair<double, double> point = std::make_pair(dovtAg.roots[n].inf.x, dovtAg.roots[n].inf.y);
        std::tuple<double, double, double> point = std::make_tuple(rootValue.inf.x, rootValue.inf.y, commandValue.inf.t);
        rootsInf.push_back(point);
        //point = std::make_pair(dovtAg.roots[n].sup.x, dovtAg.roots[n].sup.y);
        point = std::make_tuple(rootValue.sup.x, rootValue.sup.y, commandValue.inf.t);
        rootsSup.push_back(point);

        //std::pair<double, double> vel = std::make_pair(dovtAg.commands[n].inf.vel.w, dovtAg.commands[n].inf.vel.v);
        std::tuple<double, double, double> vel = std::make_tuple(commandValue.inf.vel.w, commandValue.inf.vel.v, commandValue.inf.t);
        commandsInf.push_back(vel);
        //vel = std::make_pair(dovtAg.commands[n].sup.vel.w, dovtAg.commands[n].sup.vel.v);
        vel = std::make_tuple(commandValue.sup.vel.w, commandValue.sup.vel.v, commandValue.sup.t);
        commandsSup.push_back(vel);

        if (commandValue.sup.vel.w != 0){
            std::tuple<double, double,double> radio = std::make_tuple(commandValue.sup.vel.v/commandValue.sup.vel.w, commandValue.inf.t, commandValue.sup.t);
            radios.push_back(radio);
        }
    }
    rootGP.push_back(rootsInf); rootGP.push_back(rootsSup);
    commandGP.push_back(commandsInf); commandGP.push_back(commandsSup);

    std::string str1("\n");
    if (lookAhead > 0)
        str1 = ",";
    //gpPlot << "'-' w p lc " << color[0] << " lt " << color[0] << " notitle" << str1;
    gpPlot << "'-' w p lc " << color[0] << " lt " << color[0] << " notitle , '-' w p lc " << color[lookAhead+1] << " lt " << color[lookAhead+1] << " notitle" << str1;
    gpPlotC << "'-' w p lc " << color[0] << " lt " << color[0] << " notitle , '-' w p lc " << color[lookAhead+1] << " lt " << color[lookAhead+1] << " notitle" << str1;

///*
    int size = dovtAhead.infAhead.size();
    //for (int n = 0; n<dovtAhead.infAhead.size(); n++){
    for (int n = 0; n<size; n++){
        std::vector<std::vector<std::pair<Root,Comando>>> infAheadPoint = dovtAhead.infAhead[n];

        for (int k = 0; k<lookAhead; k++) {
            //vector infAheadK contains all the positions and velocities of the root for k steps forward
            std::vector<std::pair<Root,Comando>> infAheadK = infAheadPoint[k];
            //std::vector<std::pair<double, double>> roots;
            //std::vector<std::pair<double, double>> commands;
            std::vector<std::tuple<double, double, double>> roots;
            std::vector<std::tuple<double, double, double>> commands;
            for (int m = 0; m<infAheadK.size(); m++){
                //std::pair<double, double> point = std::make_pair(infAheadK[m].first.x, infAheadK[m].first.y);
                std::tuple<double, double, double> point = std::make_tuple(infAheadK[m].first.x, infAheadK[m].first.y, infAheadK[m].second.t);
                roots.push_back(point);
                //std::pair<double, double> vel = std::make_pair(infAheadK[m].second.vel.w, infAheadK[m].second.vel.v);
                std::tuple<double, double, double> vel = std::make_tuple(infAheadK[m].second.vel.w, infAheadK[m].second.vel.v, infAheadK[m].second.t);
                commands.push_back(vel);
            }
            rootGP.push_back(roots);
            commandGP.push_back(commands);

            int c = k+1;
            std::string str(",");
            //if (k == lookAhead - 1) str = "\n";
            //if (n == size-1 && k == lookAhead-1) str = "\n";

            gpPlot << "'-' w p lc " << color[c] << " lt " << color[c] << " notitle" << str;
            gpPlotC << "'-' w p lc " << color[c] << " lt " << color[c] << " notitle" << str;

        }
    }
//*/
    ///*
    //Points sup
    size = dovtAhead.supAhead.size();
    for (int n = 0; n<size; n++){
        std::vector<std::vector<std::pair<Root,Comando>>> supAheadPoint = dovtAhead.supAhead[n];

        for (int k = 0; k<lookAhead; k++) {
            //vector infAheadK contains all the positions and velocities of the root for k steps forward
            std::vector<std::pair<Root,Comando>> supAheadK = supAheadPoint[k];
            //std::vector<std::pair<double, double>> roots;
            //std::vector<std::pair<double, double>> commands;
            std::vector<std::tuple<double, double, double>> roots;
            std::vector<std::tuple<double, double, double>> commands;
            for (int m = 0; m<supAheadK.size(); m++){
                //std::pair<double, double> point = std::make_pair(supAheadK[m].first.x, supAheadK[m].first.y);
                std::tuple<double, double, double> point = std::make_tuple(supAheadK[m].first.x, supAheadK[m].first.y, supAheadK[m].second.t);
                roots.push_back(point);
                //std::pair<double, double> vel = std::make_pair(supAheadK[m].second.vel.w, supAheadK[m].second.vel.v);
                std::tuple<double, double, double> vel = std::make_tuple(supAheadK[m].second.vel.w, supAheadK[m].second.vel.v, supAheadK[m].second.t);
                commands.push_back(vel);
            }
            rootGP.push_back(roots);
            commandGP.push_back(commands);

            int c = lookAhead+k+2;
            std::string str(",");
            //if (k == lookAhead - 1) str = "\n";
            if (n == size-1 && k == lookAhead-1) str = "\n";

            gpPlot << "'-' w p lc " << color[c] << " lt " << color[c] << " notitle" << str;
            gpPlotC << "'-' w p lc " << color[c] << " lt " << color[c] << " notitle" << str;

        }
    }
    //*/

    //LookAhead data
    gp << gpPlot.str();
    //std::vector<std::vector<std::pair<double, double>>>::reverse_iterator rIt;
    for (int m = 0; m<rootGP.size(); m++){
    //for (rIt = rootGP.rbegin(); rIt != rootGP.rend(); ++rIt){
        //std::vector<std::pair<double, double>> roots = rootGP[m];
        std::vector<std::tuple<double, double, double>> roots = rootGP[m];
        //std::vector<std::pair<double, double>> roots = (*rIt);
       /*
       if (!roots.empty()){
            std::tuple<double, double, double> r = roots.back();
            roots.clear();
            roots.push_back(r);
        }
        //*/
        gp.send1d(roots);
    }
    gp.flush();

    //Gnuplot gp1;
    gp1 << gpPlotC.str();
    //std::vector<std::vector<std::tuple<double, double, double>>>::reverse_iterator rItC;
    for (int m  = 0; m<(int)commandGP.size(); m++){
    //for (rItC = commandGP.rbegin(); rItC != commandGP.rend(); ++rItC){
        std::vector<std::tuple<double, double, double>> commands = commandGP[m];
        //std::vector<std::pair<double, double>> commands = commandGP[m];
        //std::vector<std::tuple<double, double, double>> commands = (*rItC);
        /*
        if (!commands.empty()){
            std::tuple<double, double, double> d = commands.back();
            commands.clear();
            commands.push_back(d);
        }
        //*/
        gp1.send1d(commands);
    }
    gp1.flush();

    //Gnuplot gp2; gp2 << "splot '-' w lp lc 001\n";
    //gp2.send1d(radios);
    //gp2.flush();
}

void ActiveAgent::PlotStatesAhead(const std::vector<std::vector<std::pair<Tsc,Velocidad>>>& states, const std::vector<std::vector<std::pair<Tsc,Velocidad>>>& statesCAg) {

    gp << "splot ";

    //std::vector<std::vector<std::pair<double, double>>> dataState;
    std::vector<std::vector<std::tuple<double, double, double>>> dataState;
    std::string str(",");
    for (int k = 0 ; k < (int)states.size(); k++){
        if (k+1 >= (int)states.size())
            str = "";
        //gp << "'-' w p lc " << std::to_string(k+1) << " lt " << std::to_string(k+1) << " notitle, '-' w p lc " << std::to_string(k+1) << " lt " << std::to_string(k+1) << " notitle" << str;
        gp << "'-' w p lc " << std::to_string(k+1) << " lt " << std::to_string(k+1) << " notitle" << str;
        std::vector<std::pair<Tsc,Velocidad>> data_k = states[k];
        //std::vector<std::pair<double, double>> dataGp;
        std::vector<std::tuple<double, double, double>> dataGp;
        for (auto it = data_k.begin(); it != data_k.end(); ++it){
            Tsc p = it->first; Velocidad v = it->second;
            //dataGp.push_back(std::make_pair(p.x, p.y));
            dataGp.push_back(std::make_tuple(p.x, p.y, (k+1)));
        }
        dataState.push_back(dataGp);
    }

    if (!statesCAg.empty()) {
        gp << ",";
        for (int k = 0; k < (int) statesCAg.size(); k++) {
            if (k + 1 >= (int) statesCAg.size())
                str = "\n";
            gp << "'-' w p lc " << std::to_string(k + 1) << " lt " << std::to_string(k + 1) << " notitle" << str;
            std::vector<std::pair<Tsc, Velocidad>> dataCAg_k = statesCAg[k];
            std::vector<std::tuple<double, double, double>> dataGp;
            for (auto it = dataCAg_k.begin(); it != dataCAg_k.end(); ++it) {
                Tsc p = it->first;
                Velocidad v = it->second;
                dataGp.push_back(std::make_tuple(p.x, p.y, (k + 1)));
            }
            dataState.push_back(dataGp);
        }
    }else
        gp << "\n";

    for (auto it = dataState.begin(); it != dataState.end(); ++it){
        //std::vector<std::pair<double, double>> dataGp = *it;
        std::vector<std::tuple<double, double, double>> dataGp = *it;
        gp.send1d(dataGp);
    }

    gp.flush();

}

Velocidad ActiveAgent::MotionShared(std::vector<std::unique_ptr<Agent>> *agents, double time_step, boundsVS bounds, int lookAhead){

    ///*
    GeometryCGAL geom(commands2d);
    Velocidad vScape = geom.MinDistance(Velocidad(this->GetV(), this->GetW()));
    //geom.ConvexHull(commands2d, commands2dAhead);

    DOVTS space;
    space.InsertGoal(goals[current_goal], time_step);
    //Plot VS environment
    for (auto it = commands2d.begin(); it != commands2d.end(); ++it)
        PlotVSEnvironment(it->GetCommands(), bounds, space.GetGoal(), time_step);  //PlotVSEnvironment(relativeAg->GetCommands(), boundsVS);

    SeleccionaVentana(this->GetId()+1);   //selwin(4);

    float x[5], y[5];
    x[0] = vScape.w; y[0] = vScape.v;
    linwid(12); //linwid -> sets the line width
    color("red");
    rline(x[0], y[0], x[0], y[0]);
    linwid(1);

    TerminaEjes();
    //*/

    Velocidad vel = Velocidad(this->GetV(), this->GetW());
    return vel;
}


void ActiveAgent::PlotRelativeInfo(ActiveAgent &ag){

    LinearAgent trajectory(this->loc, ag.GetLocalization(), ag.GetRealRadius() + ag.GetRealRadius(), ag.GetV(), ag.GetId());

    Eigen::Vector2d vAg1(this->GetV(), 0), vAg2(trajectory.GetVX(), trajectory.GetVY());

    std::vector<std::pair<double,double>> pointsAg1, pointsAg2;
    pointsAg1.push_back(std::make_pair(0,0));
    pointsAg1.push_back(std::make_pair(10,0));

    pointsAg2.push_back(std::make_pair(trajectory.GetLocalization().x,trajectory.GetLocalization().y));
    pointsAg2.push_back(std::make_pair(trajectory.GetLocalization().x + 10*std::cos(trajectory.GetLocalization().tita),
                                       trajectory.GetLocalization().y + 10*std::sin(trajectory.GetLocalization().tita)));

    char eti[20] = "Ag ";
    std::string id = std::to_string(this->GetId());
    strcat(eti, id.c_str());

    gp << "reset\n";
    gp << "xAg1 = " << 0 << "\n"; gp << "yAg1 = " << 0 << "\n"; gp << "eti = \"" << eti << "\"\n";
    gp << "set label at xAg1,yAg1 eti point pointtype 7 pointsize 1 lc 001\n"; //blue
    strcpy(eti, "Ag "); id = std::to_string(ag.GetId());
    strcat(eti, id.c_str());
    gp << "xAg1 = " << trajectory.GetLocalization().x << "\n"; gp << "yAg1 = " << trajectory.GetLocalization().y << "\n"; gp << "eti = \"" << eti << "\"\n";
    gp << "set label at xAg1,yAg1 eti point pointtype 7 pointsize 1 lc 002\n"; //blue

    Line trajAg = trajectory.GetTrajectory();
    Tsc raiz; int sol;
    SolDosRectas(trajAg, Line(0,1,0), raiz, sol);
    std::vector<std::tuple<double, double, double, double>> points4d, points4d1;
    if (sol > 0){
        points4d.push_back(std::make_tuple(raiz.x, raiz.y, vAg1.x(), vAg1.y()));
        points4d.push_back(std::make_tuple(raiz.x, raiz.y, vAg2.x(), vAg2.y()));
        std::cout << "Dist pto corte: " << std::sqrt(std::pow(raiz.x, 2)+std::pow(raiz.y,2)) << std::endl;

        gp << "x = " << raiz.x << "\n";
        gp << "y = " << raiz.y << "\n";
        gp << "set label at x,y \"\" point pointtype 7 pointsize 2 lc 003\n"; //blue
    }
    gp << "plot '-' w l lc 001, '-' w l lc 002, '-' w vectors filled head lc 003 lw 3\n"; //, '-' w vectors filled head lc 003 lw 3\n";
    gp.send1d(pointsAg1);
    gp.send1d(pointsAg2);
    gp.send1d(points4d);
    //gp.send1d(points4d1);

    gp.flush();
    //TerminaEjes();
}

bool IsVelReachable(double step, const Velocidad &current, const Velocidad & vel, boundsVS bounds, constraints c) {

    if (vel.v < 0 || vel.v > bounds.vlim_max) return false;
    if (vel.w > bounds.wmax_left || vel.w < bounds.wmax_right) return false;

    Velocidad v = { fabs(current.v - vel.v), fabs(current.w - vel.w) };

    const double maxv = (c.av*step) * (1 - v.w/(c.aw*step)); // Linear interpolation diamond side

    return v.v <= maxv;
}

bool VelocityInRange(std::pair<Velocidad,Velocidad> range, Velocidad point){

    bool inRange;
    if (range.second.w >= 0){
        inRange = point.w >= range.first.w && point.w <= range.second.w;
    }else{
        inRange = point.w <= range.first.w && point.w >= range.second.w;
    }
    return (inRange || point.v >= range.first.v && point.v <= range.second.v);
}

PosibleVelocidad
ActiveAgent::FindClosestReachableRec(Velocidad gV, double time_step, boundsVS bounds, std::string role, double d,
                                     bool safe) {

    Polygon pol;
    pol.InsertCommands(commands2d);

    Velocidad bestvel;
    double bestdist = DBL_MAX;
    bool found = false;

    const double dw = aw * time_step;
    const double dv = av * time_step;

    const double vstep = dw / 10.0; // Resolution of grid over reachable window

    Velocidad current = {v, w};

    bool orientate = false;
    Line lGoal({0,0},{gV.w, gV.v}); std::pair<Velocidad, Velocidad> segmentGoal = std::make_pair(Velocidad(),Velocidad(gV.v, gV.w));
    if (VelocityInRange(segmentGoal, current))
        orientate = lGoal.Distance({current.w, current.v}) > dv; //dw; //Distancia(dw, dv);
    else{
        double distP1 = Distancia(current.w, current.v);
        double distP2 = Distancia(current.w-gV.w, current.v-gV.v);
        double dist = (distP1 < distP2 ? distP1 : distP2);
        orientate = dist > dv; //dw; //Distancia(dw, dv);
    }
    for (double w = current.w - dw; w <= current.w + dw; w += vstep)
        for (double v = current.v - dv; v <= current.v + dv; v += vstep) {
            double dist = 0; double alfa = 1.0;
            if (orientate){
                if (VelocityInRange(segmentGoal, {v,w}))
                    dist = lGoal.Distance({w,v});   //Distance from a point to a segment
                else{
                    //Distance wrt the nearest point of the segment
                    double distP1 = Distancia(w,v);
                    double distP2 = Distancia(w-gV.w, v-gV.v);
                    dist = (distP1 < distP2 ? distP1 : distP2);
                }
                alfa = 0.5;
            }
            dist = dist + alfa * std::sqrt(std::pow(gV.v - v, 2) + std::pow(gV.w - w, 2));
            bool reachable = IsVelReachable(time_step, current, {v, w}, bounds, {av, aw});
            bool inside = pol.InsidePolygon({v, w});
            if ( reachable && dist < bestdist &&
                    (!safe || !inside)) {
                found = true;
                bestdist = dist;
                bestvel = {v,w};
            }
        }

    return { found, bestvel };
}

PosibleVelocidad ActiveAgent::FindClosestSafeRec(boundsVS bounds) {

    Polygon pol;
    pol.InsertCommands(commands2d);

    const Velocidad vel  = {v,w};
    const double gridsize = 100;

    Velocidad bestvel;
    double    bestdist = DBL_MAX;
    bool      found    = false;

    for (double w = bounds.wmax_right; w < bounds.wmax_left; w += (fabs(bounds.wmax_right - bounds.wmax_left)/gridsize))
        for (double v = 0; v < bounds.vlim_max; v += (bounds.vlim_max/gridsize))
            if (!pol.InsidePolygon({v, w})) {
                const double dist = sqrt(pow(vel.v - v, 2) + pow(vel.w - w, 2));
                if (dist < bestdist) {
                    found    = true;
                    bestdist = dist;
                    bestvel  = {v, w};
                }
            }

    return { found, bestvel };
}


Velocidad FollowGoal(Tpf goal, goalVS g, boundsVS bounds);
Velocidad ActiveAgent::MotionReciprocal(Velocidad velTarget, double time_step, boundsVS bounds, std::string role, double d) {

    //GeometryCGAL geom(commands2d);
    //Velocidad vScape = geom.MinDistance(Velocidad(this->GetV(), this->GetW()));

    VS space(bounds);
    Tpf goalAg;
    Tpf goal = goals[current_goal];
    transfor_inversa_p(goal.x, goal.y, &loc, &goalAg);
    space.InsertGoal(goalAg, time_step);

    ///*
    SeleccionaVentana(this->GetId()+1);   //selwin(4);
    //Plot VS environment
    if (commands2d.empty())
        PlotVSEnvironment(std::vector<Command>(), bounds, space.GetGoal(), time_step);  //PlotVSEnvironment(relativeAg->GetCommands(), boundsVS);
    else {
        for (auto it = commands2d.begin(); it != commands2d.end(); ++it)
            PlotVSEnvironment((*it).GetCommands(), bounds, space.GetGoal(),
                              time_step);  //PlotVSEnvironment(relativeAg->GetCommands(), boundsVS);
    }
    float x[2], y[2];
    x[0] = velTarget.w; y[0] = velTarget.v;
    linwid(20); //linwid -> sets the line width
    color("red");
    rline(x[0], y[0], x[0], y[0]);
    linwid(1);
    sendbf();
    //*/
    //TerminaEjes();

    //Velocidad gV = FollowGoal(goalAg, space.GetGoal(), space.GetBounds());

    //Compute the closest free velocity that minimizes a function
    PosibleVelocidad best = FindClosestReachableRec(velTarget, time_step, bounds, role, d, true);

    //SeleccionaVentana(this->GetId()+1);   //selwin(4);
    if (best.valid){
        float x[2], y[2];
        x[0] = best.vel.w; y[0] = best.vel.v;
        linwid(20); //linwid -> sets the line width
        color("blue");
        rline(x[0], y[0], x[0], y[0]);
        linwid(1);
    }
    TerminaEjes();


    if (best.valid) // Found a valid safe vel
        return best.vel;
    else { // Look for the closest safe vel
        best = FindClosestSafeRec(bounds);
        if (best.valid) {
            // std::cout << "FOUND ESCAPE VEL: " << best.vel.v << ", " << best.vel.w << std::endl;
            best = FindClosestReachableRec(best.vel, time_step, bounds, "ac", d, false); // Replace target vel with closest reachable vel
            if (!best.valid)
                throw std::runtime_error("No valid vel towards escape");
            return best.vel;
        }
        else { // No existing escape vel, emergency stop
            best = FindClosestReachableRec({0, 0}, time_step, bounds, "ac", d, false);
            if (!best.valid)
                throw std::runtime_error("No valid vel towards zero");
            // std::cout << "EMERGENCY STOP" << std::endl;
            return best.vel;
        }
    }

}

Velocidad
ActiveAgent::MotionReciprocal2(Velocidad velTarget, double time_step, boundsVS bounds, std::string role, double d, const int action,
                               const double prevUt) {

    //GeometryCGAL geom(commands2d);
    //Velocidad vScape = geom.MinDistance(Velocidad(this->GetV(), this->GetW()));

    VS space(bounds);
    Tpf goalAg;
    Tpf goal = goals[current_goal];
    transfor_inversa_p(goal.x, goal.y, &loc, &goalAg);
    space.InsertGoal(goalAg, time_step);

    ///*
    SeleccionaVentana(this->GetId()+1);   //selwin(4);
    //Plot VS environment
    if (commands2d.empty())
        PlotVSEnvironment(std::vector<Command>(), bounds, space.GetGoal(), time_step);  //PlotVSEnvironment(relativeAg->GetCommands(), boundsVS);
    else {
        for (auto it = commands2d.begin(); it != commands2d.end(); ++it)
            PlotVSEnvironment((*it).GetCommands(), bounds, space.GetGoal(),
                              time_step);  //PlotVSEnvironment(relativeAg->GetCommands(), boundsVS);
    }
    float x[2], y[2];
    x[0] = velTarget.w; y[0] = velTarget.v;
    linwid(20); //linwid -> sets the line width
    color("red");
    rline(x[0], y[0], x[0], y[0]);
    linwid(1);
    sendbf();
    //*/
    //TerminaEjes();

    //Velocidad gV = FollowGoal(goalAg, space.GetGoal(), space.GetBounds());

    //Compute the closest free velocity that minimizes a function
    PosibleVelocidad best = FindClosestReachableRec(velTarget, time_step, bounds, role, d, false);

    //SeleccionaVentana(this->GetId()+1);   //selwin(4);
    if (best.valid){
        float x[2], y[2];
        x[0] = best.vel.w; y[0] = best.vel.v;
        linwid(20); //linwid -> sets the line width
        color("blue");
        rline(x[0], y[0], x[0], y[0]);
        linwid(1);
    }
    TerminaEjes();


    if (best.valid) // Found a valid safe vel
        return best.vel;
    else { // Look for the closest safe vel
        best = FindClosestSafeRec(bounds);
        if (best.valid) {
            // std::cout << "FOUND ESCAPE VEL: " << best.vel.v << ", " << best.vel.w << std::endl;
            best = FindClosestReachableRec(best.vel, time_step, bounds, "ac", d, false); // Replace target vel with closest reachable vel
            if (!best.valid)
                throw std::runtime_error("No valid vel towards escape");
            return best.vel;
        }
        else { // No existing escape vel, emergency stop
            best = FindClosestReachableRec({0, 0}, time_step, bounds, "ac", d, false);
            if (!best.valid)
                throw std::runtime_error("No valid vel towards zero");
            // std::cout << "EMERGENCY STOP" << std::endl;
            return best.vel;
        }
    }

}

Velocidad VelDinGoal(Velocidad goal, boundsVS bounds);
//Velocidad ActiveAgent::MotionReciprocalDOVS(Tpf goalTarget, double time_step, boundsVS bounds, std::string role, double d, Velocidad velRef, Tsc locRel,
  //                                          std::vector<std::unique_ptr<Agent>> *agents) {
Velocidad ActiveAgent::MotionReciprocalDOVS(Velocidad velTarget, double time_step, boundsVS bounds, std::string role, double d,
                                         std::vector<std::unique_ptr<Agent>> *agents) {
//goalTarget: the goal recomputed to achieve the reciprocal avoiding collision
//velRef: goal velocity that should be followed by the agent to collaborate in the reciprocal avoiding collision strategy
    Strategies planner(time_step, bounds);

    Tpf goalAg;
    transfor_inversa_p(goals[current_goal].x, goals[current_goal].y, &loc, &goalAg);
    planner.space.InsertAgent(Velocidad(v, w), av, aw); //Add agent's dynamic window information
    planner.space.InsertGoal(goalAg, time_step); //Add goal information

    //VS space(bounds);
    //space.InsertAgent({v,w}, av, aw);
    //space.InsertGoal(goalTarget, time_step);

    std::vector<std::vector<Command>> commandsMerged; //commandsMerged.reserve(comm_max*obst_max);
    if (!commands2d.empty())
        //commandsMerged = planner.space.InsertDOV(commands2d, agents, *this);

    //if graph
    if (commandsMerged.empty()) PlotVSEnvironment(std::vector<Command>(), bounds, planner.space.GetGoal(), time_step);
    else {
        for (auto it = commandsMerged.begin(); it != commandsMerged.end(); ++it)
            PlotVSEnvironment((*it), bounds, planner.space.GetGoal(), time_step);    //PlotVSEnvironment(relativeAg->GetCommands(), boundsVS);
        commandsMerged.clear();
    }
    //*/

    //Search for closer goal in the free area in velocities in the direction of goalTarget/velRef
    //Velocidad vTarget = planner.ComputeClosest(space.GetGoal().commandGoal);
    Velocidad target = planner.ComputeClosest(velTarget);

    //Plot reciprocal goal
    float x[2], y[2];
    x[0] = 0; y[0] = 0;
    Velocidad vel = VelDinGoal(velTarget, bounds);
    x[1] = vel.w; y[1] = vel.v;
    DibujaRecta(this->GetId() + 2, x, y, 2, "blue", 12);

    //Plot recomputed goal
    x[0] = 0; y[0] = 0;
    x[1] = target.w; y[1] = target.v;
    DibujaRecta(this->GetId() + 2, x, y, 2, "yellow", 12);

    TerminaEjes();

    //GeometryCGAL geom(commands2d);
    //Velocidad vScape = geom.MinDistance(Velocidad(this->GetV(), this->GetW()));

    //Compute the closest free velocity that minimizes a function
    PosibleVelocidad best = FindClosestReachableRec(target, time_step, bounds, role, d, true);

    if (best.valid) // Found a valid safe vel
        return best.vel;
    else { // Look for the closest safe vel
        best = FindClosestSafeRec(bounds);
        if (best.valid) {
            // std::cout << "FOUND ESCAPE VEL: " << best.vel.v << ", " << best.vel.w << std::endl;
            best = FindClosestReachableRec(best.vel, time_step, bounds, "ac", d, false); // Replace target vel with closest reachable vel
            if (!best.valid)
                throw std::runtime_error("No valid vel towards escape");
            return best.vel;
        } else { // No existing escape vel, emergency stop
            best = FindClosestReachableRec({0, 0}, time_step, bounds, "ac", d, false);
            if (!best.valid)
                throw std::runtime_error("No valid vel towards zero");
            // std::cout << "EMERGENCY STOP" << std::endl;
            return best.vel;
        }
    }
}

Velocidad ActiveAgent::MotionReciprocalDOVTS(std::list<std::pair<Comando,int>> velTarget, double time_step, boundsVS bounds,
                                             std::string role) {

    //PARAMETERS FOR PLANNING
    std::string path = ros::package::getPath("rl_dovs");
    std::ifstream fp(path + "/data/planner3d_data.txt");
    double th; unsigned n, m; unsigned long numC;
    fp >> th;   //time horizon
    fp >> n; fp >> m; //dovt size
    fp >> numC;  //number of cells to filled downwards

    assert(th > 0 && n > 0 && m > 0 && numC >= 0);

    //Define cell sizes to the maximum linear and angular accelerations of the Agent
    //if (stepw > aw*time_step)
    n = std::ceil(std::abs(bounds.wmax_right - bounds.wmax_left) / ((aw * time_step)/2));
    //if (stepv > av*time_step)
    m = std::ceil(bounds.vlim_max / (av * time_step));

    //Planner: A*
    Astar astar(time_step, th, n, m, bounds);

    //Built of DOVTS with the obstacles, agent and goal mapped

    //Translate dynamic information of agents into DOVTS for planner
    for (auto it = commands2d.begin(); it != commands2d.end(); ++it) {
        std::vector<Command> dovt = (*it).GetCommands();
        astar.space.InsertDOVT(dovt, numC);
    }

    astar.space.FillCosts();    //Add cost to the cells
    astar.space.InsertAgent(Velocidad(v, w), av, aw); //Add agent's dynamic window information

    /*
    Tpf goalFollow = velTarget;
    if (role.compare("ac") == 0){
        transfor_inversa_p(goals[current_goal].x, goals[current_goal].y, &loc, &goalFollow);
    }
    astar.space.InsertGoal(goalFollow, time_step); //Add goal information
    //Compute the goal to be followed: any of the approximations or the exact one
    astar.ComputeFollowGoal(goalFollow);
    if (role.compare("co") == 0) astar.SetFollowGoal(velTarget);
    */

    Tpf goalAg;
    transfor_inversa_p(goals[current_goal].x, goals[current_goal].y, &loc, &goalAg);
    astar.space.InsertGoal(goalAg, time_step);

    astar.SetFollowGoal(velTarget.back().first.vel);

    //Make the plan
    std::list<Velocidad> plan;
    double cost = 0;
    astar.MakePlan(this->GetLocalization(), goals[current_goal], velTarget, plan, cost);

    std::string buffer;
    std::ostringstream buff;
    buff << std::fixed << std::setprecision(2) << cost;
    buffer = buff.str();
    //gp << "set label at graph 1,1,1 \"Cost: " << buffer << "\" lc 004\n";
    gp << "set label at graph 1,1,1 \"Id: " << this->GetId() << "\" lc 004\n";
    //Gnuplot gp1;
    //astar.space.Plot(gp, astar.GetFollowGoal(), astar.GetFollowTime());
///*
    SeleccionaVentana(this->GetId()+1);   //selwin(4);
    //Plot VS environment
    if (commands2d.empty())
        PlotVSEnvironment(std::vector<Command>(), bounds, astar.space.GetGoal(), time_step);  //PlotVSEnvironment(relativeAg->GetCommands(), boundsVS);
    else {
        for (auto it = commands2d.begin(); it != commands2d.end(); ++it)
            PlotVSEnvironment(it->GetCommands(), bounds, astar.space.GetGoal(),
                              time_step);  //PlotVSEnvironment(relativeAg->GetCommands(), boundsVS);
    }
    TerminaEjes();
//*/
    return plan.front();
}

void ActiveAgent::PlotVSData(boundsVS bounds, double time_step){
    VS space(bounds);
    Tpf goalAg;
    Tpf goal = goals[current_goal];
    transfor_inversa_p(goal.x, goal.y, &loc, &goalAg);
    space.InsertGoal(goalAg, time_step);

    //Plot VS environment
    if (commands2d.empty())
        PlotVSEnvironment(std::vector<Command>(), bounds, space.GetGoal(), time_step);  //PlotVSEnvironment(relativeAg->GetCommands(), boundsVS);
    else {
        for (auto it = commands2d.begin(); it != commands2d.end(); ++it)
            PlotVSEnvironment((*it).GetCommands(), bounds, space.GetGoal(),
                              time_step);  //PlotVSEnvironment(relativeAg->GetCommands(), boundsVS);
    }
    TerminaEjes();
}

Velocidad ActiveAgent::SelectVelocity(boundsVS bounds, double time_step) {

    double w = this->GetW();  double v = this->GetV();
    SeleccionaVentana(this->GetId()+1);
    Selecciona(w, v, bounds.wmax_left, bounds.wmax_right, bounds.vlim_max, bounds.vlim_min);
    TerminaEjes();

    return {v,w};
}

void ActiveAgent::WriteMetricsLog(std::vector<std::unique_ptr<Agent>> *agents){

    //MÉTRICAS PARA LA EVALUACIÓN DE LA NAVEGACIÓN
    //Almacenamos en el log la orientación del goal con respecto al agente,
    //la menor de todas las distancias a los obstáculos (distancia del agente con respecto a los demás agentes, como medida de distancia de seguridad),
    //velocidad lineal y angular del agente;
    //25-04-2017: coste del camino
    double d = 10000;
    for (auto it = agents->begin(); it != agents->end(); ++it){
        if ((*it)->GetId() != this->GetId()){
            double radio_seg = this->GetRealRadius() + (*it)->GetRealRadius();
            double distancia = Distancia(this->GetLocalization().x - (*it)->GetLocalization().x, this->GetLocalization().y - (*it)->GetLocalization().y);
            if (distancia - radio_seg < d) d = distancia - radio_seg;
        }
    }

    std::ofstream flog;
    if (nameLog.empty())
        throw std::runtime_error("No valid name for log file");

    Tpf goalAg;
    transfor_inversa_p(goals[current_goal].x, goals[current_goal].y, &loc, &goalAg);
    flog.open(nameLog, std::ios::app);
    // flog << atan2(goalAg.y, goalAg.x) << "\t" << d << "\t" << v << "\t" << w << "\t"; //"\t" << cost << std::endl;
    flog << atan2(goalAg.y, goalAg.x) << "\t" << d << "\t"; //"\t" << cost << std::endl;
    flog.close();
}

Command ActiveAgent::GetLastCommand(const int id) {
//Returns the left most command from the dovt computed for agent id (right turn)
//In case there is no dovt computed for id, it returns an empty command
    for (auto it = commands2d.begin(); it != commands2d.end(); ++it){
        if (it->GetId() == id){
            return it->GetRightmostCommand();
        }
    }
    return Command();
}

Command ActiveAgent::GetFirstCommand(const int id) {
//Returns the right most command from the dovt computed for agent id (left turn)
//In case there is no dovt computed for id, it returns an empty command
    for (auto it = commands2d.begin(); it != commands2d.end(); ++it){
        if (it->GetId() == id){
            return it->GetLeftmostCommand();
        }
    }
    return Command();
}

Command ActiveAgent::GetStraightLineCommand(const int id){
//Returns the command which corresponds to the straight line trajectory wrt to agent id
//If it does not exist, it returns and empty command
    for (auto it = commands2d.begin(); it != commands2d.end(); ++it){
        if (it->GetId() == id){
            return it->GetStraightCommand();
        }
    }
    return Command();
}

Command ActiveAgent::GetMinVelocityCommand(const int id){
//Returns the command which corresponds to the straight line trajectory wrt to agent id
//If it does not exist, it returns and empty command
    for (auto it = commands2d.begin(); it != commands2d.end(); ++it){
        if (it->GetId() == id){
            return it->GetMinVelocityCommand();
        }
    }
    return Command();
}

void ActiveAgent::SetBoundsVS(const double stept){

    //Build the bounds of the VS
    std::string path = ros::package::getPath("rl_dovs");
    std::ifstream f(path + "/data/velocity_constraints.txt");
    double vmin_adm, vmax_adm, wright_adm, wleft_adm, v_max, w_max_left, w_max_right, n;
    f >> vmin_adm; f >> vmax_adm; f >> wright_adm; f >> wleft_adm; f >> v_max; f >> w_max_left; f >> w_max_right; f >> n;
    f.close();

    bounds = boundsVS(vmin_adm, v_max, w_max_left, w_max_right, av, aw, n, stept);
}

std::vector<Velocidad> ActiveAgent::isGoalInDW (double time_step, std::vector<std::unique_ptr<Agent>> *agents, bool video, int iteracion, bool graph, bool debug,
                                                    std::vector<int>& goalVel, bool dqn){
    // Goal with respect to the agent
    Tpf goalAg;
    Tpf goal = goals[current_goal];
    transfor_inversa_p(goal.x, goal.y, &loc, &goalAg);

    Strategies planner(time_step, bounds);
    planner.space.InsertAgent(Velocidad(v, w), av, aw); //Add agent's dynamic window information

    planner.space.InsertGoal(goalAg, time_step);    //Add goal information
    // cout << "planner insert goal " << goalAg.x << ", " << goalAg.y <<": " << planner.space.GetGoal().commandGoal.v << ", " << planner.space.GetGoal().commandGoal.w << endl;
    if (!dqn){
        planner.space.SetDirGoal(dir_goal);
    }
    planner.space.SetDirGoal(planner.space.GetGoal().commandGoal);


    if (graph) {
        SeleccionaVentana(this->GetId()+1);
        LimpiaVentana();
    }
    if (fusionCommands){
        planner.space.SetCommands(fusion2d);
        if (graph || video){
            PlotVSEnvironment(fusion2d, bounds, planner.space.GetGoal(), time_step, video, iteracion, debug);
        }

    }else{
        std::vector<std::vector<Command>> commandsMerged;   //commandsMerged.reserve(comm_max*obst_max);
        if (!commands2d.empty())
            commandsMerged = planner.space.InsertDOV(commands2d, agents, time_step, *this);

        //if graph
        if (commandsMerged.empty()) PlotVSEnvironment(std::vector<Command>(), bounds, planner.space.GetGoal(), time_step);
        else {
            for (auto it = commandsMerged.begin(); it != commandsMerged.end(); ++it)
                PlotVSEnvironment((*it), bounds, planner.space.GetGoal(), time_step);   //PlotVSEnvironment(relativeAg->GetCommands(), boundsVS);
            commandsMerged.clear();
        }
    }

    planner.space.SetCommands(fusion2d);
    double w_max_steering, ang_left, ang_right, wz2, walign, valley, goal_valley, vorientacion, worientacion;
    std::string path = ros::package::getPath("rl_dovs");
    std::string filename = path + "/data/heuristics.txt";
    std::ifstream f;
    f.open(filename);
    f >> w_max_steering; f >> ang_left; f >> ang_right; f >> wz2; f >> walign;
    f >> valley; f >> goal_valley; f >> vorientacion; f >> worientacion;
    f.close();
    planner.SetHeuristicValues(w_max_steering, ang_left, ang_right, wz2, walign, valley, goal_valley, vorientacion, worientacion);
    if (graph) {
    //Plot the dir_goal finally computed to be reached
        SeleccionaVentana(this->GetId()+1);
        // goalVS goal_plot = planner.space.GetGoal();
        // goal_plot.dirGoal.v = planner.GetDirGoal().v;
        // goal_plot.dirGoal.w = planner.GetDirGoal().w;
        // PlotVSEnvironment(fusion2d, bounds, goal_plot, time_step, video, iteracion, debug);
        color("white");
        graf(bounds.wmax_right, bounds.wmax_left, bounds.wmax_right, 0.5, bounds.vlim_min, bounds.vlim_max, bounds.vlim_min, 0.5);
        float x[2], y[2];
        x[0] = 0.0; y[0] = 0.0;
        x[1] = planner.GetDirGoal().w; y[1] = planner.GetDirGoal().v;
        color("blue");
        linwid(1);
        rline(x[0], y[0], x[1], y[1]);
        linwid(1);
        TerminaEjes();
    }
    std::vector<Velocidad> possibleVelocities;
    possibleVelocities.clear();
    goalVel = planner.goalDirInDW(possibleVelocities);
    return possibleVelocities;
}

// add graph for greedy etc
std::vector<std::pair<int, Velocidad>> ActiveAgent::MotionStrategyRL(double time_step, std::vector<std::unique_ptr<Agent>> *agents, bool &freeActions, bool video, int iteracion, bool graph, bool debug) {

    //Goal with respect to the agent
    Tpf goalAg;
    Tpf goal = goals[current_goal];
    transfor_inversa_p(goal.x, goal.y, &loc, &goalAg);


    Strategies planner(time_step, bounds);
    planner.space.InsertAgent(Velocidad(v, w), av, aw); //Add agent's dynamic window information

    planner.space.InsertGoal(goalAg, time_step); 
    planner.space.SetDirGoal(planner.space.GetGoal().commandGoal);
    // planner.space.SetDirGoal(dir_goal);   //Add goal information
    if (graph) {
        SeleccionaVentana(this->GetId()+1);
        LimpiaVentana();
    }
    if (fusionCommands){
        planner.space.SetCommands(fusion2d);
        if (graph || video){
            PlotVSEnvironment(fusion2d, bounds, planner.space.GetGoal(), time_step, video, iteracion, debug);
        }

    }else{
        std::vector<std::vector<Command>> commandsMerged;   //commandsMerged.reserve(comm_max*obst_max);
        if (!commands2d.empty())
            commandsMerged = planner.space.InsertDOV(commands2d, agents, time_step, *this);

        //if graph
        if (commandsMerged.empty()) PlotVSEnvironment(std::vector<Command>(), bounds, planner.space.GetGoal(), time_step);
        else {
            for (auto it = commandsMerged.begin(); it != commandsMerged.end(); ++it)
                PlotVSEnvironment((*it), bounds, planner.space.GetGoal(), time_step);   //PlotVSEnvironment(relativeAg->GetCommands(), boundsVS);
            commandsMerged.clear();
        }
    }


    //PARAMETERS FOR STRATEGIES-PLANNING
    double w_max_steering, ang_left, ang_right, wz2, walign, valley, goal_valley, vorientacion, worientacion;
    std::string path = ros::package::getPath("rl_dovs");
    std::string filename = path + "/data/heuristics.txt";
    std::ifstream f;
    f.open(filename);
    f >> w_max_steering; f >> ang_left; f >> ang_right; f >> wz2; f >> walign;
    f >> valley; f >> goal_valley; f >> vorientacion; f >> worientacion;
    f.close();

    planner.SetHeuristicValues(w_max_steering, ang_left, ang_right, wz2, walign, valley, goal_valley, vorientacion, worientacion);

    Velocidad motion;
    // planner.ComputeMotion(this->GetId(), this->GetLocalization(),  goals[current_goal], agents, originInside, motion);
    std::vector<std::pair<int, Velocidad>> freeVelocities = planner.getSafeVelocities(this->GetId(), this->GetLocalization(),  goals[current_goal], agents, originInside, motion, freeActions, true);
    if (graph) {
    //Plot the dir_goal finally computed to be reached
        SeleccionaVentana(this->GetId()+1);
        // goalVS goal_plot = planner.space.GetGoal();
        // goal_plot.dirGoal.v = planner.GetDirGoal().v;
        // goal_plot.dirGoal.w = planner.GetDirGoal().w;
        // PlotVSEnvironment(fusion2d, bounds, goal_plot, time_step, video, iteracion, debug);
        color("white");
        graf(bounds.wmax_right, bounds.wmax_left, bounds.wmax_right, 0.5, bounds.vlim_min, bounds.vlim_max, bounds.vlim_min, 0.5);
        float x[2], y[2];
        x[0] = 0.0; y[0] = 0.0;
        x[1] = planner.GetDirGoal().w; y[1] = planner.GetDirGoal().v;
        color("blue");
        linwid(1);
        rline(x[0], y[0], x[1], y[1]);
        linwid(1);
        TerminaEjes();
    }
    // dir_goal = planner.GetDirGoal();
    return freeVelocities;
}

void ActiveAgent::getFreeVelocitiesVector(std::vector<float>& safeVelocities){
    safeVelocities.clear();
    Polygon pol;
    pol.InsertCommands(commands2d);

    for (int n_v = 0; n_v < 20; n_v++){
        for (int n_w = 0; n_w < 40; n_w++){
            double v = bounds.vlim_min + (n_v+0.5)*(bounds.vlim_max - bounds.vlim_min)/20;
            double w = bounds.wmax_right + (n_w+0.5)*(bounds.wmax_left - bounds.wmax_right)/40;
            // cout << "VMAX: " << bounds.vlim_max << " VMAX_RIGHT" << bounds.wmax_right << " VMAX_LEFT" << bounds.wmax_left << endl;
            // cout << "Candidate-> V: " << v << " W: " << w << " First?: " << (bounds.vlim_max/bounds.wmax_left) * w + v -bounds.vlim_max << 
            // " Second?: " << (bounds.vlim_max/bounds.wmax_right) * w + v -bounds.vlim_max << endl;
            // if (((bounds.vlim_max/bounds.wmax_left) * w + v -bounds.vlim_max <= 0) &&
            // ((bounds.vlim_max/bounds.wmax_right) * w + v -bounds.vlim_max <= 0)){
            if (!pol.InsidePolygon({v, w})) {
                // Free velocity
                safeVelocities.push_back(1);
            }
            else{
                safeVelocities.push_back(-1);
            }
            // }
            // // Done to get the images to apply the convolution
            // else{
            //     safeVelocities.push_back(-1);
            // }
        }
    }
}

PosibleVelocidad ActiveAgent::findClosestSafeVelocity() {

    Polygon pol;
    pol.InsertCommands(commands2d);

    const Velocidad vel  = {v,w};
    const double gridsize = 100;

    Velocidad bestvel;
    double    bestdist = DBL_MAX;
    bool      found    = false;

    for (double w = bounds.wmax_right; w < bounds.wmax_left; w += (fabs(bounds.wmax_right - bounds.wmax_left)/gridsize))
        for (double v = 0; v < bounds.vlim_max; v += (bounds.vlim_max/gridsize))
            if (!pol.InsidePolygon({v, w})) {
                const double dist = sqrt(pow(vel.v - v, 2) + pow(vel.w - w, 2));
                // andrew RL: value varies a lot when all velocities free
                if (dist < bestdist - 0.1) {
                    found    = true;
                    bestdist = dist;
                    bestvel  = {v, w};
                }
            }

    return { found, bestvel };
}
