//
// Created by maite16 on 6/12/17.
//

#include "dovs.h"

#include "StaticAgent.h"
#include "LinearAgent.h"
#include "CircularAgent.h"
#include "config.h"

#include <vector>

#include <iostream>
using namespace std;

//void DOVS::InsertDOV(const std::vector<Command> &commands) {
const std::vector<std::vector<Command>> DOVS::InsertDOV(const std::vector<dovt> &commandsIn, std::vector<std::unique_ptr<Agent>> *agents, const double time_step, Agent& currentAg) {
//Fusion of the DOV obstacles

    //Determine the angular beginning and end of each DOV
    //Store all DOV sequentially into one single list
    std::vector<std::vector<Command>> commandsOut; //commandsOut.reserve(obst_max*comm_max);
    commands.clear();
    //commands.reserve(obst_max*comm_max);

    ///*
    if ((int) commandsIn.size() == 1){
        auto it = commandsIn.begin();
        auto data = (*it);
        commands.insert(commands.end(), data.GetCommands().begin(), data.GetCommands().end());
        //auto data = it->GetCommands();
        //commands.insert(commands.end(), (commandsIn.begin())->GetCommands().begin(), (commandsIn.begin())->GetCommands().end());
        commandsOut.push_back(commands);

        commands.push_back(Command());
    }
    else{
    //*/
        std::vector<int> nIds;
        int n = agents->size();
        std::vector<double> ang_begin, ang_end;
        ang_begin.resize(n + 1);
        ang_end.resize(n + 1); //n+1: in order to take into account the agent itself

        std::vector<Command> totalCommands; //totalCommands.resize(obst_max*comm_max);
        std::vector<dovt>::const_iterator it;
        for (it = commandsIn.begin(); it != commandsIn.end(); ++it) {
            auto d = (*it);
            auto data = d.GetCommands();

            //loop until reaching the first command valid
            auto itData = data.begin();
            while (itData != data.end() && itData->objeto == 0){
                itData = data.erase(itData, itData+1);
            }
            if (itData != data.end()) {
                int id = itData->id;
                ang_begin[id] = atan2(data.front().sup.vel.v, data.front().sup.vel.w);
                ang_end[id] = atan2(data.back().sup.vel.v, data.back().sup.vel.w);
                nIds.push_back(id);

                totalCommands.insert(totalCommands.end(), data.begin(), data.end());
            }
        }

        //Angular order of the commands, and store
        std::vector<std::pair<bool, double>> split;
        split.resize(n + 1);
        std::vector<Rayo> totalRayos; //totalRayos.resize(obst_max*comm_max);
        std::vector<Command>::iterator it_b;
        std::vector<Command>::reverse_iterator it_e;
        for (it_b = totalCommands.begin(); it_b != totalCommands.end(); ++it_b) {
            Rayo r;
            r.obs.resize(n + 1);
            r.c.resize(n + 1);

            if ((*it_b).objeto != 0) {
                std::vector<Command>::reverse_iterator it_comp(it_b);
                for (it_e = totalCommands.rbegin(); it_e != it_comp; ++it_e) {
                    Command c1 = (*it_b);
                    Command c2 = (*it_e);
                    if (c2.objeto != 0) {
                        if (atan2(c1.sup.vel.v, c1.sup.vel.w) > atan2(c2.sup.vel.v, c2.sup.vel.w)) {

                            (*it_b) = c2;
                            (*it_e) = c1;

                            r.obs.clear();
                            r.obs.resize(n + 1);
                        } else if (atan2(c1.sup.vel.v, c1.sup.vel.w) == atan2(c2.sup.vel.v, c2.sup.vel.w)) {
                            r.obs[c2.id] = true;
                            r.c[c2.id] = c2;
                        }
                    } else {
                        //el obstáculo tiene 2 DOV, almaceno que está partido
                        auto itAux = std::next(it_e);
                        if (itAux != totalCommands.rend()){
                            Command c = (*itAux);
                            split[c.id] = std::make_pair(true, atan2(c.sup.vel.v, c.sup.vel.w));
                        }
                        it_e = std::vector<Command>::reverse_iterator(totalCommands.erase(--it_e.base()));
                        if (it_e != it_comp) it_e = prev(it_e);
                        else break;
                    }
                }
                Command c1 = (*it_b);
                r.ang = atan2(c1.sup.vel.v, c1.sup.vel.w);
                if (r.ang == M_PI / 2) r.radio = 10000;
                else if (r.ang == 0) r.radio = 0;
                else r.radio = c1.sup.vel.v / c1.sup.vel.w;
                r.obs[c1.id] = true;
                r.c[c1.id] = c1;
                totalRayos.push_back(r);
            } else {
                //el obstáculo tiene 2 DOV, almaceno que está partido
                auto itAux = std::prev(it_b);
                if (itAux != totalCommands.begin()){
                    Command c = (*itAux);
                    split[c.id] = std::make_pair(true, atan2(c.sup.vel.v, c.sup.vel.w));
                }
                it_b = totalCommands.erase(it_b);
                if (it_b != totalCommands.end()) it_b = prev(it_b);
                else break;
            }
        }

        //Fusion process
        totalCommands.clear();

        Command oneCommand;
        bool nuevo = true;
        double ang = -1;

        std::vector<bool> initialized, finished;
        initialized.resize(n + 1);
        finished.resize(n + 1);

        if (!totalRayos.empty()) {
            bool bound_inicial = true;
            Command cini;

            std::vector<Rayo>::iterator itR;
            for (itR = totalRayos.begin(); itR != totalRayos.end(); ++itR) {
                Rayo r = (*itR);

                Velocidad sup(-1,0);
                Velocidad inf(bounds.vlim_max * 10, 0);

                double t1 = 1e20;
                double t2 = 1e20;
                if (r.ang != ang) {
                    //Check if the current command being computed belongs to the initial of one DOV
                    if (nuevo) bound_inicial = true;

                    nuevo = true;

                    for (auto id:nIds) {

                        bool compare = true;

                        if (r.ang == ang_begin[id])
                            initialized[id] = true;

                        if (r.ang == ang_end[id])
                            finished[id] = true;

                        if (!r.obs[id]) {
                            //A command has not be computed for this radio
                            if (initialized[id] && !finished[id]) {

                                Tsc agLoc;
                                double agRadius = 0;
                                Velocidad agVel;
                                for (auto it = agents->begin(); it != agents->end(); ++it) {
                                    if ((*it)->GetId() == id) {
                                        agLoc = (*it)->GetLocalization();
                                        agRadius = (*it)->GetRealRadius();
                                        agVel = {(*it)->GetV(), (*it)->GetW()};
                                        break;
                                    }
                                }

                                //Compute Collision Band (CB) of the obstacle: instance of a linear or circular agent
                                std::unique_ptr<TrajectoryAgent> trajectory;
                                /*
                                if (agVel.v > 0) {
                                    if (agVel.w != 0){
                                        trajectory = std::unique_ptr<TrajectoryAgent> {new CircularAgent(currentAg.GetLocalization(), agLoc, currentAg.GetRealRadius() + agRadius, agVel.v, agVel.w, time_step, id)};
                                    }else{
                                        trajectory = std::unique_ptr<TrajectoryAgent> {
                                                new LinearAgent(currentAg.GetLocalization(), agLoc,
                                                                currentAg.GetRealRadius() + agRadius, agVel.v, id)};
                                    }
                                    //trajectory = std::unique_ptr<TrajectoryAgent> {new LinearAgent(currentAg.GetLocalization(), agLoc,currentAg.GetRealRadius() + agRadius, agVel.v, id)};

                                }else{
                                    trajectory = std::unique_ptr<TrajectoryAgent> {
                                        new StaticAgent(currentAg.GetLocalization(), agLoc, currentAg.GetRealRadius() + agRadius, agVel.v, id)};
                                }
                                //*/

                                if (agVel.v < 1e-5) agVel.v = 1e-3;
                                if (currentAg.isSegment()){
                                    agLoc.x = currentAg.getFirstPoint().x;
                                    agLoc.y = currentAg.getFirstPoint().y;
                                    Tsc sist2 = Tsc(currentAg.getLastPoint().x, currentAg.getLastPoint().y, agLoc.tita);
                                    trajectory = std::unique_ptr<TrajectoryAgent> {new StaticAgent(currentAg.GetLocalization(), agLoc,currentAg.GetRealRadius() + agRadius, agVel.v, id, true, currentAg.GetRealRadius()*config::safety_factor, sist2)};
                                }
                                else{
                                    trajectory = std::unique_ptr<TrajectoryAgent> {new LinearAgent(currentAg.GetLocalization(), agLoc,currentAg.GetRealRadius() + agRadius, agVel.v, id)};
                                }

                                Range ran = {r.radio, r.radio};
                                std::vector<Range> traj;
                                traj.push_back(ran);
                                int nradios = 1; bool stretches = false; unsigned traverse = 1;
                                double radio = (currentAg.GetRealRadius() + agRadius) * config::safety_factor;
                                dovt newCommand(id, currentAg.GetRealRadius() + agRadius, agVel.v, bounds.vlim_max);
                                trajectory->ComputeDOVT(traj, (unsigned) nradios, stretches, traverse, radio,
                                                        this->GetBounds(), 0, 0, Velocidad(), constraints(), newCommand, currentAg.GetLocalization());  //th = 0
                                if (!newCommand.GetCommands().empty() && newCommand.GetCommands().front().objeto != 0){

                                    //if (newCommand.front().sup.vel == Velocidad())
                                    //    std::cout << "MAL" << std::endl;

                                    r.c[id] = newCommand.GetCommands().front();

                                } else
                                    compare = false;


                            } else {
                                compare = false;
                            }
                        }

                        if (compare) {
                            //the greastest of the sup commands
                            if (r.c[id].sup.vel.v > sup.v) {
                                sup = r.c[id].sup.vel;
                            }
                            //the smallest of the inf commands
                            if (r.c[id].inf.vel.v < inf.v) {
                                inf = r.c[id].inf.vel;
                            }
                            //the smallest of the times
                            if (r.c[id].inf.t < t1) {
                                t1 = r.c[id].inf.t;
                            }
                            if (r.c[id].sup.t < t2) {
                                t2 = r.c[id].sup.t;
                            }
                        }

                        nuevo = nuevo && (!initialized[id] || finished[id] || (split[id].first && split[id].second == r.ang));
                        //nuevo = nuevo && (!initialized[id] || finished[id]);

                    }

                    oneCommand.objeto = 1;
                    oneCommand.sup.vel = sup;
                    oneCommand.sup.t = t2;
                    oneCommand.inf.vel = inf;
                    oneCommand.inf.t = t1;
                    commands.push_back(oneCommand);

                    //Store the bounds of DOV
                    if (bound_inicial) {
                        //Start bound
                        cini = oneCommand;
                        bound_inicial = false;
                    }

                    if (nuevo) {
                        commandsOut.push_back(commands);

                        commands.push_back(Command());

                        //Final bound
                        std::pair<Command, Command> dov = std::make_pair(cini, oneCommand);
                        boundsDOV.push_back(dov);
                    }
                    ang = r.ang;
                }
            }
            totalRayos.clear();
        }
    }
    return commandsOut;
}

std::vector<Command>& DOVS::GetCommands() {
    return commands;
}

std::vector<std::pair<Command, Command>> DOVS::GetBoundsDOV() {
    return boundsDOV;
}

void DOVS::SetCommands(const std::vector<Command>& cmds){
    // cout << "Empty? " << cmds.empty() << endl;
    commands.insert(commands.end(), cmds.begin(), cmds.end());
}
