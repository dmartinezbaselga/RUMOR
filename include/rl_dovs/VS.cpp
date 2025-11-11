//
// Created by maite14 on 5/12/17.
//

#include "VS.h"
#include "utilidades.h"

Velocidad ComputeMaximumCommand(const double ang, const boundsVS bounds);
double NormalisePI(double d);
void VS::InsertGoal(Tpf goalPos, double stept) {
    //Add goal information: projection of the goal (WS) into the VTS

    Velocidad velG, commandG, dirG;
    double dir;
    if (goalPos.x < 0.0 && goalPos.y == 0.0) {  //behind the agent

        dir = M_PI;

        velG = Velocidad(0, dir / stept);
        commandG = Velocidad(0, bounds.InsideBoundsW(velG.w));
        dirG = commandG;

    } else if (goalPos.x > 0.0 && goalPos.y == 0.0) {   //in front of the agent

        velG = Velocidad(goalPos.x / stept, 0);
        commandG = Velocidad(bounds.vlim_max, 0);
        dirG = commandG;

    } else {

        //radius of the circunference arc which lead to the goal
        double radio = (goalPos.x * goalPos.x + goalPos.y * goalPos.y) / (2.0 * goalPos.y);

        //angular displacement of the agent to reach the goalPos
        double dir1 = atan2(2 * goalPos.x * goalPos.y, goalPos.x * goalPos.x - goalPos.y * goalPos.y);

        dir = atan2(goalPos.y, goalPos.x);
        double arco = goalPos.x / radio;
        if (radio > 0) {
            if (arco < 0) {
                dir = 2 * M_PI + dir;
                dir1 = 2 * M_PI + dir1;
            }
        } else {
            if (arco > 0) {
                dir = dir - 2 * M_PI;
                dir1 = dir1 - 2 * M_PI;
            }
        }

        //Define a limit to determine when it is better to rotate rather than accelerate
        if (std::abs(dir) > 3 * M_PI / 4) {
            if (dir > 0) dir = M_PI;
            else dir = -M_PI;

            velG = Velocidad(0, dir / stept);
            commandG = Velocidad(0, bounds.InsideBoundsW(velG.w));
            dirG = commandG;
        } else {

            double w = dir1 / stept;
            velG = Velocidad(radio * w, w);

            Line r1, r2;
            Tsc vel;
            int sol;
            double ang1, ang2;
            if (radio > 0.0) {
                //ang1: angle of the goal radius mapped into VS (between 0 - pi/2)
                ang1 = std::atan(radio);
                ang1 = NormalisePI(ang1);
                //ang2: top right corner in VS bounds
                ang2 = std::atan2(bounds.vlim_max, bounds.wmax_left);
                ang2 = NormalisePI(ang2);
                if (ang1 == M_PI / 2) {
                    r1 = Line(1, 0, 0);   //x=0
                } else if (ang1 == 0) {
                    r1 = Line(0, 1, 0);   //y=0
                } else {
                    r1 = Line(Tpf(0,0), ang1);
                }

                if (ang1 >= ang2) {
                    r2 = Line(0, 1, -bounds.vlim_max);  //y = v_max
                } else {
                    r2 = Line(1, 0, -bounds.wmax_left); //x = wmax_izq
                }
                //With the new boundaries
                r2 = Line(bounds.tan_v_w, -1 , bounds.cte_v_w);

                SolDosRectas(r1, r2, vel, sol);
            } else {
                ang1 = M_PI + std::atan(radio);
                ang1 = NormalisePI(ang1);
                ang2 = std::atan2(bounds.vlim_max, bounds.wmax_right);    //left top corner in VS
                ang2 = NormalisePI(ang2);
                if (std::abs(ang1) == M_PI / 2) {
                    r1 = Line(1, 0, 0);   //recta x=0
                } else if (std::abs(ang1) == M_PI) {
                    r1 = Line(0, 1, 0);   //recta y=0
                } else {
                    r1 = Line(Tpf(0,0), ang1);
                }

                if (ang1 <= ang2) {
                    r2 = Line(0, 1, -bounds.vlim_max);  //y = v_max
                } else {
                    r2 = Line(1, 0, -bounds.wmax_right);    //x = wmax_dch
                }
                //With the new boundaries
                r2 = Line(-bounds.tan_v_w, -1 , bounds.cte_v_w);

                SolDosRectas(r1, r2, vel, sol);
            }

            commandG = Velocidad(bounds.InsideBoundsV(vel.y), bounds.InsideBoundsW(vel.x));

            double steerW = bounds.InsideBoundsW(dir / stept);
            if (steerW >= commandG.w) {
                dirG.w = bounds.InsideBoundsW(commandG.w + (steerW - commandG.w) / 2);
                dirG.v = bounds.InsideBoundsV(bounds.vlim_max + (bounds.vlim_max - commandG.v) / 2);
            } else {
                dirG.w = bounds.InsideBoundsW(commandG.w - (commandG.w - steerW) / 2);
                dirG.v = bounds.InsideBoundsV(bounds.vlim_max);
            }

            //New computation of dir_goal with the new boundaries
            if (steerW >= commandG.w) {
                dirG.w = bounds.InsideBoundsW(commandG.w + (steerW - commandG.w) / 2);
                dirG.v = bounds.ComputeMaxV_W(dirG.w);
            }else{
                dirG.w = bounds.InsideBoundsW(commandG.w - (commandG.w - steerW) / 2);
                dirG.v = bounds.ComputeMaxV_W(dirG.w);
            }
        }
    }

    goal = goalVS(velG, commandG, dirG);    //dirG converges to the goal in absence of obstacles
}

void VS::SetDirGoal (Velocidad v){
    this->goal.dirGoal = v;
}

Velocidad VS::GetDirGoal (){
    return goal.dirGoal;
}

void VS::InsertAgent(Velocidad vAgent, double aV, double aW){
    velAgent = vAgent;
    diffConstraints = constraints(aV, aW);
}


void VS::InsertBounds(boundsVS dataVS) {
    bounds = dataVS;
}

boundsVS VS::GetBounds() const {
    return bounds;
}

goalVS VS::GetGoal() const {
    return goal;
}

Velocidad VS::GetAgent() const {
    return velAgent;
}

constraints VS::GetConstraints() const {
    return diffConstraints;
}

double VS::GetMaxV(const double w) {
    //Returns the maximum linear velocity allowed for the input angular velocity

    //diffConstraints.av * (w/diffConstraints.aw + step*n);

    return 0;
}

double VS::GetMaxW(const double v) {
    //Returns the maximum angular velocity allowed for the input linear velocity


    return 0;
}


bool VS::VelReachable(double step, const Velocidad & vel) {

    if (vel.v < 0 || vel.v > bounds.vlim_max) return false;
    if (vel.w > bounds.wmax_left || vel.w < bounds.wmax_right) return false;

    Velocidad v = { fabs(velAgent.v - vel.v), fabs(velAgent.w - vel.w) };

    const double maxv = (diffConstraints.av*step) * (1 - v.w/(diffConstraints.aw*step));    // Linear interpolation diamond side

    return v.v <= maxv;
}