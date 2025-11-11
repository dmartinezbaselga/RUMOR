//
// Created by maite16 on 18/07/17.
//

#include "TrajectoryAgent.h"

int TrajectoryAgent::GetId() {
    return id;
}

Tsc TrajectoryAgent::GetLocalization() {
    return loc;
}

void TrajectoryAgent::SetLocalization(Tsc l) {
    loc = l;
}

void DiscretizeRange(double left, double right, unsigned nradios, boundsVS bounds, std::vector<double> &radius, std::vector<double> &angles) {
//Function which translates the ranges of radius to radians in the velocity space plane: 0-pi radians

    double lower_bound, upper_bound;

    //left can take the values: 0, >0, <0, -inf
    //right can take the values: >0, <0, inf, 0
    if (left == 0 && right > 0) lower_bound = 0; //necesario para distinguir entre 0 y 180
    else if (left > 0 || (left < 0 && left != -INF)) lower_bound = std::atan2(1,1/left);
    else lower_bound = M_PI_2; // if (r.izq == -10000) limite_inferior = M_PI/2;

    if (right == INF) upper_bound = M_PI_2;
    else if (right > 0 || right < 0) upper_bound = std::atan2(1,1/right);
    else upper_bound = M_PI; //if (right == 0 && left < 0) upper_bound = M_PI;

    double beta1 = atan2(bounds.vlim_max, bounds.wmax_left);
    double beta2 = atan2(bounds.vlim_max, bounds.wmax_right);
    angles.push_back(lower_bound);
    for (unsigned i=1; i<nradios; i++){
        double alfa = lower_bound + i*(upper_bound-lower_bound)/nradios;
        if (angles.back() < beta1 && alfa > beta1) angles.push_back(beta1);
        if (angles.back() < beta2 && alfa > beta2) angles.push_back(beta2);
        //if (valores[i-1] < beta1 && alfa > beta1) valores.push_back(beta1); //Incorporamos el radio correspondiente a la velocidad extrema left
        //if (valores[i-1] < beta2 && alfa > beta2) valores.push_back(beta2); //Incorporamos el radio correspondiente a la velocidad extrema right
        angles.push_back(alfa);
    }
    angles.push_back(upper_bound);

    for (unsigned i=0; i<(unsigned)angles.size(); i++)
        radius.push_back(angles[i]== M_PI_2 ? INF: std::tan(angles[i]));    //radios de curvatura asociados a los angulos
}

void ComputeStretches(const std::vector<Root>& roots, bool insideCB, bool stretches, bool intersect, std::vector<std::pair<Root, Root>>& stretch){
//From the intersection points stored in roots, this function computes the several stretches to consider for collision
//insideCB: indicates if the robot is inside or outside the collision band of the obstacle
//stretches: indicates if it should be considered only the first stretch (stretches=false) or the whole (stretches=true)
//intersect: if false: the stretch lies inside the collision band, then the roots are the tangent points of the circunference arc and a line parallel to rfront/rback

    if (intersect){
        if (insideCB){
            //the first stretch is to move out the collision band from the robot position (origin (0,0))
            stretch.push_back(std::make_pair(Root(0,0), roots[0]));
            if (stretches){
                if (roots.size() > 2) stretch.push_back(std::make_pair(roots[1], roots[2]));
                stretch.push_back(std::make_pair(roots[roots.size()-1], roots[0]));
            }
        }else {
            if (roots.size() == 1) stretch.push_back(std::make_pair(roots[0], roots[0]));
            else {
                stretch.push_back(std::make_pair(roots[0], roots[1]));
                if (stretches && roots.size() > 2) stretch.push_back(std::make_pair(roots[2], roots[3]));
            }
        }
    }else{
        stretch.push_back(std::make_pair(roots[0], roots[1]));
    }

}

void TrajectoryAgent::ComputeDOVT(const std::vector<Range> &collidingTraj, unsigned nradios, bool stretches, unsigned traverse,
                                  double radio_obs, boundsVS bounds, const double th, const bool accConst, Velocidad velCurrentAg,
                                  constraints acc, dovt &dataDOVT, Tsc agLoc) {
//This function computes the set of velocities and times which are in collision with the agent: DOVTS
//nradios: number of radios to discretize each interval in collidingTraj
//stretches: false = only the first stretch; true = the whole set of several stretches
//traverse: number of times each stretch is considered to be traversed
//npoints: number of points in which the stretch is discretized to compute the colision times and velocities
//collidingTraj: trajectories colliding with the static obstacle

    //To add an empty command
    bool primer_neg = false; bool emptyCommand = IsCBBehind(); //if the obstacle (BC) is behind the robot, then we may need to enter an empty command
    bool addEmpty = false;
    // andrew  
    if (IsCBBehind()) {
        // std::cout << "BEHIND: " << this->GetId() << std::endl;
        return;
    }
    // else std::cout << "NOT BEHIND" << std::endl;
    for (int irango = 0; irango < collidingTraj.size(); irango++) {
        Range r = collidingTraj[irango];
        std::vector<double> titaRadiusVS, radius;
        DiscretizeRange(r.first, r.second, nradios, bounds, radius, titaRadiusVS);

        //bool intersect = (r.calculos == 1);//the trajectories in the range intersect the collision band or not(the whole range lie inside the CB)
        bool intersect = IntersectTrajBand(r);  //the trajectories in the range intersect the collision band or not(the whole range lie inside the CB)
        if (!intersect && !GetOriginInside()) continue;
        for (unsigned i = 0; i < (unsigned) titaRadiusVS.size(); i++) {

            //space3d.ClearVTSpace();

            std::vector<Root> roots;
            bool intersection = intersect;
            //We modify the value of intersect for computing later the stretches only in case the agent is in zone, the acandidate is PI/2 and obstacle is moving with angle +/-PI or 0
            if (intersect) IntersectionPoints(radius[i], titaRadiusVS[i], intersection, roots); //roots are stored in ascending angular order (polar coordinates)
            else TangentPoints(radius[i], roots);   //we define the discretization range with the tangent points between the circunference and a line parallel to rfront/rback

            if (!roots.empty()) {

                std::vector<std::pair<Root, Root>> stretch;
                ComputeStretches(roots, GetOriginInside(), stretches, intersection, stretch);    //pairs of roots in angular ascending order
                
                for (unsigned tr = 0; tr < (unsigned) stretch.size(); tr++) {
                    // andrew -> mirar distancia
                    //if (sqrt(pow(agLoc.x - stretch[tr].first.x ,2) + pow(agLoc.y - stretch[tr].first.y, 2)) < 6) {
                    // if (sqrt(pow(stretch[tr].first.x ,2) + pow(stretch[tr].first.y, 2)) < 4 && sqrt(pow(stretch[tr].second.x ,2) + pow(stretch[tr].second.y, 2)) < 4) {
                    if (stretch[tr].first.radius < 4 || stretch[tr].second.radius < 4) {
                        
                        //We add an empty command for visualization purposes
                        primer_neg = emptyCommand && radius[i] < 0;
                        if (primer_neg) {
                            //emptyCommand = false;
                            if (!dataDOVT.GetCommands().empty()) {
                                addEmpty = true;
                            }
                        }

                        //3D aproximation version: minimum and maximum velocity and time to avoid collision
                        std::vector<Root> points2d;
                        Command cmd;
                        points2d.push_back(stretch[tr].first);
                        points2d.push_back(stretch[tr].second);
                        if (ComputeVT_2d(points2d, GetOriginInside(), intersect, traverse, radius[i], titaRadiusVS[i],
                                        radio_obs, bounds, th, accConst, velCurrentAg, acc, cmd)) {
                            if (addEmpty){
                                dataDOVT.Add(Command(), RootCommand());
                                addEmpty = false;
                            } else {
                                dataDOVT.Add(cmd, RootCommand(stretch[tr].first, stretch[tr].second));
                            }
                        }
                        else{
                            // std::cout << "VEL NOT EMPTY" << std::endl;
                        }

                    }else {
                        //std::cout << "Filtrada posicion: " << stretch[tr].first.x << ", " << stretch[tr].first.y << std::endl;
                    }
                } 
                    //almacenar los stretchs según su ocurrencia
                //comparar por tipo de stretch (primer stretch, segundo...) para comprobar si hay hueco entre velocidades en el VT
            }
        }
    }
}