//
// Created by maite16 on 18/07/17.
//

#include "LinearAgent.h"

#include "utilidades.h"

#include "TData.h"

#include <cmath>
#include <algorithm>

bool operator < (const Root& l, const Root& r) {	//Ascending order by angle (asumption: the roots belong to the same arc of circunference so that the angle has same sign)
    return (std::abs(l.angle) < std::abs(r.angle)) ||
           ((l.angle == r.angle) && (l.radius < r.radius));
}

bool operator < (const Velocidad& l, const Velocidad& r) {	//Ascending order by v
    //Assumption: velocities l and r lie in the same radius

    Tpfp lim_l, lim_r;
    car2pol_vel(l, &lim_l);
    car2pol_vel(r, &lim_r);

    return (std::abs(lim_l.t) - std::abs(lim_r.t) < 1e-5 && lim_l.r < lim_r.r);
}

/*
// l < r <=> r > l
// l == r <=> !(l<r) && !(l>r)
bool operator < (const Command& l, const Command& r) {	//Ascending order by angle
    return (atan2(l.sup.vel.v, l.sup.vel.w) < atan2(r.sup.vel.v, r.sup.vel.w));
}
//*/

bool IsObsBehind(const Root& pRob, const Root& pObs, const Eigen::Vector2d& u);
bool LinearAgent::ConsiderRange(Range range){
//Returns true or false if the candidate range of trajectories is free of collision (false) or may collide with the agent (true)
//Discard ranges which do not intersect the moving direction of the agent, i.e, which are free of collision

    if (GetOriginInside()) {
        //TODO: implement this case of the function

        //We compute if the lines Rfront/Rback lie behind the roots; if so, we compute the distances from the lines Rfront/Rback to the points Root1/Root2, otherwise the distance is 0
        Tsc raiz; int sol = 0;
        Line R1(Tpf(0,0), TrajectoryAgent::GetLocalization().tita); //line which passes through the origin
        Line Rfront(cornerAg[0], cornerAg[1]);
        SolDosRectas(R1, Rfront, raiz, sol); Root pointObs = Root(raiz.x, raiz.y); sol = 0;
        //Computes if the agent is moving towards the origin
        //if (IsObsBehind(Root(0,0), pointObs, Eigen::Vector2d(v_x,v_y))) //Line (Recta) front is behind root 1
            return true;
        //else{
        //}
    }else{

        return IntersectTrajBand(range);
    }
}

void LinearAgent::IntersectingTrajectories(std::vector<Range>& rangeTraj) {

    //vector of radius to determine the ranges of possible colliding trajectories for which DOVT has to be computed later
    std::vector<double> positiveR;
    std::vector<double> negativeR;

    //Compute the radius of the circunference arcs which pass through each corner of the square that inscribes the agent
    for (int i = 0; i<cornerAg.size(); i++){
        Tpf corner = cornerAg[i];
        double radius = (corner.x*corner.x + corner.y*corner.y)/(2.0*corner.y);
        if (radius >= 0) positiveR.push_back(radius);
        else negativeR.push_back(radius);
    }

    //Compute the radius of the tangent circunference arcs to the trajectory band of the agent
    std::pair<double, double> tanRup = upper.TangentRadius();
    std::pair<double, double> tanRlow = lower.TangentRadius();

    //Add these conditions to avoid the case in which both are parallel and lie one each respective side
    //if (!positiveR.empty() && tanRup.first > 0)
        positiveR.push_back(tanRup.first);
    //if (!negativeR.empty() && tanRup.second < 0)
        negativeR.push_back(tanRup.second);
    //if (!positiveR.empty() && tanRlow.first > 0)
        positiveR.push_back(tanRlow.first);
    //if (!negativeR.empty() && tanRup.second < 0)
        negativeR.push_back(tanRlow.second);

    //ascending order of the radius
    std::sort(positiveR.begin(), positiveR.end());
    std::sort(negativeR.begin(), negativeR.end());

    //Compute candidate ranges of circunference arc trajectories
    //ComputeRange(positiveR, negativeR, rangeTraj);
    double left = 0;
    for (int i = 0; i<positiveR.size(); i++){
        double right = positiveR[i];
        Range r = {left, right};
        if (std::abs(left - right) < 1e-5) continue;
        else if (ConsiderRange(r)) rangeTraj.push_back(r);
        left = right;
    }
    if (ConsiderRange({left,INF}))
        rangeTraj.push_back({left, INF});

    left = -INF;
    for (int i = 0; i<negativeR.size(); i++){
        double right = negativeR[i];
        Range r = {left, right};
        if (std::abs(left - right) < 1e-5) continue;
        else if (ConsiderRange(r)) rangeTraj.push_back(r);
        left = right;
    }
    if (left != 0 && ConsiderRange({left,0})) rangeTraj.push_back({left,0});
}



void ComputeRoot(const Root rin, Line R, const double dist, Root& rout){
//This function computes the root which is at a distance 'dist' from root rin
//dist = sqrt((rin.x-rout.x)*(rin.x-rout.x) + (rin.y-rout.y)*(rin.y-rout.y))
//both roots rin and rout lie in R

    double A = R.GetA(); double B = R.GetB(); double C = R.GetC();
    double x1 = rin.x; double y1 = rin.y;
    if (A != 0){
        double a = 1;
        double b = (2*C*B + 2*B*A*x1 - 2*y1*A*A)/(1+A*A);
        double c = (C*C + 2*C*A*x1 + x1*x1*A*A + y1*y1*A*A - A*A*dist*dist)/(1+A*A);

        double y21 = (-b + std::sqrt(b*b - 4*a*c))/(2*a);
        double y22 = (-b - std::sqrt(b*b - 4*a*c))/(2*a);

        double x21 = - (C + B*y21)/A;
        double x22 = - (C + B*y22)/A;

        //Me quedo con la mayor; TODO: me debería quedar con aquella teniendo en cuenta el signo del ángulo de dirección del obstáculo
        if (x21 > x1) rout = Root(x21, y21);
        else rout = Root(x22, y22);
    }else{
        double x21 = x1 + dist;
        double x22 = x1 - dist;

        //Me quedo con la mayor; TODO: me debería quedar con aquella teniendo en cuenta el signo del ángulo de dirección del obstáculo
        if (x21 > x1) rout = Root(x21, y1);
        else rout = Root(x22, y1);
    }

}

void LinearAgent::IntersectionPoints(double rcandidate, double acandidate, bool& intersection, std::vector<Root>& roots){
//This function computes the intersecting points between a candidate trajectory and the collision band of the linear agent

    if(acandidate != M_PI_2){

        double raices[2][2];
        int num_sol;

        //intersection points between the trajectory of the linear agent (collision band) and the circumference arc (candidate trajectory for the active agent)
        SolCirculoRecta(upper.GetA(), upper.GetB(), upper.GetC(), 0, rcandidate, rcandidate, raices, num_sol);
        for (int i=0; i<num_sol; i++){
            roots.push_back(Root(raices[i][0], raices[i][1], 1));
        }
        SolCirculoRecta(lower.GetA(), lower.GetB(), lower.GetC(), 0, rcandidate, rcandidate, raices, num_sol);
        for (int i=0; i<num_sol; i++){
            roots.push_back(Root(raices[i][0], raices[i][1], 0));
        }

        if (!roots.empty()){

            std::sort(roots.begin(), roots.end());  //angular sort of the roots

            if (roots.size() == 3){ //Delete the second root (tangent circumference arc), which is useless (it does not represent a point to exit or enter the collision band)
                //Delete the root located at a different line
                unsigned i = 0;
                if (roots[0].i == roots[1].i) i = 2;
                if (roots[1].i == roots[2].i) i = 0;
                if (roots[0].i == roots[2].i) i = 1;
                roots.erase(roots.begin()+i);
            }
        }

    }else{

        //intersection points between the trajectory of the linear agent (collision band) and the straight line (candidate trajectory for the active agent)
        //only roots in front of the robot can be reachable
        Line r(0.,1.,0.);   //axis 'x': y=0
        Tsc raiz1, raiz2;
        int sol1 = 0; int sol2 = 0;
        SolDosRectas(upper, r, raiz1, sol1);
        if (sol1 != 0 && raiz1.x >= 0) roots.push_back(Root(raiz1.x, raiz1.y, 1));
        SolDosRectas(lower, r, raiz2, sol2);
        if (sol2 != 0 && raiz2.x >= 0) roots.push_back(Root(raiz2.x, raiz2.y, 0));

        if (!roots.empty()) {
            std::sort(roots.begin(), roots.end());
        }else{
            if (GetOriginInside()){

                Tpf plus45 = cornerAg[0]; Tpf minus45 = cornerAg[1];
                Tpf plus135 = cornerAg[2]; Tpf minus135 = cornerAg[3];

                intersection = false;

                Tsc loc = TrajectoryAgent::GetLocalization();
                if (std::abs(loc.tita) == M_PI){

                    if (loc.x > 0){ //obstacle moves towards the robot
                        Tsc raiz; int sol = 0;
                        //Line R(0,0,0);
                        Line Rfront(plus45, minus45);
                        SolDosRectas(r, Rfront, raiz, sol);
                        roots.push_back(Root(0,0));
                        roots.push_back(Root(raiz.x, raiz.y));
                    }

                }else{  //alfa = 0

                    //compute the longest distance the obstacle can traverse for the time horizon considered
                    //double th = space3d.GetVTSpace().data3d.size()*time_step;
                    //TODO: check if the value for th is indifferent
                    double th = 20;

                    double dist = module*th;
                    //Line R(0,0,0);
                    Root rout(0,0);
                    if (loc.x > 0){ //obstacle moves away the robot

                        Tsc raiz; int sol = 0;
                        Line Rback(plus135, minus135);
                        SolDosRectas(r, Rback, raiz, sol);
                        //roots.push_back(Root(raiz.x, raiz.y));
                        ComputeRoot(Root(raiz.x, raiz.y), r, dist, rout);   //compute root which is at distance 'modulo*th' from the initial root
                        //roots.push_back(Root(raiz.x, raiz.y)
                        roots.push_back(Root(0,0));
                        roots.push_back(rout);

                    }else{  //obstacle moves towards the robot

                        roots.push_back(Root(0,0));

                        Tsc raiz; int sol = 0;
                        Line Rfront(plus45, minus45);
                        SolDosRectas(r, Rfront, raiz, sol);
                        //ComputeRoot(Root(raiz.x, raiz.y), R, dist, rout);      //compute root which is at distance 'v*th' from the initial root
                        ComputeRoot(Root(0,0), r, dist, rout);      //compute root which is at distance 'modulo*th' from the initial root
                        roots.push_back(rout);
                    }
                }

            }//else::¿¿¿CONSIDERAR CASO EN EL QUE LAS RECTAS DE LA BANDA COINCIDEN CON EJE X DEL ROBOT???
        }
    }
}

void LinearAgent::TangentPoints(double rcandidate, std::vector<Root>& roots) {
//This function computes the tangent points of the trajectory of the robot with a line parallel to rfront/rback
//Returns only the tangent point before the robot

    Tpf plus45 = cornerAg[0]; Tpf minus45 = cornerAg[1];
    Line R(plus45, minus45);
    double e = std::numeric_limits<double>::epsilon();

    //¿¿¿QUÉ PASA SI EL RADIO ES 0???
    if (R.GetA() == 0) { //line with slope 0
        double A, B, C;
        double raiz[2][2];
        int n = 0;

        A = 0.0; B = 1.0; C = -2.0 * rcandidate;
        SolCirculoRecta(A, B, C, 0, rcandidate, rcandidate, raiz, n);

        if (n > 0) {
            roots.push_back(Root(0, 0));
            roots.push_back(Root(raiz[0][0], raiz[0][1]));
        }

    }else{
        double A1, B1, C1, A2, B2, C2;
        double raices1[2][2], raices2[2][2];
        int n1 = 0; int n2 = 0;

        if (R.GetB() == 0){  //line with infinite slope
            A1 = 1.0; B1 = 0.0; C1 = rcandidate;
            A2 = 1.0; B2 = 0.0; C2 = -rcandidate;
        }else{	//recta con pendiente diferente de cero e inf.
            double m = -(R.GetA()/R.GetB());
            A1 = -m; B1 = 1.0; C1 = -((1.0 + std::sqrt(1.0+m*m))*rcandidate);
            A2 = -m; B2 = 1.0; C2 = -((1.0 - std::sqrt(1.0+m*m))*rcandidate);
        }
        SolCirculoRecta(A1, B1, C1, 0.0, rcandidate, rcandidate, raices1, n1);
        SolCirculoRecta(A2, B2, C2, 0.0, rcandidate, rcandidate, raices2, n2);

        if (n1 > 0 && n2 > 0) {
            //roots.push_back(Root(raices1[0][0], raices1[0][1]));
            //roots.push_back(Root(raices2[0][0], raices2[0][1]));
            //std::sort(roots.begin(), roots.end());

            Root rootTg;
            if (raices1[0][0] > 0) rootTg = Root(raices1[0][0], raices1[0][1]);
            else rootTg = Root(raices2[0][0], raices2[0][1]);
            roots.push_back(Root(0, 0));
            roots.push_back(rootTg);
        }
    }
}

bool IsObsBehind(const Root& pRob, const Root& pObs, const Eigen::Vector2d& u){
//Function which computes if pointObs lies behind pRob in a line wrt the direction of the line

    Eigen::Vector3d vObs3d(pRob.x - pObs.x, pRob.y - pObs.y, 0);
    Eigen::Vector3d u3d(u.x(), u.y(), 0);

    //Compute the angle between vObsToRob and u, to check if it has same angle i.e., they have same direction: atan2(norm(cross(v1,v2)),dot(v1,v2))
    double angle = std::atan2((vObs3d.cross(u3d)).norm(), vObs3d.dot(u3d));

    return (std::abs(angle) < 1e-5);
}


double AngDisplacement(const Root point, const double rcandidate, const unsigned k){

    //Compute the angular movement the robot should perform to reach the point
    double tita = atan2(2*point.y*point.x, point.x*point.x - point.y*point.y);
    if (rcandidate > 0 && tita < 0) tita += 2*M_PI;	//Normalize to 0-2pi
    if (rcandidate < 0 && tita > 0) tita -= 2*M_PI;

    //Add k turns of 2pi radii
    if (rcandidate > 0) tita += k*2*M_PI;
    else tita -= k*2*M_PI;

    return tita;
}

Velocidad ComputeVelocity(const double distance, const double rcandidate, const double acandidate, const double time, const Velocidad max) {
//Velocity to traverse distance 'distance' in time 'time'
//distance = angular displacement along the circunference arc of radius rcandidate or linear displacement following a straight line

    //Velocidad vel = (acandidate == M_PI_2) ? Velocidad(distance/time, 0) : Velocidad((distance / time) * rcandidate, distance / time);
    //return (max < vel) ? max : vel;
    return (acandidate == M_PI_2) ? Velocidad(distance/time, 0) : Velocidad((distance / time) * rcandidate, distance / time);
}

//void Obstaculo::ComputeVT_2d(const std::vector<Root>& points, bool insideBC, bool intersect, unsigned traverse, double rcandidate, double acandidate, double radio_obs) { //rcandidate, radio_obs = obs candidate trajectory, and inflated radius
bool LinearAgent::ComputeVT_2d(const std::vector<Root> &points, bool insideBC, const bool intersection, unsigned traverse,
                               double rcandidate, double acandidate, double radio_obs, boundsVS bounds, const int th,
                               const bool accConst, Velocidad velCurrentAg, constraints acc,
                               Command &cmdOut) {
// rcandidate = robot's candidate trajectory, radio_obs = inflated radius
// This function computes the minimum and maximum velocities the robot should perform to avoid a collision by reaching points, which are angular ascending ordered
// It returns if a command has been computed, i.e., if the robot's candidate trajectory is dangerous

    bool intersect = intersection;

    Tpf plus45 = cornerAg[0]; Tpf minus45 = cornerAg[1];
    Tpf plus135 = cornerAg[2]; Tpf minus135 = cornerAg[3];

    Line Rfront = Line(plus45, minus45);
    Line Rback = {plus135, minus135};

    Root root1 = points[0];
    Root root2 = points[1];

    Tsc loc = TrajectoryAgent::GetLocalization();
    Line R1(Tpf(root1.x,root1.y), loc.tita);
    Line R2(Tpf(root2.x,root2.y), loc.tita);

    //We compute if the lines Rfront/Rback lie behind the roots; if so, we compute the distances from the lines Rfront/Rback to the points Root1/Root2, otherwise the distance is 0
    Tsc raiz; int sol = 0;
    SolDosRectas(R1, Rfront, raiz, sol); Root pointObs = Root(raiz.x, raiz.y); sol = 0;
    bool RfBR1 = IsObsBehind(root1, pointObs, Eigen::Vector2d(v_x,v_y)); //Line (Recta) front is behind root 1
    double dRfrontR1 = (RfBR1 ? std::sqrt((pointObs.x - root1.x)*(pointObs.x - root1.x) + (pointObs.y - root1.y)*(pointObs.y - root1.y)) : 0);

    SolDosRectas(R2, Rfront, raiz, sol); pointObs = Root(raiz.x, raiz.y); sol = 0;
    bool RfBR2 = IsObsBehind(root2, pointObs, Eigen::Vector2d(v_x,v_y)); //Line (Recta) front is behind root 2
    double dRfrontR2 = (RfBR2 ? std::sqrt((pointObs.x - root2.x)*(pointObs.x - root2.x) + (pointObs.y - root2.y)*(pointObs.y - root2.y)) : 0);

    SolDosRectas(R1, Rback, raiz, sol); pointObs = Root(raiz.x, raiz.y); sol = 0;
    bool RbBR1 = IsObsBehind(root1, pointObs, Eigen::Vector2d(v_x,v_y)); //Line (Recta) back is behind root 1
    double dRbackR1 = (RbBR1 ? std::sqrt((pointObs.x - root1.x)*(pointObs.x - root1.x) + (pointObs.y - root1.y)*(pointObs.y - root1.y)) : 0);

    SolDosRectas(R2, Rback, raiz, sol); pointObs = Root(raiz.x, raiz.y);
    bool RbBR2 = IsObsBehind(root2, pointObs, Eigen::Vector2d(v_x,v_y)); //Line (Recta) back is behind root 2
    double dRbackR2 = (RbBR2 ? std::sqrt((pointObs.x - root2.x)*(pointObs.x - root2.x) + (pointObs.y - root2.y)*(pointObs.y - root2.y)) : 0);

    Root rootMin, rootMax;
    rootMin = root2; rootMax = root1;

    //Check if the obstacle reaches the tangent point of the trajectory before than the root to escape of collision
    //Were the case, then the times to escape the collision band are updated wrt the distance to the tangent point
    if (insideBC && intersect){ // && acandidate != M_PI_2){
        Tpf tanPoint; int towardsRobot;
        DireccionMovimiento(alfa, module, loc.x, loc.y, towardsRobot);
        LineTanR2(rcandidate, Rfront, alfa, towardsRobot, tanPoint);

        Root tanRoot(tanPoint.x, tanPoint.y);
        if (std::abs(tanRoot.angle) < std::abs(root2.angle)){   //Update the times to escape the collision band

            Tsc raiz; int sol;
            R2 = Line(Tpf(tanRoot.x, tanRoot.y), alfa);
            SolDosRectas(R2, Rfront, raiz, sol); pointObs = Root(raiz.x, raiz.y);
            RfBR2 = IsObsBehind(tanRoot, pointObs, Eigen::Vector2d(v_x,v_y)); //Line (Recta) front is behind tangent root
            dRfrontR2 = (RfBR2? std::sqrt((pointObs.x - tanRoot.x)*(pointObs.x - tanRoot.x) + (pointObs.y - tanRoot.y)*(pointObs.y - tanRoot.y)) : 0);

            SolDosRectas(R2, Rback, raiz, sol); pointObs = Root(raiz.x, raiz.y);
            RbBR2 = IsObsBehind(tanRoot, pointObs, Eigen::Vector2d(v_x,v_y)); //Line (Recta) back is behind tangent root
            dRbackR2 = (RbBR2 ? std::sqrt((pointObs.x - tanRoot.x)*(pointObs.x - tanRoot.x) + (pointObs.y - tanRoot.y)*(pointObs.y - tanRoot.y)) : 0);

            root2 = tanRoot;    //we are interested in root2 = tanRoot when we wish that rootMax = root2 so that the DOV is continuous in VS
        }
    }

    //We compute the correspondance between minimum time to exit the collision band (at point rootMin) before the object arrives, and
    //the maximum time before entering the collision band (at point rootMax) when the object has just left
    double tMin = 1e-5; double tMax = 1e-5;
    if (!RbBR1 && !RbBR2) return false;	//The obstacle does not intersect the robot's candidate trajectory
    else{
        if (!insideBC){
            //if (dRfrontR2 < dRfrontR1){
            if (dRbackR2 < dRbackR1){
                tMin = dRfrontR2/module; //rootMin = root2;
                tMax = dRbackR1/module; //rootMax = root1;
            }else{
                tMin = dRfrontR1/module; //rootMin = root2;
                tMax = dRbackR2/module; //rootMax = root1;
            }
        }else{

            if (RfBR1){ //} && RfBR2){
                if (dRfrontR2 < dRfrontR1){
                    tMin = dRfrontR2/module; //rootMin = root2;
                    tMax = dRfrontR1/module;
                }else{
                    //if (intersect){
                    tMin = dRfrontR1/module; //rootMin = root2;
                    tMax = tMin;
                    //}else{
                    //    tMin = dRfrontR2/modulo; // wrt the tangent point
                    //    tMax = dRfrontR1/modulo; // wrt the origin
                    //}
                }
            }else{
                //check for the rback line with respect to the roots
                if (!RbBR1){
                    if (!intersect)
                        intersect = true; //so that it computes the velocity to reach the rootMax in time tMax

                    rootMax = root2;
                    tMax = dRbackR2/module;
                }
            }
        }
    }
    if (tMin < 1e-5) tMin = 1e-5;
    if (tMax < 1e-5) tMax = 1e-5;

    //Consideration of a time horizon (13/09/2018): Check if the collision times are higher than the time horizon
    //Inside collision band: should we consider the th as well?
    //Candidate trajectories which lie inside the collision band (inside collision band): should we consider the th as well?
    //Could we distinguish different th candidates depending on this three cases???
    if (th > 0){
        //if (!intersect) th *= 2;
        //else if (insideBC) th *= 1.5;

        assert(tMax >= tMin);

        if (tMax > th) return false;
        else{}
            //std::cout << " tMax: " << tMax << std::endl;
    }

    //if (acandidate == M_PI_2)
        //std::cout << "tita pi/2" << std::endl;

    unsigned ntraverse = (acandidate == M_PI_2) ? 1 : traverse;
    for (unsigned k = 0; k<ntraverse; k++) {    //Update the forbidden velocities and times

        Velocidad max;
        max = bounds.ComputeMaximumCommand(acandidate);    //ObtenerComandoMaximo(acandidate, bounds.vlim_max, bounds.wmax_left, bounds.wmax_right, max.v, max.w);

        if (!intersect){
            Comando sup = Comando(max, tMin);
            Comando inf = Comando(Velocidad(0,0), tMax);
            cmdOut = Command(sup, inf, 1, this->GetId()); //commands2d.push_back(Command(sup, inf, 1));
        }else{

            bool computeCommand = (tMax > 1e-5 || insideBC);
            if (computeCommand) {

                //Computes the angular displacement the agent should perform to reach the point
                double distance = (acandidate == M_PI_2 || std::abs(rootMin.y) < 1e-5) ? copysign(rootMin.x, rcandidate) : AngDisplacement(rootMin, rcandidate, k);
                Velocidad vel;
                if (tMin > 1e-5) vel = ComputeVelocity(distance, rcandidate, acandidate, tMin, max);
                else vel = max;
                //Comando sup = Comando((max < vel) ? max : vel, tMin);   //Comando sup = Comando(vel, tMin);

                //INI: dynamic constraints of the agent
                if (accConst){
                    double tw = std::abs(vel.w - velCurrentAg.w)/acc.aw;
                    double tv = std::abs(vel.v - velCurrentAg.v)/acc.av;

                    double partialDist;
                    if (acandidate == M_PI_2 || std::abs(rootMax.y) < 1e-5){
                        partialDist = velCurrentAg.v*tw + velCurrentAg.v*tv + 0.5*acc.av*tv*tv;
                    }else{
                        double ang = std::abs(velCurrentAg.w)*tw + 0.5*acc.aw*tw*tw;
                        partialDist = ang + std::abs(vel.w)*tv;
                    }
                    distance = std::abs(distance)-std::abs(partialDist);
                    if (distance > 0){
                        distance = copysign(distance, rcandidate);
                        tMin -= (tw + tv);
                        if (tMin < 1e-5){
                            tMin = 1e-5;
                            vel = max;
                        }else
                            vel = ComputeVelocity(distance, rcandidate, acandidate, tMin, max);
                    }
                }
                //FIN: dynamic constraints of the agent
                Comando sup = Comando((max < vel) ? max : vel, tMin);   //Comando sup = Comando(vel, tMin);

                double distanceMin = distance;
                if (!(rootMax == Root(0, 0))) {
                    distance = (acandidate == M_PI_2 || std::abs(rootMax.y) < 1e-5) ? copysign(rootMax.x, rcandidate) : AngDisplacement(rootMax, rcandidate, k);
                    vel = ComputeVelocity(distance, rcandidate, acandidate, tMax, max);

                    //INI: dynamic constraints of the agent
                    if (accConst) {
                        double tw = std::abs(vel.w - velCurrentAg.w) / acc.aw;
                        double tv = std::abs(vel.v - velCurrentAg.v) / acc.av;

                        double partialDist;
                        if (acandidate == M_PI_2 || std::abs(rootMax.y) < 1e-5){
                            partialDist = velCurrentAg.v*tw + velCurrentAg.v*tv + 0.5*acc.av*tv*tv;
                        }else{
                            double ang = std::abs(velCurrentAg.w)*tw + 0.5*acc.aw*tw*tw;
                            partialDist = ang + std::abs(vel.w)*tv;
                        }
                        distance = std::abs(distance)-std::abs(partialDist);
                        if (distance > 0){
                            distance = copysign(distance, rcandidate);
                            tMax += (tw + tv);
                            if (tMax < 1e-5) return false;
                            vel = ComputeVelocity(distance, rcandidate, acandidate, tMax, max);
                        }//else if (std::abs(distanceMin)-std::abs(ang) > 0) vel = Velocidad(0, 0);
                    }
                    //FIN: dynamic constraints of the agent

                    if (max < vel) {    //the forbidden velocity lies outside the velocity bounds of the agent
                        return false;   //computeCommand = false; break;
                    }
                }else{
                    vel = Velocidad(0, 0);
                }
                Comando inf = Comando(vel, tMax);
                cmdOut = Command(sup, inf, 1, this->GetId()); //commands2d.push_back(Command(sup, inf, 1));

            }else
                return false;
        }
    }

    return true;  //to know if a command has been computed
}

bool LinearAgent::IntersectTrajBand(Range r){
//Returns true if any of the radius of the interval intersects the collision band

    double points[2][2];
    int n1 = 0; int n2 = 0; int n3 = 0; int n4 = 0;

    SolCirculoRecta(upper.GetA(), upper.GetB(), upper.GetC(), 0, r.first, r.first, points, n1);
    if (n1 < 2){  //the radius does not intersect the collision band or it intersects in the tangent point
        SolCirculoRecta(lower.GetA(), lower.GetB(), lower.GetC(), 0, r.first, r.first, points, n2);
        if (n2 < 2){
            SolCirculoRecta(upper.GetA(), upper.GetB(), upper.GetC(), 0, r.second, r.second, points, n3);
            if (n3 < 2)
                SolCirculoRecta(lower.GetA(), lower.GetB(), lower.GetC(), 0, r.second, r.second, points, n4);
        }
    }

    return (n1 >= 2 || n2 >= 2 || n3 >= 2 || n4 >= 2) || ((n1 == 1 || n2 == 1) && r.second == INF) || ((n3 == 1 || n4 == 1) && r.first == -INF);
}

bool LinearAgent::IsCBBehind(){
    //std::cout << "ISCBBEHING OF AGENT: " << this->GetId() << std::endl;
    return upper.GetA() != 0 && (-upper.GetC()/upper.GetA() < 0  && -lower.GetC()/lower.GetA() < 0);
}

/*
//std::vector<Command> LinearAgent::ComputeDOVT(const std::vector<Range>& collidingTraj, unsigned nradios, bool stretches, unsigned traverse, double radio_obs, boundsVS bounds){
void LinearAgent::ComputeDOVT(const std::vector<Range> &collidingTraj, unsigned nradios, bool stretches,
                              unsigned traverse,
                              double radio_obs, boundsVS bounds, const double th, dovt &dataDOVT) {
//This function computes the set of velocities and times which are in collision with the agent: DOVTS
//nradios: number of radios to discretize each interval in collidingTraj
//stretches: false = only the first stretch; true = the whole set of several stretches
//traverse: number of times each stretch is considered to be traversed
//npoints: number of points in which the stretch is discretized to compute the colision times and velocities

    //std::vector<Command> commands; //commands.reserve(2*comm_max);
    //dovt dataDOVT; //commands.reserve(2*comm_max);

    //To add an empty command
    bool primer_neg = false; bool emptyCommand = IsAgentBehind(); //if the obstacle (BC) is behind the robot, then we may need to enter an empty command
    for (int irango = 0; irango<collidingTraj.size(); irango++) {
        Range r = collidingTraj[irango];

        //if (!(std::abs(r.first) == INF || std::abs(r.second) == INF)) continue;

        std::vector<double> titaRadiusVS, radius;
        DiscretizeRange(r.first, r.second, nradios, bounds, radius, titaRadiusVS);

        //bool intersect = (r.calculos == 1);//the trajectories in the range intersect the collision band or not(the whole range lie inside the CB)
        bool intersect = IntersectTrajBand(r);  //the trajectories in the range intersect the collision band or not(the whole range lie inside the CB)
        for (unsigned i = 0; i < (unsigned) titaRadiusVS.size(); i++) {

            //space3d.ClearVTSpace();

            std::vector<Root> roots; bool intersection = intersect;
            //We modify the value of intersect for computing later the stretches only in case the agent is in zone, the acandidate is PI/2 and obstacle is moving with angle +/-PI or 0
            if (intersect) IntersectionPoints(radius[i], titaRadiusVS[i], intersection, roots); //roots are stored in ascending angular order (polar coordinates)
            else TangentPoints(radius[i], roots);   //we define the discretization range with the tangent points between the circunference and a line parallel to rfront/rback

            if (!roots.empty()) {

                std::vector<std::pair<Root, Root>> stretch;
                ComputeStretches(roots, GetOriginInside(), stretches, intersection, stretch);    //pairs of roots in angular ascending order

                for (unsigned tr = 0; tr < (unsigned) stretch.size(); tr++) {

                    //We add an empty command for visualization purposes
                    primer_neg = emptyCommand && radius[i] < 0;
                    if (primer_neg){
                        emptyCommand = false;
                        if (!dataDOVT.GetCommands().empty()){
                            dataDOVT.Add(Command(), RootCommand());
                        }
                    }

                    //3D aproximation version: minimum and maximum velocity and time to avoid collision
                    std::vector<Root> points2d; Command cmd;
                    points2d.push_back(stretch[tr].first);
                    points2d.push_back(stretch[tr].second);
                    if (ComputeVT_2d(points2d, GetOriginInside(), intersect, traverse, radius[i], titaRadiusVS[i], radio_obs, bounds, th, cmd)){
                        dataDOVT.Add(cmd, RootCommand(stretch[tr].first, stretch[tr].second));
                    }

                }
                //almacenar los stretchs según su ocurrencia
                //comparar por tipo de stretch (primer stretch, segundo...) para comprobar si hay hueco entre velocidades en el VT
            }
        }
    }

    //return dataDOVT;
}
*/

bool LinearAgent::GetOriginInside() {
    // returns if the origin lies inside the TrajectoryAgent, the trajectory band swept by the agent

    return ((upper.GetC() <= 0 && lower.GetC() >= 0) || (-lower.GetC() >= 0 && -upper.GetC() <= 0));
}
