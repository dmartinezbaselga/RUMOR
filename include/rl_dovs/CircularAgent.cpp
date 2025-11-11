//
// Created by maite16 on 18/07/17.
//

#include "CircularAgent.h"

void CircularAgent::IntersectingTrajectories(std::vector<Range>& rangeTraj){

    //Tangent radius to the circumference, defining the collision band
    double r1_up, r2_up, r1_down, r2_down; Tpf tan1, tan2;
    RadioPtoTanCircunf(upper.x(), upper.y(), upper.r(), r1_up, tan1, r2_up, tan2);
    RadioPtoTanCircunf(lower.x(), lower.y(), lower.r(), r1_down, tan1, r2_down, tan2);

    std::vector<double> r_pos, r_neg;
    if (r1_up >= 0) r_pos.push_back(r1_up); else r_neg.push_back(r1_up);
    if (r2_up >= 0) r_pos.push_back(r2_up); else r_neg.push_back(r2_up);
    if (r1_down >= 0) r_pos.push_back(r1_down); else r_neg.push_back(r1_down);
    if (r2_down >= 0) r_pos.push_back(r2_down); else r_neg.push_back(r2_down);
    sort(r_pos.begin(), r_pos.end());
    sort(r_neg.begin(), r_neg.end());

    //Intersection of the circumferences with the 'x' axis
    double raiz_up[2][2], raiz_down[2][2]; int n_sol_up, n_sol_down;
    SolCirculoRecta(0, 1, 0, upper.x(), upper.y(), upper.r(), raiz_up, n_sol_up);
    SolCirculoRecta(0, 1, 0, lower.x(), lower.y(), lower.r(), raiz_down, n_sol_down);

    //special cases: agent point (0,0) is tangent to upper or lower circumferences
    bool computed = false;
    switch (n_sol_up){
        case 0:
            if (n_sol_down == 1){
                Range r = {0, r1_up};
                rangeTraj.push_back({0, r1_up});
                rangeTraj.push_back({r1_up, r2_up});
                rangeTraj.push_back({r2_up, lower.r()});
                computed = true;
            }
            break;
        case 1:
            if (n_sol_down == 2){
                rangeTraj.push_back({upper.r(), r1_down});
                rangeTraj.push_back({r1_down, INF});
                rangeTraj.push_back({-INF, r2_down});
                rangeTraj.push_back({r2_down, 0});
                computed = true;
            }
            break;
    }

    //Computation of the range of trajectories
    bool insideCB = GetOriginInside();
    if (!computed){
        if (!r_pos.empty()){
            if (insideCB) rangeTraj.push_back({0, r_pos[0]});
            for (int i=0; i<r_pos.size()-1; i++){
                rangeTraj.push_back({r_pos[i], r_pos[i+1]});
            }
            if (!r_neg.empty()) rangeTraj.push_back({r_pos[r_pos.size()-1], INF});
        }

        if (!r_neg.empty()){
            if (!r_pos.empty()) rangeTraj.push_back({-INF, r_neg[0]});
            for (int i=0; i<r_neg.size()-1; i++){
                    rangeTraj.push_back({r_neg[i], r_neg[i+1]});
            }
            if (insideCB) rangeTraj.push_back({r_neg[r_neg.size()-1], 0});
        }
    }

    //Check that lower is the nearest circumference to the robot, and the first with which candidate trajectories intersect
    if (!insideCB){
        if (std::min(std::abs(r1_up), std::abs(r2_up)) < std::min(std::abs(r1_down), std::abs(r2_down))){
            //Interchange of the circumferences
            Circumference caux = lower;
            lower = upper; upper = caux;
        }
    }else{
        double min_r =  *std::min_element(r_pos.begin(), r_pos.end());
        if ( min_r == r1_up || min_r == r2_up){
            //Interchange of the circumferences
            Circumference caux = lower;
            lower = upper; upper = caux;
        }
    }
}

bool CircularAgent::GetOriginInside() {
    // returns if the origin lies inside the TrajectoryAgent, the trajectory band swept by the agent

    double raiz_up[2][2], raiz_down[2][2]; int n_sol_up, n_sol_down;

    //Intersection of the circumferences with the 'x' axis
    SolCirculoRecta(0, 1, 0, upper.x(), upper.y(), upper.r(), raiz_up, n_sol_up);
    SolCirculoRecta(0, 1, 0, lower.x(), lower.y(), lower.r(), raiz_down, n_sol_down);

    switch (n_sol_up){
        case 0:
        case 1:
            if (n_sol_down > 1){
                if ((raiz_down[0][0] <= 0 && raiz_down[1][0] >= 0) || (raiz_down[0][0] >= 0 && raiz_down[1][0] <= 0)){
                    return true;
                }
            }
            break;
        case 2:
            if ((raiz_up[0][0] >= 0 && raiz_up[1][0] >= 0) || (raiz_up[0][0] <= 0 && raiz_up[1][0] <= 0)){
                if (n_sol_down > 1){
                    if ((raiz_down[0][0] <= 0 && raiz_down[1][0] >= 0) || (raiz_down[0][0] >= 0 && raiz_down[1][0] <= 0)){
                        return true;
                    }
                }
            }
            break;
    }

    return false;
}

bool CircularAgent::IsCBBehind(){
//Compute if the CB intersects in front of the robot, or if the obstacle is before the robot

    bool behind = true;

    double raices[2][2]; int sol;
    SolCirculoRecta(0, 1, 0, lower.x(), lower.y(), lower.r(), raices, sol);
    if (sol > 0){
        for (int i=0; i<sol; i++){
            if (raices[i][0] > 0){
                behind = false;
                break;
            }
        }
    }else{
        if (GetLocalization().x > 0) behind = false;
    }

    return behind;
}

bool CircularAgent::IntersectTrajBand(Range r){
//Returns true if any of the radius of the interval intersects the collision band

    double points[2][2];
    int n1 = 0; int n2 = 0; int n3 = 0; int n4 = 0;

    Line l;
    Circumference cradio(0, r.first, r.first);
    RectaDosCircunf(cradio, lower, l);  //line that goes through the intersection points between the circumferences
    SolCirculoRecta(l.GetA(), l.GetB(), l.GetC(), lower.x(), lower.y(), lower.r(), points, n1);	//Intersection points with lower circumference
    if (n1 < 2) {  //the radius does not intersect the collision band or it intersects in the tangent point
        RectaDosCircunf(cradio, upper, l);  //line that goes through the intersection points between the circumferences
        SolCirculoRecta(l.GetA(), l.GetB(), l.GetC(), upper.x(), upper.y(), upper.r(), points, n2);	//Intersection points with upper circumference
        if (n2 < 2){
            cradio = Circumference(0, r.second, r.second);
            RectaDosCircunf(cradio, lower, l);
            SolCirculoRecta(l.GetA(), l.GetB(), l.GetC(), lower.x(), lower.y(), lower.r(), points, n3);
            if (n3 < 2){
                RectaDosCircunf(cradio, upper, l);
                SolCirculoRecta(l.GetA(), l.GetB(), l.GetC(), upper.x(), upper.y(), upper.r(), points, n4);
            }
        }
    }

    return (n1 >= 2 || n2 >= 2 || n3 >= 2 || n4 >= 2) || ((n1 == 1 || n2 == 1) && r.second == INF) || ((n3 == 1 || n4 == 1) && r.first == -INF);
}

void CircularAgent::IntersectionPoints(double rcandidate, double acandidate, bool& intersection, std::vector<Root>& roots){
//This function computes the intersecting points between a candidate trajectory and the collision band of the circular agent

    int nsol = 0;
    double raices[2][2];

    double size = lower.r() - upper.r();

    if (acandidate != M_PI_2){

        Line l;
        Circumference cradio(0, rcandidate, rcandidate);    //circular trajectory to consider as candidate
        RectaDosCircunf(cradio, lower, l);  //line that goes through the intersection points between the circumferences
        SolCirculoRecta(l.GetA(), l.GetB(), l.GetC(), lower.x(), lower.y(), lower.r(), raices, nsol);	//Intersection points with lower circumference
        for (int i=0; i<nsol; i++){
            roots.push_back(Root(raices[i][0], raices[i][1], 0));
        }

        if (upper.r() > size){ //If the interior hole is not big enough so the agent can fit inside, then we consider there is no space
            RectaDosCircunf(cradio, upper, l);
            SolCirculoRecta(l.GetA(), l.GetB(), l.GetC(), upper.x(), upper.y(), upper.r(), raices, nsol);
            if (nsol > 1){
                for (int i=0; i<nsol; i++){
                    roots.push_back(Root(raices[i][0], raices[i][1], 1));
                }
            }
        }

    }else{	//radius INF

        double raices[2][2];
        SolCirculoRecta(0, 1, 0, lower.x(), lower.y(), lower.r(), raices, nsol);
        for (int i=0; i<nsol; i++){
            if (raices[i][0] >= 0){
                roots.push_back(Root(raices[i][0], raices[i][1], 0));
            }
        }

        if (upper.r() > size){
            SolCirculoRecta(0, 1, 0, upper.x(), upper.y(), upper.r(), raices, nsol);
            if (nsol > 1){
                for (int i=0; i<nsol; i++){
                    if (raices[i][0] >= 0){
                        roots.push_back(Root(raices[i][0], raices[i][1], 1));
                    }
                }
            }
        }
    }

    if (!roots.empty())
        std::sort(roots.begin(), roots.end());  //angular sort of the roots

}

void CircularAgent::TangentPoints(double rcandidate, std::vector<Root>& roots){

    roots.push_back(Root(0, 0));
    roots.push_back(Root(0, 0));
}

struct Compare {
    Tpf p;
    Compare(Tpf point) {this->p = point;}
    //Sort depending on the distance wrt to point p
    bool operator()(const std::pair<double, double>& l, const std::pair<double, double>& r){
        return ((p.x - l.first)*(p.x - l.first) + (p.y - l.second)*(p.y - l.second) <  (p.x - r.first)*(p.x - r.first) + (p.y - r.second)*(p.y - r.second));
    }
};

// l < r <=> r > l
// l == r <=> !(l<r) && !(l>r)
struct CompareTpfp {
    double w;
    Tsc sist;
    CompareTpfp(Tsc sistema, double velW){this->sist = sistema; this->w = velW;}
    //Criterio de ordenación según coordenadas polares de los puntos
    //Para comprobar si el obstáculo está dentro del intervalo de los puntos,
    //que definen la trayectoria de movimiento del obstáculo
    /*bool operator()(const Tpfp& l, const Tpfp& r){
        return ((w > 0 && l.t < r.t) || (w < 0 && l.t > r.t));
    }*/
    //bool CompareTpfp(const Tpfp& l, const Tpfp& r){
    bool operator()(const Root& l, const Root& r){
        Tpfp left = Tpfp(l.radius, l.angle);
        Tpfp right = Tpfp(r.radius, r.angle);
        Tpf pl, pr;
        //los puntos a ordenar están en coordenadas polares en su sistema original
        //calculo sus coordenadas cartesianas
        pol2car(&left, &pl); pol2car(&right, &pr);
        //std::cout << "Ordenamos: " << std::endl;
        //std::cout << pl.x << ", " << pl.y << ", " << left.t << ", " << left.r << std::endl;
        //std::cout << pr.x << ", " << pr.y << ", " << right.t << ", " << right.r << std::endl;
        Tpf sol, sol1;
        //transformo sus coordenadas cartesianas en el sistema 'sist'
        transfor_inversa_p(pl.x, pl.y, &sist, &sol);
        transfor_inversa_p(pr.x, pr.y, &sist, &sol1);
        //std::cout << "respecto al centro de la circ: " << sol.x << ", " << sol.y << std::endl;
        //std::cout << "respecto al centro de la circ: " << sol1.x << ", " << sol1.y << std::endl;
        return (w > 0 &&
                ((pl.x < 0 && pr.x < 0 && (l.angle < r.angle || (l.angle == r.angle && l.radius > r.radius))) ||
                 (pl.x >= 0 && pr.x >= 0 && (l.angle < r.angle || (l.angle == r.angle && l.radius < r.radius))) ||
                 ((pl.x < 0 && pr.x >= 0 || pl.x >= 0 && pr.x <0) &&
                  (((sol.y >= 0 && sol1.y >= 0) && l.angle < r.angle) ||
                   ((sol.y <= 0 && sol1.y <= 0) && l.angle > r.angle))))) ||
               (w < 0 &&
                ((pl.x < 0 && pr.x < 0 && (l.angle > r.angle || (l.angle == r.angle && l.radius > r.radius))) ||
                 (pl.x >= 0 && pr.x >= 0 && (l.angle > r.angle || (l.angle == r.angle && l.radius < r.radius))) ||
                 ((pl.x < 0 && pr.x >= 0 || pl.x >= 0 && pr.x <0) &&
                  (((sol.y >= 0 && sol1.y >= 0) && l.angle < r.angle) ||
                   ((sol.y <= 0 && sol1.y <= 0) && l.angle > r.angle)))));

        /*return (pl.x < 0 && pr.x < 0 && (l.t < r.t || (l.t == r.t && l.r > r.r))) ||
                (pl.x >= 0 && pr.x >= 0 && (l.t < r.t || (l.t == r.t && l.r < r.r))) ||
                ((pl.x < 0 && pr.x >= 0 || pl.x >= 0 && pr.x <0) &&
                  (((sol.y >= 0 && sol1.y >= 0) && l.t < r.t) ||
                  ((sol.y <= 0 && sol1.y <= 0) && l.t > r.t)) ||
                  ((sol.y >= 0 && sol1.y <= 0 || sol.y <=0 && sol1.y >= 0) && l.t < r.t));
                  */
        //(pl.x < 0 && pr.x >= 0 && l.t > r.t);
    }
};


bool clockwise(Tpf v1, Tpf v2){
    //std::cout << "clockwise: " << -v1.x*v2.y + v1.y*v2.x << std::endl;
    return (-v1.x*v2.y + v1.y*v2.x > 0);
}

bool InsideInterval(std::vector<Root> points, Tsc loc_obj, double w, Tpf center){

    //Busco los puntos que estén separados a mayor distancia
    //Compruebo si la localización del obstáculo con respecto al robot está entre esos puntos extremos
    int max_dist = 0; Tpf max_point1, max_point2;
    /*for (int i=0; i<(int)points.size(); i++){
        Tpf pto1;
        pol2car(&points[i], &pto1);
        for (int j=i+1; j<(int)points.size(); j++){
            Tpf pto2;
            pol2car(&points[j], &pto2);
            double dist = std::sqrt((pto1.x - pto2.x)*(pto1.x - pto2.x) + (pto1.y - pto2.y)*(pto1.y - pto2.y));
            if (dist > max_dist){
                max_dist = dist;
                max_point1 = pto1;
                max_point2 = pto2;
            }
        }
    }*/

    Tpf point; point.x = loc_obj.x; point.y = loc_obj.y;
    Tpfp p0 = Tpfp(points[0].radius, points[0].angle);
    Tpfp p1 = Tpfp(points[(int)points.size()-1].radius, points[(int)points.size()-1].angle);
    pol2car(&p0, &max_point1);
    pol2car(&p1, &max_point2);
    //return (!clockwise(max_point1, point) && clockwise(max_point2, point));
    Tsc sistema; Tpf sol1, sol2, sol1_center, sol2_center;
    sistema.x = loc_obj.x; sistema.y = loc_obj.y; sistema.tita = loc_obj.tita;
    //Trasnformamos los puntos al sistema del robot (están en el sistema del obstáculo)
    transfor_directa_p(max_point1.x, max_point1.y, &sistema, &sol1);
    transfor_directa_p(max_point2.x, max_point2.y, &sistema, &sol2);
    //if (!clockwise(max_point1, point) && clockwise(max_point2, point)){
    //Transformamos los puntos en el sistema del robot al sistema situado en el centro de la circunferencia
    sistema.x = center.x; sistema.y = center.y; sistema.tita = 0;
    transfor_inversa_p(sol1.x, sol1.y, &sistema, &sol1_center);
    transfor_inversa_p(sol2.x, sol2.y, &sistema, &sol2_center);
    transfor_inversa_p(loc_obj.x, loc_obj.y, &sistema, &point);

    if (w > 0){
        if (!clockwise(sol1_center, point) && clockwise(sol2_center, point)){
            return true;
        }
    }else{
        if (clockwise(sol1_center, point) && !clockwise(sol2_center, point)){
            return true;
        }
    }
    return false;

    /*
    if ((loc_obj_rob.x <= max_point1.x && loc_obj_rob.x >= max_point2.x || loc_obj_rob.x >= max_point1.x && loc_obj_rob.x <= max_point2.x) &&
        (loc_obj_rob.y <= max_point1.y && loc_obj_rob.y >= max_point2.y || loc_obj_rob.y >= max_point1.y && loc_obj_rob.y <= max_point2.y)){
        std::cout << "Inside interval" << std::endl;
        return true;
    }

    std::cout << "Not inside interval" << std::endl;
    return false;
    */

}

Velocidad ComputeVelocity(const double distance, const double rcandidate, const double acandidate, const double time, const Velocidad max);
double AngDisplacement(const Root point, const double rcandidate, const unsigned k);
bool CircularAgent::ComputeVT_2d(const std::vector<Root> &points, bool insideBC, const bool intersection, unsigned traverse,
                                 double rcandidate, double acandidate, double radio_obs, boundsVS bounds, const int th,
                                 const bool accConst, Velocidad velCurrentAg, constraints acc, Command &cmdOut) {

    bool intersect = intersection;
    double tMin = 1e-5; double tMax = 1e-5;

    Root root1 = points[0];
    Root root2 = points[1];

    double size = lower.r() - upper.r();

    std::vector<std::pair<double,double>> centers;
    Line l1(Tpf(root1.x, root1.y), Tpf(lower.x(), lower.y()));
    Line l2(Tpf(root2.x, root2.y), Tpf(lower.x(), lower.y()));

    std::vector<Root> positions;
    std::vector<Root> totalPoints;
    //Compute the circumferences of radius 'size' tangent to line 'l' that go through the agent trajectory
    //These circumferences correspond to the obstacle in different positions, at different instants
    CircunfTanRecta(trajectory, l1, size, centers);
    if (!centers.empty()){
        //Sort of the circumferences wrt distance to the intersecting root; the two first ones are of interest
        std::sort(centers.begin(), centers.end(), Compare(Tpf(root1.x, root1.y)));

        Tsc sist; Tpf sol1, sol2;
        sist.x = GetLocalization().x; sist.y = GetLocalization().y; sist.tita = GetLocalization().tita;
        transfor_inversa_p(centers[0].first, centers[0].second, &sist, &sol1);
        transfor_inversa_p(centers[1].first, centers[1].second, &sist, &sol2);
        positions.push_back(Root(sol1.x, sol1.y)); positions.push_back(Root(sol2.x, sol2.y));
        totalPoints.push_back(Root(sol1.x, sol1.y)); totalPoints.push_back(Root(sol2.x, sol2.y));

        //Angular displacement the obstacle should do to reach the positions
        double distance1 = std::abs(sol1.y) < 1e-5 ? copysign(sol1.x, trajectory.r()) : AngDisplacement(Root(sol1.x, sol1.y), trajectory.r(), 0);
        double distance2 = std::abs(sol2.y) < 1e-5 ? copysign(sol2.x, trajectory.r()) : AngDisplacement(Root(sol2.x, sol2.y), trajectory.r(), 0);

        //Calculo el centro de la circunferencia con respecto al obstáculo
        Tpf sol;
        transfor_inversa_p(trajectory.x(), trajectory.y(), &sist, &sol);
        Tsc sistCenter; sistCenter.x = sol.x; sistCenter.y = sol.y; sistCenter.tita = 0; //loc_obj_rob.tita;
        std::sort(positions.begin(), positions.end(), CompareTpfp(sistCenter, w));

        if (InsideInterval(positions, GetLocalization(), w, Tpf(trajectory.x(), trajectory.y()))){
            if (distance1 < distance2) tMax = distance1/w;
            else tMax = distance2/w;
        }else{
            if (distance1 > distance2) tMax = distance1/w;
            else tMax = distance2/w;
        }
    }

    centers.clear(); positions.clear();
    CircunfTanRecta(trajectory, l2, size, centers);
    if (!centers.empty()){
        //Sort of the circumferences wrt distance to the intersecting root; the two first ones are of interest
        std::sort(centers.begin(), centers.end(), Compare(Tpf(root2.x, root2.y)));

        Tsc sist; Tpf sol1, sol2;
        sist.x = GetLocalization().x; sist.y = GetLocalization().y; sist.tita = GetLocalization().tita;
        transfor_inversa_p(centers[0].first, centers[0].second, &sist, &sol1);
        transfor_inversa_p(centers[1].first, centers[1].second, &sist, &sol2);
        positions.push_back(Root(sol1.x, sol1.y)); positions.push_back(Root(sol2.x, sol2.y));
        totalPoints.push_back(Root(sol1.x, sol1.y)); totalPoints.push_back(Root(sol2.x, sol2.y));

        //Angular displacement the obstacle should do to reach the positions
        double distance1 = std::abs(sol1.y) < 1e-5 ? copysign(sol1.x, trajectory.r()) : AngDisplacement(Root(sol1.x, sol1.y), trajectory.r(), 0);
        double distance2 = std::abs(sol2.y) < 1e-5 ? copysign(sol2.x, trajectory.r()) : AngDisplacement(Root(sol2.x, sol2.y), trajectory.r(), 0);

        //Calculo el centro de la circunferencia con respecto al obstáculo
        Tpf sol;
        transfor_inversa_p(trajectory.x(), trajectory.y(), &sist, &sol);
        Tsc sistCenter; sistCenter.x = sol.x; sistCenter.y = sol.y; sistCenter.tita = 0; //loc_obj_rob.tita;
        std::sort(positions.begin(), positions.end(), CompareTpfp(sistCenter, w));
        std::sort(totalPoints.begin(), totalPoints.end(), CompareTpfp(sistCenter, w));

        if (!InsideInterval(totalPoints, GetLocalization(), w, Tpf(trajectory.x(), trajectory.y()))){
            if (distance1 < distance2) tMin = distance1/w;
            else tMin = distance2/w;
        }
    }

    Root rootMin, rootMax;
    rootMin = root2; rootMax = root1;

    ///*
    if (th > 0){
        //if (!intersect) th *= 2;
        //else if (insideBC) th *= 1.5;

        //assert(tMax >= tMin);

        if (tMax > th) return false;
        else{}
        //std::cout << " tMax: " << tMax << std::endl;
    }
    //*/

    unsigned ntraverse = (acandidate == M_PI_2) ? 1 : traverse;
    for (unsigned k = 0; k<ntraverse; k++){ //Update the forbidden velocities and times

        Velocidad max;
        max = bounds.ComputeMaximumCommand(acandidate);    //ObtenerComandoMaximo(acandidate, bounds.vlim_max, bounds.wmax_left, bounds.wmax_right, max.v, max.w);

        if (!intersect){
            Comando sup = Comando(max, tMin);
            Comando inf = Comando(Velocidad(0,0), tMax);
            cmdOut = Command(sup, inf, 1, this->GetId());
        }else{

            bool computeCommand = (tMax > 1e-5 || insideBC);
            if (computeCommand) {

                //Computes the angular displacement the agent should perform to reach the point
                double distance = (acandidate == M_PI_2 || std::abs(rootMin.y) < 1e-5) ? copysign(rootMin.x, rcandidate) : AngDisplacement(rootMin, rcandidate, k);
                Velocidad vel;
                if (tMin > 1e-5) vel = ComputeVelocity(distance, rcandidate, acandidate, tMin, max);
                else vel = max;
                Comando sup = Comando((max < vel) ? max : vel, tMin);

                if (!(rootMax == Root(0, 0))) {
                    distance = (acandidate == M_PI_2 || std::abs(rootMax.y) < 1e-5) ? copysign(rootMax.x, rcandidate) : AngDisplacement(rootMax, rcandidate, k);
                    vel = ComputeVelocity(distance, rcandidate, acandidate, tMax, max);
                    if (max < vel) {    //the forbidden velocity lies outside the velocity bounds of the agent
                        return false;   //computeCommand = false; break;
                    }
                }else{
                    vel = Velocidad(0, 0);
                    /*///caso especial cuando una de las circunferencias de la banda de colision coincide
                    //con alguno de los ejes en el sistema del robot y radio es 0
                    if (loc_obj_rob.y > 0){
                        //giros a izquierda prohibidos porque nos metemos en la banda de colision
                        c.sup.vel = Velocidad(0.0,wmax_izq);
                    }else{
                        //giros a derecha prohibidos porque nos metemos en la banda de colision
                        c.sup.vel = Velocidad(0.0,wmax_dch);
                    }*/
                }
                Comando inf = Comando(vel, tMax);
                cmdOut = Command(sup, inf, 1, this->GetId());

            }else
                return false;
        }

    }

    return true;

}