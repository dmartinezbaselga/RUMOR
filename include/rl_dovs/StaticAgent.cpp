//
// Created by maite on 24/09/18.
//

#include "StaticAgent.h"

#include "utilidades.h"
#include <tuple>
#include <vector>
#include <iostream>

double offset = 1e-3*0.25/2;

double NormalisePI(double d){
//Transforms the value into +/-pi range

    while (d > M_PI) d -= 2*M_PI;
    while (d < -M_PI) d += 2*M_PI;

    if (d>=-offset && d<=offset) return 0.0;
    else if (d>=M_PI-offset && d<=M_PI+offset) return M_PI;
    else if (d>=-(M_PI+offset) && d<=-(M_PI-offset)) return -M_PI;
    else if (d>=M_PI_2-offset && d<=M_PI_2+offset) return M_PI_2;
    else if (d>=-(M_PI_2+offset) && d<=-(M_PI_2-offset)) return -M_PI_2;

    return d;
}

bool IntersectTrajObj(const Tpf p1, const Tpf p2, const double radio){

    double points[2][2];
    int n = 0;

    Line l(p1, p2);
    if (std::abs(radio) == INF){
        Tsc point;
        SolDosRectas(l, Line(0,1,0), point, n);
        if (n>0){points[0][0] = point.x; points[0][1] = point.y;}
    }
    else
        SolCirculoRecta(l.GetA(), l.GetB(), l.GetC(), 0, radio, radio, points, n);

    //Check if the intersection point(s) are in between the segment endpoints
    for (int i = 0; i<n; i++){
        if ((points[i][0] >= p1.x && points[i][0] <= p2.x || points[i][0] >= p2.x && points[i][0] <= p1.x) &&
            (points[i][1] >= p1.y && points[i][1] <= p2.y || points[i][1] >= p2.y && points[i][1] <= p1.y)){
            return true;
        }
    }
    return false;
}

bool StaticAgent::ConsiderRange(Range range){
//Returns true if any of the radius of the interval intersects the obstacle

    std::vector<std::pair<Tpf,Tpf>> corners = {std::make_pair(cornerAg[0], cornerAg[1]), std::make_pair(cornerAg[1], cornerAg[3]),
                                               std::make_pair(cornerAg[3], cornerAg[2]), std::make_pair(cornerAg[2], cornerAg[0])};
    std::vector<double> radios = {range.first, range.second};
    //Each radio defining the range should intersect the object
    for (auto r:radios){
        bool inter = false;
        for (auto pto:corners){
            inter =  inter || IntersectTrajObj(pto.first, pto.second, r);
        }
        if (!inter) return false;
    }

    return true;
}

void StaticAgent::IntersectionPoints(double rcandidate, double acandidate, bool& intersection, std::vector<Root>& roots){

    std::vector<std::pair<Tpf,Tpf>> corners = {std::make_pair(cornerAg[0], cornerAg[1]), std::make_pair(cornerAg[1], cornerAg[3]),
                                               std::make_pair(cornerAg[3], cornerAg[2]), std::make_pair(cornerAg[2], cornerAg[0])};
    for (auto pto:corners){
        Line l(pto.first, pto.second);
        double points[2][2]; int n = 0;

        if (acandidate == M_PI_2){ //if (std::abs(rcandidate) == INF){
            Tsc point;
            SolDosRectas(l, Line(0,1,0), point, n);
            if (n>0){points[0][0] = point.x; points[0][1] = point.y;}
        }else
            SolCirculoRecta(l.GetA(), l.GetB(), l.GetC(), 0, rcandidate, rcandidate, points, n);

        for (int i = 0; i<n; i++){
            if ((points[i][0] >= pto.first.x && points[i][0] <= pto.second.x || points[i][0] >= pto.second.x && points[i][0] <= pto.first.x) &&
                (points[i][1] >= pto.first.y && points[i][1] <= pto.second.y || points[i][1] >= pto.second.y && points[i][1] <= pto.first.y)){
                roots.push_back(Root(points[i][0], points[i][1], 0));
            }
        }
    }
    if (!roots.empty()) std::sort(roots.begin(), roots.end()); //angular sort of the roots
}

double AngDisplacement(const Root point, const double rcandidate, const unsigned k);
Velocidad ComputeVelocity(const double distance, const double rcandidate, const double acandidate, const double time, const Velocidad max);
bool StaticAgent::ComputeVT_2d(const std::vector<Root> &points, bool insideBC, const bool intersection, unsigned traverse,
                               double rcandidate, double acandidate, double radio_obs, boundsVS bounds, const int th,
                               const bool accConst, Velocidad velCurrentAg, constraints acc, Command &cmdOut) {

    Root root = points[0];

    //The whole range of velocities that lead the robot to follow the circunference arc will make the robot to collide with the object at a time in future
    Velocidad max;
    max = bounds.ComputeMaximumCommand(acandidate);    //ObtenerComandoMaximo(acandidate, bounds.vlim_max, bounds.wmax_left, bounds.wmax_right, max.v, max.w);
    double distance = (acandidate == M_PI_2 || std::abs(root.y) < 1e-5) ? root.x : AngDisplacement(root, rcandidate, 0)*rcandidate; //double distance = AngDisplacement(roots[0], radius[i], 0);
    distance = std::max(0., distance-0.1);
    double t = (acandidate == M_PI_2 || std::abs(root.y) < 1e-5) ? distance/max.v : distance/max.w;
    if (th > 0 && t > th) return false;
    else{
        Comando sup = Comando(max, t);
        // if (th > 0)
        //     inf = Comando(ComputeVelocity(distance, rcandidate, std::abs(root.y) < 1e-5 ? M_PI_2 : acandidate, th, max), th);
        // else {
        double v_inf = sqrt(2*distance*acc.av);
        if (v_inf >= max.v) return false;
        Comando inf = Comando(Velocidad(v_inf, v_inf/rcandidate), distance/v_inf);
        // inf = Comando(Velocidad(),0);
        // std::cout << "Distancia: " << distance << std::endl;
        // std::cout << "max: " << max.v << ", " << max.w << std::endl;
        
        // std::cout << "inf: " << inf.vel.v << ", " << inf.vel.w << std::endl;
        //std::cout << "max: " << max.v << ", " << max.w << std::endl;
        cmdOut = Command(sup, inf, 1, this->GetId());
    }
    return true;
}
