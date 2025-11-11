//
// Created by maite16 on 18/07/17.
//

#ifndef VS_CIRCULARAGENT_H
#define VS_CIRCULARAGENT_H

#include "TrajectoryAgent.h"
#include "utilidades.h"

double NormalisePI(double d);

class CircularAgent: public TrajectoryAgent {

    Circumference upper, lower, trajectory;

    double v, w;

    bool IsCBBehind();
    bool IntersectTrajBand(Range r);
    void IntersectionPoints(double rcandidate, double acandidate, bool& intersection, std::vector<Root>& roots);
    void TangentPoints(double rcandidate, std::vector<Root>& roots);
    bool ComputeVT_2d(const std::vector<Root> &points, bool insideBC, const bool intersection, unsigned traverse,
                          double rcandidate, double acandidate, double radio_obs, boundsVS bounds, const int th,
                          const bool accConst, Velocidad velCurrentAg, constraints acc, Command &cmdOut);

public:

    CircularAgent(int nId):TrajectoryAgent(nId){};
    CircularAgent(Tsc sistRef, Tsc sistIn, double size, double vIn, double wIn, double stept, int nId):TrajectoryAgent(nId){

        //Compute location of one agent (sistIn) with respect to another agent (sistRef)
        Tsc l;
        transfor_inversa_sis(sistIn, &sistRef, &l);
        TrajectoryAgent::SetLocalization(l);

        v = vIn; w = wIn;

        //Equation of the circular agent's trajectory from three points
        Tsc p0 = l; Tsc p1, p2;

        p1.x = p0.x - (v/w)*std::sin(p0.tita) + (v/w)*std::sin(p0.tita+w*stept);
        p1.y = p0.y + (v/w)*std::cos(p0.tita) - (v/w)*std::cos(p0.tita+w*stept);
        p1.tita = NormalisePI(p0.tita + w*stept);

        p2.x = p1.x - (v/w)*std::sin(p1.tita) + (v/w)*std::sin(p1.tita+w*stept);
        p2.y = p1.y + (v/w)*std::cos(p1.tita) - (v/w)*std::cos(p1.tita+w*stept);
        p2.tita = NormalisePI(p1.tita + w*stept);

        Tpf pto0, pto1, pto2;
        pto0.x = p0.x; pto0.y = p0.y;
        pto1.x = p1.x; pto1.y = p1.y;
        pto2.x = p2.x; pto2.y = p2.y;
        // printf("WIN: %f\n", wIn);
        trajectory = ComputeCircumference(pto0, pto1, pto2);

        upper = Circumference(trajectory.x(), trajectory.y(), trajectory.r() - size);   //interior circumference
        lower = Circumference(trajectory.x(), trajectory.y(), trajectory.r() + size);   //exterior circumference
    };
    ~CircularAgent(){};

    bool GetOriginInside();
    void IntersectingTrajectories(std::vector<Range>& rangeTraj);
};

#endif //VS_CIRCULARAGENT_H
