//
// Created by maite on 24/09/18.
//

#ifndef VS_STATICAGENT_H
#define VS_STATICAGENT_H

#include "LinearAgent.h"

class StaticAgent: public LinearAgent{

    bool ConsiderRange(Range range);

    bool IntersectTrajBand(Range r) { return true; };
    void IntersectionPoints(double rcandidate, double acandidate, bool& intersection, std::vector<Root>& roots);
    void TangentPoints(double rcandidate, std::vector<Root>& roots){};
    bool ComputeVT_2d(const std::vector<Root> &points, bool insideBC, const bool intersection, unsigned traverse,
                          double rcandidate, double acandidate, double radio_obs, boundsVS bounds, const int th,
                          const bool accConst, Velocidad velCurrentAg, constraints acc, Command &cmdOut);

public:

    StaticAgent(Tsc sistRef, Tsc sistIn, double size, double vIn, int nId, bool segment = false, double active_size = 0, 
    Tsc sistIn2 = Tsc())
            : LinearAgent(sistRef, sistIn, size, vIn, nId, segment, active_size, sistIn2){};
    ~StaticAgent(){};

    bool GetOriginInside() { return  false; };
    void IntersectingTrajectories(std::vector<Range>& rangeTraj, bool(TrajectoryAgent::*f)(Range r));
};


#endif //VS_STATICAGENT_H
