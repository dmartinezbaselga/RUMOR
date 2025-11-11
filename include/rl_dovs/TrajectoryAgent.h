//
// Created by maite16 on 18/07/17.
//

#ifndef VS_TRAJECTORYAGENT_H
#define VS_TRAJECTORYAGENT_H

#include <vector>

#include "TData.h"
#include "dovts.h"


class TrajectoryAgent {
//Class which is in charge of making the operations relative to the trajectory of one agent
//Location information in the class is relative between two systems, so TrajectoryAgent information is stored with respect to another system (and thus, that system will be the origin in the TrajectoryAgent class)

    int id;
    Tsc loc;

    virtual bool IsCBBehind() = 0;
    virtual bool IntersectTrajBand(Range r) = 0;
    virtual void IntersectionPoints(double rcandidate, double acandidate, bool& intersection, std::vector<Root>& roots) = 0;
    virtual void TangentPoints(double rcandidate, std::vector<Root>& roots) = 0;
    virtual bool ComputeVT_2d(const std::vector<Root> &points, bool insideBC, const bool intersection, unsigned traverse,
                                  double rcandidate, double acandidate, double radio_obs, boundsVS bounds, const int th,
                                  const bool accConst, Velocidad velCurrentAg, constraints acc, Command &cmdOut) = 0;
public:
    TrajectoryAgent(int i):id(i){};
    ~TrajectoryAgent(){};

    int GetId();
    Tsc GetLocalization();
    void SetLocalization(Tsc l);

    //void ComputeDOVT(const std::vector<Range> &collidingTraj, unsigned nradios, bool stretches, unsigned traverse, double radio_obs, boundsVS bounds, const double th, dovt &dataDOVT);
    void ComputeDOVT(const std::vector<Range> &collidingTraj, unsigned nradios, bool stretches, unsigned traverse,
                     double radio_obs, boundsVS bounds, const double th, const bool accConst, Velocidad velCurrentAg,
                     constraints acc, dovt &dataDOVT, Tsc agLoc);

    virtual bool GetOriginInside() = 0;
    virtual void IntersectingTrajectories(std::vector<Range>& rangeTraj) = 0;   //Computes the different ranges of circular trajectories which intersect with or lie inside the trajectory of the relative agent
};

#endif //VS_TRAJECTORYAGENT_H