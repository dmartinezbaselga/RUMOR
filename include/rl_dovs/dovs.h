//
// Created by maite16 on 6/12/17.
//

#ifndef VS_DOVS_H
#define VS_DOVS_H

#include "VS.h"
#include "Agent.h"

class DOVS: public VS {

    //std::vector<std::vector<Command>> commands; //support for bounded dovt: minimum and maximum times for escape or enter the collision band
    std::vector<Command> commands; //sequential list of minimum and maximum times for escape or enter the collision band
    std::vector<std::pair<Command, Command>> boundsDOV;    //store the bounds of the different DOVs
public:
    DOVS(){};
    DOVS(boundsVS bounds):VS(bounds){};
    ~DOVS(){
        commands.clear();
        boundsDOV.clear();
    };

    const std::vector<std::vector<Command>> InsertDOV(const std::vector<dovt> &commandsIn, std::vector<std::unique_ptr<Agent>> *agents, const double time_step, Agent& currentAg);

    void SetCommands(const std::vector<Command>& cmds);
    std::vector<Command>& GetCommands();
    std::vector<std::pair<Command, Command>> GetBoundsDOV();

};


#endif //VS_DOVS_H
