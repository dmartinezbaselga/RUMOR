//
// Created by maite14 on 27/06/17.
//

#ifndef VS_PASSIVEAGENT_H
#define VS_PASSIVEAGENT_H

#include "Agent.h"

class PassiveAgent: public Agent {
public:
    PassiveAgent(int i, double x, double y, double tita, double v, double w, double r, bool segment = false, 
    double distance_segment = 0, Tpf first_point = Tpf(), Tpf last_point = Tpf()) :
            Agent(i, x, y, tita, v, w, r, false, true, segment, distance_segment, first_point, last_point){
                //     std::cout <<"New agent: " << x << ", "<< y << ", " << tita << ", " << r << std::endl;
            }
    ~PassiveAgent(){};

    Velocidad
    MotionPlan(double time_step, double time_horizon, unsigned ndata, unsigned mdata, unsigned long numC,
                   int lookAhead) { return {v, w};}
};


#endif //VS_PASSIVEAGENT_H
