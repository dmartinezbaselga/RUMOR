//
// Created by maite14 on 5/12/17.
//

#ifndef VS_VS_H
#define VS_VS_H

#include "TData.h"
#include <iostream>
bool IntersectionPoint(Velocidad s1_ini, Velocidad s1_fin, Velocidad s2_ini, Velocidad s2_fin, std::vector<Velocidad> &points);

//struct to store the velocity bounds in the VS
typedef struct boundsVS{
    double vlim_min, vlim_max, wmax_left, wmax_right;    //velocity bounds on VS
    //double vmax_adm, wleft_adm, wright_adm;    //velocity constraints for the agent
    double tan_v_w, cte_v_w, tan_w_v, cte_w_v; //tangent and constant factors of the equation line which define the triangle constraints in VS

    boundsVS(){};
    boundsVS(double vmin, double vmax, double wleft, double wright, double av, double aw, int n, double stept) : vlim_min(vmin), vlim_max(vmax), wmax_left(wleft), wmax_right(wright){
        //parameters for equation wrt left side of VS space
        // tan_v_w = -av/aw; //cte_v_w = av*n*stept; //v(w)
        // tan_w_v = -aw/av; //cte_w_v = aw*n*stept; //w(v)
        std::cout << "vmax: " << vmax << ", wmax:  " << wleft << "av: " << av << "aw: " << aw << std::endl;
        tan_v_w = -vmax/wleft; cte_v_w = vmax; //v(w), v = -vmax*|w| + vmax
        tan_w_v = -wleft/vmax; cte_w_v = wleft; //w(v)
        //wrt right side of VS space: -tan_v_w, (-tan_w_v, -cte_v_w)
    };
    boundsVS(const boundsVS& data) : vlim_min(data.vlim_min), vlim_max(data.vlim_max), wmax_left(data.wmax_left), wmax_right(data.wmax_right),
                                    tan_v_w(data.tan_v_w), cte_v_w(data.cte_v_w), tan_w_v(data.tan_w_v), cte_w_v(data.cte_w_v){};

    double InsideBoundsW(double w){
        //Returns a value within bounds

        if (w > wmax_left) return wmax_left;
        else if (w < wmax_right) return wmax_right;
        else return w;
    }

    double InsideBoundsV(double v){
        //Returns a value within bounds

        if (v > vlim_max) return vlim_max;
        else if (v < vlim_min) return vlim_min;
        else return v;
    }

    bool IsInsideBounds(Velocidad vel){
        return (vel.v <= vlim_max && vel.w >= wmax_right && vel.w <= wmax_left);
    }

    double ComputeMaxV_W(const double w){
        //Returns the maximum linear velocity the robot can have with the given angular velocity
        return (tan_v_w*std::abs(w) + cte_v_w);
    }

    double ComputeMaxW_V(const double v){
        //Returns the maximum angular velocity (absolute value) the robot can have with the given linear velocity
        return (tan_w_v*v + cte_w_v);
    }

    Velocidad ComputeMaximumCommand(const double ang){

        Velocidad max;

        // se consideran todas las regiones a las cuales pertenence el "angulo de curvatura" que describe el robot
        // if (ang == 0.0){
        //     max.w = cte_v_w/(std::tan(ang)-tan_v_w); max.v = 0; //vmax = 0.0; wmax = w_max_left;
        // }
        // else if (ang == M_PI_2){
        //     max.w = 0; max.v = cte_v_w; //vmax = vmax_adm; wmax = 0.0;
        // }
        // else if (ang == M_PI){
        //     max.w = -(cte_v_w/(std::tan(ang)-tan_v_w)); max.v = 0; //vmax = 0.0; wmax = w_max_right
        // }
        // else if (0.0 < ang && ang < M_PI_2){
        //     max.w = cte_v_w/(std::tan(ang)-tan_v_w); max.v = tan_v_w*max.w + cte_v_w;   //wmax = w_max_left; vmax = std::tan(angulo)*wmax;
        // }
        // else if (M_PI_2 < ang && ang < M_PI){
        //     max.w = cte_v_w/(std::tan(ang)+tan_v_w); max.v = -tan_v_w*max.w + cte_v_w;   //vmax = vmax_adm; wmax = vmax/std::tan(angulo);
        // }

        // Consider no kinodynamic restrictions
        // se consideran todas las regiones a las cuales pertenence el "angulo de curvatura" que describe el robot
        if (ang == 0.0){
            max.v = 0.0; 
            max.w = wmax_left;
        }
        else if (ang == M_PI_2){
            max.v = vlim_max; 
            max.w = 0.0;
        }
        else if (ang == M_PI){
            max.v = 0.0; 
            max.w = wmax_right;
        }
        else if (0.0 < ang && ang < M_PI_2){
            if (std::tan(ang)*wmax_left > vlim_max){
                max.v = vlim_max;
                max.w = vlim_max/std::tan(ang);
            }
            else{
                max.w = wmax_left;
                max.v = wmax_left*std::tan(ang);
            }
        }
        else if (M_PI_2 < ang && ang < M_PI){
            if (std::tan(ang)*wmax_right > vlim_max){
                max.v = vlim_max;
                max.w = vlim_max/std::tan(ang);
            }
            else{
                max.w = wmax_right;
                max.v = wmax_right*std::tan(ang);
            }
        }
        return max;
    }

    Velocidad GetBoundVelocity(const Velocidad current, const Velocidad v, const Velocidad v1, const Velocidad v2){
    //Compute the intersection between segment (v-v1) and the bounds, as well as (v-v2) and the bounds
    //If there is intersection, v is updated with the intersection point
        Velocidad vel = current;
        std::vector<Velocidad> points;
        Velocidad bound1 = Velocidad(cte_v_w,0); Velocidad bound2 = Velocidad(0, -cte_v_w/tan_v_w);
        if (IntersectionPoint(v1,v, bound1, bound2, points) || IntersectionPoint(v,v2, bound1, bound2, points)){
            //assume there only one intersection point
            vel = points[0];
        }else {
            Velocidad bound1 = Velocidad(cte_v_w,0); Velocidad bound2 = Velocidad(0,cte_v_w/tan_v_w);
            if (IntersectionPoint(v1,v, bound1, bound2, points) || IntersectionPoint(v, v2, bound1, bound2, points)){
                vel = points[0];
            }
        }
        return vel;
    }
};

//struct to store goal information in the VS
typedef struct goalVS{
    //velGoal is the velocity to reach the goal in the next step; it is assumed that the time associated is stept
    //comandoGoal is the maximum velocity reachable following the circunference arc of the goal, within VS bounds; or the dir_goal (which converges to goal)
    Velocidad velGoal, commandGoal, dirGoal;   //double steering_w, steering_dir;

    goalVS(){};
    goalVS(Velocidad v, Velocidad c, Velocidad d):velGoal(v), commandGoal(c), dirGoal(d){};
};

//struct to store the dynamic window of the agent given a velocity
//define the controls the agent can perform at an instant from a velocity (center)
typedef struct constraints{
    double av, aw; //the triangle of the DW for a differential-drive robot is defined by acceleration constraints
    double k;   //offset to obtain more or less velocity values from the control space VS

    constraints(){};
    //constraints(double aV, double aW, double deltat):av(aV), aw(aW), t(deltat){};
    constraints(double aV, double aW):av(aV), aw(aW){};
};

class VS {
protected:
    boundsVS bounds;    //define the control space of the agent: set of possible velocities for the agent
private:
    goalVS goal;    //Goal information

    //Agent information
    Velocidad velAgent; //current agent velocity
    constraints diffConstraints;

public:
    VS(){};
    VS(boundsVS dataVS):bounds(dataVS){};
    virtual ~VS(){};

    void InsertGoal(Tpf goal, double stept);
    void InsertAgent(Velocidad vAgent, double aV, double aW);

    void InsertBounds(boundsVS dataVS);

    bool VelReachable(double step, const Velocidad & vel);

    void SetDirGoal (Velocidad v);
    Velocidad GetDirGoal ();

    boundsVS GetBounds() const;
    goalVS GetGoal() const;
    Velocidad GetAgent() const;
    constraints GetConstraints() const;

    double GetMaxV(const double w);
    double GetMaxW(const double v);
};
enum movement {UP, DOWN, LEFT, RIGHT, EQUAL};   //in dynamic window, seen from current velocity
typedef struct DW{
    Velocidad up, down, left, right, equal;

    DW(){};
    DW(Velocidad current, constraints c, boundsVS bounds, double t, bool ignoreBounds = false){

        double maxw_v = bounds.InsideBoundsW(bounds.ComputeMaxW_V(current.v));
        double maxv_w = bounds.InsideBoundsV(bounds.ComputeMaxV_W(current.w));
        if (!ignoreBounds) {
            right = { current.v, (std::abs(current.w + c.aw*t) <= maxw_v ? current.w + c.aw*t : copysign(maxw_v, current.w + c.aw*t))};
            left = { current.v, (std::abs(current.w - c.aw*t) <= maxw_v ? current.w - c.aw*t : copysign(maxw_v, current.w - c.aw*t))};
            up = {current.v + c.av*t <= maxv_w ? current.v + c.av*t : maxv_w, current.w};
            down = {bounds.InsideBoundsV(current.v - c.av*t) <= maxv_w ? bounds.InsideBoundsV(current.v - c.av*t) : maxv_w, current.w};
        } else {
            right = { current.v, (std::abs(current.w + c.aw*t) <= maxw_v ? current.w + c.aw*t : -5)};
            left = { current.v, (std::abs(current.w - c.aw*t) <= maxw_v ? current.w - c.aw*t : -5)};
            up = {current.v + c.av*t <= maxv_w ? current.v + c.av*t : -5, current.w};
            down = {bounds.InsideBoundsV(current.v - c.av*t) <= maxv_w ? bounds.InsideBoundsV(current.v - c.av*t) : -5, current.w};
        }

        equal = current;

        //Special checking if the current velocity of the robot is already at the maximum reachable
        if (std::abs(current.w - maxw_v <= 5e-3) && std::abs(current.v - maxv_w) <= 5e-3 && !ignoreBounds){
            if (right == current){
                right = bounds.GetBoundVelocity(right, {current.v, current.w + c.aw*t}, {current.v + c.av*t, current.w}, {current.v - c.av*t, current.w});
            }
            if (left == current){
                left = bounds.GetBoundVelocity(left, {current.v, current.w - c.aw*t}, {current.v + c.av*t, current.w}, {current.v - c.av*t, current.w});
            }
            if (up == current){
                up = bounds.GetBoundVelocity(up, {current.v + c.av*t, current.w}, {current.v, current.w - c.aw*t}, {current.v, current.w + c.aw*t});
            }
            if (down == current){
                down = bounds.GetBoundVelocity(down, {current.v - c.av*t, current.w}, {current.v, current.w - c.aw*t}, {current.v, current.w + c.aw*t});
            }
        }
    }
    ~DW(){};
    Velocidad GetVelocity(movement mov){
        switch (mov){
            case UP:
                return up;
            case DOWN:
                return down;
            case LEFT:
                return left;
            case RIGHT:
                return right;
            case EQUAL:
                return equal;
        }
    };
};


#endif //VS_VS_H
