                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    //
// Created by maite14 on 4/04/17.
//

#ifndef VS_ASTAR_H
#define VS_ASTAR_H

#include <queue>

#include "dovts.h"

    class State {

        Velocidad vel;
        double t;
        //Tsc pos;    //Store also the position information associated to the state (for computing costs)

    public:

        State() {
            vel = Velocidad(0, 0);
            t = 0;
            //pos.x = 0; pos.y = 0; pos.tita = 0;
        };

        //State(const Velocidad v, const double time, const Tsc p) : vel(v), t(time), pos(p) {};
        State(const Velocidad v, const double time) : vel(v), t(time) {};

        State(const State &stateIn) {
            vel = stateIn.vel;
            t = stateIn.t;
            //pos = stateIn.pos;
        };

        ~State() {};

        bool operator==(const State &stateIn){
            return (this->vel == stateIn.vel && this->t == stateIn.t);
        };

        bool operator<(const State &stateIn) const
        {
            return t < stateIn.t or (not (stateIn.t < t) and vel.v < stateIn.vel.v);
        }

        bool operator>(const State &stateIn) const
        {
            return t > stateIn.t or (not (stateIn.t > t) and vel.v > stateIn.vel.v);
        }

        friend class Astar;
        friend class Node;
        friend class Cost;
    };

    struct Key{

        unsigned i, j, k;   // to know the cell numbers in the space (w,v,t): the key corresponding to the state

        Key(){ i=-1; j=-1; k=-1;};
        Key(std::pair<keyWV, unsigned> k):i(k.first.first), j(k.first.second), k(k.second){};

        bool operator==(const Key kIn){
            return (i == kIn.i && j == kIn.j && k == kIn.k);
        }

        bool operator<(const Key &kIn) const {
            return (k < kIn.k || (k == kIn.k && j < kIn.j) || (k == kIn.k && j == kIn.j && i < kIn.i));
        }
        bool operator()(const Key& l, const Key& r){
            return l.k < r.k || (l.k == r.k && l.j < r.j) || (l.k == r.k && l.j == r.j && l.i < r.i);
        }

        bool SameVel(const Key &kIn){
            return (i == kIn.i && j == kIn.j);
        }

    };

    class Node {

        State state;
        Key key;    //key corresponding to the state

        double cellCost; //Cost of cell which represents

        double f, g, h; //cost associated to the node

        //pointer to father
        std::shared_ptr<Node> ptr;

    public:

        Node(){

            //key.i = -1; key.j = -1; key.k = -1;
            f = 0; g = 0; h = 0;

            cellCost = 0;

            ptr = std::shared_ptr<Node>(nullptr);
        };

        Node(const Node& p){
            state = p.state;

            key.i = p.key.i; key.j = p.key.j; key.k = p.key.k;
            f = p.f; g = p.g; h = p.h;

            cellCost = p.cellCost;

            ptr = p.ptr;

        }

        Node(const State& s, std::shared_ptr<Node> p, const unsigned w_index, const unsigned v_index, const unsigned t_index, const double cf, const double cg, const double ch, const double cost = 0){

            state = s;

            key.i = w_index; key.j = v_index; key.k = t_index;
            f = cf; g = cg; h = ch;

            cellCost = cost;

            ptr = p;
        };

        ~Node(){};

        bool operator() (const Node& l, const Node& r){
            return (l.f < r.f || (l.f == r.f && l.h < r.h));
        }

        bool operator< (const Node& nin){
            return (this->f < nin.f || (this->f == nin.f && this->h < nin.h));
        }

        bool operator== (const Node &nodeIn){
            return key == nodeIn.key;
        }

        void operator= (const Node &in){
            state = in.state;
            key.i = in.key.i; key.j = in.key.j; key.k = in.key.k;
            f = in.f; g = in.g; h = in.h;

            cellCost = in.cellCost;

            ptr = in.ptr;
        }

        void show(){
            std::cout << "State: " << state.vel.v << ", " << state.vel.w << ", " << state.t << std::endl;
            std::cout << "Index: " << key.i << ", " << key.j << ", " << key.k << std::endl;
            std::cout << "Costs: " << f << ", " << g << ", " << h << std::endl;
            std::cout << "Parent: -> " << std::endl;
            if (ptr) ptr->show(); std::cout << std::endl;
        }

        friend class Astar;
    };

    class Astar {

        double time_step;
        //goal to be followed and reached in time 'followTime', within limits of the VTS
        Velocidad followGoal; double followTime;

        //bool IsAdmissible(const State s);
        bool ComputePlan2(const State start, std::list<State> &goal, std::list<State> &plan, bool (Astar::* function)(Key s));
        //bool ComputePlan(const State start, State goal, int action, std::list<State> &plan, bool (Astar::* function)(Key s));
        void GetNeighbours(const Node current, std::list<Node>& neighbours, bool (Astar::* function)(Key s));

        //Tsc ComputePosition(Tsc current, const Velocidad newVel, const double interval);

        //cost functions
        bool distToObsVS(const Key key, double& time);
        double NumCells(const Key key);

        //bool expand(State s);
        bool expand(Key key);
        bool expand2(const State s);
    public:
        DOVTS space;
        double totalCost;

        Astar(double t, double time_horizon,  unsigned ndata, unsigned mdata,
              boundsVS bounds):time_step(t), totalCost(-1) {

            space = DOVTS((unsigned) std::ceil(time_horizon/time_step), time_step, ndata, mdata, bounds);

        };
        ~Astar(){};

        Velocidad GetFollowGoal();
        double GetFollowTime();
        void ComputeFollowGoal(Tpf goal);
        void SetFollowGoal(Velocidad gF);

        //bool MakePlan(std::list<Velocidad> &plan);
        bool MakePlan(Tsc posR, Tpf posG, std::list<std::pair<Comando, int>> velTarget, std::list<Velocidad> &plan, double& totalCost);

        bool ComputePlan(std::list<Velocidad> &plan);

    };

#endif //VS_ASTAR_H
