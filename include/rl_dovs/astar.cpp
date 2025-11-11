//
// Created by maite14 on 4/04/17.
//


#include <cstddef>

#include "astar.h"

#include <queue>

/*
Node::Node(){

    parent = NULL;
    i = -1; j = -1; k = -1;
    f = 0; g = 0; h = 0;

    //ptr = nullptr;
    //ptr = std::unique_ptr<Node>(nullptr);
}


Node::Node(State s, Node *p, unsigned w_index, unsigned v_index, unsigned t_index, double cf, double cg, double ch){

    state = s;
    parent = p;
    i = w_index; j = v_index; k = t_index;
    f = cf; g = cg; h = ch;

    //ptr = std::move(p->ptr);

}

Node::~Node(){

    //TODO: check the correct delete of pointers (primero recorre la lista eliminando los estados y después regresa eliminando los padres; es decir, va desde final al principio eliminando los state y del principio al final eliminando los padres)
    if (parent)
        delete parent;

    //ptr.reset(nullptr);
}
 //*/



double NormalisePI(double d);
Tsc ComputePosition(Tsc current, const Velocidad newVel, const double interval){

    Tsc loc = current;

    if (std::abs(newVel.w) < 1e-5){
        loc.x = loc.x + newVel.v*std::cos(loc.tita)*interval;
        loc.y = loc.y + newVel.v*std::sin(loc.tita)*interval;
    }
    else{	//w != 0
        //Differential robot
        loc.x = loc.x - (newVel.v/newVel.w)*std::sin(loc.tita) + (newVel.v/newVel.w)*std::sin(loc.tita+newVel.w*interval);
        loc.y = loc.y + (newVel.v/newVel.w)*std::cos(loc.tita) - (newVel.v/newVel.w)*std::cos(loc.tita+newVel.w*interval);
        loc.tita = NormalisePI(loc.tita + newVel.w * interval);
    }

    return loc;
}


void Astar::GetNeighbours(const Node current, std::list<Node>& neighbours, bool (Astar::* function)(Key s)) {
//Compute the adjacents nodes of the current one, taking into account the dynamic constraints of the agent
//Check if the candidate state lies inside the VS bounds

    unsigned k = current.key.k;
    std::shared_ptr<Node> parent = std::make_shared<Node>(current);
    for (int dw=-1; dw <= 1; dw++)
        for (int dv=-1; dv <= 1; dv++) {
            if (dv != 0 && dw != 0)
                continue;

            int dt = 1;

            if (k+dt >= space.GetSizeVTS())
                return;

            keyWV newKeyVel = std::make_pair(current.key.i + dw, current.key.j + dv);
            Key newKey = std::make_pair(newKeyVel,k+dt);

            auto &layer = space.GetLayer(k+dt); //auto &layer = space.GetVTSpace().data3d[k+dt].mapVW;
            const auto search = layer.mapVW.find(newKeyVel);  //returns an iterator to the element if found, otherwise it returns an iterator to map::end.

            if (search != layer.mapVW.end() && (this->*function)(newKey)) {
                State s;
                double cost = search->second.currentCost;
                Node newNode(s, parent, newKey.i, newKey.j, newKey.k, 0, 0, 0, cost);
                //if (search->second.currentCost > 0)
                    //std::cout << "celda ocupada" << std::endl;
                neighbours.push_front(newNode);
            }
        }
}

double distanceVS(const Key from, const Key to){
//Distance to the goal state
    return std::sqrt(std::pow((double)from.i - (double)to.i, 2) + std::pow((double)from.j - (double)to.j, 2));
}

bool Astar::distToObsVS(const Key key, double& time) {
//steps/time during which a velocity can be maintained (operation performed on the VS)
//returns if there is roof (a cell occupied by an obstacle) above the cell

    unsigned i;
    for (i = key.k + 1; i < (unsigned) space.GetVTSpace().data3d.size(); i++) { //time layers
        layerVW input = space.GetVTSpace().data3d[i];
        if (!input.mapVW.empty()) {
            keyWV kWV = std::make_pair(key.i, key.j);
            if (input.mapVW.find(kWV) == input.mapVW.end()) //the element does not exist, cell is free in the layer
                continue;
            else    //velocity cell is occupied at layer time i
                break;
        }
    }

    time = 0;
    if (i == (unsigned) space.GetVTSpace().data3d.size()){

        time = ((unsigned) space.GetVTSpace().data3d.size() - key.k - 1) * time_step;
        return false;

    }else{
        time = (i - key.k - 1)*time_step;

        //TODO: it should be considered the time needed to reach a safe velocity
        //TODO: Buscar la velocidad más próxima para la cuál el obstáculo no está encima, i.e, es libre durante el horizonte
        return true;
    }
}

double Astar::NumCells(const Key key){
//it returns the number of cells/steps until a safe one, i.e, until a cell without roof of velocity
//if all the cells lie below the velocity roof, then it returns 0

    std::priority_queue<double, std::vector<double>, std::greater<double>> cellSteps;
    double time = 0;
    for (unsigned j = 0; j<space.sizev; j++){
        for (unsigned i = 0; i<space.sizew; i++){
            keyWV key_ = std::make_pair(i,j);
            Key newKey(std::make_pair(key_, key.k));
            if (!distToObsVS(newKey, time)){
                double numSteps = std::abs((double)newKey.i - (double)key.i) + std::abs((double)newKey.j - (double)key.j);
                cellSteps.push(numSteps);
            }
        }
    }
    if (!cellSteps.empty())
        return cellSteps.top();
    else
        return 0;
}

bool Astar::ComputePlan2(const State start, std::list<State> &goal, std::list<State> &plan, bool (Astar::* function)(Key s)) {

    std::list<Node> open;
    std::map<Key, Node, Key> explored;

    State sGoal = goal.front();

    std::pair<keyWV, unsigned> key;
    space.GetKey(sGoal.vel, sGoal.t, key);

    State s = start;
    std::pair<keyWV, unsigned> startKey;
    space.GetKey(s.vel, s.t, startKey);
    Node start_node(s, NULL, startKey.first.first, startKey.first.second, startKey.second, 0, 0, 0);

    Key goal_key(key);
    Key start_key(startKey);
    Key parentGoal;
    State finalGoal = goal.front(); //when we do not want to associate time to the goal, so that it returns the time as soon as the goal velocity is reached

    goal.pop_front();

    double nodesGenerated = 0;
    double nodesExplored = 0;

    open.push_back(start_node);  //Initialize the open list

    Key zero = Key(std::make_pair(space.GetKeyWV({0, 0}), 0));
    Line lGoal({zero.i, zero.j}, {goal_key.i, goal_key.j});
    bool orientate = false;
    double dmax = 1;    //dw; //Distancia(dw, dv);
    if (start_key.i >= zero.i && start_key.i <= goal_key.i || start_key.j >= zero.j && start_key.j <= goal_key.j){
        double d = lGoal.Distance({start_key.i, start_key.j});
        orientate = d > dmax; //dw; //Distancia(dw, dv);
    }else{
        //Compute distance from current node to the extreme points of the segment
        double distP1 = distanceVS(start_key, zero);
        double distP2 = distanceVS(start_key, goal_key);
        double d = (distP1 < distP2 ? distP1 : distP2);
        orientate = d > dmax;
    }

    //if (!orientate) std::cout << "no orient" << std::endl;

    while (!open.empty()){

        Node current = open.front();
        open.pop_front();

        std::pair<keyWV, unsigned> key = std::make_pair(std::make_pair(current.key.i, current.key.j), current.key.k);

        //if (current.key.operator==(goal_key)){    //to consider the velocity and time as requirement for reaching the goal
        if (current.key.i == goal_key.i && current.key.j == goal_key.j && current.key.k == space.GetVTSpace().data3d.size()-1){    //to consider only the velocity as requirement for reaching the goal

            if (goal_key.k != current.key.k){   //the goal velocity has been reached at a different time
                goal_key.k = current.key.k;
                //finalGoal.t = current.state.t;  //update the time at which the goal is reached
            }

            //check if the first node is already the goal; i.e, the start and goal nodes are the same
            if (current.ptr)
                parentGoal = current.ptr->key;
            else
                parentGoal = goal_key;

            totalCost = current.g;

            break;
        }

        explored.insert({current.key, current});
        nodesExplored++;

        std::list<Node> neighbours;
        GetNeighbours(current, neighbours, function);
        for (std::list<Node>::iterator it = neighbours.begin(); it != neighbours.end(); ++it){

            Node neighbourNode = (*it);

            double time = 0; bool roof = distToObsVS(neighbourNode.key, time);

            //Compute the cost to reach the cell: cost to reach the parent + the cost from parent to cell
            layerVW layer = space.GetVTSpace().data3d[neighbourNode.key.k];
            keyWV keyVel = std::make_pair(neighbourNode.key.i, neighbourNode.key.j);
            auto search = layer.mapVW.find(keyVel);
            if (search != layer.mapVW.end()){

                double loss = 0;
                if (neighbourNode.key.SameVel(goal_key))
                    loss = 0;   // already at goal
                else{
                    if (current.key.SameVel(neighbourNode.key)) {
                        loss     = 0.99;
                    }
                    else {

                        //TODO: probar como heurística el producto vectorial entre el vector que va between the start to goal vector and the current point to goal vector
                        //Added to consider the angular distance for the cost of the cell
                        const double dw = space.sizew;
                        double stepw = std::abs(space.GetBounds().wmax_right - space.GetBounds().wmax_left) / space.sizew;

                        double current_distGoalLine = 0;
                        Key zero = Key(std::make_pair(space.GetKeyWV({0, 0}), 0));
                        Line lGoal({zero.i, zero.j}, {goal_key.i, goal_key.j});
                        if (current.key.i >= zero.i && current.key.i <= goal_key.i || current.key.j >= zero.j && current.key.j <= goal_key.j){
                            current_distGoalLine = lGoal.Distance({current.key.i, current.key.j});
                            //orientate = current_distGoalLine > 1; //dw; //Distancia(dw, dv);
                        }else{
                            //Compute distance from current node to the extreme points of the segment
                            double distP1 = distanceVS(current.key, zero);
                            double distP2 = distanceVS(current.key, goal_key);
                            current_distGoalLine = (distP1 < distP2 ? distP1 : distP2);
                        }

                        double neighbour_distGoalLine = 0;
                        if (neighbourNode.key.i >= zero.i && neighbourNode.key.i <= goal_key.i || neighbourNode.key.j >= zero.j && neighbourNode.key.j <= goal_key.j){
                            neighbour_distGoalLine = lGoal.Distance({neighbourNode.key.i, neighbourNode.key.j});
                        }else{
                            //Compute distance from current node to the extreme points of the segment
                            double distP1 = distanceVS(neighbourNode.key, zero);
                            double distP2 = distanceVS(neighbourNode.key, goal_key);
                            neighbour_distGoalLine = (distP1 < distP2 ? distP1 : distP2);
                        }

                        double alfa = 1.0; double beta = 1.0;
                        if (orientate)
                            alfa = 0.15;
                        else
                            beta = 0.25;

                        //Cost to reach the velocity cell
                        double current_distVS = distanceVS(current.key, goal_key);
                        double neighbour_distVS = distanceVS(neighbourNode.key, goal_key);
                        loss = alfa*(1 - (current_distVS - neighbour_distVS)) + beta*(1 - (current_distGoalLine - neighbour_distGoalLine));

                        //Cost to reach the velocity cell
                        //double current_distVS = distanceVS(current.key, goal_key);
                        //double neighbour_distVS = distanceVS(neighbourNode.key, goal_key);
                        //loss = 1 - (current_distVS - neighbour_distVS);
                    }
                }

                neighbourNode.g = current.g
                                  + search->second.currentCost  //cost of the cell in the map (the node belongs to a cell in the map)
                                  + 1
                                  + loss/10;
                //+ distAngular/10;
                //+ cost of future occupied cells
                //+ cost of effort for doing a manoeuvre: avoiding manoeuvre, keep in a non-collision course
            }

            neighbourNode.h = std::abs((double)goal_key.k - neighbourNode.key.k);

            neighbourNode.f = neighbourNode.g + neighbourNode.h;

            std::list<Node>::iterator itNode = std::find(open.begin(), open.end(), neighbourNode);
            if (itNode != open.end() && neighbourNode.g < (*itNode).g){  //it is already in the open list
                //Replace the node with the node with fewer cost in the open list
                open.remove(*itNode);
                open.push_back(neighbourNode);
            } else if (itNode == open.end() && explored.find(neighbourNode.key) == explored.end()){
                //it has not been explored yet
                open.push_back(neighbourNode);
            }

            nodesGenerated++;
        }
        //sort the open list in ascending cost order
        open.sort([](const Node &l, const Node &r) { return (l.f < r.f || (l.f==r.f && l.g<r.g));});
    }

    //std::cout << "START key: " << start_key.i << ", " << start_key.j << ", " << start_key.k << std::endl;
    //std::cout << "GOAL key: " << goal_key.i << ", " << goal_key.j << ", " << goal_key.k << std::endl;

    //The plan is computed
    Key parentKey = parentGoal;
    while (!(parentKey == start_key)){

        std::map<Key, Node, Key>::iterator it = explored.find(parentKey);
        if (it != explored.end()){
            keyWV keyVel = std::make_pair(it->second.key.i, it->second.key.j);
            State s(space.GetVelocity(keyVel), space.GetTime(it->second.key.k)); //Tsc pos; State s(space.GetVelocity(keyVel), space.GetTime(it->second.key.k), pos);
            plan.push_back(s);
            //std::cout << "key: " << keyVel.first << ", " << keyVel.second << ", " << it->second.key.k << std::endl;
            //std::cout << "Data: " << s.vel.v << ", " << s.vel.w << ", " << s.t << std::endl;

            parentKey = it->second.ptr->key;
            //it->second.show();
        }else
            return false;
    }
    plan.push_front(finalGoal);

    return true;
}




//bool Astar::ComputePlan(const State start1, State goal1, int action, std::list<State> &plan, bool (Astar::* function)(Key s)) {
bool Astar::ComputePlan(std::list<Velocidad> &plan) {

    State start(space.GetAgent(), 0.0); //State start(space.GetAgent(), 0.0, posR);
    State goal(followGoal, followTime); //State goal(followGoal, followTime, Tsc(posG.x,posG.y,0));

    std::list<Node> open;
    std::map<Key, Node, Key> explored;

    State sGoal = goal;
    std::pair<keyWV, unsigned> key;
    space.GetKey(sGoal.vel, sGoal.t, key);

    State s = start;
    std::pair<keyWV, unsigned> startKey;
    space.GetKey(s.vel, s.t, startKey);
    Node start_node(s, NULL, startKey.first.first, startKey.first.second, startKey.second, 0, 0, 0);

    Key goal_key(key);
    Key start_key(startKey);
    Key parentGoal;
    State finalGoal = goal; //when we do not want to associate time to the goal, so that it returns the time as soon as the goal velocity is reached

    double nodesGenerated = 0;
    double nodesExplored = 0;

    open.push_back(start_node);  //Initialize the open list

    double totalTime = 0.0;
    while (!open.empty()){

        //clock_t tstart = clock();
        //clock_t tfinish = clock();
        //tstart = clock();
        //tfinish = clock();
        //totalTime += ((double)(tfinish-tstart))/CLOCKS_PER_SEC;
        //fdataI << ((double)(tfinish-tstart))/CLOCKS_PER_SEC << std::endl;


        Node current = open.front();
        open.pop_front();

        std::pair<keyWV, unsigned> key = std::make_pair(std::make_pair(current.key.i, current.key.j), current.key.k);

        //if (current.key.operator==(goal_key)){    //to consider the velocity and time as requirement for reaching the goal
       if (current.key.i == goal_key.i && current.key.j == goal_key.j){ // && current.key.k == space.GetSizeVTS()-1){    //to consider only the velocity as requirement for reaching the goal
       //if (action == 0 && current.key.i == goal_key.i && current.key.j == goal_key.j || current.key.operator==(goal_key)){    //to consider only the velocity as requirement for reaching the goal

            if (goal_key.k != current.key.k){   //the goal velocity has been reached at a different time
                goal_key.k = current.key.k;
                //finalGoal.t = current.state.t;  //update the time at which the goal is reached
            }

            //check if the first node is already the goal; i.e, the start and goal nodes are the same
            if (current.ptr)
                parentGoal = current.ptr->key;
            else
                parentGoal = goal_key;

            totalCost = current.g;

            break;
        }

        explored.insert({current.key, current});

        nodesExplored++;

        std::list<Node> neighbours, neighToAdd;
        GetNeighbours(current, neighbours, &Astar::expand);
        for (std::list<Node>::iterator it = neighbours.begin(); it != neighbours.end(); ++it) {

            Node neighbourNode = (*it);

            double loss = 0;
            if (neighbourNode.key.SameVel(goal_key))
                loss = 0;   // already at goal
            else {
                if (current.key.SameVel(neighbourNode.key)) {
                    loss = 0.99;
                } else {
                    //Cost to reach the velocity cell
                    double current_distVS = distanceVS(current.key, goal_key);
                    double neighbour_distVS = distanceVS(neighbourNode.key, goal_key);
                    loss = 1 - (current_distVS - neighbour_distVS);
                }
            }

            neighbourNode.g = current.g
                              +
                              neighbourNode.cellCost  //cost of the cell in the map (the node belongs to a cell in the map)
                              //+ search->second.currentCost
                              + 1
                              + loss / 10
                              ;
            //+ distAngular/10;
            //+ cost of future occupied cells
            //+ cost of effort for doing a manoeuvre: avoiding manoeuvre, keep in a non-collision course
            //}

            neighbourNode.h = std::abs((double) goal_key.k - (double) neighbourNode.key.k);

            neighbourNode.f = neighbourNode.g + neighbourNode.h;

            std::list<Node>::iterator itNode = std::find(open.begin(), open.end(), neighbourNode);
            if (itNode != open.end() && neighbourNode.g < (*itNode).g){  //it is already in the open list
                //Replace the node with the node with fewer cost in the open list
                open.remove(*itNode);
                neighToAdd.push_back(neighbourNode);
                //open.push_back(neighbourNode);
            } else if (itNode == open.end() && explored.find(neighbourNode.key) == explored.end()){
                //it has not been explored yet
                //open.push_back(neighbourNode);
                neighToAdd.push_back(neighbourNode);
                nodesGenerated++;
            }
        }
        //sort the open list in ascending cost order
            //open.sort([](const Node &l, const Node &r) { return (l.f < r.f || (l.f==r.f && l.h<r.h)); });
        ///*
        if (!neighToAdd.empty()){
            neighToAdd.sort();
            open.merge(neighToAdd);
        }//*/
    }

    std::ofstream fdata;
    fdata.open("pathAstar.dat", std::ios::trunc);

    //The plan is computed
    Key parentKey = parentGoal;
    while (!(parentKey == start_key)){

        std::map<Key, Node, Key>::iterator it = explored.find(parentKey);
        if (it != explored.end()){
            keyWV keyVel = std::make_pair(it->second.key.i, it->second.key.j);
            State s(space.GetVelocity(keyVel), space.GetTime(it->second.key.k)); //Tsc pos; State s(space.GetVelocity(keyVel), space.GetTime(it->second.key.k), pos);
            plan.push_back(s.vel);
            //plan.push_back(s);

            fdata << "\n" << it->second.key.i << "\t" << it->second.key.j << "\t" << it->second.key.k << std::endl;
            //fdata.clear();
            fdata.seekp(0);
            parentKey = it->second.ptr->key;
            //it->second.show();
        }else
            return false;
    }
    //plan.push_front(finalGoal);
    plan.push_front(followGoal);

    fdata.close();

    plan.reverse();

    return true;
}

bool Astar::expand(const Key key){
//Function to determine the condition for expanding a node

    return true;    //all the candidate cells are expanded
    return space.IsCellFree(std::make_pair(std::make_pair(key.i, key.j), key.k));   //expansion only of the free cells
}

bool Astar::MakePlan(Tsc posR, Tpf posG, std::list<std::pair<Comando, int>> velTarget, std::list<Velocidad> &plan, double& cost) {

    bool computed = true;

    State start(space.GetAgent(), 0.0); //State start(space.GetAgent(), 0.0, posR);
    State goal(followGoal, followTime); //State goal(followGoal, followTime, Tsc(posG.x,posG.y,0));

    std::ofstream fdata; //("pathAstar.dat", std::ios::trunc);  //file to log data

    //while (!velTarget.empty()){

        //std::list<State> statePlan;
        std::list<Velocidad> statePlan;

        //State goal(velTarget.front().first.vel, velTarget.front().first.t);
        int action = velTarget.front().second;
        velTarget.pop_front();

        std::cout << "START: " << start.vel.w << ", " << start.vel.v << ", " << start.t << std::endl;
        std::cout << "GOAL: " << goal.vel.w << ", " << goal.vel.v << ", " << goal.t << std::endl;

        //TODO: COMPUTE HERE DIRECTLY THE ASTAR, TO AVOID THE CALLING OF THE COMPUTEPLAN FUNCTION AND THE PARAMETER PASSING, WHICH TAKES UNNECESSARY EXTRA TIME
        //computed = computed && ComputePlan(start, goal, action, statePlan, &Astar::expand);
        computed = computed && ComputePlan(statePlan);

        cost = totalCost;

        fdata.open("pathAstar.dat", std::ios::trunc);
        //Translate the computed plan in a sequence of velocities
        statePlan.reverse();
        while (!statePlan.empty()){
            /*
            State s = statePlan.front();
            statePlan.pop_front();
            plan.push_back(s.vel);

            //Log data
            std::pair<keyWV, unsigned> key;
            space.GetKey(s.vel, s.t, key);

            //fdata << s.vel.w << "\t" << s.vel.v << "\t" << s.t << std::endl;
            fdata << key.first.first << "\t" << key.first.second << "\t" << key.second << std::endl;
            */
            Velocidad vel = statePlan.front();
            statePlan.pop_front();
            plan.push_back(vel);
        }
        fdata << std::endl;
        fdata.close();

        if (computed) start = State(goal);
    //}

    //fdata.close();

    return computed;
}

void Astar::ComputeFollowGoal(Tpf goal){
    //Decide the goal to be reached

    double min_dist = 2;
    double dist = std::sqrt(std::pow(goal.x, 2) + std::pow(goal.y, 2));

    if (space.GetBounds().IsInsideBounds(space.GetGoal().velGoal)){
        followGoal = space.GetGoal().velGoal;
    }else{
        if (dist < min_dist)
            followGoal = space.GetGoal().commandGoal;
        else
            followGoal = space.GetGoal().dirGoal;
    }
    followTime = space.GetTh();
    //CheckVelocityGoal(followGoal, robot, goal); //Tsc robot
}

void Astar::SetFollowGoal(Velocidad gF){
    followGoal = gF;
    followTime = space.GetTh();
}

Velocidad Astar::GetFollowGoal() {
    return followGoal;
}

double Astar::GetFollowTime() {
    return followTime;
}
