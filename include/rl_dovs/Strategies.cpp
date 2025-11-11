//
// Created by maite14 on 7/12/17.
//

#include "Strategies.h"
#include <iostream>
using namespace std;

double dist_medium2 = 7*7;
double dist_near2 = 5*5;

std::vector<Command> commands;
//Velocidad right, left, up, down;
bool estrategia_valle;

enum Movimiento {AR, AB, IZ, DE, EQ};
struct DW_Info{
    Velocidad vel;
    bool preferred, inCollision, futureCollision;
    double titaGoal, distGoal; // with respect to the position of the robot after applying the velocity and then stopping
    double titaGoal_next, distGoal_next; // with respect to next position of the robot (in workspace)
    double titaVS, distVS; // with respect to the goal (the command (v,w) that would lead the robot to the goal if applied at next time stept) in the VS
};
std::vector<DW_Info> dw;

/*
// l < r <=> r > l
// l == r <=> !(l<r) && !(l>r)
bool operator == (const Velocidad& l, const Velocidad& r) {	//Ascending order by angle
    return (l.v == r.v && l.w == r.w);
}

// l < r <=> r >
// l == r <=> !(l<r) && !(l>r)
bool operator < (const Velocidad& l, const Velocidad& r) {	//Ascending order by angle
    Velocidad left = l; Velocidad right = r;
    Tpfp pl, pr;
    car2pol_vel(&left,&pl); car2pol_vel(&right, &pr);
    return (std::abs(pl.t) < std::abs(pr.t)) ||
           ((pl.t == pr.t) && (pl.r < pr.r));
}
*/

// andrew
std::ofstream fstrategies("logs/strategies.txt");

// andrew: make it generic for limits!
float Strategies::maxAngularVelocity(float linearVelocity) {
    return (linearVelocity-1.5)/-1.5;
}

// TODO: change to appropiate place
// andrew
Velocidad calculateVelocities(Velocidad dir_goal , DW dwAg) {
    Velocidad vel;
    if (dir_goal.w > dwAg.equal.w) {
        vel = Velocidad((dwAg.right.v-dwAg.down.v)/2+dwAg.down.v, abs(dwAg.right.w-dwAg.down.w)/2+dwAg.down.w);
        // std::cout << "(" << dwAg.right.v << " - " << dwAg.down.v << ")/3 + " << dwAg.down.v << " = " << vel.v << std::endl;
        // std::cout << "abs(" << dwAg.right.w << "-" << dwAg.down.w << ")/3 + " << dwAg.down.w << " = " << vel.w<< std::endl;
    } else {
        vel = Velocidad((dwAg.left.v - dwAg.down.v)/2+dwAg.down.v, dwAg.down.w - abs(dwAg.down.w-dwAg.left.w)/2);
        // std::cout << "(" << dwAg.left.v << " - " << dwAg.down.v << ")/3 + " << dwAg.down.v << " = " << vel.v << std::endl;
        // std::cout << dwAg.down.w << " - abs(" << dwAg.down.w << "-" << dwAg.left.w << ")/3 = " << vel.w << std::endl;
    }
    return vel;
}

void Strategies::SetHeuristicValues(double wsteer, double al, double ar, double wz2, double waling, double valley, double goal_valley, double vorient, double worient){

    wmax_steering = wsteer;
    
    ang_left = al;
    ang_right = ar;
    wlim_z2 = wz2;
    
    valley_depth = valley;
    cte_goal_valley = goal_valley;
    
    vorientacion = vorient;
    worientacion = worient;
    
    waling_lim = waling;
}

void Strategies::ComputeSpaces(){
//This function classifies the hollows of free velocities: hollows outside DOV (space_outDOV),
//hollows inside DOV which are not valleys (space_inDOV) and hollows inside DOV which are valleys (valles_z2)

    boundsVS bounds = space.GetBounds();

    // cout << "Zonas libres: " << zonaLibres.size() << endl;
    for (int i=0; i<(int)zonaLibres.size(); i++){
        // std::cout << "Free zone" << std::endl;
        ParAng z = zonaLibres[i];
        ParVel v = velLibres[i];

        bool inside = false;
        for (int k=0; k<(int)limites_DOV.size(); k++){

            std::pair<Command, Command> dov = limites_DOV[k];

            double ang_ini = atan2(dov.first.sup.vel.v, dov.first.sup.vel.w);
            double ang_fin = atan2(dov.second.sup.vel.v, dov.second.sup.vel.w);

            //Check if the free zone lies inside of any of the bounds of a DOV
            if (z.ini >= ang_ini && z.fin <= ang_fin){

                inside = true;
                break;

            }
        }

        Space sp;
        if (!inside){
            //The free velocity zone lies outside the bounds of the DOV
            sp.ang = z;
            sp.vel = v;
            sp.width = z.fin - z.ini;
            sp.depth = Velocidad(0,0);	//The whole range is completely free

            space_outDOV.push_back(sp);

            /*
            //Comprobamos si hay algún valle en el hueco para actualizar los indices del valle
            int i=0;
            if (!valles_z2.empty()){

                while (i<(int)valles_z2.size()){

                    infoValle valle = valles_z2[i];

                    if (atan2(commands[valle.ini].sup.vel.v, commands[valle.ini].sup.vel.w) >= z.ini &&
                        atan2(commands[valle.fin].sup.vel.v, commands[valle.fin].sup.vel.w) <= z.fin){

                        //La zona libre contiene un valle, buscamos el DOV en el que se encuentra el valle
                        //y modificamos los indices del valle para que coincidan con los del hueco y límites del DOV
                        for (int k=0; k<(int)limites_DOV.size(); k++){

                            std::pair<Command, Command> dov = limites_DOV[k];

                            double ang_ini = atan2(dov.first.sup.vel.v, dov.first.sup.vel.w);
                            double ang_fin = atan2(dov.second.sup.vel.v, dov.second.sup.vel.w);

                            //Comprobamos si la zona libre está dentro de los límites de algún DOV
                            if (atan2(commands[valle.ini].sup.vel.v, commands[valle.ini].sup.vel.w) >= ang_ini &&
                                    atan2(commands[valle.fin].sup.vel.v, commands[valle.fin].sup.vel.w) <= ang_fin){

                                //Modificamos el índice del inicio del valle
                                if (std::abs(z.ini - atan2(commands[valle.ini].sup.vel.v, commands[valle.ini].sup.vel.w)) <
                                        std::abs(ang_ini - atan2(commands[valle.ini].sup.vel.v, commands[valle.ini].sup.vel.w))){

                                    for (int j=0; j<(int)commands.size(); j++){
                                        if (atan2(commands[j].sup.vel.v, commands[j].sup.vel.w) == z.ini){
                                            valle.ini = j;
                                            break;
                                        }
                                    }
                                }else{

                                    for (int j=0; j<(int)commands.size(); j++){
                                        if (atan2(commands[j].sup.vel.v, commands[j].sup.vel.w) == ang_ini){
                                            valle.ini = j;
                                            break;
                                        }
                                    }
                                }

                                //Modificamos el ínidice del fin del valle
                                if (std::abs(z.fin - atan2(commands[valle.fin].sup.vel.v, commands[valle.fin].sup.vel.w)) <
                                        std::abs(ang_fin - atan2(commands[valle.fin].sup.vel.v, commands[valle.fin].sup.vel.w))){

                                    for (int j=0; j<(int)commands.size(); j++){
                                        if (atan2(commands[j].sup.vel.v, commands[j].sup.vel.w) == z.fin){
                                            valle.fin = j;
                                            break;
                                        }
                                    }
                                }else{

                                    for (int j=0; j<(int)commands.size(); j++){
                                        if (atan2(commands[j].sup.vel.v, commands[j].sup.vel.w) == ang_fin){
                                            valle.fin = j;
                                            break;
                                        }
                                    }
                                }

                                valles_z2[i] = valle;
                                break;

                            }
                        }
                        //break;
                    }
                    i++;
                }
            }
            //*/

        }else{
            //Separamos los huecos libres que corresponden a valles
            int i=0;
            if (!valles_z2.empty()){

                while (i<(int)valles_z2.size()){

                    infoValle valle = valles_z2[i];

                    if (atan2(commands[valle.ini].sup.vel.v, commands[valle.ini].sup.vel.w) >= z.ini &&
                        atan2(commands[valle.fin].sup.vel.v, commands[valle.fin].sup.vel.w) <= z.fin){

                        //La zona libre contiene un valle, no la almacenamos
                        //Modificamos los indices del valle para que coincidan con los del hueco dentro del DOV
                        for (int j=0; j<(int)commands.size(); j++){
                            if (atan2(commands[j].sup.vel.v, commands[j].sup.vel.w) == z.ini) valle.ini = j;

                            if (atan2(commands[j].sup.vel.v, commands[j].sup.vel.w) == z.fin){
                                valle.fin = j;
                                break;
                            }
                        }
                        valles_z2[i] = valle;
                        break;
                    }
                    i++;
                }
            }

            if (i == (int)valles_z2.size()){

                //La zona de velocidades libre dentro de los límites del DOV no contiene ningún valle
                sp.ang = z;
                sp.vel = v;
                sp.width = z.fin - z.ini;

                //Buscamos la profundidad del hueco (minima velocidad que debería llevar el robot para alcanzar el hueco)
                Velocidad minVel = Velocidad(bounds.vlim_max,0);
                for (int j=0; j<(int)commands.size(); j++){
                    if (atan2(commands[j].sup.vel.v, commands[j].sup.vel.w) >= z.ini &&
                        atan2(commands[j].sup.vel.v, commands[j].sup.vel.w) <= z.fin){

                        if (commands[j].sup.vel.v < minVel.v) minVel = commands[j].sup.vel;

                    }
                }
                sp.depth = minVel;

                space_inDOV.push_back(sp);
            }
        }
    }
}

bool PointInInterval(Velocidad raiz, Velocidad ini, Velocidad fin){
//Check if a point lies in between interval [ini, fin]

    double epsilon = 1e-5;

    return ((raiz.w >= ini.w - epsilon && raiz.w <= fin.w + epsilon) || (raiz.w >= fin.w - epsilon && raiz.w <= ini.w + epsilon)) &&
           ((raiz.v >= ini.v - epsilon && raiz.v <= fin.v + epsilon) || (raiz.v >= fin.v - epsilon && raiz.v <= ini.v + epsilon));
}

bool IntersectionPoint(Velocidad s1_ini, Velocidad s1_fin, Velocidad s2_ini, Velocidad s2_fin, std::vector<Velocidad> &points){
//Given two segments, the function returns true o false if they intersect

    bool found = false;

    //Compute two lines from the points
    Tpf p1(s1_ini.w, s1_ini.v);
    Tpf p2(s1_fin.w, s1_fin.v);
    Line r1 = Line(p1, p2);

    p1 = Tpf(s2_ini.w, s2_ini.v);
    p2 = Tpf(s2_fin.w, s2_fin.v);
    Line r2 = Line(p1, p2);

    Tsc raiz; int sol;
    SolDosRectas(r1, r2, raiz, sol);

    //Comprobamos si el punto de intersección entre las rectas está entre los límites de los segmentos
    if (sol==0){	// son paralelas
        //Comprobar si los segmentos se superponen
        found = true;
        if (PointInInterval(s1_ini, s2_ini, s2_fin) && PointInInterval(s1_fin, s2_ini, s2_fin)){
            points.push_back(s1_ini);
            points.push_back(s1_fin);
        }else if (PointInInterval(s2_ini, s1_ini, s1_fin) && PointInInterval(s2_fin, s1_ini, s1_fin)){
            points.push_back(s2_ini);
            points.push_back(s2_fin);
        }else if (PointInInterval(s1_ini, s2_ini, s2_fin) && !PointInInterval(s1_fin, s2_ini, s2_fin)){
            if (PointInInterval(s2_ini, s1_ini, s1_fin) && !PointInInterval(s2_fin, s1_ini, s1_fin)){
                points.push_back(s2_ini);
                points.push_back(s1_ini);
            }else if (!PointInInterval(s2_ini, s1_ini, s1_fin) && PointInInterval(s2_fin, s1_ini, s1_fin)){
                points.push_back(s2_fin);
                points.push_back(s1_ini);
            }
        }else if (!PointInInterval(s1_ini, s2_ini, s2_fin) && PointInInterval(s1_fin, s2_ini, s2_fin)){
            if (PointInInterval(s2_ini, s1_ini, s1_fin) && !PointInInterval(s2_fin, s1_ini, s1_fin)){
                points.push_back(s2_ini);
                points.push_back(s1_fin);
            }else if (!PointInInterval(s2_ini, s1_ini, s1_fin) && PointInInterval(s2_fin, s1_ini, s1_fin)){
                points.push_back(s2_fin);
                points.push_back(s1_fin);
            }
        }else{
            found = false;
        }
    }else{
        //comprobar que el punto de interseccion esté entre los límites de los dos segmentos
        Velocidad point;
        point.w = raiz.x; point.v = raiz.y;
        found = PointInInterval(point, s1_ini, s1_fin) && PointInInterval(point, s2_ini, s2_fin);
        if (found) points.push_back(point);
    }

    return found;
}

double NormalisePI(double d);
bool Strategies::IsSafeVelocity(Velocidad vel){

    //Predict the final position of the robot and obstacles
    Tsc position = posAg;
    double velV = vel.v;
    int n = 1; //number of times in advanced until the robot stops

    while (true){
        if (std::abs(vel.w) < 1e-5){ //0.00001 && w > -0.00001){
            position.x = position.x + velV*cos(position.tita)*stept;
            position.y = position.y + velV*sin(position.tita)*stept;
        }
        else{	//w != 0
            position.x = position.x - (velV/vel.w)*sin(position.tita) + (velV/vel.w)*sin(position.tita+vel.w*stept);
            position.y = position.y + (velV/vel.w)*cos(position.tita) - (velV/vel.w)*cos(position.tita+vel.w*stept);
            position.tita = NormalisePI(position.tita + vel.w*stept);
            NormalisePI(position.tita);
        }

        if (velV > 0){
            velV = velV - space.GetConstraints().av*stept; n++;
        }
        else break;
    }

    if (agents != NULL){
        std::vector<std::unique_ptr<Agent>>::iterator it;
        for (it = agents->begin(); it != agents->end(); ++it){
            if ((*it)->GetId() != idAg){
                Tsc posObs = (*it)->GetLocalization();
                double v = (*it)->GetV(); double w = (*it)->GetW();
                if (v > 0){
                    for (int j=0; j<n; j++){
                        if (std::abs(w) < 1e-5){
                            posObs.x = posObs.x + v*cos(posObs.tita)*stept;
                            posObs.y = posObs.y + v*sin(posObs.tita)*stept;
                        }
                        else{	//w != 0
                            posObs.x = posObs.x - (v/w)*sin(posObs.tita) + (v/w)*sin(posObs.tita+w*stept);
                            posObs.y = posObs.y + (v/w)*cos(posObs.tita) - (v/w)*cos(posObs.tita+w*stept);
                            posObs.tita = NormalisePI(posObs.tita + vel.w*stept);
                        }
                    }
                    //Check if the robot is in collision with the obstacle
                    // andrew: aumentado rango
                    // double radio_seg = (*it)->GetRealRadius() + (*it)->GetRealRadius()+ (*it)->GetRealRadius();
                    double radio_seg = (*it)->GetRealRadius() + (*it)->GetRealRadius();
                    Tpf sol;
                    // nueva posicion calculada del agente y no posAg
                    //transfor_inversa_p(posObs.x, posObs.y, &posAg, &sol);
                    transfor_inversa_p(posObs.x, posObs.y, &position, &sol);
                    double distancia = Distancia(sol.x, sol.y);

                    if (distancia - radio_seg <= 1e-5){
                        return false;
                    }
                    // float theta = posAg.tita;
                    // std::vector<double> currentDirection {cos(theta), sin(theta)};
                    // std::vector<double> obsDirection {(*it)->GetLocalization().x - posAg.x,(*it)->GetLocalization().y - posAg.y};
                    // float dot = currentDirection[0] * obsDirection[0] + currentDirection[1]*obsDirection[1];   
                    // float det = currentDirection[0]*obsDirection[1] - currentDirection[1]*obsDirection[0];    
                    // float angle = atan2(det, dot); 

                    // // if not behind, check this
                    // if ( !(angle > 3.1415/2+0.2 || angle < -3.1415/2-0.2) ) {
                        
                    //     // andrew: mirar a ver como estan ahora tambien
                    //     transfor_inversa_p(posObs.x, posObs.y, &posAg, &sol);
                    //     distancia = Distancia(sol.x, sol.y);
                    //     if (distancia - radio_seg <= 1e-5){
                    //         return false;
                    //     }
                    //     transfor_inversa_p((*it)->GetLocalization().x, (*it)->GetLocalization().y, &position, &sol);
                    //     distancia = Distancia(sol.x, sol.y);
                    //     if (distancia - radio_seg <= 1e-5){
                    //         return false;
                    //     }
                    // }
                }
            }
        }
    }

    return true;
}

bool Strategies::MaxFreeVelocityInRange(DW_Range points, Velocidad &best_vel){
//Función que devuelve la máxima velocidad dentro del rango que es libre de colisión
//Devuelve true si ha encontrado una velocidad; si no, devuelve false
//Pre: el polígono de puntos ya construido a partir de los DOV, y points.num > 0
//points: contains the pair of velocities which define the interval to look for the maximum command to apply; or just a velocity (in the border)
//points[0]: the upper velocity; points[1]: is the lower velocity

    boundsVS bounds = space.GetBounds();
    //Velocidad dir_goal = space.GetGoal().dirGoal;

    bool free_obs = false;

    if (points.num > 0){

        if (points.num == 1){
            best_vel = points.ini;
            free_obs = !pol.InsidePolygon(best_vel);
        }else{
            //points contains a pair of velocities

            std::vector<DW_Range> free_range;
            free_range.push_back(points);	// initially, the whole range in DW is assumed as free

            Velocidad vel_ini = points.ini;
            Velocidad vel_fin = points.fin;

            Velocidad vel_found = vel_ini;	//The maximum in the range
            best_vel = vel_found;

            bool found = false;	//variable to determine if the dir_goal range in the DW has been found in the DOV considered
            for (int i=0; i+1<(int)commands.size(); i++){
                Command ci = commands.operator [](i);
                Command ci_1 = commands.operator [](i+1);
                if (ci_1.objeto != 0){
                    if (!found){
                        if (atan2(dir_goal.v, dir_goal.w) == atan2(ci.sup.vel.v, ci.sup.vel.w)){

                            //std::vector<std::vector<Velocidad>>::iterator it;
                            std::vector<DW_Range>::iterator it;
                            for (it=free_range.begin(); it != free_range.end(); ++it){
                                Velocidad vel_ini = (*it).ini;
                                Velocidad vel_fin = (*it).fin;

                                if (vel_ini.v > ci.sup.vel.v){
                                    if (vel_fin.v < ci.sup.vel.v){
                                        (*it).fin = ci.sup.vel;
                                        if (vel_fin.v < ci.inf.vel.v){
                                            //std::vector<Velocidad> new_range;
                                            //DW_Range new_range;
                                            //new_range.push_back(ci.inf.vel);
                                            //new_range.push_back(vel_fin);
                                            DW_Range new_range;
                                            new_range.ini = ci.inf.vel;
                                            new_range.fin = vel_fin;
                                            it = free_range.insert(next(it), new_range);
                                        }
                                    }
                                }else{
                                    if(vel_ini.v > ci.inf.vel.v){
                                        if (vel_fin.v >= ci.inf.vel.v){
                                            it = free_range.erase(it);
                                            if (it != free_range.end())	it = prev(it);
                                            else break;
                                        }else{
                                            (*it).ini = ci.inf.vel;
                                        }

                                    }
                                }
                            }
                            found = true;
                        }else if (atan2(dir_goal.v, dir_goal.w) > atan2(ci.sup.vel.v, ci.sup.vel.w) && atan2(dir_goal.v, dir_goal.w) < atan2(ci_1.sup.vel.v, ci_1.sup.vel.w)){

                            //std::vector<std::vector<Velocidad>>::iterator it;
                            std::vector<DW_Range>::iterator it;
                            for (it=free_range.begin(); it != free_range.end(); ++it){
                                Velocidad vel_ini = (*it).ini;
                                Velocidad vel_fin = (*it).fin;

                                bool intersection_sup = false; Velocidad vel_fin_ant;
                                //Comprobamos el rango de velocidades que queda libre con respecto a los valores superiores
                                if ((ci.sup.vel.v != bounds.vlim_max || ci_1.sup.vel.v != bounds.vlim_max) && (ci.sup.vel.w != bounds.wmax_left || ci_1.sup.vel.w != bounds.wmax_left) &&
                                    (ci.sup.vel.w != bounds.wmax_right || ci_1.sup.vel.w != bounds.wmax_right)){
                                    std::vector<Velocidad> point;
                                    if (IntersectionPoint(vel_ini, vel_fin, ci.sup.vel, ci_1.sup.vel, point)){
                                        vel_fin_ant = vel_fin;
                                        (*it).fin = point.front();
                                        intersection_sup = true;
                                    }else{
                                        if (vel_fin.v > std::min(ci.sup.vel.v, ci_1.sup.vel.v)){	//rango libre
                                            found = true;
                                            continue;
                                        }
                                    }
                                }

                                //Comprobamos el rango de velocidades que queda libre con respecto a los valores inferiores
                                if ((ci.inf.vel.v != bounds.vlim_min || ci_1.inf.vel.v != bounds.vlim_min) && (ci.inf.vel.w != bounds.wmax_left || ci_1.inf.vel.w != bounds.wmax_left) &&
                                    (ci.inf.vel.w != bounds.wmax_right || ci_1.inf.vel.w != bounds.wmax_right)){
                                    std::vector<Velocidad> point;
                                    if (IntersectionPoint(vel_ini, vel_fin, ci.inf.vel, ci_1.inf.vel, point)){
                                        if (!intersection_sup) (*it).ini = point.front();
                                        else{
                                            //std::vector<Velocidad> new_range;
                                            //new_range.push_back(point);
                                            //new_range.push_back(vel_fin_ant);
                                            DW_Range new_range;
                                            new_range.ini = point.front();
                                            new_range.fin = vel_fin_ant;
                                            it = free_range.insert(next(it), new_range);
                                        }
                                    }else{
                                        if (vel_ini.v < std::max(ci.inf.vel.v, ci_1.inf.vel.v)){	//rango libre
                                            found = true;
                                            continue;
                                        }else{
                                            if (!intersection_sup){	//the range lies in the middle
                                                it = free_range.erase(it);
                                                if (it != free_range.end())	it = prev(it);
                                                else break;
                                            }
                                        }
                                    }
                                }else{
                                    if (!intersection_sup){	//the range lies in the middle
                                        it = free_range.erase(it);
                                        if (it != free_range.end())	it = prev(it);
                                        else break;
                                    }
                                }
                            }
                            found = true;
                        }
                    }
                }else{
                    if (!found){
                        //ci es el último comando de velocidad prohibido del DOV actual
                        if (atan2(dir_goal.v, dir_goal.w) == atan2(ci.sup.vel.v, ci.sup.vel.w)){

                            std::vector<DW_Range>::iterator it;
                            for (it=free_range.begin(); it != free_range.end(); ++it){
                                Velocidad vel_ini = (*it).ini;
                                Velocidad vel_fin = (*it).fin;

                                if (vel_ini.v > ci.sup.vel.v){
                                    if (vel_fin.v < ci.sup.vel.v){
                                        (*it).fin = ci.sup.vel;
                                        if (vel_fin.v < ci.inf.vel.v){
                                            DW_Range new_range;
                                            new_range.ini = ci.inf.vel;
                                            new_range.fin = vel_fin;
                                            it = free_range.insert(next(it), new_range);
                                        }
                                    }
                                }else{
                                    if(vel_ini.v > ci.inf.vel.v){
                                        if (vel_fin.v >= ci.inf.vel.v){
                                            it = free_range.erase(it);
                                            if (it != free_range.end())	it = prev(it);
                                            else break;
                                        }else{
                                            (*it).ini = ci.inf.vel;
                                        }

                                    }
                                }
                            }
                        }
                    }

                    found = false;	// hemos alcanzado el último comando del DOV
                }

                if (free_range.empty()) break;
            }
            //Nos quedamos con la primera velocidad del rango de velocidades libres
            //Las velocidades que definen el rango son distintas; devuelvo

            if (!free_range.empty()){

                best_vel = free_range.front().ini;
                if (pol.InsidePolygon(best_vel) && !IsSafeVelocity(best_vel)){
                    best_vel.v = free_range.front().ini.v - (free_range.front().ini.v - free_range.front().fin.v)/2;
                    best_vel.w = free_range.front().ini.w - (free_range.front().ini.w - free_range.front().fin.w)/2;
                }
                free_obs = true;

            }else free_obs = false;

            free_range.clear();
        }
    }

    return free_obs;
}

//DW_Range Strategies::DW_VelocityRange(){
bool Strategies::IntersectRangeDW_Goal(DW_Range& range){
    //Returns the range of velocities which lead to the goal within the dynamic window

    std::vector<Velocidad> points;

    double alfa = atan2(dir_goal.v, dir_goal.w);
    // cout << "Alpha: " << alfa << endl;

    std::vector<Velocidad> dw;
    dw.push_back(dwAg.right); dw.push_back(dwAg.up); dw.push_back(dwAg.left); dw.push_back(dwAg.down); dw.push_back(dwAg.right);
    //We check first if the DW intersects the dir_goal in VS
    for (int i=0; i+1<(int)dw.size(); i++){
        double betai = atan2(dw[i].v, dw[i].w);
        double betai_1 = atan2(dw[i+1].v, dw[i+1].w);
        if (alfa >= betai && alfa <= betai_1 || alfa >= betai_1 && alfa <= betai){
            //Velocidad point;
            std::vector<Velocidad> point;
            if (IntersectionPoint(dw[i], dw[i+1], Velocidad(), dir_goal, point)){
                //If the two segments ((dcha, arriba)-(cero, dir_goal)) intersect, it returns the intersection point
                for (int k=0; k<(int)point.size(); k++){
                    points.push_back(point[k]);
                    // cout << "Point (" << k << ")-> " << "V: " << point[k].v << "W: " << point[k].w << endl;
                }
            }
        }
    }
    std::sort(points.begin(), points.end(), [](const Velocidad &v1, const Velocidad &v2){
        return (v1.v > v2.v || (v1.v == v2.v && std::abs(v1.w) > std::abs(v2.w)));
    });
    points.erase(unique(points.begin(), points.end(), [](const Velocidad &v1, const Velocidad &v2){
        return (v1.v == v2.v && v1.w == v2.w);}), points.end());	//The value returned by the predicate of unique indicates whether both arguments are considered equivalent

    //Relleno la estructura
    range.num = (int)points.size();
    if (range.num == 1){
        range.ini = points[0];
        range.fin = range.ini;
    }else if (range.num > 1){
        range.ini = points.front();
        range.fin = points.back();
    }

    return range.num > 0;
}

bool Strategies::IsVelInZone2(Velocidad vel){
    //Comprueba si la velocidad vel está en los límites de velocidad que definen la zona2(zona de velocidades lineales más altas)

    boundsVS bounds = space.GetBounds();
    if (atan2(vel.v, vel.w) >= atan2(bounds.vlim_max, bounds.wmax_left) &&
        atan2(vel.v, vel.w) <= atan2(bounds.vlim_max, bounds.wmax_right)){
        return true;
    }else{
        return false;
    }
}

bool Strategies::IntersectionDinGoal(Velocidad ini, Velocidad fin){
//Returns if the dynamic window intersects with the segment ini-fin

    bool intersect = false;

    Point_2 right = Point_2(dwAg.right.w, dwAg.right.v);
    Point_2 left = Point_2(dwAg.left.w, dwAg.left.v);
    Point_2 up = Point_2(dwAg.up.w, dwAg.up.v);
    Point_2 down = Point_2(dwAg.down.w, dwAg.down.v);

    std::vector<Segment_2> segments;
    if (!(dwAg.right == dwAg.up)){
        Segment_2 s = Segment_2(right, up);
        segments.push_back(s);
    }
    if (!(dwAg.up == dwAg.left)){
        Segment_2 s = Segment_2 (up, left);
        segments.push_back(s);
    }
    if (!(dwAg.left == dwAg.down)){
        Segment_2 s = Segment_2 (left, down);
        segments.push_back(s);
    }
    if (!(dwAg.down == dwAg.right)){
        Segment_2 s = Segment_2 (down, right);
        segments.push_back(s);
    }
    if (Point_2 (ini.w, ini.v) != Point_2 (fin.w, fin.v)){
        Segment_2 s = Segment_2 (Point_2 (ini.w, ini.v), Point_2 (fin.w, fin.v));
        segments.push_back(s);
    }

    if (do_curves_intersect (segments.begin(), segments.end()))
        intersect = true;
    else{
        Velocidad ag = space.GetAgent();
        if (ag.v == 0 &&
            (dwAg.right == ini || dwAg.left == ini ||
             dwAg.right == fin || dwAg.left == fin))
        {
            intersect = true;
        }
    }

    return intersect;
}

Velocidad ComputeVel(double ang, boundsVS bounds){

    Tsc vel_sol;
    Velocidad vel;
    int sol;
    Line r1, r2;
    Tpf p1, p2;

    if (ang == M_PI/2 || ang == -M_PI/2){
        r1 = Line(1.0, 0.0, 0.0); //recta x=0
    }else if (ang == 0 || ang == M_PI || ang == -M_PI){
        r1 = Line(0.0, 1.0, 0.0);	//recta y=0
    }else{
        r1 = Line(Tpf(0.0, 0.0), ang);	//ecuacion de la recta de R_goal en VS, pasa por (0,0)
    }

    double ang1 = atan2(bounds.vlim_max, bounds.wmax_left);	//angulo de la esquina superior derecha en VS
    ang1 = NormalisePI(ang1);
    double ang2 = std::atan2(bounds.vlim_max, bounds.wmax_right); //angulo de la esquina superior izquierda en VS
    ang2 = NormalisePI(ang2);
    if (ang < ang1){
        r2 = Line(1.0, 0.0, -bounds.wmax_left);
    }else if (ang > ang2){
        r2 = Line(1.0, 0.0, -bounds.wmax_right);
    }else{
        r2 = Line(0.0, 1.0, -bounds.vlim_max);	//y = vmax_adm  or  vlim_max
    }

    SolDosRectas(r1, r2, vel_sol, sol);
    vel.w = vel_sol.x;
    vel.v = vel_sol.y;

    return vel;
}

Velocidad VelDinGoal(Velocidad goal, boundsVS bounds){
    // std::cout << VelDinGoal << std::endl;
    double ang = atan2(goal.v, goal.w);
    return ComputeVel(ang, bounds);
}

Velocidad Strategies::SeguirGoal(){
    // std::cout << "Strategy: follow goal" << std::endl;
//Basic motion: choose velocities which lead to the goal

    Velocidad vel;

    DW_Range dwRange;
    if (IntersectRangeDW_Goal(dwRange)){    //if (IntersectionDinGoal(Velocidad(), dir_goal)){
        //Elegir velocidades dentro de la ventana dinámica que llevan al goal

        // andrew
        float distance = sqrt(pow(this->goal.x - this->posAg.x, 2) + pow(this->goal.y - this->posAg.y, 2));
        float distance2 = distance;
        float dif = dwAg.equal.v - dwAg.down.v;
        float currentV = dwAg.equal.v;
        while (currentV > 0) {
            distance -= currentV;
            currentV -= dif;
        }

        if (distance <= 0.1) {
            return dwAg.down;
        }

        if (distance < 2) {
            return dwAg.equal;
        }

        //if (robot.v < vs->GetVMaxAdm()){
        //vel = up;
        //(29/03/2016): seguimos el radio que lleva al goal
        //DW_Range dwRange = DW_VelocityRange();	//Computes the range of velocities inside the DW that belongs to dir_goal radio
        if (!MaxFreeVelocityInRange(dwRange, vel)) vel = dwAg.down;

    }else{
        //Elegir velocidades de aceleración máxima que acerquen al robot al goal
        if (atan2(dir_goal.v, dir_goal.w) < atan2(space.GetAgent().v, space.GetAgent().w)){
            vel = dwAg.right; //to the right
        }else{  //to the left
            vel = dwAg.left;
        }
    }

    return vel;
}

Velocidad Strategies::AlineacionGoal(){
    // std::cout << "Strategy: alligning with goal" << std::endl;
//Agent disoriented wrt goal

    Velocidad vel;
    DW_Range dwRange;
    if (IntersectRangeDW_Goal(dwRange)){    //if (IntersectionDinGoal(Velocidad(), dir_goal)){
        //(29/03/2016): seguimos el radio que lleva al goal
        //DW_Range dwRange = DW_VelocityRange();	//Computes the range of velocities inside the DW that belongs to dir_goal radio
        // Andrew
        if (!MaxFreeVelocityInRange(dwRange, vel))
            // vel = calculateVelocities(dir_goal, dwAg);
            // andrew no estbaa
            vel = dwAg.down;
    }else{
        if (space.GetAgent().v > dir_goal.v){
            // vel = calculateVelocities(dir_goal, dwAg);
            vel = dwAg.down;
        }else{
            //A buscar el goal a máxima aceleración
            if (dir_goal.w > space.GetAgent().w){
                vel = dwAg.right;
            }else if (dir_goal.w < space.GetAgent().w){
                vel = dwAg.left;
            }else{
                if (dwAg.up.v <= dir_goal.v){
                    vel = dwAg.up;
                }else{
                    vel = VelDinGoal(dir_goal, space.GetBounds());
                }
            }
        }
    }

    return vel;
}

Velocidad Strategies::FreeMotion(bool noObstacles){
    // std::cout << "Strategy: free motion" << std::endl;
    Velocidad vel;
    //std::cout << noObstacles << std::endl;

    if (IsVelInZone2(dir_goal)){
        // std::cout << "GOAL ZONE 2" << std::endl;
        //dir_goal lies in zona2 (zone with maximum linear velocities), the agent is oriented towards the goal

        /*if (std::abs(steeringDir) >= vs->GetWMaxSteering()){	//muy desorientado
            if (steeringDir > 0.0){	//goal a la izquierda
                vel = right;	//dinamica a la derecha
            }else{	//goal a la derecha
                vel = left;	//dinamica a la izquierda
            }
        }else{
        */
        vel = SeguirGoal();
        //}
    }else{
        // andrew
        // std::cout << "NEW DIR FREEMOTION, " << noObstacles << std::endl;
        // NewDirGoal(noObstacles);
        //Alineacion con el goal ya que estamos muy desorientados
        vel = AlineacionGoal();
    }

    return vel;
}

/*
bool IsDirGoalFree(Velocidad dir_goal){
    //analiza la lista con las zonas de comandos ocupados y devuelve si dir_goal esta en zona libre o no
    //Se comprueba si el dir_goal está está en zona completamente ocuapada por el DOV; si el dir_goal está
    //en zona de velocidades parcialmente libres devuelve true

    for (int i=0; i<(int)zonaOcupados.size(); i++){
        Command c_ini = commands[zonaOcupados[i].ini];
        Command c_fin = commands[zonaOcupados[i].fin];

        if (atan2(dir_goal.v, dir_goal.w) > atan2(c_ini.sup.vel.v, c_ini.sup.vel.w) &&
            (atan2(dir_goal.v, dir_goal.w) < atan2(c_fin.sup.vel.v, c_fin.sup.vel.w)))
            return false;
    }

    return true;
}
*/

void Strategies::AnalyseCommands(){

    boundsVS bounds = space.GetBounds();

    int k = 0;
    int z1 = 0; int z2 = 0; int z3 = 0;
    bool primero = false;
    bool ocupado = false;
    int ini = 0; int fin = 0;

    //Valleys...
    bool hay_valle = false;
    int valle_inicia, valle_piso, valle_termina;
    int valle = 0;

    bool cmndo_valle = false;
    int comando_valle = 0;

    int mi_valle = 0;
    mi_valle = -3;

    //Canal...
    bool hay_canal = false;
    Velocidad canalRob;	//velocity of canal which is nearest to the agent

    hay_comandos_z1 = false;  hay_comandos_z2 = false;  hay_comandos_z3 = false;
    zona1 = false; zona2 = false; zona3 = false;
    int z1_inicia = 0; int z1_fin = 0; int z2_inicia = -1; int z2_fin = -1; int z3_inicia = 0; int z3_fin = 0;

    int menor = 0;

    bool primero_z2 = false;
    bool ultimo_z2 = false;
    int c_z2_ini = 0; int c_z2_fin = 0;	//comandos que marcan el inicio y fin de la zona2

    double ang_fin_obj, ang_ini_obj;

    bool hay_hueco_left = false; bool hay_hueco_right = false;
    int comando_hueco_left; int comando_hueco_right;

    bool walign_found = false;

    bool evitando = false;

    //voy a distinguir entre valles que pertenecen solo a la zona2, o a cada lado en el VS
    //guardamos una lista con todos los valles de cada zona
    infoValle valle_left, valle_z2, valle_right;
    bool hay_valle_left = false; bool hay_valle_z2 = false; bool hay_valle_right = false;

    std::vector<Par> listaObjsDin;	//indices comandos inicio y fin de cada objeto dinamico en el VS
    Par lim_obs;

    Par zona;

    zonaLibres.clear(); velLibres.clear(); zonaOcupados.clear();
    valles_left.clear(); valles_z2.clear(); valles_right.clear();

    bool init_wright = false;
    double walign_left = bounds.wmax_left; double walign_right = bounds.wmax_right;

    Velocidad canal_left, canal_right;
    double t_canal_left, t_canal_right;
    double dist_canal = 5.0; int ind_canal = -1;
    bool new_canal = true; bool new_walign = true;
    bool cl_found = false; bool cr_found = false;
    canal_left.v = 0; canal_left.w = 0; t_canal_left = 0;
    canal_right.v = 0; canal_right.w = 0; t_canal_right = 0;

    walign.clear(); canal.clear();

    std::vector<Command>::iterator it;

    int ind_last_command = (int)commands.size()-2;
    int n_commands = (int)commands.size()-1;
    int ind_i = 0;	//para actualizar el comando de referencia a partir del cual se comprueba si hay valles
    // cout << "NUM COMANDOS: " << n_commands << endl;
    for (int i=0; i<n_commands; i++) {
        Command ci = commands[i];

        //cuando hay fusion no buscamos canal ya que puede haber varios; simplemente nos
        //quedamos con el mayor y el menor radio de todos asi como los limites de cada obs en el VS
        if (i == 0){
            lim_obs.ini = i;
        }

        if (ci.objeto == 0){
            //marca un nuevo obstaculo en el VS
            lim_obs.fin = i-1;
            listaObjsDin.push_back(lim_obs);
            lim_obs.ini = i+1;	//marcamos ya el inicio del siguiente objeto dinamico

            if (ocupado){
                zona.fin = i-1;
                zonaOcupados.push_back(zona);
                ocupado = false;
            }

            valle = 0;
            ind_i = i+1;

            walign.push_back(std::make_pair(walign_left, walign_right));
            Velocidad cl, cr;
            if (!cl_found){
                canal_left = canal_right;
                t_canal_left = t_canal_right;
            }
            if (!cr_found){
                canal_right = canal_left;
                t_canal_right = t_canal_left;
            }
            cl = canal_left; cr = canal_right;
            canal.push_back(std::make_pair(std::make_pair(cl, cr), std::make_pair(t_canal_left, t_canal_right)));
            new_walign = true; init_wright = false;
            new_canal = true; primero = false;
            cl_found = false; cr_found = false;

            continue;
        }

        double maxw_v = bounds.InsideBoundsW(bounds.ComputeMaxW_V(ci.sup.vel.v));
        double maxv_w = bounds.InsideBoundsV(bounds.ComputeMaxV_W(ci.sup.vel.w));

        //comprobamos si hay valles
        if (i>=ind_i + 2){
            Command ci_1 = commands[i-1];
            Command ci_2 = commands[i-2];

            if (((ci.sup.vel.v < ci_1.sup.vel.v && ci_1.sup.vel.v >= ci_2.sup.vel.v) ||
                 (ci.sup.vel.v < ci_1.sup.vel.v && ci_1.sup.vel.v < ci_2.sup.vel.v && i-2 == ind_i)) &&
                //comandos[i-2].sup.w >= comandos[i-1].sup.w &&
                //comandosIdFusion[i-1].sup.w > comandosIdFusion[i].sup.w &&
                valle==0 && atan2(ci_1.sup.vel.v, ci_1.sup.vel.w) !=
                            atan2(ci_2.sup.vel.v, ci_2.sup.vel.w)){
                //std::abs(comandos[i-2].sup.w) >= std::abs(comandos[i-1].sup.w) &&
                //std::abs(comandos[i-1].sup.w) > std::abs(comandos[i].sup.w)
                valle = 1;
                valle_inicia = i;
            }else if(ci.sup.vel.v > ci_1.sup.vel.v &&
                     ci_1.sup.vel.v < ci_2.sup.vel.v && valle==1){
                valle = 2;
                valle_piso = i-1;

                //Check if the valley lies within bounds
                double maxv = space.GetBounds().ComputeMaxV_W(ci_1.sup.vel.w);
                if (ci_1.sup.vel.v > maxv) valle = 0;

            }else if (((ci.sup.vel.v <= ci_1.sup.vel.v && ci_1.sup.vel.v > ci_2.sup.vel.v) ||
                       (i == ind_last_command && (ci.sup.vel.v >= ci_1.sup.vel.v && ci_1.sup.vel.v > ci_2.sup.vel.v)))
                      &&
                      //(std::abs(ci_2.sup.w) < wmax_left && atan2(ci.sup.v, ci.sup.w) != atan2(ci_1.sup.v, ci_1.sup.w) && valle == 2))
                      (atan2(ci.sup.vel.v, ci.sup.vel.w) != atan2(ci_1.sup.vel.v, ci_1.sup.vel.w) && valle == 2))
            {
                valle_termina = i-2;
                hay_valle = true;
                if (atan2(commands[valle_termina].sup.vel.v, commands[valle_termina].sup.vel.w) - atan2(commands[valle_inicia].sup.vel.v, commands[valle_inicia].sup.vel.w) < 5*M_PI/180){
                    //la anchura del valle es muy pequeña: no lo consideramos valle
                    hay_valle = false;
                    valle = 0;
                }

                if (hay_valle){
                    if (atan2(commands[valle_inicia].sup.vel.v, commands[valle_inicia].sup.vel.w) <
                        atan2(bounds.vlim_max, bounds.wmax_left) &&
                        atan2(commands[valle_termina].sup.vel.v, commands[valle_termina].sup.vel.w) <
                        atan2(bounds.vlim_max, bounds.wmax_left)){

                        //el valle esta completamente en la zona1 del VS, radios a izquierda
                        if (hay_valle_left == 0){
                            hay_valle_left = 1;
                        }

                        valle_left.ini = valle_inicia;
                        valle_left.fin = valle_termina;
                        valle_left.piso = valle_piso;
                        valles_left.push_back(valle_left);

                    }else if (atan2(commands[valle_inicia].sup.vel.v, commands[valle_inicia].sup.vel.w) >
                              atan2(bounds.vlim_max, bounds.wmax_right) &&
                              atan2(commands[valle_termina].sup.vel.v, commands[valle_termina].sup.vel.w) >
                              atan2(bounds.vlim_max, bounds.wmax_right)){

                        //valle esta en zona3
                        if (hay_valle_right == 0){
                            hay_valle_right = 1;
                        }

                        valle_right.ini = valle_inicia;
                        valle_right.fin = valle_termina;
                        valle_right.piso = valle_piso;
                        valles_right.push_back(valle_right);

                    }else{
                        //valle esta completamente o parte en zona2
                        if (hay_valle_z2 == 0){// && comandosIdFusion[valle_piso].sup.v <= valley_depth){
                            hay_valle_z2 = 1;
                        }

                        valle_z2.ini = valle_inicia;
                        valle_z2.fin = valle_termina;
                        valle_z2.piso = valle_piso;
                        valles_z2.push_back(valle_z2);
                    }
                }

                if (ci.sup.vel.v < ci_1.sup.vel.v && ci_1.sup.vel.v > ci_2.sup.vel.v &&
                    //comandos[i-2].sup.w >= comandos[i-1].sup.w &&
                    //comandosIdFusion[i-1].sup.w > comandosIdFusion[i].sup.w &&
                    atan2(ci_1.sup.vel.v, ci_1.sup.vel.w) != atan2(ci_2.sup.vel.v, ci_2.sup.vel.w)){
                    //el final de un valle puede estar marcando el principio de otro
                    valle = 1;
                    valle_inicia = i;

                }else{
                    valle = 0;
                }
            }
        }
        //análisis de zona1, zona2, zona3
        if (0 <= atan2(ci.sup.vel.v, ci.sup.vel.w) && atan2(ci.sup.vel.v, ci.sup.vel.w) < atan2(bounds.vlim_max, bounds.wmax_left)){
            //ZONA1, buscamos zonas de comandos libres (w)
            hay_comandos_z1 = true;
            if (z1 == 0){
                if (ci.sup.vel.w < maxw_v){   //if (ci.sup.vel.w < bounds.wmax_left){
                    z1_inicia = i; z1 = 1; zona1 = true;
                }
            }else{	//z1 == 1
                if (ci.sup.vel.w == maxw_v){  //if (ci.sup.vel.w == bounds.wmax_left){
                    z1_fin = i; z1 = 0;
                }
            }

            //zona de comandos ocupados
            if (ci.sup.vel.w == maxw_v && !ocupado){  //if (ci.sup.vel.w == bounds.wmax_left && !ocupado){
                zona.ini = i; ocupado = true;
            }else if (ci.sup.vel.w < maxw_v && ocupado){  //}else if (ci.sup.vel.w < bounds.wmax_left && ocupado){
                ocupado = false; zona.fin = i-1; zonaOcupados.push_back(zona);
            }

        }else if (atan2(bounds.vlim_max, bounds.wmax_left) <= atan2(ci.sup.vel.v, ci.sup.vel.w) &&
                  atan2(ci.sup.vel.v, ci.sup.vel.w) <= atan2(bounds.vlim_max, bounds.wmax_right)){
            //ZONA2, buscamos zonas de comandos libres (vel. lineales)
            hay_comandos_z2 = true;
            if (z1 == 1){	//la zona1 ha terminado con comando libre
                if (z1_inicia == 0){	//el primer comando pertenece a la zona1 y es libre
                    hay_hueco_left = true;
                    comando_hueco_left = i-1;
                }
                z1 = 0; z1_fin = i-1;
            }
            if (!primero_z2){
                c_z2_ini = i;
                primero_z2 = true;
                /*
                //caso en que los comandos de z1 no alcanzan valores límites y el primero de z2 esta en el limite
                if (hay_hueco_left && atan2(ci.sup.v, ci.sup.w) == atan2(vlim_max, wmax_left)
                        && ci.sup.v == vlim_max){
                    hay_hueco_left = false;
                }else{
                    comando_hueco_left = i;
                }
                */

            }
            /*
            if (i==0){
                if (atan2(ci.sup.v, ci.sup.w) > atan2(vlim_max, wmax_left) ||
                    (atan2(ci.sup.v, ci.sup.w) == atan2(vlim_max, wmax_left) && ci.sup.v < vlim_max)){
                    hay_hueco_left = true;	//hay hueco en z2 a la dcha del objeto dinamico en el lado dcho del VS, giros a izda
                    comando_hueco_left = i;
                }
            }
            if (i == ind_last_command){
                if (atan2(ci.sup.v, ci.sup.w) < atan2(vlim_max, wmax_right) ||
                    (atan2(ci.sup.v, ci.sup.w) == atan2(vlim_max, wmax_right) && ci.sup.v < vlim_max)){
                    hay_hueco_right = true;	//hueco en z2 a la izda del objeto dinamico en el lado izdo del VS, giros a dcha
                    comando_hueco_right = i;
                }
            }
            */
            if (z2 == 0){
                if (ci.sup.vel.v < maxv_w){    //if (ci.sup.vel.v < bounds.vlim_max){
                    z2 = 1; z2_inicia = i; menor = i; zona2 = true;
                }
            }else{	//z2 == 1
                if (ci.sup.vel.v == maxv_w){   //if (ci.sup.vel.v == bounds.vlim_max){
                    z2 = 0; z2_fin = i;
                }
            }

            /*
            if (hay_hueco_left){
                if(ci.sup.v == vlim_max && !walign_found){
                    walign_left = ci.sup.w;
                    walign_found = true;
                }
            }

            if (atan2(ci.sup.v, ci.sup.w) > atan2(vlim_max, 0.0)){
                if (ci.sup.v == vlim_max)
                    walign_right = ci.sup.w;
            }
            */

            //zona de comandos ocupados
            if (ci.sup.vel.v == maxv_w && !ocupado){   //if (ci.sup.vel.v == bounds.vlim_max && !ocupado){
                zona.ini = i; ocupado = true;
            }else if (ci.sup.vel.v < maxv_w && ocupado){   //}else if (ci.sup.vel.v < bounds.vlim_max && ocupado){
                ocupado = false; zona.fin = i-1; zonaOcupados.push_back(zona);
            }

        }else{ //ZONA3, atan2(vlim_max, wmax_right) < atan2(comandos.GetComando(i).sup.v, comandos.GetComando(i).sup.w)
            hay_comandos_z3 = true;
            if (z2 == 1){ z2 = 0; z2_fin = i-1;}
            if (!ultimo_z2){
                c_z2_fin = i-1;
                ultimo_z2 = true;
                /*
                if (!hay_hueco_right){
                    if (i >= 1){
                        if (atan2(commands[i-1].c.sup.v, commands[i-1].c.sup.w) < atan2(vlim_max, wmax_right) ||
                            (atan2(commands[i-1].c.sup.v, commands[i-1].c.sup.w) == atan2(vlim_max, wmax_right) && commands[i-1].c.sup.v < vlim_max)){
                            hay_hueco_right = true;
                            comando_hueco_right = i-1;
                        }
                    }
                }
                */
            }
            if (z3 == 0){
                    if (ci.sup.vel.w > -maxw_v){  //if (ci.sup.vel.w > bounds.wmax_right){
                    z3_inicia = i; z3 = 1; zona3 = true;
                }else
                    hay_hueco_right = false;
            }else{ //z3 == 1
                if (ci.sup.vel.w == -maxw_v){ //if (ci.sup.vel.w == bounds.wmax_right){
                    z3_fin = i; z3 = 0; hay_hueco_right = false;
                }
            }
            //zona de comandos ocupados
            if (ci.sup.vel.w == -maxw_v && !ocupado){ //if (ci.sup.vel.w == bounds.wmax_right && !ocupado){
                zona.ini = i; ocupado = true;
            }else if (ci.sup.vel.w > -maxw_v && ocupado){ //}else if (ci.sup.vel.w > bounds.wmax_right && ocupado){
                ocupado = false; zona.fin = i-1; zonaOcupados.push_back(zona);
            }
        }

        //Almacenamos las velocidades w de alineación left/right
        //if (i==0){
        if (new_walign){
            walign_left = ci.sup.vel.w;
            walign_right = ci.sup.vel.w;
            new_walign = false;
        }else{
            if (ci.sup.vel.w > walign_left){
                walign_left = ci.sup.vel.w; init_wright = true;
            }else{
                if (init_wright){
                    walign_right = ci.sup.vel.w; init_wright = false;
                }else{
                    if (ci.sup.vel.w < walign_right){
                        walign_right = ci.sup.vel.w;
                    }
                }
            }
        }

        //almacenamos una velocidad limite inferior, como si fuera la de canal
        if (atan2(ci.inf.vel.v, ci.inf.vel.w) <= atan2(bounds.vlim_max, 0)){
            //radios positivos, giros a izquierda
            hay_canal = true;
            cl_found = true;

            if (new_canal){
                canal_left.v = ci.inf.vel.v;
                canal_left.w = ci.inf.vel.w;
                t_canal_left = ci.sup.t;
                ind_canal = i;
                new_canal = false;
            }else{
                Command ci_1 = commands[i-1];
                if (i-1 == ind_canal && ci.inf.vel.v < ci_1.inf.vel.v && ci.inf.vel.w < ci_1.inf.vel.w){	//son consecutivos y menor que el anterior
                    canal_left.v = ci.inf.vel.v;
                    canal_left.w = ci.inf.vel.w;
                    t_canal_left = ci.sup.t;
                    ind_canal = i;
                    //nos quedamos también con aquellas v,w canal mas proxima al wrobot
                    if (std::abs(ci.inf.vel.w - space.GetAgent().w) < dist_canal){
                        canalRob.v = ci.inf.vel.v;
                        canalRob.w = ci.inf.vel.w;
                        dist_canal = std::abs(ci.inf.vel.w - space.GetAgent().w);
                    }
                }
            }
        }

        // std::cout << "Canal left: " << canal_left.v << ", " << canal_left.w << std::endl;

        if (atan2(ci.inf.vel.v, ci.inf.vel.w) >= atan2(bounds.vlim_max, 0)){
            //radios negativos, giros a derecha
            if (!primero){
                //me quedo con el primero, que sera el menor de todos
                canal_right.v = ci.inf.vel.v;
                canal_right.w = ci.inf.vel.w;
                // std::cout << "t_canal_right: " << ci.inf.t << ", " << ci.sup.t << std::endl;
                //t_canal_right = ti.t1;
                t_canal_right = ci.sup.t;
                primero = true;
                hay_canal = true;
                cr_found = true;
                ind_canal = i;
            }else{
                Command ci_1 = commands[i-1];
                if (i-1 == ind_canal && ci.inf.vel.v < ci_1.inf.vel.v && ci.inf.vel.w < ci_1.inf.vel.w){	//son consecutivos y menor que el anterior
                    canal_right.v = ci.inf.vel.v;
                    canal_right.w = ci.inf.vel.w;
                    t_canal_right = ci.sup.t;
                    ind_canal = i;

                    //me quedo con la v,w del canal mas proximo al robot
                    if (std::abs(ci.inf.vel.w - space.GetAgent().w) < dist_canal){
                        canalRob.v = ci.inf.vel.v;
                        canalRob.w = ci.inf.vel.w;
                        dist_canal = std::abs(ci.inf.vel.w - space.GetAgent().w);
                    }
                }
            }
        }
    }

    if (z1 == 1){ z1_fin = ind_last_command; z1 = 0;}	// i = último radio
    if (z2 == 1){ z2_fin = ind_last_command; z2 = 0;}
    if (z3 == 1){ z3_fin = ind_last_command; z3 = 0;}

    if (primero_z2 && !ultimo_z2){
        c_z2_fin = ind_last_command;
        ultimo_z2 = true;
    }

    //marca un nuevo obstaculo en el VS
    lim_obs.fin = (int)commands.size()-2;
    listaObjsDin.push_back(lim_obs);

    if (ocupado){
        zona.fin = (int)commands.size()-2;
        zonaOcupados.push_back(zona);
    }

    walign.push_back(std::make_pair(walign_left, walign_right));
    Velocidad cl, cr;
    if (!cl_found){
        canal_left = canal_right;
        t_canal_left = t_canal_right;
    }
    if (!cr_found){
        canal_right = canal_left;
        t_canal_right = t_canal_left;
    }
    cl = canal_left; cr = canal_right;
    canal.push_back(std::make_pair(std::make_pair(cl, cr), std::make_pair(t_canal_left, t_canal_right)));

    if (walign_left < bounds.wmax_left) hay_hueco_left = true; // < 0.75
    if (walign_right > bounds.wmax_right) hay_hueco_right = true; // > -0.75

    if ((int)listaObjsDin.size() > 1){
        Par objAnt = listaObjsDin[0];
        for (int i=1; i<(int)listaObjsDin.size(); i++){
            Par obj = listaObjsDin[i];
            if (atan2(commands[objAnt.fin].sup.vel.v, commands[objAnt.fin].sup.vel.w) != atan2(commands[obj.ini].sup.vel.v, commands[obj.ini].sup.vel.w)){
                //Los objetos dinámicos no están seguidos, hay espacio libre entre ellos
                if (atan2(commands[obj.ini].sup.vel.v, commands[obj.ini].sup.vel.w) <= atan2(bounds.vlim_max, bounds.wmax_left)){
                    zona1 = true;
                }else if (atan2(commands[obj.ini].sup.vel.v, commands[obj.ini].sup.vel.w) < atan2(bounds.vlim_max, bounds.wmax_right)){
                    zona2 = true;
                }else{
                    zona3 = true;
                }
            }
        }
    }

    //Calculamos las zonas libres a partir de las ocupadas
    int j=0;
    ParAng zonaLibre;
    ParVel velLibre ;
    if (zonaOcupados.size() != 0){
        //si no hay zona de comandos ocupados todo el VS esta libre, la lista es vacia
        //Par zonaOcup = zonaOcupados[0];
        if (atan2(commands[zonaOcupados[0].ini].sup.vel.v, commands[zonaOcupados[0].ini].sup.vel.w) != 0.0){
            zonaLibre.ini = 0.0;
            zonaLibre.fin = atan2(commands[zonaOcupados[0].ini].sup.vel.v, commands[zonaOcupados[0].ini].sup.vel.w);
            velLibre.ini.v = 0.0; velLibre.ini.w = bounds.wmax_left;
            velLibre.fin.v = commands[zonaOcupados[0].ini].sup.vel.v; velLibre.fin.w = commands[zonaOcupados[0].ini].sup.vel.w;
            if (abs(zonaLibre.ini - zonaLibre.fin) > 0.1){
                // cout << "Zona libre: Ini-> "<< zonaLibre.ini << ". End-> " << zonaLibre.fin << ". Vini: " << velLibre.ini.v << "Vend: " << velLibre.fin.v << endl;
                zonaLibres.push_back(zonaLibre);
                velLibres.push_back(velLibre);
            }
        }

        zonaLibre.ini = atan2(commands[zonaOcupados[0].fin].sup.vel.v, commands[zonaOcupados[0].fin].sup.vel.w);
        velLibre.ini.v = commands[zonaOcupados[0].fin].sup.vel.v; velLibre.ini.w = commands[zonaOcupados[0].fin].sup.vel.w;
        j++;
        while (j<zonaOcupados.size()){
            // std::cout << "Zona ocupada: " << zonaOcupados[j].ini << "\t" << zonaOcupados[j].fin << std::endl;
            //zonaOcup = zonaOcupados[j];
            zonaLibre.fin = atan2(commands[zonaOcupados[j].ini].sup.vel.v, commands[zonaOcupados[j].ini].sup.vel.w);
            velLibre.fin.v = commands[zonaOcupados[j].ini].sup.vel.v; velLibre.fin.w = commands[zonaOcupados[j].ini].sup.vel.w;
            if (abs(zonaLibre.ini - zonaLibre.fin) > 0.2){
                // cout << "Zona libre: Ini-> "<< zonaLibre.ini << ". End-> " << zonaLibre.fin << ". Vini: " << velLibre.ini.v << "Vend: " << velLibre.fin.v << endl;
                zonaLibres.push_back(zonaLibre);
                velLibres.push_back(velLibre);
            }

            zonaLibre.ini = atan2(commands[zonaOcupados[j].fin].sup.vel.v, commands[zonaOcupados[j].fin].sup.vel.w);
            velLibre.ini.v = commands[zonaOcupados[j].fin].sup.vel.v; velLibre.ini.w = commands[zonaOcupados[j].fin].sup.vel.w;
            j++;
        }
        if (atan2(commands[zonaOcupados[j-1].fin].sup.vel.v, commands[zonaOcupados[j-1].fin].sup.vel.w) != M_PI){
            zonaLibre.fin = M_PI;
            velLibre.fin.v = 0.0; velLibre.fin.w = bounds.wmax_right;
            if (abs(zonaLibre.ini - zonaLibre.fin) > 0.2){
                // cout << "Zona libre: Ini-> "<< zonaLibre.ini << ". End-> " << zonaLibre.fin << ". Vini: " << velLibre.ini.v << "Vend: " << velLibre.fin.v << endl;
                zonaLibres.push_back(zonaLibre);
                velLibres.push_back(velLibre);
            }
        }
    }else{
        zonaLibre.ini = 0.0;
        zonaLibre.fin = M_PI;
        zonaLibres.push_back(zonaLibre);
        velLibre.ini.v = 0.0; velLibre.ini.w = bounds.wmax_left;
        velLibre.fin.v = 0.0; velLibre.fin.w = bounds.wmax_right;
        velLibres.push_back(velLibre);
    }

    //Delete from zonaLibres the zones corresponding to valleys with few depth, they are occupied zones
    //if (!valles_z2.empty()) DeleteValleys();

    //Orden de las zonas libres de mayor a menor tamaño angular
    for (int i=0; i<(int)zonaLibres.size(); i++){
        ParAng zi = zonaLibres[i];
        ParVel vi = velLibres[i];
        for (int j=(int)zonaLibres.size()-1; j>=i; j--){
            ParAng zj = zonaLibres[j];
            ParVel vj = velLibres[j];
            if ((zi.fin - zi.ini) < (zj.fin - zj.ini)){
                ParAng auxz; ParVel auxv;
                auxz = zi;	auxv = vi;
                zi = zj;	vi = vj;
                zj = auxz;	vj = auxv;
            }
        }
    }

    listaObjsDin.clear();
}

bool Strategies::IsVelInGoalSpace(Velocidad vel, Velocidad dir_goal){
//Asumimos que el dir_goal está en zona de velocidades libres dentro de los límites de algún DOV
//Queremos comprobar que la velocidad vel está en el mismo hueco de velocidades

    double alfa_vel = atan2(vel.v, vel.w);
    double alfa_goal = atan2(dir_goal.v, dir_goal.w);
    for (int i=0; i<(int)zonaLibres.size(); i++){
        ParAng z = zonaLibres[i];

        if (alfa_goal >= z.ini && alfa_goal <= z.fin){
            //Comprobamos que vel está dentro de estos limites
            if(alfa_vel >= z.ini && alfa_vel <= z.fin){
                return true;
            }
            return false;
        }
    }

    return false;
}

bool Strategies::IsVelocityFree(Velocidad vel){
    //analiza la lista con las zonas de comandos ocupados y devuelve si vel esta en zona libre o no
    //Se comprueba si el vel está está en zona completamente ocuapada por el DOV; si el vel está
    //en zona de velocidades parcialmente libres devuelve true

    for (int i=0; i<(int)zonaOcupados.size(); i++){
        Command c_ini = commands[zonaOcupados[i].ini];
        Command c_fin = commands[zonaOcupados[i].fin];

        if (atan2(vel.v, vel.w) > atan2(c_ini.sup.vel.v, c_ini.sup.vel.w) &&
            (atan2(vel.v, vel.w) < atan2(c_fin.sup.vel.v, c_fin.sup.vel.w)))
            return false;
    }

    return true;
}

bool ApproachToGoal(Tsc ag, Tpf goal, Velocidad velAg, Velocidad vGoal, boundsVS bounds){
    //returns true if the robot is near the goal and then the strategy is to prioritize reaching the goal
    double distMax = sqrt((bounds.wmax_left - bounds.wmax_right) * (bounds.wmax_left - bounds.wmax_right) +
                                  (bounds.vlim_max - bounds.vlim_min)*(bounds.vlim_max - bounds.vlim_min));
    double distRobGoal = sqrt((vGoal.w - velAg.w) * (vGoal.w - velAg.w) +
                                      (vGoal.v - velAg.v) * (vGoal.v - velAg.v));

    Tpf sol;
    transfor_inversa_p(goal.x, goal.y, &ag, &sol);
    //return (sol.x*sol.x + sol.y*sol.y <= dist_medium2 && distRobGoal <= 8*distMax);
    return false;
}

bool Strategies::ReachableValley(Command cini, Command cpiso, Command cfin){

    Velocidad robot = space.GetAgent();
    double aw = space.GetConstraints().aw;
    double alfa_robot = atan2(robot.v, robot.w);
    double alfa_ini = atan2(cini.sup.vel.v, cini.sup.vel.w);
    double alfa_fin = atan2(cfin.sup.vel.v, cfin.sup.vel.w);
    double alfa_piso = atan2(cpiso.sup.vel.v, cpiso.sup.vel.w);

    //Almaceno el valor angular del límite del DOV más cercano al valle
    double alfa_dov = 0; bool outside_dov = false;
    for (int i=0; i<(int)space_outDOV.size(); i++){

        Space sp = space_outDOV[i];

        //El robot está en un espacio fuera de los límites del DOV
        if (alfa_robot >= sp.ang.ini && alfa_robot <= sp.ang.fin){
            //Comparo qué lado está más cerca del valle
            if (std::abs(sp.ang.ini - alfa_piso) < std::abs(sp.ang.fin - alfa_piso)){
                alfa_dov = sp.ang.ini;
            }else alfa_dov = sp.ang.fin;
            outside_dov = true;
            break;
        }
    }

    double left_robot = atan2(robot.v, robot.w - aw*stept);
    double size_allowed = std::abs(alfa_robot - atan2(robot.v, robot.w - aw*stept));

    return ((cpiso.sup.vel.v < valley_depth && robot.v < cpiso.sup.vel.v) ||
            (robot.v > cpiso.sup.vel.v && robot.w <= cini.sup.vel.w && robot.w >= cfin.sup.vel.w) ||
            ((alfa_robot > 0 && alfa_robot >= alfa_ini && alfa_robot <= alfa_fin) && ((robot.v > cpiso.sup.vel.v) || (cpiso.sup.vel.v - robot.v <= valley_depth/2))) ||
            ((alfa_robot <= alfa_ini || alfa_robot >= alfa_fin) && (robot.v > cpiso.sup.vel.v) &&
             //(std::abs(alfa_robot - alfa_piso) <= std::abs(alfa_robot - atan2(robot.v, robot.w - aw*stept)) ||
             ((std::abs(alfa_robot - alfa_ini) <= size_allowed ||
               std::abs(alfa_robot - alfa_fin) <= std::abs(alfa_robot - atan2(robot.v, robot.w - aw*stept))) ||
              outside_dov && std::abs(cini.sup.vel.w - cfin.sup.vel.w) >= aw*stept && std::abs(alfa_dov - alfa_piso) <= 2*std::abs(alfa_robot - atan2(robot.v, robot.w - aw*stept))))
    );
}

bool Strategies::ReachableSpace(Space sp){

    Velocidad robot = space.GetAgent();
    double aw = space.GetConstraints().aw;
    double alfa_robot = atan2(robot.v, robot.w);
    double alfa_ini = sp.ang.ini;
    double alfa_fin = sp.ang.fin;
    double alfa_piso = atan2(sp.depth.v, sp.depth.w);

    //Almaceno el valor angular del límite del DOV más cercano al valle
    double alfa_dov = 0; bool outside_dov = false;
    for (int i=0; i<(int)space_outDOV.size(); i++){

        Space sp = space_outDOV[i];

        //El robot está en un espacio fuera de los límites del DOV
        if (alfa_robot >= sp.ang.ini && alfa_robot <= sp.ang.fin){
            //Comparo qué lado está más cerca del valle
            if (std::abs(sp.ang.ini - alfa_piso) < std::abs(sp.ang.fin - alfa_piso)){
                alfa_dov = sp.ang.ini;
            }else alfa_dov = sp.ang.fin;
            outside_dov = true;
            break;
        }
    }

    return ((sp.depth.v < valley_depth && robot.v < sp.depth.v) ||
            (robot.v > sp.depth.v && robot.w <= sp.vel.ini.w && robot.w >= sp.vel.fin.w) ||
            ((alfa_robot > 0 && alfa_robot >= alfa_ini && alfa_robot <= alfa_fin) && ((robot.v > sp.depth.v) || (sp.depth.v - robot.v <= valley_depth/2))) ||
            ((alfa_robot <= alfa_ini || alfa_robot >= alfa_fin) && (robot.v >= sp.depth.v) &&
             ((std::abs(alfa_robot - alfa_ini) <= std::abs(alfa_robot - atan2(robot.v, robot.w - aw*stept)) ||
               std::abs(alfa_robot - alfa_fin) <= std::abs(alfa_robot - atan2(robot.v, robot.w - aw*stept))) ||
              outside_dov && std::abs(sp.vel.ini.w - sp.vel.fin.w) >= aw*stept && std::abs(alfa_dov - alfa_piso) <= 2*std::abs(alfa_robot - atan2(robot.v, robot.w - aw*stept))))
    );
}


bool Strategies::NewDirGoal(bool noObstacles){
//Intentamos recalcular el dir_goal en algún hueco dentro del rango de velocidades angulares en la misma dirección que el dir_goal,
//priorizando velocidades lineales altas
//Según la distancia que esté el robot del goal, priorizamos recalcular el dir_goal en huecos en otras direcciones, o bien en
//velocidades angulares altas y lineales bajas
//1º: huecos y valles dentro del DOV con velocidades angulares en la misma dirección que el dir_goal
//2º: huecos y valles dentro del DOV con velocidades angulares en dirección contraria al dir_goal
//3º: huecos fuera del DOV hacia velocidades angulares del dir_goal

    boundsVS bounds = space.GetBounds();
    double steeringDir = space.GetGoal().velGoal.w;    
    // cout << "COMPUTING NEW GOAL" << endl;

    //Delimitamos las zonas angulares preferentes en las que buscar velocidades libres dentro de valles y huecos dentro del DOV
    double ang_ini, ang_fin; Velocidad cmd_goal = space.GetGoal().commandGoal;
    dir_goal = cmd_goal;
    if (std::abs(steeringDir) > wmax_steering){
        // std::cout << "MAX STEERING IN NEW DIR " << wmax_steering << " " << steeringDir << std::endl;
        if (dir_goal.w > 0){
            ang_ini = atan2(0, bounds.wmax_left);
            ang_fin = atan2(cmd_goal.v, cmd_goal.w);
        }
        else if (dir_goal.w < 0){
            ang_fin = atan2(0, bounds.wmax_right);
            ang_ini = atan2(cmd_goal.v, cmd_goal.w);
        }else{
            ang_ini = atan2(cmd_goal.v, cmd_goal.w);
            ang_fin = ang_ini;
        }
    }else{
        if (dir_goal.w > 0){
            ang_ini = atan2(dir_goal.v, steeringDir);
            ang_fin = atan2(cmd_goal.v, cmd_goal.w);
        }
        else if (dir_goal.w < 0){
            ang_fin = atan2(dir_goal.v, steeringDir);
            ang_ini = atan2(cmd_goal.v, cmd_goal.w);
        }else{
            ang_ini = atan2(cmd_goal.v, cmd_goal.w);
            ang_fin = ang_ini;
        }
    }

    //Buscamos huecos en velocidades angulares hacia el dir_goal
    // cout << cmd_goal.v << ", " << cmd_goal.w << ", " << dir_goal.v << ", " << dir_goal.w << endl;
    // cout << atan2(dir_goal.v, dir_goal.w) << ", " << atan2(cmd_goal.v, cmd_goal.w) << endl;
    double ang_new_goal = atan2(dir_goal.v, dir_goal.w);
    // cout << "ang1: " << ang_new_goal << endl;

    //double min_width = aw*stept/4;
    double aw = space.GetConstraints().aw;
    double min_width = atan2(bounds.vlim_max, aw*stept - aw*stept/4) - atan2(bounds.vlim_max, aw*stept);
    double alfa_goal = atan2(dir_goal.v, dir_goal.w);
    // cout << "ang1.1: " << ang_new_goal << endl;
    bool hayValle = noObstacles;
    // bool hayValle = false;
    if (!valles_z2.empty()){
        //Hay algún valle que contenga una zona de velocidades en la misma dirección que el dir_goal
        infoValle valleGoal;
        // cout << "HAY VALLES EN Z2" << endl;
        // cout << "SIZE VALLES: " << valles_z2.size() << endl;
        for (int i=0; i<(int)valles_z2.size(); i++){

            infoValle valle = valles_z2[i];
            double ang_valle_ini = atan2(commands[valle.ini].sup.vel.v, commands[valle.ini].sup.vel.w);
            double ang_valle_fin = atan2(commands[valle.fin].sup.vel.v, commands[valle.fin].sup.vel.w);

            //El valle empieza o termina en la zona de velocidades hacia el dir_goal, o el valle contiene toda la zona
            if ((ang_valle_ini >= ang_ini && ang_valle_ini <= ang_fin) || (ang_valle_fin >= ang_ini && ang_valle_fin <= ang_fin)  ||
                (ang_valle_ini <= ang_ini && ang_valle_fin >= ang_fin)){

                //Comprobamos que el valle sea alcanzable por el robot
                if (ReachableValley(commands[valle.ini], commands[valle.piso], commands[valle.fin])){

                    if (hayValle){
                        //Me quedo con el más profundo
                        if (commands[valle.piso].sup.vel.v < commands[valleGoal.piso].sup.vel.v){
                            ang_new_goal = atan2(commands[valle.piso].sup.vel.v, commands[valle.piso].sup.vel.w);
                            valleGoal = valle;
                        }

                    }else{
                        ang_new_goal = atan2(commands[valle.piso].sup.vel.v, commands[valle.piso].sup.vel.w);
                        valleGoal = valle;
                        hayValle = true;
                    }
                }

            }
        }
    }
    // cout << "HAY VALLE: " << hayValle << endl;
    //andrew
    // estrategia_valle = hayValle;
    // cout << "ang1.5: " << ang_new_goal << endl;
    if (!hayValle){
        // cout << "NO HAY VALLES EN Z2" << endl;
        //Busco huecos dentro del DOV en zonas de velocidades en la misma dirección que el dir_goal
        bool haySpace = false;
        Space spaceGoal;
        for (int i=0; i<(int)space_inDOV.size(); i++){

            Space sp = space_inDOV[i];

            //El hueco empieza o termina en la zona de velocidades hacia el dir_goal, o el hueco contiene toda la zona
            if ((sp.ang.ini >= ang_ini && sp.ang.ini <= ang_fin) || (sp.ang.fin >= ang_ini && sp.ang.fin <= ang_fin) ||
                (sp.ang.ini <= ang_ini && sp.ang.fin >= ang_fin)){

                //El hueco tiene cierta anchura
                //if (std::abs(sp.vel.ini.w - sp.vel.fin.w) > min_width){
                if (std::abs(sp.ang.fin - sp.ang.ini) > min_width){

                    //Comprobamos que el hueco sea alcanzable por el robot
                    if (ReachableSpace(sp)){

                        if (haySpace){
                            //Me quedo con el más profundo
                            if (sp.depth.v < spaceGoal.depth.v){
                                ang_new_goal = atan2(sp.depth.v, sp.depth.w);
                                spaceGoal = sp;
                            }

                        }else{
                            ang_new_goal = atan2(sp.depth.v, sp.depth.w);
                            spaceGoal = sp;
                            haySpace = true;
                        }

                    }

                }
            }
        }

        if (!haySpace){

            //Busco huecos fuera del DOV con velocidades en la misma dirección que el dir_goal
            Space spaceGoal; bool haySpaceOut = false;
            for (int i=0; i<(int)space_outDOV.size(); i++){

                Space sp = space_outDOV[i];

                //El hueco empieza o termina en la zona de velocidades en la misma dirección al dir_goal, y en zona2
                if (((sp.ang.ini >= ang_ini && sp.ang.ini <= ang_fin) || (sp.ang.fin >= ang_ini && sp.ang.fin <= ang_fin) ||
                     (sp.ang.ini <= ang_ini && sp.ang.fin >= ang_fin)) &&
                    ((sp.ang.ini >= atan2(bounds.vlim_max, bounds.wmax_left) && sp.ang.ini <= atan2(bounds.vlim_max, bounds.wmax_right) &&
                      sp.ang.fin >= atan2(bounds.vlim_max, bounds.wmax_left) && sp.ang.fin <= atan2(bounds.vlim_max, bounds.wmax_right)) ||
                     (sp.ang.ini < atan2(bounds.vlim_max, bounds.wmax_left) &&
                      sp.ang.fin >= atan2(bounds.vlim_max, bounds.wmax_left) && sp.ang.fin <= atan2(bounds.vlim_max, bounds.wmax_right)) ||
                     (sp.ang.ini >= atan2(bounds.vlim_max, bounds.wmax_left) && sp.ang.ini <= atan2(bounds.vlim_max, bounds.wmax_right) &&
                      sp.ang.fin > atan2(bounds.vlim_max, bounds.wmax_right)) )){

                    //El hueco tiene cierta anchura
                    //if (std::abs(sp.vel.ini.w - sp.vel.fin.w) > min_width){
                    if (std::abs(sp.ang.fin - sp.ang.ini) > min_width){
                        //Comprobamos que el hueco sea alcanzable por el robot
                        if (ReachableSpace(sp)){

                            //21/09/2016: modifico los limites de la zona libre para calcular el nuevo dir_goal entorno a la zona de velocidades en dirección al goal
                            /*//si el robot está cerca del goal (para que no se desoriente demasiado)
                            Tpf goal_rob;
                            transfor_inversa_p(goal.x, goal.y, &posRobot, &goal_rob);
                            if (goal_rob.x*goal_rob.x + goal_rob.y*goal_rob.y < dist_medium2){
                                if (sp.ang.ini > ang_ini) sp.ang.ini = ang_ini;
                                if (sp.ang.fin < ang_fin) sp.ang.fin = ang_fin;
                                sp.width = sp.ang.fin - sp.ang.ini;
                            }
                            //*/

                            if (haySpaceOut){
                                //Me quedo con el más próximo al dir_goal
                                if (std::abs(alfa_goal - sp.ang.ini) < std::abs(alfa_goal - ang_new_goal) ||
                                    std::abs(alfa_goal - sp.ang.fin) < std::abs(alfa_goal - ang_new_goal)){

                                    int n = std::ceil(sp.width/(atan2(bounds.vlim_max, aw*stept) - atan2(bounds.vlim_max, 2*aw*stept)));
                                    if (n < 2) n = 2;

                                    if (std::abs(sp.ang.ini - alfa_goal) < std::abs(sp.ang.fin - alfa_goal)){
                                        ang_new_goal = sp.ang.ini + std::abs(sp.ang.fin - sp.ang.ini)/n;
                                    }else{
                                        ang_new_goal = sp.ang.fin - std::abs(sp.ang.fin - sp.ang.ini)/n;
                                    }
                                    //ang_new_goal = sp.ang.ini + std::abs(sp.ang.fin - sp.ang.ini)/n;
                                    spaceGoal = sp;
                                }
                            }else{

                                int n = std::ceil(sp.width/(atan2(bounds.vlim_max, aw*stept) - atan2(bounds.vlim_max, 2*aw*stept)));
                                if (n < 2) n = 2;

                                if (std::abs(sp.ang.ini - atan2(dir_goal.v, dir_goal.w)) <
                                    std::abs(sp.ang.fin - atan2(dir_goal.v, dir_goal.w))){
                                    ang_new_goal = sp.ang.ini + std::abs(sp.ang.fin - sp.ang.ini)/n;
                                }else{
                                    ang_new_goal = sp.ang.fin - std::abs(sp.ang.fin - sp.ang.ini)/n;
                                }

                                spaceGoal = sp;
                                haySpaceOut = true;
                            }

                        }
                    }
                }
            }


            if (!haySpaceOut){	//No se ha encontrado ningún hueco de velocidades en dirección al dir_goal
                // cout << "No hay huecos en dir_goal" << endl;

                //Si el robot no está muy desorientado, busco valles y huecos en zonas en dirección diferentes al dir_goal
                //if (std::abs(steeringDir) < (M_PI/2)/stept){
                if (std::abs(steeringDir) < 3*wmax_steering){

                    bool hayValle2 = false; bool haySpace2 = false; bool haySpaceOut2 = false;

                    infoValle valleGoal;
                    if (!valles_z2.empty()){
                        // cout << "Valle en direccion distinta" << endl;
                        //Hay algún valle que contenga una zona de velocidades en distinta dirección que el dir_goal
                        for (int i=0; i<(int)valles_z2.size(); i++){

                            infoValle valle = valles_z2[i];
                            double ang_valle_ini = atan2(commands[valle.ini].sup.vel.v, commands[valle.ini].sup.vel.w);
                            double ang_valle_fin = atan2(commands[valle.fin].sup.vel.v, commands[valle.fin].sup.vel.w);

                            //El valle empieza o termina en la zona de velocidades diferente al dir_goal
                            if (ang_valle_ini >= ang_fin || ang_valle_fin <= ang_ini){

                                //Comprobamos que el valle sea alcanzable por el robot
                                if (ReachableValley(commands[valle.ini], commands[valle.piso], commands[valle.fin])){

                                    if (hayValle2){
                                        //Me quedo con el más cercano al dir_goal original, para no alejarnos demasiado en la dirección contraria
                                        if (std::abs(atan2(commands[valle.piso].sup.vel.v, commands[valle.piso].sup.vel.w) - alfa_goal) <
                                            std::abs(atan2(commands[valleGoal.piso].sup.vel.v, commands[valleGoal.piso].sup.vel.w) - alfa_goal)){
                                            ang_new_goal = atan2(commands[valle.piso].sup.vel.v, commands[valle.piso].sup.vel.w);
                                            valleGoal = valle;
                                        }

                                    }else{
                                        ang_new_goal = atan2(commands[valle.piso].sup.vel.v, commands[valle.piso].sup.vel.w);
                                        valleGoal = valle;
                                        hayValle2 = true;
                                    }

                                }

                            }
                        }
                    }

                    //if (!hayValle2){
                    //Busco huecos dentro del DOV en zonas de velocidades en dirección diferente al dir_goal
                    Space spaceGoal;
                    for (int i=0; i<(int)space_inDOV.size(); i++){

                        Space sp = space_inDOV[i];

                        //El hueco empieza o termina en la zona de velocidades diferente al dir_goal
                        if (sp.ang.ini >= ang_fin || sp.ang.fin <= ang_ini){

                            //El hueco tiene cierta anchura
                            //if (std::abs(sp.vel.ini.w - sp.vel.fin.w) > min_width){
                            if (std::abs(sp.ang.fin - sp.ang.ini) > min_width){
                                //Comprobamos que el hueco sea alcanzable por el robot
                                if (ReachableSpace(sp)){

                                    if (haySpace2){
                                        //Me quedo con el más próximo a direcciones hacia el dir_goal
                                        if (std::abs(atan2(sp.depth.v, sp.depth.w) - alfa_goal) < std::abs(atan2(spaceGoal.depth.v, spaceGoal.depth.w) - alfa_goal)){
                                            ang_new_goal = atan2(spaceGoal.depth.v, spaceGoal.depth.w);
                                            spaceGoal = sp;
                                        }

                                    }else{
                                        if (hayValle2){
                                            double alfa_piso = atan2(commands[valleGoal.piso].sup.vel.v, commands[valleGoal.piso].sup.vel.w);
                                            if (std::abs(alfa_piso - alfa_goal) < std::abs(atan2(sp.depth.v, sp.depth.w) - alfa_goal)){
                                                //Mantengo el valle como mejor opcion
                                            }else{
                                                ang_new_goal = atan2(sp.depth.v, sp.depth.w);
                                                spaceGoal = sp;
                                                haySpace2 = true;
                                                hayValle2 = false;
                                            }
                                        }else{
                                            ang_new_goal = atan2(sp.depth.v, sp.depth.w);
                                            spaceGoal = sp;
                                            haySpace2 = true;
                                        }
                                    }

                                }

                            }
                        }
                    }

                    //if (!haySpace2){
                    //Busco huecos fuera del DOV, primero en zona2
                    for (int i=0; i<(int)space_outDOV.size(); i++){

                        Space sp = space_outDOV[i];

                        //El hueco empieza o termina en la zona de velocidades diferente al dir_goal
                        if (sp.ang.ini >= ang_fin || sp.ang.fin <= ang_ini){

                            //El hueco tiene cierta anchura
                            //if (std::abs(sp.vel.ini.w - sp.vel.fin.w) > min_width){
                            if (std::abs(sp.ang.fin - sp.ang.ini) > min_width){
                                //Comprobamos que el hueco sea alcanzable por el robot
                                if (ReachableSpace(sp)){

                                    if (haySpaceOut2){
                                        //Me quedo con el más próximo a direcciones hacia el dir_goal
                                        if (std::abs(alfa_goal - sp.ang.ini) < std::abs(alfa_goal - ang_new_goal) ||
                                            std::abs(alfa_goal - sp.ang.fin) < std::abs(alfa_goal - ang_new_goal)){

                                            int n = std::ceil(sp.width/(atan2(bounds.vlim_max, aw*stept) - atan2(bounds.vlim_max, 2*aw*stept)));
                                            if (n < 2) n = 2;

                                            if (std::abs(sp.ang.ini - alfa_goal) < std::abs(sp.ang.fin - alfa_goal)){
                                                ang_new_goal = sp.ang.ini + std::abs(sp.ang.fin - sp.ang.ini)/n;
                                            }else{
                                                ang_new_goal = sp.ang.fin - std::abs(sp.ang.fin - sp.ang.ini)/n;
                                            }
                                            //ang_new_goal = sp.ang.ini + std::abs(sp.ang.fin - sp.ang.ini)/n;
                                            spaceGoal = sp;
                                        }
                                    }else{
                                        if (hayValle2){
                                            double alfa_piso = atan2(commands[valleGoal.piso].sup.vel.v, commands[valleGoal.piso].sup.vel.w);
                                            if (std::abs(alfa_piso - alfa_goal) < std::abs(alfa_goal - sp.ang.ini) &&
                                                std::abs(alfa_piso - alfa_goal) < std::abs(alfa_goal - sp.ang.fin)){
                                                //Mantengo el valle como mejor opcion
                                            }else{
                                                int n = std::ceil(sp.width/(atan2(bounds.vlim_max, aw*stept) - atan2(bounds.vlim_max, 2*aw*stept)));
                                                if (n < 2) n = 2;

                                                if (std::abs(sp.ang.ini - alfa_goal) < std::abs(sp.ang.fin - alfa_goal)){
                                                    ang_new_goal = sp.ang.ini + std::abs(sp.ang.fin - sp.ang.ini)/n;
                                                }else{
                                                    ang_new_goal = sp.ang.fin - std::abs(sp.ang.fin - sp.ang.ini)/n;
                                                }
                                                //ang_new_goal = sp.ang.ini + std::abs(sp.ang.fin - sp.ang.ini)/n; //ang_new_goal = atan2(sp.depth.v, sp.depth.w);
                                                spaceGoal = sp;
                                                haySpaceOut2 = true;
                                                hayValle2 = false;
                                            }
                                        }else if (haySpace2){
                                            if (std::abs(ang_new_goal - alfa_goal) < std::abs(sp.ang.ini - alfa_goal) &&
                                                std::abs(ang_new_goal - alfa_goal) < std::abs(sp.ang.fin - alfa_goal)){
                                                //Mantengo el hueco dentro de DOV como mejor opcion
                                            }else{
                                                int n = std::ceil(sp.width/(atan2(bounds.vlim_max, aw*stept) - atan2(bounds.vlim_max, 2*aw*stept)));
                                                if (n < 2) n = 2;
                                                if (std::abs(sp.ang.ini - alfa_goal) < std::abs(sp.ang.fin - alfa_goal)){
                                                    ang_new_goal = sp.ang.ini + std::abs(sp.ang.fin - sp.ang.ini)/n;
                                                }else{
                                                    ang_new_goal = sp.ang.fin - std::abs(sp.ang.fin - sp.ang.ini)/n;
                                                }
                                                //ang_new_goal = sp.ang.ini + std::abs(sp.ang.fin - sp.ang.ini)/n; //ang_new_goal = atan2(sp.depth.v, sp.depth.w);
                                                spaceGoal = sp;
                                                haySpaceOut2 = true;
                                                haySpace2 = false;
                                            }
                                        }else{

                                            int n = std::ceil(sp.width/(atan2(bounds.vlim_max, aw*stept) - atan2(bounds.vlim_max, 2*aw*stept)));
                                            if (n < 2) n = 2;

                                            if (std::abs(sp.ang.ini - alfa_goal) < std::abs(sp.ang.fin - alfa_goal)){
                                                ang_new_goal = sp.ang.ini + std::abs(sp.ang.fin - sp.ang.ini)/n;
                                            }else{
                                                ang_new_goal = sp.ang.fin - std::abs(sp.ang.fin - sp.ang.ini)/n;
                                            }

                                            spaceGoal = sp;
                                            haySpaceOut2 = true;
                                        }
                                    }

                                }

                            }
                        }
                    }

                    //}
                    //}

                    if (!hayValle2 && !haySpace2 && !haySpaceOut2){
                        // cout << "No hay nada de nada" << endl;
                        //Busco el hueco de velocidades más grande fuera del DOV
                        double minDist = 0;
                        for (int i=0; i<(int)space_outDOV.size(); i++){

                            Space sp = space_outDOV[i];
                            if (std::abs(sp.ang.fin - sp.ang.ini) > minDist){
                                minDist = std::abs(sp.ang.fin - sp.ang.ini);

                                int n = std::ceil(sp.width/(atan2(bounds.vlim_max, aw*stept) - atan2(bounds.vlim_max, 2*aw*stept)));
                                if (n < 2) n = 2;

                                if (std::abs(sp.ang.ini - alfa_goal) < std::abs(sp.ang.fin - alfa_goal)){
                                    ang_new_goal = sp.ang.ini + std::abs(sp.ang.fin - sp.ang.ini)/n;
                                }else{
                                    ang_new_goal = sp.ang.fin - std::abs(sp.ang.fin - sp.ang.ini)/n;
                                }
                            }
                        }
                    }

                }else{
                    // cout << "Robot muy desorientado. Size out DOV: "<< space_outDOV.size() << endl;
                    //El robot está bastante desorientado del goal, buscamos el hueco más cercano al dir_goal fuera del DOVs
                    double minDist = 3*M_PI;
                    for (int i=0; i<(int)space_outDOV.size(); i++){

                        Space sp = space_outDOV[i];

                        int n = std::ceil(sp.width/(atan2(bounds.vlim_max, aw*stept) - atan2(bounds.vlim_max, 2*aw*stept)));
                        if (n < 2) n = 2;

                        if (std::abs(alfa_goal - sp.ang.ini) < minDist){
                            minDist = std::abs(alfa_goal - sp.ang.ini);
                            ang_new_goal = sp.ang.ini + std::abs(sp.ang.fin - sp.ang.ini)/n;
                        }

                        if (std::abs(alfa_goal - sp.ang.fin) < minDist){
                            minDist = std::abs(alfa_goal - sp.ang.fin);
                            ang_new_goal = sp.ang.fin - std::abs(sp.ang.fin - sp.ang.ini)/n;
                        }
                        // cout << "Ang_new_goal: " << ang_new_goal << endl;
                    }
                }

            }//endif: !haySpaceOut en velocidades hacia dirección del dir_goal
        }//endif: !haySpace en velocidades hacia dirección del dir_goal
    }//endif: !hay_valle en velocidades hacia dirección del dir_goal

    // cout << "ang2: " << ang_new_goal << endl;
    if (ang_new_goal <= atan2(bounds.vlim_max, bounds.wmax_left)){
        // cout <<"a" <<endl;
        dir_goal.v = bounds.wmax_left*tan(ang_new_goal);
        dir_goal.w = bounds.wmax_left;
        // dir_goal.w = bounds.vlim_max/tan(ang_new_goal);

    }else if (ang_new_goal <= atan2(bounds.vlim_max, bounds.wmax_right)){
        // cout <<"b" <<endl;
        dir_goal.v = bounds.vlim_max;
        dir_goal.w = bounds.vlim_max/tan(ang_new_goal);
    }else{
        // cout <<"c" <<endl;
        dir_goal.v = bounds.wmax_right*tan(ang_new_goal);
        dir_goal.w = bounds.wmax_right;
        // dir_goal.w = bounds.vlim_max/tan(ang_new_goal);
    }
    // cout << "Selected: " << atan2(dir_goal.v, dir_goal.w) << endl;

    // /*
    // float x[2], y[2];
    // x[0] = 0.0; y[0] = 0.0;
    // x[1] = dir_goal.w; y[1] = dir_goal.v;
    // DibujaRecta(4, x, y, 2, "blue");
    // //*/
    

    return true;
}

bool Strategies::InValleyZ2(Velocidad vel, infoValle& valleOut){
//Función que devuelve true o false si la velocidad vel está dentro de algún valle

    bool enValle = false;

    if (!valles_z2.empty()){

        for (int i=0; i<(int)valles_z2.size(); i++){

            infoValle valle = valles_z2[i];
            if (atan2(vel.v, vel.w) >= atan2(commands[valle.ini].sup.vel.v, commands[valle.ini].sup.vel.w) &&
                atan2(vel.v, vel.w) <= atan2(commands[valle.fin].sup.vel.v, commands[valle.fin].sup.vel.w)){

                //enValle = ReachableValley(commands[valle.ini], commands[valle.piso], commands[valle.fin]);
                enValle = true;
                valleOut = valle;
                break;
            }
        }

    }

    return enValle;
}

bool Strategies::InSpaceDOV(Velocidad vel, Space& spOut){
//Función que devuelve true o false si la velocidad vel está dentro de algún hueco

    bool enHueco = false;

    if (!space_inDOV.empty()){

        for (int i=0; i<(int)space_inDOV.size(); i++){

            Space sp = space_inDOV[i];
            if (atan2(vel.v, vel.w) >= sp.ang.ini && atan2(vel.v, vel.w) <= sp.ang.fin){
                //El hueco tiene cierta anchura y es alcanzable
                //enHueco = (std::abs(sp.vel.ini.w - sp.vel.fin.w) > aw*stept) && ReachableSpace(sp);
                enHueco = true;
                spOut = sp;
                break;
            }
        }

    }

    return enHueco;
}

Velocidad Strategies::PassingBefore(Velocidad piso){
    // std::cout << "Strategy: passing before" << std::endl;
    Velocidad vel;
    Velocidad robot = space.GetAgent();

    boundsVS bounds = space.GetBounds();

    if (!IntersectionDinGoal(Velocidad(), dir_goal)){
        if (atan2(robot.v, robot.w) > atan2(dir_goal.v, dir_goal.w)){
            vel = dwAg.right;
        }else{
            vel = dwAg.left;
        }
    }else{
        if (robot.v <= piso.v * 1.20){	//if (robot.v <= commands[valle.piso].sup.vel.v * 1.20){
            //el robot no ha alcanzado el hueco todavia
            vel = dwAg.up;

        }else{

            if (piso.v >= valley_depth && robot.v >= piso.v){
                DW_Range dwRange;
                if (IntersectRangeDW_Goal(dwRange)){   //if (IntersectionDinGoal(Velocidad(), dir_goal)){
                    //DW_Range dwRange = DW_VelocityRange();	//Computes the range of velocities inside the DW that belongs to dir_goal radio
                    if (!MaxFreeVelocityInRange(dwRange, vel)){
                        if (robot.v < bounds.vlim_max) vel = dwAg.up;
                        else vel = VelDinGoal(dir_goal, space.GetBounds());
                    }
                }else{	//no estoy en el valle
                    if (atan2(dir_goal.v, dir_goal.w) < atan2(robot.v, robot.w)){
                        vel = dwAg.right;
                    }else{
                        vel = dwAg.left;
                    }
                }

            }else{

                //comprobar si el robot elegido cae dentro o fuera de los objetos dinamicos
                if (pol.InsidePolygon(robot)){
                    //el robot ha alcanzado el valle pero esta dentro de alguno de los objetos dinamicos,
                    //debe hacer movimiento a izda o dcha para meterse dentro del valle
                    if (atan2(robot.v, robot.w) < atan2(dir_goal.v, dir_goal.w)){
                        vel = dwAg.left;
                    }else{
                        vel = dwAg.right;
                    }

                    if (pol.InsidePolygon(vel)){
                        vel = dwAg.up;
                    }

                }else{	//el robot está fuera del objeto dinámico
                    if (robot.v < bounds.vlim_max) vel = dwAg.up;
                    else vel = VelDinGoal(dir_goal, space.GetBounds());
                }
            }
        }
    }

    estrategia_valle = true;
    return vel;
}

bool Strategies::InSpaceInsideDOV(Velocidad vel){
    //Pre: vel está en zona libre
    //Comprobamos si la velocidad 'vel' está en algún hueco dentro de los límites de algún DOV

    assert(!pol.InsidePolygon(vel));

    Velocidad robot = space.GetAgent();

    bool in_space = false;

    //Comprobamos los valles
    std::vector<infoValle> valles = valles_left;
    std::vector<infoValle>::iterator it;
    for (it=valles.begin(); it != valles.end(); ++it){

        infoValle valle = (*it);
        if (atan2(vel.v, vel.w) >= atan2(commands[valle.ini].sup.vel.v, commands[valle.ini].sup.vel.w) &&
            atan2(vel.v, vel.w) <= atan2(commands[valle.fin].sup.vel.v, commands[valle.fin].sup.vel.w) &&
            robot.v >= commands[valle.piso].sup.vel.v){

            return true;
        }
    }

    valles = valles_right;
    for (it=valles.begin(); it != valles.end(); ++it){

        infoValle valle = (*it);
        if (atan2(vel.v, vel.w) >= atan2(commands[valle.ini].sup.vel.v, commands[valle.ini].sup.vel.w) &&
            atan2(vel.v, vel.w) <= atan2(commands[valle.fin].sup.vel.v, commands[valle.fin].sup.vel.w) &&
            robot.v >= commands[valle.piso].sup.vel.v){

            return true;
        }
    }

    valles = valles_z2;
    for (it=valles.begin(); it != valles.end(); ++it){

        infoValle valle = (*it);
        if (atan2(vel.v, vel.w) >= atan2(commands[valle.ini].sup.vel.v, commands[valle.ini].sup.vel.w) &&
            atan2(vel.v, vel.w) <= atan2(commands[valle.fin].sup.vel.v, commands[valle.fin].sup.vel.w) &&
            robot.v >= commands[valle.piso].sup.vel.v){

            return true;
        }
    }

    //Comprobamos los huecos entre DOVs
    for (int k=0; k<pol.Size(); k++){
        std::vector<Par> lim_pol = pol.GetLimPol();
        std::vector<Velocidad> points = pol.GetPoints();
        if (atan2(vel.v, vel.w) >= atan2(points[lim_pol[k].ini].v, points[lim_pol[k].ini].w) &&
            atan2(vel.v, vel.w) <= atan2(points[lim_pol[k].fin].v, points[lim_pol[k].fin].w)){

            Velocidad vel, vel_canal; double t_canal;
            for (int i=0; i<(int)canal.size(); i++){	//Busco una velocidad de canal del DOV

                if (atan2(canal[i].first.first.v, canal[i].first.first.w) >= atan2(points[lim_pol[k].ini].v, points[lim_pol[k].ini].w) &&
                    atan2(canal[i].first.first.v, canal[i].first.first.w) <= atan2(points[lim_pol[k].fin].v, points[lim_pol[k].fin].w)){

                    double dist = std::sqrt((vel.w - canal[i].first.first.w)*(vel.w - canal[i].first.first.w) + (vel.v - canal[i].first.first.v)*(vel.v - canal[i].first.first.v));
                    if (vel.v > canal[i].first.first.v & dist > 2*space.GetConstraints().aw*stept){
                        return true;
                    }
                }
            }
        }
    }

    return in_space;
}

Velocidad Strategies::AlineacionWObstacle(double walign){
    // std::cout << "Strategy: allign with obstacle" << std::endl;
    //El robot se alinea hacia la dirección walign

    Velocidad vel;
    Velocidad robot = space.GetAgent();
    boundsVS bounds = space.GetBounds();

    double steeringDir = space.GetGoal().velGoal.w;
    if (steeringDir > 3*wmax_steering){
        // std::cout << "alineacionobstacle" << std::endl;
        vel = UnderDirGoal();

    }else{
        //Si el robot está en algún hueco libre dentro de los límites del DOV, la estrategia es seguir el goal
        if(!pol.InsidePolygon(robot) && InSpaceInsideDOV(robot)){
            // std::cout << "alineacionobstacle" << std::endl;
            if (steeringDir > wmax_steering) vel = UnderDirGoal();
            else vel = SeguirGoal();

            fstrategies << "PasarPrimero\t";
        }
        else{
            if (walign < 0){
                //waling es negativa, direccion de alineacion hacia la derecha
                if (dwAg.left.w >= walign || (dwAg.left.w < walign && robot.w >= walign)){
                    vel = dwAg.left;
                }else if (dwAg.left.w < walign && robot.w < walign && dwAg.right.w >= walign){
                    if (robot.v < bounds.vlim_max) vel = dwAg.up;
                    else vel = VelDinGoal(dir_goal, bounds);
                }else{
                    vel = dwAg.right;
                }
            }else{
                //waling es positiva, direccion de alineacion hacia la izquierda
                if (dwAg.left.w < walign && robot.w <= walign){
                    vel = dwAg.right;
                }else if (dwAg.left.w <= walign){
                    if (robot.v < bounds.vlim_max) vel = dwAg.up;
                    else vel = VelDinGoal(dir_goal, bounds);
                }else
                    vel = dwAg.left;
            }
            fstrategies << "Alineacion\t";
        }
    }

    return vel;
}

Velocidad Strategies::ObjSinValleZ2ConHueco(double walign){
    // std::cout << "Strategy: ObjSinValleZ2ConHueco" << std::endl;
    Velocidad vel;
    Velocidad robot = space.GetAgent();

    //Busco la velocidad del canal en torno a la que debe estar el robot
    Velocidad vel_canal; double t_canal;
    double v_canal = space.GetBounds().vlim_min;
    for (int i=0; i<(int)canal.size(); i++){
        //Nos quedamos con la velocidad de canal que tenga la velocidad lineal más alta
        if (canal[i].first.first.v > v_canal){
            vel_canal = canal[i].first.first;
            t_canal = canal[i].second.first;
            v_canal = vel_canal.v;
        }

        if (canal[i].first.second.v > v_canal){
            vel_canal = canal[i].first.second;
            t_canal = canal[i].second.second;
            v_canal = vel_canal.v;
        }
    }

    if (pol.InsidePolygon(robot)){
        fstrategies << "\tRobotDentroPoligono";
        double timeStop = (robot.v - vel_canal.v)/(space.GetConstraints().av*stept);
        if (timeStop <= t_canal){ // me da tiempo a frenar
            if (std::abs(walign - robot.w) < std::abs(vel_canal.v - robot.v)){
                //el robot tarda menos en alinearse que en frenar
                vel = AlineacionWObstacle(walign);
            }else{
                vel = ObjSinValleSinHueco();
            }
        }else{
            //no me da tiempo a frenar y evitar la colision, intento alinearme
            vel = AlineacionWObstacle(walign);
        }
    }else{
        fstrategies << "\tRobotFueraPoligono";
        if (std::abs(walign) > wlim_z2){
            vel = ObjSinValleSinHueco();
        }else{
            vel = AlineacionWObstacle(walign);
        }
    }

    return vel;
}


Velocidad Strategies::DetrasObjeto(double ang){
    // std::cout << "Strategy: behind object. CHANGES DIR GOAL" << std::endl;
    Velocidad vel;
    Velocidad robot = space.GetAgent();
    boundsVS bounds = space.GetBounds();
    float x[2], y[2];

    if (!IsDirGoalRadiusFree() && !IsVelInGoalSpace(robot, dir_goal)){
        //Recalcula el dir_goal en la zona libre más próxima al hueco más grande
        Velocidad velo;
        if (ang <= atan2(bounds.vlim_max, bounds.wmax_left)){
            velo.v = bounds.wmax_left*tan(ang);
            velo.w = bounds.wmax_left;
        }else if (ang <= atan2(bounds.vlim_max, bounds.wmax_right)){
            velo.v = bounds.vlim_max;
            velo.w = bounds.vlim_max/tan(ang);
        }else{
            velo.v = bounds.wmax_right*tan(ang);
            velo.w = bounds.wmax_right;
        }

        dir_goal = ComputeNewVel(velo);
    }

    //(21/04/2016): si el robot está muy desorientado, forzamos a que se oriente primero
    Tpf sol;
    transfor_inversa_p(goal.x, goal.y, &posAg, &sol);
    double distGoalRob = sol.x*sol.x + sol.y*sol.y;
    double steeringDir = space.GetGoal().velGoal.w;
    if (std::abs(steeringDir) > 1.5*wmax_steering && distGoalRob <= dist_medium2)
        vel = UnderDirGoal();
    else{

        //estrategia a seguir
        DW_Range dwRange;
        if (IntersectRangeDW_Goal(dwRange)){    //if (IntersectionDinGoal(cero, dir_goal)){

            if (distGoalRob < dist_near2){

                //DW_Range dwRange = DW_VelocityRange();	//Computes the range of velocities inside the DW that belongs to dir_goal radio
                if (!MaxFreeVelocityInRange(dwRange, vel)){

                    //No hay velocidad del radio goal dentro de la DW que sea libre
                    if (atan2(robot.v, robot.w) == 0){
                        if (dir_goal.w > 0) vel = dwAg.right;
                        else vel = dwAg.left;
                    }else{

                        if (atan2(robot.v, robot.w) < atan2(dir_goal.v, dir_goal.w)){

                            //(12/02/2016): Escojo la velocidad que más me acerca al goal
                            Tpf sol1, sol2, sol3;
                            //Tsc pos1 = PredictPosition(robot.v, robot.w);
                            //transfor_inversa_p(goal.x, goal.y, &pos1, &sol1);
                            //double tita1 = atan2(sol.y, sol.x);
                            Tsc pos2 = PredictPosition(dwAg.left.v, dwAg.left.w);
                            transfor_inversa_p(goal.x, goal.y, &pos2, &sol2);
                            double tita2 = atan2(sol2.y, sol2.x);
                            Tsc pos3 = PredictPosition(dwAg.up.v, dwAg.up.w);
                            transfor_inversa_p(goal.x, goal.y, &pos3, &sol3);
                            double tita3 = atan2(sol3.y, sol3.x);

                            if (std::abs(tita2) < std::abs(tita3)) vel = dwAg.left;
                            else vel = dwAg.up;

                        }else{

                            //(12/02/2016): Escojo la velocidad que más me acerca al goal
                            Tpf sol1, sol2, sol3;
                            //Tsc pos1 = PredictPosition(robot.v, robot.w);
                            //transfor_inversa_p(goal.x, goal.y, &pos1, &sol1);
                            //double tita1 = atan2(sol.y, sol.x);
                            Tsc pos2 = PredictPosition(dwAg.right.v, dwAg.right.w);
                            transfor_inversa_p(goal.x, goal.y, &pos2, &sol2);
                            double tita2 = atan2(sol2.y, sol2.x);
                            Tsc pos3 = PredictPosition(dwAg.up.v, dwAg.up.w);
                            transfor_inversa_p(goal.x, goal.y, &pos3, &sol3);
                            double tita3 = atan2(sol3.y, sol3.x);

                            if (std::abs(tita2) < std::abs(tita3)) vel = dwAg.right;
                            else vel = dwAg.up;
                        }
                    }
                }

            }else{

                //if (robot.v < bounds.vlim_max){
                if (std::abs(robot.v - bounds.InsideBoundsV(bounds.ComputeMaxV_W(robot.w))) > 1e-5){
                    vel = dwAg.up;
                }else{
                    //DW_Range dwRange = DW_VelocityRange();	//Computes the range of velocities inside the DW that belongs to dir_goal radio
                    if (!MaxFreeVelocityInRange(dwRange, vel)){
                        vel = dwAg.down;
                    }
                    //vel = VelDinGoal(dir_goal, bounds);
                }
            }

        }else{
            if (atan2(dir_goal.v, dir_goal.w) < atan2(robot.v, robot.w)){
                vel = dwAg.right;
            }else{
                vel = dwAg.left;
            }
        }
    }

    bool new_computed = false;
    if (pol.InsidePolygon(robot)){

        if (pol.InsidePolygon(vel)){

            //Buscamos si hay hueco para alineación, lo más cerca del robot
            double min_dist = 5; double w_selected; bool selected = false;
            for (int i=0; i<(int)walign.size(); i++){

                if (walign[i].first < waling_lim){	//hay_hueco_left
                    if (std::abs(walign[i].first - robot.w) < min_dist){
                        w_selected = walign[i].first;
                        min_dist = std::abs(walign[i].first - robot.w);
                        selected = true;
                    }
                }

                if (walign[i].second > -waling_lim){
                    if (std::abs(walign[i].second - robot.w) < min_dist){	//right
                        w_selected = walign[i].second;
                        min_dist = std::abs(walign[i].second - robot.w);
                        selected = true;
                    }
                }
            }

            //Estrategia alineacion o frenado
            if (selected){
                vel = ObjSinValleZ2ConHueco(w_selected);
            }else{
                vel = ObjSinValleSinHueco();
            }
            new_computed = true;
        }

    }

    //if (!new_computed) fstrategies << "DetrasObjeto(EvitandoObjeto)\t";

    return vel;
}

Tsc Strategies::PredictPosition_Kstepts(double v, double w, int k){
//Devuelve la posicion final a partir de la posicion actual del robot y la velocidad a aplicar en los siguientes k instantes de tiempo

    Tsc position = posAg;

    for (int i=0; i<k; i++){
        if (std::abs(w) < 1e-5){
            position.x = position.x + v*cos(position.tita)*stept;
            position.y = position.y + v*sin(position.tita)*stept;
        }
        else{	//w != 0
            position.x = position.x - (v/w)*sin(position.tita) + (v/w)*sin(position.tita+w*stept);
            position.y = position.y + (v/w)*cos(position.tita) - (v/w)*cos(position.tita+w*stept);
            position.tita = NormalisePI(position.tita + w*stept);
        }
    }

    return position;
}

Tsc Strategies::PredictPosition(double v, double w){
//Devuelve la posicion final a partir de la posicion actual del robot y la velocidad a aplicar en el siguiente
//instante de tiempo; a partir de dicho instante, el robot decelera hasta detenerse por completo

    Tsc position = posAg;
    double velV = v;

    while (true){
        if (std::abs(w) < 1e-5){ //0.00001 && w > -0.00001){
            position.x = position.x + velV*cos(position.tita)*stept;
            position.y = position.y + velV*sin(position.tita)*stept;
        }
        else{	//w != 0
            position.x = position.x - (velV/w)*sin(position.tita) + (velV/w)*sin(position.tita+w*stept);
            position.y = position.y + (velV/w)*cos(position.tita) - (velV/w)*cos(position.tita+w*stept);
            position.tita = NormalisePI(position.tita + w*stept);
        }

        if (velV > 0) velV = velV - space.GetConstraints().av*stept;
        else break;
    }
    return position;
}

Velocidad Strategies::ComputeNewVel(Velocidad vel){

    boundsVS bounds = space.GetBounds();

    double min_dist_ang = 2*M_PI;
    Velocidad velo = vel; double ang = atan2(vel.v, vel.w); //por defecto, si no encuentra hueco libre
    for (int k=0; k<(int)zonaLibres.size(); k++){
        //busco de entre las zonas libres aquel comando que este mas cerca de la velocidad vel
        ParAng z = zonaLibres[k];
        ParVel v = velLibres[k];
        if (std::abs(atan2(vel.v, vel.w) - z.ini) < min_dist_ang){
            min_dist_ang = std::abs(atan2(vel.v, vel.w) - z.ini);
            velo = v.ini;
            ang = atan2(velo.v, velo.w); // + cte_goal_valley;
        }
        if (std::abs(atan2(vel.v, vel.w) - z.fin) < min_dist_ang){
            min_dist_ang = std::abs(atan2(vel.v, vel.w) - z.fin);
            velo = v.fin;
            ang = atan2(velo.v, velo.w); // - cte_goal_valley;
        }
    }

    if (ang <= atan2(bounds.vlim_max, bounds.wmax_left)){
        velo.v = bounds.wmax_left*tan(ang);
        velo.w = bounds.wmax_left;
    }else if (ang <= atan2(bounds.vlim_max, bounds.wmax_right)){
        velo.v = bounds.vlim_max;
        velo.w = bounds.vlim_max/tan(ang);
    }else{
        velo.v = bounds.wmax_right*tan(ang);
        velo.w = bounds.wmax_right;
    }

    /*
    float x[2], y[2];
    x[0] = 0.0; y[0] = 0.0;
    x[1] = velo.w; y[1] = velo.v;
    DibujaRecta(4, x, y, 2, "blue");
    //*/

    return velo;
}

bool Strategies::IsDirGoalRadiusFree(){
//Función que devuelve true o false si el radio dir_goal es completamente libre,
//es decir, no está dentro de los límites de ningún DOV

    for (int i=0; i<(int)limites_DOV.size(); i++){

        std::pair<Command, Command> limite = limites_DOV[i];

        double alfa_goal = atan2(dir_goal.v, dir_goal.w);
        double alfa_first = atan2(limite.first.sup.vel.v, limite.first.sup.vel.w);
        double alfa_second = atan2(limite.second.sup.vel.v, limite.second.sup.vel.w);
        if (alfa_goal > alfa_first && alfa_goal < alfa_second)
            return false;

    }

    return true;

}

Velocidad Strategies::UnderDirGoal(){
    // std::cout << "Strategy: maintaining velocities under dir_goal, because robot is far away from goal" << std::endl;
//Estrategia de navegación que mantiene al robot a velocidades por debajo del dir_goal, cuando el robot está muy desorientado del goal
//Así se consiguen trayectorias de curvatura menor, para orientarse antes hacia el goal

    Velocidad vel;
    Velocidad robot = space.GetAgent();

    if (dir_goal.w > 0){
        // std::cout << "UNDERDIRGOAL: direction of goal is positive" << std::endl;

        if (atan2(robot.v, robot.w) < atan2(dir_goal.v, dir_goal.w)){	//trayectoria de arco de circunferencia menor que el radio goal
            // std::cout << "Arco menor" << std::endl;
            if (robot.w > worientacion){
                if (atan2(dwAg.left.v, dwAg.left.w) >= atan2(dir_goal.v, dir_goal.w)) {
                    vel = dwAg.down;
                    // vel = calculateVelocities(dir_goal, dwAg);
                }
                else vel = dwAg.left;
            }else{
                if (robot.v > vorientacion){
                    vel = dwAg.down;
                    // vel = calculateVelocities(dir_goal, dwAg);
                }else{
                    vel = dwAg.up;
                    if (dwAg.up.v > vorientacion){
                        vel.v = vorientacion; vel.w = robot.w;
                    }
                    if (atan2(vel.v, vel.w) >= atan2(dir_goal.v, dir_goal.w)){
                        if (dwAg.right.w > worientacion){
                            vel.v = robot.v; vel.w = worientacion;
                        }else vel = dwAg.right;
                    }
                }
            }

        }else{	//trayectoria de arco de circunferencia mayor que el radio goal
            // std::cout << "Arco menor" << std::endl; 
            if (robot.v > vorientacion) {
                // std::cout << "robot.v > vorientacion" << std::endl;
                vel = dwAg.down;
                // vel = calculateVelocities(dir_goal, dwAg);

            }
            else{
                // std::cout << "robot.v <= vorientacion" << std::endl;
                if (robot.w < worientacion){
                    if (dwAg.right.w > worientacion){
                        vel.v = robot.v; vel.w = worientacion;
                    }else vel = dwAg.right;
                }else {
                    vel = dwAg.down;
                    // vel = calculateVelocities(dir_goal, dwAg);

                }
            }

        }

    }else{
        // std::cout << "UNDERDIRGOAL: direction of goal is negative" << std::endl;
        if (atan2(robot.v, robot.w) > atan2(dir_goal.v, dir_goal.w)){	//trayectoria de arco de circunferencia menor que el radio goal

            if (robot.w < -worientacion){
                if (atan2(dwAg.right.v, dwAg.right.w) <= atan2(dir_goal.v, dir_goal.w)) {
                    vel = dwAg.down;
                    // vel = calculateVelocities(dir_goal, dwAg);

                }
                else vel = dwAg.right;
            }else{
                if (robot.v > vorientacion){
                    vel = dwAg.down;
                    // vel = calculateVelocities(dir_goal, dwAg);

                }else{
                    vel = dwAg.up;
                    if (dwAg.up.v > vorientacion){
                        vel.v = vorientacion; vel.w = robot.w;
                    }
                    if (atan2(vel.v, vel.w) <= atan2(dir_goal.v, dir_goal.w)){	//vel conduciria a trayectoria por encima del dir_goal
                        if (dwAg.left.w < -worientacion){
                            vel.v = robot.v; vel.w = -worientacion;
                        }else vel = dwAg.left;
                    }
                }
            }

        }else{	//trayectoria de arco de circunferencia mayor que el radio goal

            if (robot.v > vorientacion) {
                vel = dwAg.down;
                // vel = calculateVelocities(dir_goal, dwAg);

            }
            else{
                if (robot.w > -worientacion){
                    if (dwAg.left.w < -worientacion){
                        vel.v = robot.v; vel.w = -worientacion;
                    }else vel = dwAg.left;
                }else {
                    // vel = calculateVelocities(dir_goal, dwAg);
                    vel = dwAg.down;
                }
            }
        }
    }

    return vel;
}

Velocidad Strategies::ComputeVelGoalRobotEnZona(){
    // std::cout << "Strategy: computeVelGoalRobotEnZona (free velocities closest to the robot). CHANGES DIR GOAL" << std::endl;
    //Se selecciona la zona de velocidades libres más próxima al robot, y que menos tiempo le cuesta alcanzar

    Velocidad vGoal, vel;
    Velocidad robot = space.GetAgent();
    boundsVS bounds = space.GetBounds();

    double av = space.GetConstraints().av;
    double aw = space.GetConstraints().aw;

    double min_dist_ang = 0;
    double ang = atan2(dir_goal.v, dir_goal.w); vel = dir_goal;	//por defecto, si no encuentra hueco libre
    double tmin = 100;
    for (int k=0; k<(int)zonaLibres.size(); k++){
        ParAng z = zonaLibres[k];
        ParVel v = velLibres[k];
        //Comprobamos el tamaño de la zona con velocidades libres
        if ((v.ini.w == bounds.wmax_left && v.fin.w == bounds.wmax_left) || (v.ini.w == bounds.wmax_right && v.fin.w == bounds.wmax_right)){
            if (std::abs(v.ini.v - v.fin.v) < av*stept){ //1.5*aw*stept){
                continue;
            }
            if (std::abs(robot.v - v.ini.v) < std::abs(robot.v - v.fin.v)){
                vGoal = v.ini;
            }else{
                vGoal = v.fin;
            }

            if (robot.v < 1){ //Robot no va muy rápido
                if (std::abs(robot.v - vGoal.v)/(av*stept) < tmin){
                    vel = vGoal;
                    tmin = std::abs(robot.v - vGoal.v)/(av*stept);
                    ang = z.ini + (z.fin - z.ini)/2;
                }
            }else{
                if ((std::abs(robot.v - vGoal.v)/(av*stept) + std::abs(robot.w - vGoal.w)/(aw*stept)) < tmin){
                    vel = vGoal;
                    tmin = std::abs(robot.v - vGoal.v)/(aw*stept) + std::abs(robot.w - vGoal.w)/(aw*stept);
                    ang = z.ini + (z.fin - z.ini)/2;
                }
            }
        }else{
            if (std::abs(v.ini.w - v.fin.w) < aw*stept){
                continue;
            }
            if (std::abs(robot.w - v.ini.w) < std::abs(robot.w - v.fin.w)){
                vGoal = v.ini;
            }else{
                vGoal = v.fin;
            }

            if (robot.v < 1){ //Robot no va muy rápido
                if (std::abs(robot.w - vGoal.w)/(aw*stept) < tmin){
                    vel = vGoal;
                    tmin = std::abs(robot.w - vGoal.w)/(aw*stept);
                    ang = z.ini + (z.fin - z.ini)/2;
                }
            }else{
                if ((std::abs(robot.w - vGoal.w)/(aw*stept) + std::abs(robot.v - vGoal.v)/(av*stept)) < tmin){
                    vel = vGoal;
                    tmin = std::abs(robot.w - vGoal.w)/(aw*stept) + std::abs(robot.v - vGoal.v)/(av*stept);
                    ang = z.ini + (z.fin - z.ini)/2;
                }
            }
        }
    }

    if (ang <= atan2(bounds.vlim_max, bounds.wmax_left)){
        dir_goal.v = bounds.wmax_left*tan(ang);
        dir_goal.w = bounds.wmax_left;

    }else if (ang <= atan2(bounds.vlim_max, bounds.wmax_right)){
        dir_goal.v = bounds.vlim_max;
        dir_goal.w = bounds.vlim_max/tan(ang);
    }else{
        dir_goal.v = bounds.wmax_right*tan(ang);
        dir_goal.w = bounds.wmax_right;
    }
    vel = dir_goal;
    /*
    float x[2], y[2];
    x[0] = 0.0; y[0] = 0.0;
    x[1] = dir_goal.w; y[1] = dir_goal.v;
    DibujaRecta(4, x, y, 2, "blue");

    fstrategies << "GoalNoLibre\t";
    */

    return vel;
}

Velocidad Strategies::OrientacionGoalCanal(Velocidad canal){
    // std::cout << "Strategy: orientacion goal canal" << std::endl;
    Velocidad vel;
    Velocidad robot = space.GetAgent();

    Velocidad ini,fin;
    fin = dir_goal;
    bool computed = false;

    DW_Range dwRange;
    if (IntersectRangeDW_Goal(dwRange)){    //if (IntersectionDinGoal(ini, fin)){
        //(29/03/2016): seguimos el radio que lleva al goal
        //DW_Range dwRange = DW_VelocityRange();	//Computes the range of velocities inside the DW that belongs to dir_goal radio
        computed = MaxFreeVelocityInRange(dwRange, vel);
    }

    if (!computed){
        Velocidad command;

        if (robot.w >= 0.0){//cuadrante derecho, giros a izquierda
            if (dwAg.left.w < canal.w){
                command.v = dwAg.up.v + (dwAg.up.v/3.0); command.w = robot.w;
                if (pol.InsidePolygon(command)){
                    vel = robot;
                }else{
                    vel = dwAg.up;
                }

            }else{
                //comprobamos que el valor resultante de velocidad no cae en el objeto dinamico
                command.v = robot.v; command.w = dwAg.left.w;
                if (pol.InsidePolygon(command)){
                    vel = robot;
                }else{
                    vel = dwAg.left;
                }
            }
        }else{//cuadrante izquierdo, giros a derecha
            if (dwAg.right.w <= canal.w){
                //comprobamos que el valor resultante de velocidad no cae en el objeto dinamico
                command.v = robot.v; command.w = dwAg.right.w;
                if (pol.InsidePolygon(command)){
                    vel = robot;
                }else{
                    vel = dwAg.right;
                }
            }
            else{
                command.v = dwAg.up.v + (dwAg.up.v/3.0); command.w = robot.w;
                if (pol.InsidePolygon(command)){
                    vel = robot;
                }else{
                    vel = dwAg.up;
                }
            }
        }
    }

    return vel;
}


Velocidad Strategies::ObjSinValleSinHueco(){
    // std::cout << "Strategy: there are no velocities free in Z2. CHANGES DIR GOAL" << std::endl;
    //zona2 no tiene comandos de velocidad libres, está totalmente ocupado

    Velocidad vel;
    Velocidad robot = space.GetAgent();

    double steeringDir = space.GetGoal().velGoal.w;
    
    bool computed = false;
    if (pol.InsidePolygon(robot)){

        //Se recalcula el dir_goal en la zona de velocidades libres más próxima al robot, y que menos tiempo le cuesta alcanzar
        dir_goal = ComputeVelGoalRobotEnZona();

        infoValle valleGoal; Space spGoal;

        if (InValleyZ2(dir_goal, valleGoal) && ReachableValley(commands[valleGoal.ini], commands[valleGoal.piso], commands[valleGoal.fin])){

            vel = PassingBefore(commands[valleGoal.piso].sup.vel);
            computed = true;

        }else if (InSpaceDOV(dir_goal, spGoal) && ((std::abs(spGoal.vel.ini.w - spGoal.vel.fin.w) > space.GetConstraints().aw*stept) && ReachableSpace(spGoal))){

            vel = PassingBefore(spGoal.depth);
            computed = true;
        }
    }

    if (!computed){

        //Busco la velocidad del canal en torno a la que debe estar el robot
        Velocidad vel_canal; double t_canal;
        double v_canal = space.GetBounds().vlim_min;
        for (int i=0; i<(int)canal.size(); i++){
            //Nos quedamos con la velocidad de canal que tenga la velocidad lineal más alta
            if (canal[i].first.first.v > v_canal){
                vel_canal = canal[i].first.first;
                t_canal = canal[i].second.first;
                v_canal = vel_canal.v;
            }

            if (canal[i].first.second.v > v_canal){
                vel_canal = canal[i].first.second;
                t_canal = canal[i].second.second;
                v_canal = vel_canal.v;
            }
        }

        //Recalculo el dir_goal el la zona libre donde está la velocidad del canal
        dir_goal = ComputeNewVel(vel_canal);

        if (robot.v > vel_canal.v){

            if (robot.w > 0.0){	//lado derecho del VS, giros a izquierda

                if (dir_goal.w > 0){
                    if (pol.InsidePolygon(dwAg.left)){
                        //vel = down;
                        //23/02/2016: estrategia de acercamiento hacia el goal
                        if (dwAg.down.v > 0) vel = dwAg.down;
                        else{
                            //(12/02/2016): Me quedo con la que me acerca más al goal
                            Tpf sol1, sol2;
                            Tsc pos1 = PredictPosition(robot.v, robot.w);
                            transfor_inversa_p(goal.x, goal.y, &pos1, &sol1);
                            //double tita1 = atan2(sol1.y, sol1.x);
                            Tsc pos2 = PredictPosition(dwAg.down.v, dwAg.down.w);
                            transfor_inversa_p(goal.x, goal.y, &pos2, &sol2);

                            //Eligo la velocidad que más me acerca al goal
                            if (sol1.x < sol2.x) vel = robot;
                            else vel = dwAg.down;
                        }

                    }else{
                        vel = dwAg.left;
                    }
                }else{
                    // std::cout << "OBJSINVALLEHUECO" << std::endl;
                    vel = UnderDirGoal();
                }

            }else if (robot.w < 0.0){
                //compruebo si puedo hacer DIN_LATERAL, es decir, si al hacerlo no me meto en el obs

                if (dir_goal.w < 0){
                    if (pol.InsidePolygon(dwAg.right)){
                        if (dwAg.down.v > 0) vel = dwAg.down;
                        else{
                            //(12/02/2016): Me quedo con la que me acerca más al goal
                            Tpf sol1, sol2;
                            Tsc pos1 = PredictPosition(robot.v, robot.w);
                            transfor_inversa_p(goal.x, goal.y, &pos1, &sol1);

                            Tsc pos2 = PredictPosition(dwAg.down.v, dwAg.down.w);
                            transfor_inversa_p(goal.x, goal.y, &pos2, &sol2);

                            //Eligo la velocidad que más me acerca al goal
                            if (sol1.x < sol2.x) vel = robot;
                            else vel = dwAg.down;
                        }
                    }else{
                        vel = dwAg.right;
                    }
                }else{
                    // std::cout << "OBJSINVALLEHUECO" << std::endl;
                    vel = UnderDirGoal();
                }

            }else{
                // std::cout << "OBJSINVALLEHUECO" << std::endl;
                if (steeringDir > wmax_steering) vel = UnderDirGoal();
                else vel = robot;
            }

        }else{	//nos orientamos hacia el goal considerando la velocidad de canal
            vel = OrientacionGoalCanal(vel_canal);
        }

        fstrategies << "SlowingDown\t";

    }//else fstrategies << "PasarPrimero\t";

    return vel;
}

bool Strategies::NewDirGoalOutsideDOV(){
//Intentamos recalcular el dir_goal en algún hueco fuera del DOV dentro del rango de velocidades angulares en la misma dirección que el dir_goal,
//priorizando velocidades lineales altas

    // cout << "New dir goal outside dov" << endl;
    boundsVS bounds = space.GetBounds();

    //Delimitamos las zonas angulares preferentes en las que buscar velocidades libres dentro de valles y huecos dentro del DOV
    double ang_ini, ang_fin; Velocidad cmd_goal = space.GetGoal().commandGoal;
    double steeringDir = space.GetGoal().velGoal.w;
    if (steeringDir > wmax_steering){
        if (dir_goal.w > 0){
            ang_ini = atan2(0, bounds.wmax_left);
            ang_fin = atan2(cmd_goal.v, cmd_goal.w);
        }
        else if (dir_goal.w < 0){
            ang_fin = atan2(0, bounds.wmax_right);
            ang_ini = atan2(cmd_goal.v, cmd_goal.w);
        }else{
            ang_ini = atan2(cmd_goal.v, cmd_goal.w);
            ang_fin = ang_ini;
        }
    }else{
        if (dir_goal.w > 0){
            ang_ini = atan2(dir_goal.v, steeringDir);
            ang_fin = atan2(cmd_goal.v, cmd_goal.w);
        }
        else if (dir_goal.w < 0){
            ang_fin = atan2(dir_goal.v, steeringDir);
            ang_ini = atan2(cmd_goal.v, cmd_goal.w);
        }else{
            ang_ini = atan2(cmd_goal.v, cmd_goal.w);
            ang_fin = ang_ini;
        }
    }

    //Restringo los huecos fuera del DOV que estén dentro de los limites de algún DOV
    std::vector<Space> spaces_search;
    for (int i=0; i<(int)space_outDOV.size(); i++){

        Space sp = space_outDOV[i];
        //Elimino el rango que está dentro del DOV
        Space space_new = sp;
        for (int k=0; k<(int)limites_DOV.size(); k++){

            std::pair<Command, Command> dov = limites_DOV[k];

            double ang_ini = atan2(dov.first.sup.vel.v, dov.first.sup.vel.w);
            double ang_fin = atan2(dov.second.sup.vel.v, dov.second.sup.vel.w);

            //Comprobamos si la zona libre está dentro de los límites de algún DOV
            if (sp.ang.ini >= ang_ini && sp.ang.ini <= ang_fin){
                space_new.ang.ini = ang_fin;
            }

            if (sp.ang.fin >= ang_ini && sp.ang.fin <= ang_fin){
                space_new.ang.fin = ang_ini;
            }
        }
        spaces_search.push_back(space_new);
    }

    //Busco huecos fuera del DOV con velocidades en la misma dirección que el dir_goal
    bool haySpaceOut = false; double ang_new_goal = atan2(dir_goal.v, dir_goal.w);
    double aw = space.GetConstraints().aw;
    double min_width = atan2(bounds.vlim_max, aw*stept - aw*stept/4) - atan2(bounds.vlim_max, aw*stept);
    for (int i=0; i<(int)spaces_search.size(); i++){

        Space sp = spaces_search[i];

        //El hueco tiene cierta anchura
        if (std::abs(sp.ang.fin - sp.ang.ini) > min_width){
            //Comprobamos que el hueco sea alcanzable por el robot
            if (ReachableSpace(sp)){

                if (haySpaceOut){
                    //Me quedo con el más próximo al dir_goal
                    if (std::abs(sp.ang.ini - atan2(dir_goal.v, dir_goal.w)) < std::abs(ang_new_goal - atan2(dir_goal.v, dir_goal.w))){
                        ang_new_goal = sp.ang.ini;
                    }
                    if (std::abs(sp.ang.fin - atan2(dir_goal.v, dir_goal.w)) < std::abs(ang_new_goal - atan2(dir_goal.v, dir_goal.w))){
                        ang_new_goal = sp.ang.fin;
                    }
                }else{

                    int n = std::ceil(sp.width/(atan2(bounds.vlim_max, aw*stept) - atan2(bounds.vlim_max, 2*aw*stept)));
                    if (n < 2) n = 2;

                    if (std::abs(sp.ang.ini - atan2(dir_goal.v, dir_goal.w)) <
                        std::abs(sp.ang.fin - atan2(dir_goal.v, dir_goal.w))){
                        ang_new_goal = sp.ang.ini;
                    }else{
                        ang_new_goal = sp.ang.fin;
                    }

                    haySpaceOut = true;
                }

            }
        }
    }


    if (ang_new_goal <= atan2(bounds.vlim_max, bounds.wmax_left)){
        dir_goal.v = bounds.wmax_left*tan(ang_new_goal);
        dir_goal.w = bounds.wmax_left;
    }else if (ang_new_goal <= atan2(bounds.vlim_max, bounds.wmax_right)){
        dir_goal.v = bounds.vlim_max;
        dir_goal.w = bounds.vlim_max/tan(ang_new_goal);
    }else{
        dir_goal.v = bounds.wmax_right*tan(ang_new_goal);
        dir_goal.w = bounds.wmax_right;
    }

    /*
    float x[2], y[2];
    x[0] = 0.0; y[0] = 0.0;
    x[1] = dir_goal.w; y[1] = dir_goal.v;
    DibujaRecta(4, x, y, 2, "blue");

    //*/
    // fstrategies << "GoalNoLibre\t";

    return haySpaceOut;
}


Velocidad Strategies::NoHayVallesZ2(double ang_ini, double ang_fin){
    // std::cout << "Strategy: no Z2 valleys. CHANGES DIR GOAL" << std::endl;
    Velocidad vel;
    Velocidad robot = space.GetAgent();
    boundsVS bounds = space.GetBounds();

    if (!hay_comandos_z2){
        //no hay ningun comando del objeto dinamico que pertenezca a la zona2, ocupará la z1 o z3

        if (!IsDirGoalRadiusFree() && !IsVelInGoalSpace(robot, dir_goal)){
            //Recalcula el dir_goal en la zona libre más próxima al dir_goal original
            dir_goal = ComputeNewVel(dir_goal);
        }

        vel = FreeMotion(true);

    }else{	//hay comandos en la zona2 del VS

        bool hay_hueco_left = false; bool hay_hueco_right = false;

        if (pol.Size() > 1){
            //Hay más de un obstáculo: analizamos las zonas libres para saber si hay hueco

            if (!IsDirGoalRadiusFree() && !IsVelInGoalSpace(robot, dir_goal)){
                // cout << "NEW DIR NO VALLES Z2" << endl;
                NewDirGoal(false);	//Recalcular el dir_goal en espacios fuera de los DOV
            }

            vel = DetrasObjeto(atan2(dir_goal.v, dir_goal.w));

        }else{

            //double waling_lim = std::abs(wmax_left); // - 0.25*std::abs(wmax_left);
            double wleft = bounds.wmax_left; double wright = bounds.wmax_right;
            for (int i=0; i<(int)walign.size(); i++){
                // std::cout << "walign_left: " << walign[i].first << ", walign_right: " << walign[i].second << std::endl;
                if (walign[i].first < waling_lim){	//if (walign[i].first < 0.75){
                    hay_hueco_left = true; wleft = walign[i].first;
                }
                if (walign[i].second > -waling_lim){	//if (walign[i].second > -0.75){
                    hay_hueco_right = true; wright = walign[i].second;
                }
            }

            //(22/04/2016) INI: Añadimos condiciones adicionales para aprovechar más los huecos debajo de los limites del DOV si no hay hueco a ningún lado
            if (!hay_hueco_left && !hay_hueco_right){
                //Comprobamos que los limites donde empiezan los polígonos están en zona2 o próximos, dejando libres las zonas1 o 3 con más velocidades angulares
                hay_hueco_left = true; double alfa = atan2(bounds.vlim_max/3, bounds.wmax_left);
                for (int k=0; k<(int)limites_DOV.size(); k++){
                    std::pair<Command, Command> dov = limites_DOV[k];
                    double ang_ini = atan2(dov.first.sup.vel.v, dov.first.sup.vel.w);

                    if (ang_ini < alfa){
                        hay_hueco_left = false;
                        break;
                    }
                }


                //Comprobamos que los limites donde empiezan los polígonos están en zona2 o próximos, dejando libres las zonas1 o 3 con más velocidades angulares
                hay_hueco_right = true; double alfa_r = atan2(bounds.vlim_max/3, bounds.wmax_right);
                for (int k=0; k<(int)limites_DOV.size(); k++){
                    std::pair<Command, Command> dov = limites_DOV[k];
                    double ang_fin = atan2(dov.second.sup.vel.v, dov.second.sup.vel.w);

                    if (ang_fin > alfa_r){
                        hay_hueco_right = false;
                        break;
                    }
                }
            }
            //(22/04/2016) FIN

            if (!hay_hueco_left && !hay_hueco_right){

                //el objeto dinamico tiene sus comandos inicial y final en z1 y z3 respectivamente
                vel = ObjSinValleSinHueco();

            }else if (hay_hueco_left && hay_hueco_right){

                if (!IsDirGoalRadiusFree() && !IsVelInGoalSpace(robot, dir_goal)){
                    NewDirGoalOutsideDOV();
                }

                vel = DetrasObjeto(atan2(dir_goal.v, dir_goal.w));

            }else if (hay_hueco_left){

                if (ang_ini > ang_left){
                    vel = DetrasObjeto(ang_ini);
                }else{
                    if (ang_fin < atan2(bounds.vlim_max/3, bounds.wmax_right)){
                        vel = DetrasObjeto(ang_ini);
                    }else{
                        vel = ObjSinValleZ2ConHueco(wleft);
                    }
                }
            }else{

                if (ang_fin < ang_right){ //angulo_right=93 grados
                    vel = DetrasObjeto(ang_fin);
                }else{
                    if (ang_ini > atan2(bounds.vlim_max/3, bounds.wmax_left)){	//atan2(vlim_max, wmax_left)){
                        //vel = EvitandoObjeto();
                        vel = DetrasObjeto(ang_fin);
                    }else{
                        vel = ObjSinValleZ2ConHueco(wright);
                    }
                }
            }

        }
    }
    return vel;
}


Velocidad Strategies::SelectStrategy(){
    // std::cout << "Strategy: selecting strategy" << std::endl;
    Velocidad vel;

    infoValle valleGoal; Space spGoal;

    double aw = space.GetConstraints().aw;
    double min_width = aw*stept/4;
    //Comprobamos que el dir_goal no esté en zona libre dentro de algún valle o hueco no alcanzables
    if ( (InValleyZ2(dir_goal, valleGoal) && !ReachableValley(commands[valleGoal.ini], commands[valleGoal.piso], commands[valleGoal.fin])) ||
         (InSpaceDOV(dir_goal, spGoal) && !((std::abs(spGoal.vel.ini.w - spGoal.vel.fin.w) > min_width) && ReachableSpace(spGoal))) )
        // cout << "NEW DIR SELECT STRATEGY" << endl;
        NewDirGoal(false);

    if (InValleyZ2(dir_goal, valleGoal) && ReachableValley(commands[valleGoal.ini], commands[valleGoal.piso], commands[valleGoal.fin])){

        vel = PassingBefore(commands[valleGoal.piso].sup.vel);

    }else if (InSpaceDOV(dir_goal, spGoal) && ((std::abs(spGoal.vel.ini.w - spGoal.vel.fin.w) > min_width) && ReachableSpace(spGoal))){

        vel = PassingBefore(spGoal.depth);

    }else{
        // std::cout << "No hay valles" << std::endl;
        vel = NoHayVallesZ2(atan2(commands[0].sup.vel.v, commands[0].sup.vel.w),
                              atan2(commands[commands.size()-2].sup.vel.v, commands[commands.size()-2].sup.vel.w));
    }

    return vel;
}

void Strategies::Evaluate_DWMaxAccCommands(){
//Evaluate the different possible commands of the robot for next stept

    DW_Info data;

    dw.clear();

    data.vel.w = dwAg.up.w; data.vel.v = dwAg.up.v > space.GetBounds().vlim_max ? space.GetBounds().vlim_max : dwAg.up.v ; dw.push_back(data);
    data.vel.w = dwAg.up.w; data.vel.v = dwAg.down.v < space.GetBounds().vlim_min ? space.GetBounds().vlim_min : dwAg.down.v ; dw.push_back(data);
    data.vel.v = dwAg.left.v; data.vel.w = dwAg.left.w < -maxAngularVelocity(data.vel.v) ? -maxAngularVelocity(data.vel.v): dwAg.left.w; dw.push_back(data);
    data.vel.v = dwAg.right.v; data.vel.w = dwAg.right.w > maxAngularVelocity(data.vel.v) ? maxAngularVelocity(data.vel.v): dwAg.right.w; dw.push_back(data);
    data.vel = space.GetAgent(); dw.push_back(data);
    // data.vel = calculateVelocities(Velocidad(0, 1.5), dwAg); data.vel.w = data.vel.w > maxAngularVelocity(data.vel.v) ? maxAngularVelocity(data.vel.v): data.vel.w; dw.push_back(data);
    // data.vel = calculateVelocities(Velocidad(0, -1.5), dwAg); data.vel.w = data.vel.w < -maxAngularVelocity(data.vel.v) ? -maxAngularVelocity(data.vel.v): data.vel.w; dw.push_back(data);

    //Evaluamos cada uno de los posibles movimientos del robot, dentro de su ventana dinámica
    for (int i=0; i<(int)dw.size(); i++){
        DW_Info data = dw[i];

        //double vGoal = vs->GetSteeringW() * vs->GetRadioGoal(); double wGoal = vs->GetSteeringW();

        Velocidad vGoal = space.GetGoal().velGoal;
        if (!pol.InsidePolygon(data.vel) && IsSafeVelocity(data.vel)){
            data.inCollision = false;
            //if (IsSafeVelocity(data.vel))	//da tiempo de que el robot frene sin colisionar // or it is not a safe velocity, then compute the instant when the collision occurs

            //Computes the position of the robot after applying the new velocity during the next time stept and then it starts breaking until it stops
            Tsc pos = PredictPosition(data.vel.v, data.vel.w);
            Tpf sol;
            transfor_inversa_p(goal.x, goal.y, &pos, &sol);
            data.titaGoal = atan2(sol.y, sol.x);
            data.distGoal = sqrt(sol.x*sol.x + sol.y*sol.y);

            //Computes the position of the robot after applying the new velocity during the next time stept
            pos = PredictPosition_Kstepts(data.vel.v, data.vel.w, 1);
            transfor_inversa_p(goal.x, goal.y, &pos, &sol);
            data.titaGoal_next = atan2(sol.y, sol.x);
            data.distGoal_next = sqrt(sol.x*sol.x + sol.y*sol.y);

            //Computes the distance and orientation with respect to the velocity in the VS
            data.titaVS = abs(vGoal.w - data.vel.w);
            data.distVS = sqrt((vGoal.w - data.vel.w) * (vGoal.w - data.vel.w) + (vGoal.v - data.vel.v) * (vGoal.v - data.vel.v));

        }else{
            data.inCollision = true;
        }

        dw[i] = data;
    }
}

Velocidad Strategies::AlineacionGoal2(){
    // std::cout << "Strategy: alligning with goal 2" << std::endl;   
    Velocidad vel;
    Velocidad robot = space.GetAgent();
    boundsVS bounds = space.GetBounds();
    
    //Compruebo si el dir_goal está en algún valle
    bool recalcular = false;
    bool enValle = false; infoValle valleGoal;
    for (int i=0; i<(int)valles_z2.size(); i++){

        infoValle valle = valles_z2[i];
        if (atan2(dir_goal.v, dir_goal.w) >= atan2(commands[valle.ini].sup.vel.v, commands[valle.ini].sup.vel.w) &&
            atan2(dir_goal.v, dir_goal.w) <= atan2(commands[valle.fin].sup.vel.v, commands[valle.fin].sup.vel.w)){

            if (ReachableValley(commands[valle.ini], commands[valle.piso], commands[valle.fin])){
                enValle = true;
                valleGoal = valle;
                break;
            }
        }
    }

    if (enValle){
        // std::cout << "DIR GOAL IS IN A VALLEY" << std::endl;
        fstrategies << "PasarPrimero\t";
        //Ir a por el valle
        vel = PassingBefore(commands[valleGoal.piso].sup.vel);
        return vel;
    }

    //Estrategia AlineacionGoal2
    Tpf goal_rob;
    transfor_inversa_p(goal.x, goal.y, &posAg, &goal_rob);
    double distGoalRob = goal_rob.x*goal_rob.x + goal_rob.y*goal_rob.y;    
    if (distGoalRob < dist_near2){
        // std::cout << "ROBOT IS LESS THEN HALF A METER FROM GOAL" << std::endl;
        //Si el robot está a menos de medio metro del goal, sigo el ComandoGoal directamente y no el GoalMedio
        
        Velocidad dir_goalAnt = dir_goal;
        dir_goal = space.GetGoal().commandGoal;
        if (!IsVelocityFree(dir_goal)) dir_goal = dir_goalAnt;

    }

    //if (vs->GetSteeringDir() > wmax_steering) vel = UnderDirGoal();
    double steeringDir = space.GetGoal().velGoal.w; double radioGoal = space.GetGoal().velGoal.v / space.GetGoal().velGoal.w;
    if ((steeringDir > wmax_steering || distGoalRob > dist_medium2) && radioGoal != 0) {
        // std::cout << "STEERING TO GOAL IS MORE THEN MAXIMUM" << std::endl;
        // std::cout << "alineaciongoal2" << std::endl;
        vel = UnderDirGoal();
    }
    else{
        // std::cout << "STEERING TO GOAL IS OK" << std::endl;
        DW_Range dwRange;
        if (!IntersectRangeDW_Goal(dwRange)){   //if (!IntersectionDinGoal(Velocidad(), dir_goal)){
            // std::cout << "NOT IntersectRangeDW_Goal" << std::endl;
            if (robot.v > dir_goal.v){
                // std::cout << "ROBOT VELOCITY IS HIGHER THEN DIR_GOAL: " << dir_goal.w << std::endl;
                // andrew
                vel = dwAg.down;
                // vel = calculateVelocities(dir_goal, dwAg);

            }else{
                // std::cout << "ROBOT VELOCITY IS LOWER THEN DIR_GOAL" << std::endl;
                if (dir_goal.w > robot.w){	//HABRIA QUE TENER EN CUENTA WL WMAXSTEERING O ALINEARSE SEGÚN INTERSECTION CON EL DIR_GOAL
                    // std::cout << "ROBOT ANGULAR VELOCITY IS LOWER THEN DIR_GOAL" << std::endl;
                    vel = dwAg.right;
                }else if (dir_goal.w < robot.w){
                    // std::cout << "ROBOT ANGULAR VELOCITY IS HIGHER THEN DIR_GOAL" << std::endl;
                    vel = dwAg.left;
                }else{
                    // std::cout << "ROBOT LINEAL VELOCITY IS HIGHER THEN DIR_GOAL" << std::endl;
                    if (dwAg.up.v <= dir_goal.v){
                        vel = dwAg.up;
                    }else{
                        vel = VelDinGoal(dir_goal, bounds);
                    }
                }
            }

        }else{
            // std::cout << "YES IntersectRangeDW_Goal" << std::endl;
            //DW_Range dwRange = DW_VelocityRange();	//Computes the range of velocities inside the DW that belongs to dir_goal radio
            if (!MaxFreeVelocityInRange(dwRange, vel)){	//Si no encuentra velocidad libre en el rango
                // std::cout << "NO FREE VELOCITIES INSIDE THE DW" << std::endl;
                if (dir_goal.w > 0 && (atan2(robot.v, robot.w) >= atan2(dir_goal.v, dir_goal.w) && robot.w < bounds.wmax_left)){
                    // std::cout << "GOAL RADIUS POSITIVE, TURN LEFT" << std::endl;
                    //Radio goal positivo, giro hacia la izquierda; el radio que describe el robot es mayor que el radio al goal
                    vel = dwAg.right;
                }else if(dir_goal.w < 0 && (atan2(robot.v, robot.w) <= atan2(dir_goal.v, dir_goal.w) && robot.w > bounds.wmax_right)){
                    // std::cout << "GOAL RADIUS NEGATIVE, TURN RIGHT" << std::endl;
                    //Radio goal negativo, giro hacia la derecha; el radio que describe el robot es menor que el radio al goal
                    vel = dwAg.left;
                }else{
                    // std::cout << "GOAL RADIUS NONE??, TURN LEFT" << std::endl;
                    if (std::abs(dir_goal.w) > std::abs(robot.w)){
                        vel = dwAg.up;
                    }else{
                        if (robot.v < dir_goal.v){
                                // std::cout << "Aqui" << std::endl;
                            if (dwAg.down.v >= dir_goal.v){
                                // vel = calculateVelocities(dir_goal, dwAg);
                                vel = dwAg.down;
                            }else{
                                vel = VelDinGoal(dir_goal, bounds);
                            }
                        }else{
                            if (dwAg.up.v <= dir_goal.v){
                                vel = dwAg.up;
                            }else{
                                vel = VelDinGoal(dir_goal, bounds);
                            }
                        }
                    }
                }
            }
        }

        //El robot puede frenar después de aplicar el comando vel sin colisionar
        if (pol.InsidePolygon(vel) && IsSafeVelocity(vel)) estrategia_valle = true;
    }

    fstrategies << "AlineacionGoal2\t";

    return vel;
}

Velocidad Strategies::Estrategia_escaparBC(){
    // std::cout << "Strategy: escape collision band" << std::endl;
    //Seguir el goal recalculado en zona fuera de la banda de colision
    Velocidad vel;
    Velocidad robot = space.GetAgent();
    boundsVS bounds = space.GetBounds();

    Velocidad ini;
    Velocidad fin(dir_goal.v, dir_goal.w);

    //(23/09/2016): si el robot está muy desorientado y cerca del goal, forzamos a que baje la velocidad y para que no se desoriente y se pase del goal
    Tpf sol;
    transfor_inversa_p(goal.x, goal.y, &posAg, &sol);
    double distGoalRob = sol.x*sol.x + sol.y*sol.y;
    double steeringDir = space.GetGoal().velGoal.w;
    if (steeringDir > 1.5*wmax_steering && distGoalRob <= dist_medium2)	{//dist_near2
        // std::cout << "escaparBC" << std::endl;
        vel = UnderDirGoal();
    } else{

        DW_Range dwRange;
        if (IntersectRangeDW_Goal(dwRange)){   //if (IntersectionDinGoal(ini, fin)){

            //DW_Range dwRange = DW_VelocityRange();	//Computes the range of velocities inside the DW that belongs to dir_goal radio
            vel = dwRange.ini;

        }else{
            if (robot.w > dir_goal.w){
                // std::cout << "Escape left" << std::endl;
                vel = dwAg.left;
            }else if (robot.w < dir_goal.w){
                // std::cout << "Escape right, robot w=" << robot.w << ", goal w=" << dir_goal.w << std::endl;
                vel = dwAg.right;
            }else{
                if (robot.v > dir_goal.v){
                    // std::cout << "Escape down" << std::endl;
                    vel = dwAg.down;
                }else{
                    if (robot.v < dir_goal.v)	vel = dwAg.up;
                    else vel = VelDinGoal(dir_goal, bounds);
                }
            }
        }

        // andrew
        if (!IsSafeVelocity(vel)) {
            // std::cout << "BC NOT SAFE" << std::endl;
            if (IsSafeVelocity(dwAg.up)) {
                // std::cout << "ACCELERATING!!" << std::endl;
                vel = dwAg.up;
            } else if (IsSafeVelocity(dwAg.down)) {
                vel = dwAg.down;
            } else {
                vel = dwAg.up;
            }
        }
    }
    fstrategies << "AvoidingCollision\t";

    return vel;

}

Velocidad Strategies::EscapeCB(bool originInside){
    // std::cout << "Strategy: escape collision band 2. CHANGES DIR GOAL" << std::endl;
    Velocidad vel;
    Velocidad robot = space.GetAgent();
    boundsVS bounds = space.GetBounds();
    double aw = space.GetConstraints().aw;

    bool hay_hueco_left = false; bool hay_hueco_right = false;
    for (int i=0; i<(int)walign.size(); i++){
        if (walign[i].first < waling_lim) hay_hueco_left = true;	//if (walign[i].first < 0.75) hay_hueco_left = true;
        if (walign[i].second > -waling_lim) hay_hueco_right = true;	//if (walign[i].second > -0.75) hay_hueco_right = true;
    }

    double wh = 0.27;
    if (hay_comandos_z2 && !zona2 && !hay_hueco_left && !hay_hueco_right){
        //la zona2 está totalmente ocupada
        // std::cout << "La zona 2 esta totalmente ocupada" << std::endl;

        //Se selecciona la zona de velocidades libres más próxima al robot, que menos tiempo cuesta alcanzar
        Velocidad dir_goal_ant = dir_goal;
        Velocidad vGoal;
        dir_goal = ComputeVelGoalRobotEnZona();
        if (dir_goal.v == dir_goal_ant.v && dir_goal.w == dir_goal_ant.w) dir_goal = ComputeNewVel(robot);

        vGoal = dir_goal;
        //(22/01/2016): Intentamos restringir la dir_goal.w a la wh, si está libre de todos los objetos dinámicos
        if (std::abs(vGoal.w) > wh){
            //Recalculamos el goal restringiendo el valor de w
            Line r1,r2; Tpf pto1, pto2;
            pto1.x = 0; pto1.y = 0; pto2.x = vGoal.v; pto2.y = vGoal.w;
            r1 = Line(pto1, pto2);
            if (vGoal.w > 0){ pto1.x = wh; pto1.y = 0; pto2.x = wh; pto2.y = vGoal.v;}
            else {pto1.x = -wh; pto1.y = 0; pto2.x = -wh; pto2.y = vGoal.v;}
            r2 = Line(pto1, pto2);
            Tsc raiz; int sol;
            SolDosRectas(r1, r2, raiz, sol);
            Velocidad v(raiz.y, raiz.x);
            if (!pol.InsidePolygon(v)){
                vGoal = v;
                /*
                float x[2], y[2];
                x[0] = 0.0; y[0] = 0.0;
                x[1] = vGoal.w; y[1] = vGoal.v;
                DibujaRecta(4, x, y, 2, "blue");
                */
            }
        }

        //vel = Estrategia_escaparBC();

        Velocidad ini;
        Velocidad fin(dir_goal.v, dir_goal.w);
        DW_Range dwRange;
        if (IntersectRangeDW_Goal(dwRange)){   //if (IntersectionDinGoal(ini, fin)){

            //DW_Range dwRange = DW_VelocityRange();	//Computes the range of velocities inside the DW that belongs to dir_goal radio
            vel = dwRange.ini;

        }else{
            if (robot.w > dir_goal.w){
                vel = dwAg.left;
            }else if (robot.w < dir_goal.w){
                vel = dwAg.right;
            }else{
                if (robot.v > dir_goal.v){
                    vel = dwAg.down;
                }else{
                    if (robot.v < dir_goal.v)	vel = dwAg.up;
                    else vel = VelDinGoal(dir_goal, bounds);
                }
            }
        }


        if (hay_comandos_z1 && !zona1 && hay_comandos_z3 && !zona3) {} fstrategies << "Certain Collision\t";

    }else{
            // std::cout << "Hay hueco en Zona2" << std::endl;
        //Si el dir_goal no está en zona libre ni en zona heurística, recalcularlo
        if (!(IsVelocityFree(dir_goal) && (atan2(dir_goal.v, dir_goal.w) >= atan2(1.5, wh) && atan2(dir_goal.v, dir_goal.w) <= atan2(1.5, -wh)))){
            // if (IsVelocityFree(dir_goal)) std::cout << "GOAL SI ESTA EN ZONA LIBRE PERO NO EN LA ZONA OPTIMA: " << dir_goal.v << ", " << dir_goal.w << std::endl;
            // else std::cout << "GOAL NO ESTA EN ZONA LIBRE" << std::endl;
            //Delimitamos las zonas angulares preferentes en las que buscar velocidades libres dentro de valles y huecos dentro del DOV
            double ang_ini, ang_fin; Velocidad cmd_goal = space.GetGoal().commandGoal;
            double steeringDir = space.GetGoal().velGoal.w;
            if (steeringDir > wmax_steering){
                if (dir_goal.w > 0){
                    ang_ini = atan2(0, bounds.wmax_left);
                    ang_fin = atan2(cmd_goal.v, cmd_goal.w);
                }
                else if (dir_goal.w < 0){
                    ang_fin = atan2(0, bounds.wmax_right);
                    ang_ini = atan2(cmd_goal.v, cmd_goal.w);
                }else{
                    ang_ini = atan2(cmd_goal.v, cmd_goal.w);
                    ang_fin = ang_ini;
                }
            }else{
                if (dir_goal.w > 0){
                    ang_ini = atan2(dir_goal.v, steeringDir);
                    ang_fin = atan2(cmd_goal.v, cmd_goal.w);
                }
                else if (dir_goal.w < 0){
                    ang_fin = atan2(dir_goal.v, steeringDir);
                    ang_ini = atan2(cmd_goal.v, cmd_goal.w);
                }else{
                    ang_ini = atan2(cmd_goal.v, cmd_goal.w);
                    ang_fin = ang_ini;
                }
            }


            double ang = atan2(dir_goal.v, dir_goal.w);
            bool hayValle = false; bool computed = false;
            if (!valles_z2.empty()){
                // std::cout << "valleys z2 are not empty" << std::endl;
                //Comprobamos si hay algún valle que contenga una zona de velocidades en la misma dirección que el dir_goal
                double alfa_wh_pos = atan2(1.5, wh); double alfa_wh_neg = atan2(1.5, -wh);
                infoValle valleGoal; double dist = M_PI;
                for (int i=0; i<(int)valles_z2.size(); i++){

                    infoValle valle = valles_z2[i];
                    double ang_valle_ini = atan2(commands[valle.ini].sup.vel.v, commands[valle.ini].sup.vel.w);
                    double ang_valle_fin = atan2(commands[valle.fin].sup.vel.v, commands[valle.fin].sup.vel.w);

                    //El valle empieza o termina en la zona de velocidades hacia el dir_goal, o el valle contiene toda la zona
                    if ((ang_valle_ini >= ang_ini && ang_valle_ini <= ang_fin) || (ang_valle_fin >= ang_ini && ang_valle_fin <= ang_fin) ||
                        (ang_valle_ini <= ang_ini && ang_valle_fin >= ang_fin)){

                        if (alfa_wh_pos >= ang_valle_ini && alfa_wh_neg <= ang_valle_fin){
                            //la zona definida en la heurística es libre de colision
                            if (std::abs(atan2(dir_goal.v, dir_goal.w) - atan2(1.5, wh)) < std::abs(atan2(dir_goal.v, dir_goal.w) - atan2(1.5, -wh))){
                                ang = atan2(1.5, wh);
                            }else{
                                ang = atan2(1.5, -wh);
                            }
                            dist = std::abs(atan2(dir_goal.v, dir_goal.w) - ang);
                            hayValle = true;
                        }else if (alfa_wh_pos <= ang_valle_ini && alfa_wh_neg > ang_valle_ini  && alfa_wh_neg <= ang_valle_fin){
                            if (std::abs(atan2(dir_goal.v, dir_goal.w) - atan2(1.5, -wh)) < dist){
                                ang = atan2(1.5, -wh);
                                dist = std::abs(atan2(dir_goal.v, dir_goal.w) - ang);
                                hayValle = true;
                            }
                        }else if (alfa_wh_pos >= ang_valle_ini && alfa_wh_pos < ang_valle_fin  && alfa_wh_neg >= ang_valle_fin){
                            if (std::abs(atan2(dir_goal.v, dir_goal.w) - atan2(1.5, wh)) < dist){
                                ang = atan2(1.5, wh);
                                dist = std::abs(atan2(dir_goal.v, dir_goal.w) - ang);
                                hayValle = true;
                            }
                        }else if (alfa_wh_pos <= ang_valle_ini && alfa_wh_neg >= ang_valle_fin){
                            if (std::abs(atan2(dir_goal.v, dir_goal.w) - (ang_valle_ini + std::abs(ang_valle_fin-ang_valle_ini)/2)) < dist){
                                ang = ang_valle_ini + std::abs(ang_valle_fin-ang_valle_ini)/2;
                                dist = std::abs(atan2(dir_goal.v, dir_goal.w) - ang);
                                hayValle = true;
                            }
                        }else{
                            //Los límites que delimitan el valle están fuera de la zona heurística
                            //Sin embargo, existe la posibilidad de zona de velocidades libres si la profundidad del valle están entre los límites +/-wh de la zona heurística
                            //Sólo puede interesar recalcular el dir_goal en dicho hueco si el robot no está dentro de la BC de más de un obstáculo (ya que al salir de una
                            //banda se le cerraria el hueco) y si el robot está bastante desorientado del goal (la zona de velocidades angulares en las que está el dir_goal no
                            //está dentro de la zona heurística)
                            if (ang_ini > alfa_wh_neg || ang_fin < alfa_wh_pos){	//zona angular de velocidades está fuera de la zona heurística

                                std::vector<std::unique_ptr<Agent>>::iterator it; bool en_zona = false; bool consider = false;
                                /*
                                for (it=agents->begin(); it!=agents->end(); ++it){
                                    if ((*it).GetConsider() && (*it).GetEnZona()){
                                        if (!originInside){
                                            consider = true;
                                            en_zona = true;
                                        }else{
                                            consider = false;
                                            break;
                                        }
                                    }
                                }
                                */
                                if (consider && std::abs(commands[valle.piso].sup.vel.w) < wh){
                                    double ang_valle_piso = atan2(commands[valle.piso].sup.vel.v, commands[valle.piso].sup.vel.w);
                                    if (std::abs(atan2(dir_goal.v, dir_goal.w) - ang_valle_piso) < dist){
                                        ang = ang_valle_piso;
                                        dist = std::abs(atan2(dir_goal.v, dir_goal.w) - ang);
                                        hayValle = true;
                                    }
                                }
                            }
                        }
                    }

                }
            }
            
            // andrew
            // estrategia_valle = hayValle;

            double min_width = atan2(bounds.vlim_max, aw*stept - aw*stept/4) - atan2(bounds.vlim_max, aw*stept);
            if (!hayValle){
                // std::cout << "No hay valle" << std::endl;
                //Busco huecos dentro del DOV en zonas de velocidades en la misma dirección que el dir_goal
                bool haySpace = false; double dist = M_PI;
                Space spaceGoal;
                for (int i=0; i<(int)space_inDOV.size(); i++){

                    Space sp = space_inDOV[i];

                    //El hueco empieza o termina en la zona de velocidades hacia el dir_goal, o el hueco contiene toda la zona
                    if ((sp.ang.ini >= ang_ini && sp.ang.ini <= ang_fin) || (sp.ang.fin >= ang_ini && sp.ang.fin <= ang_fin) ||
                        (sp.ang.ini <= ang_ini && sp.ang.fin >= ang_fin)){

                        //El hueco tiene cierta anchura
                        if (std::abs(sp.ang.fin - sp.ang.ini) > min_width){

                            if (atan2(1.5, wh) >= sp.ang.ini && atan2(1.5, -wh) <= sp.ang.fin){
                                //la zona definida en la heurística es libre de colision
                                if (std::abs(atan2(dir_goal.v, dir_goal.w) - atan2(1.5, wh)) < std::abs(atan2(dir_goal.v, dir_goal.w) - atan2(1.5, -wh))){
                                    ang = atan2(1.5, wh);
                                }else{
                                    ang = atan2(1.5, -wh);
                                }
                                dist = std::abs(atan2(dir_goal.v, dir_goal.w) - ang);
                                haySpace = true;
                            }else if (atan2(1.5, wh) <= sp.ang.ini && atan2(1.5, -wh) > sp.ang.ini  && atan2(1.5, -wh) <= sp.ang.fin){
                                if (std::abs(atan2(dir_goal.v, dir_goal.w) - atan2(1.5, -wh)) < dist){
                                    ang = atan2(1.5, -wh);
                                    dist = std::abs(atan2(dir_goal.v, dir_goal.w) - ang);
                                    haySpace = true;
                                }
                            }else if (atan2(1.5, wh) >= sp.ang.ini && atan2(1.5, wh) < sp.ang.fin  && atan2(1.5, -wh) >= sp.ang.fin){
                                if (std::abs(atan2(dir_goal.v, dir_goal.w) - atan2(1.5, wh)) < dist){
                                    ang = atan2(1.5, wh);
                                    dist = std::abs(atan2(dir_goal.v, dir_goal.w) - ang);
                                    haySpace = true;
                                }
                            }else if (atan2(1.5, wh) <= sp.ang.ini && atan2(1.5, -wh) >= sp.ang.fin){
                                if (std::abs(atan2(dir_goal.v, dir_goal.w) - (sp.ang.ini + std::abs(sp.ang.fin - sp.ang.ini)/2)) < dist){
                                    ang = sp.ang.ini + std::abs(sp.ang.fin - sp.ang.ini)/2;
                                    dist = std::abs(atan2(dir_goal.v, dir_goal.w) - ang);
                                    haySpace = true;
                                }
                            }else{
                                //Los límites que delimitan el valle están fuera de la zona heurística
                                //Sin embargo, existe la posibilidad de zona de velocidades libres si la profundidad del valle están entre los límites +/-wh de la zona heurística
                                //Sólo puede interesar recalcular el dir_goal en dicho hueco si el robot no está dentro de la BC de más de un obstáculo (ya que al salir de una
                                //banda se le cerraria el hueco) y si el robot está bastante desorientado del goal (la zona de velocidades angulares en las que está el dir_goal no
                                //está dentro de la zona heurística)
                                if (ang_ini > atan2(1.5, -wh) || ang_fin < atan2(1.5, wh)){	//zona angular de velocidades está fuera de la zona heurística

                                    std::vector<std::unique_ptr<Agent>>::iterator it; bool en_zona = false; bool consider = false;
                                    /*
                                    for (it=agents->begin(); it!=agents->end(); ++it){
                                        if ((*it).GetConsider() && (*it).GetEnZona()){
                                            if (!en_zona){
                                                consider = true;
                                                en_zona = true;
                                            }else{
                                                consider = false;
                                                break;
                                            }
                                        }
                                    }
                                     */
                                    if (consider && std::abs(sp.depth.w) < wh){
                                        double ang_valle_piso = atan2(sp.depth.v, sp.depth.w);
                                        if (std::abs(atan2(dir_goal.v, dir_goal.w) - ang_valle_piso) < dist){
                                            ang = ang_valle_piso;
                                            dist = std::abs(atan2(dir_goal.v, dir_goal.w) - ang);
                                            haySpace = true;
                                        }
                                    }
                                }
                            }

                        }
                    }
                }

                if (!haySpace){	//No hay ninguna zona libre que contenga velocidades en la dirección del goal que estén en zona heurística
                    // std::cout << "No hay space" << std::endl;
                    double dist = M_PI;
                    for (int k=0; k<(int)zonaLibres.size(); k++){

                        ParAng z = zonaLibres[k];
                        ParVel v = velLibres[k];

                        if (atan2(1.5, wh) >= z.ini && atan2(1.5, -wh) <= z.fin){
                            //la zona definida en la heurística es libre de colision
                            //Comprobamos el tamaño de la zona con velocidades libres
                            if (std::abs(v.ini.w - v.fin.w) < aw*stept){
                                continue;
                            }

                            if (std::abs(atan2(robot.v, robot.w) - atan2(1.5, wh)) < std::abs(atan2(robot.v, robot.w) - atan2(1.5, -wh))){
                                ang = atan2(1.5, wh);
                            }else{
                                ang = atan2(1.5, -wh);
                            }
                            dist = std::abs(atan2(robot.v, robot.w) - ang);
                            computed = true;
                        }else if (atan2(1.5, wh) <= z.ini && atan2(1.5, -wh) > z.ini  && atan2(1.5, -wh) <= z.fin){
                            //Comprobamos el tamaño de la zona con velocidades libres
                            if (std::abs(v.ini.w - v.fin.w) < aw*stept){
                                continue;
                            }
                            if (std::abs(atan2(robot.v, robot.w) - atan2(1.5, -wh)) < dist){
                                ang = atan2(1.5, -wh);
                                dist = std::abs(atan2(robot.v, robot.w) - ang);
                                computed = true;
                            }
                        }else if (atan2(1.5, wh) >= z.ini && atan2(1.5, wh) < z.fin  && atan2(1.5, -wh) >= z.fin){
                            //Comprobamos el tamaño de la zona con velocidades libres
                            if (std::abs(v.ini.w - v.fin.w) < aw*stept){
                                continue;
                            }
                            if (std::abs(atan2(robot.v, robot.w) - atan2(1.5, wh)) < dist){
                                ang = atan2(1.5, wh);
                                dist = std::abs(atan2(robot.v, robot.w) - ang);
                                computed = true;
                            }
                        }else if (atan2(1.5, wh) <= z.ini && atan2(1.5, -wh) >= z.fin){
                            //Comprobamos el tamaño de la zona con velocidades libres
                            if (std::abs(v.ini.w - v.fin.w) < aw*stept){
                                continue;
                            }
                            if (std::abs(atan2(dir_goal.v, dir_goal.w) - (z.ini + std::abs(z.fin - z.ini)/2)) < dist){
                                ang = z.ini + std::abs(z.fin - z.ini)/2;
                                dist = std::abs(atan2(dir_goal.v, dir_goal.w) - ang);
                                computed = true;
                            }
                        }
                    }
                    if (!computed){
                        // std::cout << "NOt computed" << std::endl;
                        //Se selecciona la zona de velocidades libres más próxima al robot, que menos tiempo cuesta alcanzar
                        Velocidad dir_goal_ant = dir_goal;
                        Velocidad dir_Goal;
                        //Velocidad vGoal = ComputeVelGoalRobotEnZona();
                        dir_goal = ComputeVelGoalRobotEnZona();
                        if (dir_goal.v == dir_goal_ant.v && dir_goal.w == dir_goal_ant.w) dir_goal = ComputeNewVel(robot);

                        dir_Goal = dir_goal;

                        //(22/01/2016): Intentamos restringir la dir_goal.w a la wh, si está libre de todos los objetos dinámicos
                        if (std::abs(dir_Goal.w) > wh){
                            //Recalculamos el goal restringiendo el valor de w
                            Line r1,r2; Tpf pto1, pto2;
                            pto1.x = 0; pto1.y = 0; pto2.x = dir_Goal.w; pto2.y = dir_Goal.v;
                            r1 = Line(pto1, pto2);
                            if (dir_Goal.w > 0){ pto1.x = wh; pto1.y = 0; pto2.x = wh; pto2.y = dir_Goal.v;}
                            else {pto1.x = -wh; pto1.y = 0; pto2.x = -wh; pto2.y = dir_Goal.v;}
                            r2 = Line(pto1, pto2);
                            Tsc raiz; int sol;
                            SolDosRectas(r1, r2, raiz, sol);
                            Velocidad v(raiz.y, raiz.x);
                            if (!pol.InsidePolygon(v)){
                                dir_Goal = v;
                                /*
                                float x[2], y[2];
                                x[0] = 0.0; y[0] = 0.0;
                                x[1] = dir_Goal.w; y[1] = dir_Goal.v;
                                DibujaRecta(4, x, y, 2, "blue");
                                */
                            }
                        }
                        ang = atan2(dir_Goal.v, dir_Goal.w);
                        computed = true;
                    }
                }
            }

            if (!computed){
                if (ang <= atan2(bounds.vlim_max, bounds.wmax_left)){
                    dir_goal.v = bounds.wmax_left*tan(ang);
                    dir_goal.w = bounds.wmax_left;
                }else if (ang <= atan2(bounds.vlim_max, bounds.wmax_right)){
                    dir_goal.v = bounds.vlim_max;
                    dir_goal.w = bounds.vlim_max/tan(ang);
                }else{
                    dir_goal.v = bounds.wmax_right*tan(ang);
                    dir_goal.w = bounds.wmax_right;
                }
                /*
                float x[2], y[2];
                x[0] = 0.0; y[0] = 0.0;
                x[1] = dir_goal.w; y[1] = dir_goal.v;
                DibujaRecta(4, x, y, 2, "blue");
                */
            }

        }

        vel = Estrategia_escaparBC();
    }

    return vel;
}

Velocidad Strategies::ReachGoal(){
    // std::cout << "Strategy: reach goal" << std::endl;
    //Strategy to reach the goal, the robot is close to it
    //The objects around the robot are considered for AvoidingCollision but the priority is to reach the goal

    Velocidad vel;
    Velocidad robot = space.GetAgent();
    boundsVS bounds = space.GetBounds();
    
    Tpf sol;
    transfor_inversa_p(goal.x, goal.y, &posAg, &sol);
    double distGoalRob = sol.x*sol.x + sol.y*sol.y;
    if (IsVelInZone2(dir_goal) && distGoalRob >= dist_near2){
        DW_Range dwRange;
        if (IntersectRangeDW_Goal(dwRange)){   //if (IntersectionDinGoal(cero, dir_goal)){

            //DW_Range dwRange = DW_VelocityRange();	//Computes the range of velocities inside the DW that belongs to dir_goal radio
            if (!MaxFreeVelocityInRange(dwRange, vel)){

                if (dwAg.up.v <= dir_goal.v){
                    if (robot.v > bounds.vlim_max){
                        vel = dwAg.down;
                    }else if (robot.v < bounds.vlim_max){	//dinamica arriba
                        vel = dwAg.up;
                    }else{	//dinamica goal_media
                        vel = VelDinGoal(dir_goal, bounds); //radio medio entre r_goal y steering_dir
                    }
                }else{
                    if (robot.v > bounds.vlim_min){
                        vel = dwAg.down;
                    }else{
                        if (dir_goal.w > robot.w){	//HABRIA QUE TENER EN CUENTA WL WMAXSTEERING O ALINEARSE SEGÚN INTERSECTION CON EL DIR_GOAL
                            vel = dwAg.right;
                        }else if (dir_goal.w < robot.w){
                            vel = dwAg.left;
                        }else{
                            if (dwAg.up.v <= dir_goal.v){
                                vel = dwAg.up;
                            }else{
                                vel = VelDinGoal(dir_goal, bounds);
                            }
                        }
                    }
                }
            }
        }else{
            if (robot.v > dir_goal.v){
                vel = dwAg.down;
            }else{
                if (robot.v > bounds.vlim_max){
                    vel = dwAg.down;
                }else{
                    if (dir_goal.w > robot.w){	//HABRIA QUE TENER EN CUENTA WL WMAXSTEERING O ALINEARSE SEGÚN INTERSECTION CON EL DIR_GOAL
                        vel = dwAg.right;
                    }else if (dir_goal.w < robot.w){
                        vel = dwAg.left;
                    }else{
                        if (dwAg.up.v <= dir_goal.v){
                            vel = dwAg.up;
                        }else{
                            vel = VelDinGoal(dir_goal, bounds);
                        }
                    }
                }
            }
        }
    }else{

        //Alineacion porque está muy desorientado del goal
        DW_Range dwRange;
        if (!IntersectRangeDW_Goal(dwRange)){   //if (!IntersectionDinGoal(cero, dir_goal)){

            if (robot.v > dir_goal.v){
                vel = dwAg.down;
                if (pol.InsidePolygon(vel)){
                    if (dir_goal.w > robot.w){	//HABRIA QUE TENER EN CUENTA WL WMAXSTEERING O ALINEARSE SEGÚN INTERSECTION CON EL DIR_GOAL
                        vel = dwAg.right;
                        //Intento mover el robot hacia el lado contrario, si está libre
                        if (pol.InsidePolygon(vel)){
                            vel = dwAg.left;
                        }
                    }else if (dir_goal.w < robot.w){
                        vel = dwAg.left;
                        //Intento mover el robot hacia el lado contrario, si está libre
                        if (pol.InsidePolygon(vel)){
                            vel = dwAg.right;
                        }
                    }
                }
            }else{
                if (dir_goal.w > robot.w){
                    vel = dwAg.right;
                    if (pol.InsidePolygon(vel)){
                        vel = dwAg.down;
                    }
                }else if (dir_goal.w < robot.w){
                    vel = dwAg.left;
                    if (pol.InsidePolygon(vel)){
                        vel = dwAg.down;
                    }
                }else{
                    if (dwAg.up.v <= dir_goal.v){
                        vel = dwAg.up;
                    }else{
                        vel = VelDinGoal(dir_goal, bounds);
                    }
                }
            }

        }else{

            //DW_Range dwRange = DW_VelocityRange();	//Computes the range of velocities inside the DW that belongs to dir_goal radio
            if (!MaxFreeVelocityInRange(dwRange, vel)){

                // std::cout << "No hay rango libre" << std::endl;

                if (dir_goal.w > 0 && (atan2(robot.v, robot.w) >= atan2(dir_goal.v, dir_goal.w) && robot.w < bounds.wmax_left)){
                    //Radio goal positivo, giro hacia la izquierda; el radio que describe el robot es mayor que el radio al goal
                    vel = dwAg.right;
                    if (pol.InsidePolygon(vel)){
                        vel = dwAg.down;
                    }
                }else if(dir_goal.w < 0 && (atan2(robot.v, robot.w) <= atan2(dir_goal.v, dir_goal.w) && robot.w > bounds.wmax_right)){
                    //Radio goal negativo, giro hacia la derecha; el radio que describe el robot es menor que el radio al goal
                    vel = dwAg.left;
                    if (pol.InsidePolygon(vel)){
                        vel = dwAg.down;
                    }
                }else{
                    if (std::abs(dir_goal.w) > std::abs(robot.w)){
                        vel = dwAg.up;
                    }else{
                        if (robot.v > dir_goal.v){
                            if (dwAg.down.v >= dir_goal.v){
                                vel = dwAg.down;
                            }else{
                                vel = VelDinGoal(dir_goal, bounds);
                            }
                        }else{
                            if (dwAg.up.v <= dir_goal.v){
                                vel = dwAg.up;
                            }else{
                                vel = VelDinGoal(dir_goal, bounds);
                            }
                        }
                    }
                }
            }
        }
    }

    return vel;
}

Velocidad Strategies::FollowGoal(Tpf goal){
    //Decide the goal to be reached

    Velocidad followGoal;
    double min_dist = 2;
    double dist = std::sqrt(std::pow(goal.x, 2) + std::pow(goal.y, 2));
    // cout << "Dist follow goal: " << dist << endl;
    if (space.GetBounds().IsInsideBounds(space.GetGoal().velGoal)){
        // cout << 1 << endl;
        followGoal = space.GetGoal().velGoal;
    }else{
        if (dist < min_dist){
            // cout << "Chose commandGoal: " << space.GetGoal().commandGoal.v << ", " << space.GetGoal().commandGoal.w << endl;
            followGoal = space.GetGoal().commandGoal;
        }
        else{
            // cout << "Chose dirGoal: " << space.GetGoal().dirGoal.v << ", " << space.GetGoal().dirGoal.w << endl;
            followGoal = space.GetGoal().dirGoal;
        }
    }

    return followGoal;
}

// int debug_counter = 0;
bool Strategies::ComputeMotion(int agId, Tsc posR, Tpf posG, std::vector<std::unique_ptr<Agent>> *agentsIn, bool originInside, Velocidad &motion){

    //motion = space.GetAgent();
    dir_goal = space.GetGoal().dirGoal;
    NewDirGoal(false);
    space.SetDirGoal(dir_goal);
    // if (dir_goal.v == 0 && dir_goal.w == 0){
    //     NewDirGoal(true);
    //     space.SetDirGoal(dir_goal);
    // }
    // cout << "FIRST: Motion at the beginning of compute motion: " << atan2(dir_goal.v, dir_goal.w) << std::endl; 

    commands.resize(comm_max*obst_max);
    commands = space.GetCommands();
    // cout << "COMMANDS: " << commands.size() << endl;
    for (auto c:commands){
        // cout << "COM: " << c.sup.vel.v << ", " << c.sup.vel.w << " --- " << c.inf.vel.v << ", " << c.inf.vel.w << endl;
    }
    limites_DOV = space.GetBoundsDOV();
    posAg = posR;
    goal = posG;
    agents = agentsIn;
    idAg = agId;

    //Construct the polygon(s) with the different velocity points
    pol.InsertCommands(commands);

    //dir_goal = space.GetGoal().dirGoal;
    //Goal with respect to the agent
    Tpf goalAg;
    transfor_inversa_p(goal.x, goal.y, &posR, &goalAg);
    // cout << "Motion before followGoal: " << atan2(dir_goal.v, dir_goal.w) << std::endl;
    // printf("GOAL USED: %f, %f\n", goalAg.x, goalAg.y);
    dir_goal = FollowGoal(goalAg);
    // cout << "Motion after followGoal: " << atan2(dir_goal.v, dir_goal.w) << std::endl;

    Velocidad velAg = space.GetAgent();
    boundsVS bounds = space.GetBounds();

    dwAg = DW(velAg, space.GetConstraints(), bounds, stept);
    //right = dwAg.right; left = dwAg.left; up = dwAg.up; down = dwAg.down;
    //right = (velAg.w + space.GetConstraints().aw*stept > bounds.wmax_left) ? Velocidad(velAg.v, bounds.wmax_left) : Velocidad(velAg.v, space.GetAgent().w + space.GetConstraints().aw*stept);
    //left = (velAg.w - space.GetConstraints().aw*stept < bounds.wmax_right) ? Velocidad(velAg.v, bounds.wmax_right) : Velocidad(velAg.v, space.GetAgent().w - space.GetConstraints().aw*stept);
    //up = (velAg.v + space.GetConstraints().av*stept > bounds.vlim_max) ? Velocidad(bounds.vlim_max, velAg.w) : Velocidad(velAg.v + space.GetConstraints().av*stept,velAg.w);
    //down = (velAg.v - space.GetConstraints().av*stept < bounds.vlim_min) ? Velocidad(bounds.vlim_min, velAg.w) : Velocidad(velAg.v - space.GetConstraints().av*stept, velAg.w);

    // andrew
    estrategia_valle = false;
    // estrategia_valle = true;

    AnalyseCommands();

    ComputeSpaces();	//classifies the hollows in space_outDOV, space_inDOV y valles_z2 (03/05/2016)
    
    // andrew
    float distance = sqrt(pow(this->goal.x - this->posAg.x, 2) + pow(this->goal.y - this->posAg.y, 2));
    float distance2 = distance;
    float dif = dwAg.equal.v - dwAg.down.v;
    float currentV = dwAg.equal.v;
    while (currentV > 0) {
        distance -= currentV*stept;
        currentV -= dif;
    }



    //Compute the best navigation strategy
    if (commands.empty()){
        // std::cout << "COMMANDS ARE EMPTY, APPLYING FREE MOTION " << std::endl;
        dir_goal = space.GetGoal().commandGoal;
        NewDirGoal(true);
        motion = FreeMotion(true);
    }else{
        // std::cout << "There are commands " << debug_counter++ << std::endl;
        //28/09/2016: si el robot está orientado al goal (dir_goal.w = 0, wrobot ~ 0) y están en hueco libre, entonces mantenerse en el goal
        if (IsVelocityFree(dir_goal) && IsVelInGoalSpace(velAg, dir_goal) && dir_goal.w == 0 && velAg.w == dir_goal.w){
            //Seguir DirGoal
            // std::cout << "Robot is alligned with goal and there is free space, carrying on towards goal" << std::endl;
            motion = FreeMotion(false);
            return true;
        }
        // andrew: devuelve siempre false, implementacion sencilla
        //Comprobamos la distancia entre el robot y el goal, en el espacio de velocidad
        // if (ApproachToGoal(posR, goal, velAg, space.GetGoal().velGoal, bounds)){
        //     // std::cout << "Robot is close to goal, prioritizing goal" << std::endl;
        //     Tpf sol;
        //     transfor_inversa_p(goal.x, goal.y, &posR, &sol);

        //     if (!IsVelocityFree(dir_goal)){
        //         // std::cout << "Can't move directly, computing new goal direction outside of DOV" << std::endl;
        //         NewDirGoalOutsideDOV();
        //     }else{
        //         //comprueba que el radio goal está en zona de velocidades libre pero está dentro de los límites de algún DOV,
        //         //y que el robot está dentro de la misma zona de velocidades libres
        //         if (!IsDirGoalRadiusFree() && !IsVelInGoalSpace(velAg, dir_goal)){
        //             dir_goal = space.GetGoal().commandGoal;

        //             if (!IsDirGoalRadiusFree() && velAg.v < 0.5){
        //                 //Recalcula el dir_goal en la zona libre más próxima al dir_goal original
        //                 //dir_goal = ComputeNewVel(dir_goal);
        //                 NewDirGoalOutsideDOV();
        //             }
        //         }
        //     }

        //     motion = ReachGoal();

        // 
        
        // braking
        if (distance <= 0.05 || brake) {
            // std::cout << "distance: " << distance << ", brake: " << brake << std::endl;
            brake = true;
            // std::cout << "BRAKING!!" << std::endl;
            motion = dwAg.down;
        } else {
            brake = false;
            if (!originInside){
                // std::cout << "Not inside collision band, tan dir: "<< atan2(dir_goal.v, dir_goal.w) << std::endl;
                //Seleccionamos la mejor estrategia en función del espacio de velocidades ocupado por los obstáculos y la dirección hacia el goal

                bool goalFree = true; bool recalcular = false;
                if (!IsVelocityFree(dir_goal)){
                    // std::cout << "Goal direction is not free, computing new goal direction" << std::endl;
                    recalcular = true;
                    goalFree = NewDirGoal(false);
                }
                else{
                    // cout << "GOAL DIRECTION FREE" << endl;
                }

                /*//(26/04/2016)
                if (IsVelInZone2(dir_goal)){

                    Tpf sol;
                    transfor_inversa_p(goal.x, goal.y, &posRobot, &sol);
                    double distGoalRob = sol.x*sol.x + sol.y*sol.y;
                    if (distGoalRob > dist_near2){
                        goalFree = ComputeDirGoal();
                    }else{
                        if (!EsDirGoalFree()){
                            goalFree = ComputeNewDirGoal(dir_goal);
                        }
                    }

                }else{
                    goalFree = EsDirGoalFree();
                }
                //(26/04/2016)*/

                if (IsVelInZone2(dir_goal)){	//dir_goal en zona2, zona de velocidades lineales altas en el VS
                    // std::cout << "Motion to goal is in Zone 2" << std::endl;
                    motion = SelectStrategy();
                }else{
                    // std::cout << "Motion to goal is not in Zone 2" << std::endl;
                    motion = AlineacionGoal2();
                    // cout << "Dir goal at the end of AlineacionGoal2: " << atan2(dir_goal.v, dir_goal.w) << endl;
                }

                //Si el comando es de colision, comprobar si puede saltar a velocidades libres
                if (pol.InsidePolygon(motion) && !estrategia_valle){
                    // std::cout << "New command will create a collision" << std::endl;
                    Velocidad prueba; double aw = space.GetConstraints().aw; double av = space.GetConstraints().av;
                    // if (motion == dwAg.right){ prueba.v = motion.v; prueba.w = motion.w + aw*stept;}
                    // else if (motion == dwAg.left){ prueba.v = motion.v; prueba.w = motion.w - aw*stept;}
                    // else if (motion == dwAg.down){ prueba.v = motion.v - av*stept; prueba.w = motion.w;}
                    // else if (motion == dwAg.up){ prueba.v = motion.v + av*stept; prueba.w = motion.w;}
                    // else prueba = motion;
                    // andrew: prueba
                    if (motion == velAg) prueba = motion;
                    else if (motion == dwAg.right){ prueba.v = motion.v; prueba.w = ((motion.w + aw*stept > maxAngularVelocity(motion.v)) ? maxAngularVelocity(motion.v) : motion.w + aw*stept);}// std::cout << "MOTION: RIGHT, for V=" << motion.v << " max angular velocity: " << maxAngularVelocity(motion.v) << std::endl;}
                    else if (motion == dwAg.left){ prueba.v = motion.v; prueba.w = ((motion.w - aw*stept < -maxAngularVelocity(motion.v)) ? -maxAngularVelocity(motion.v) : motion.w - aw*stept);} // std::cout << "MOTION: LEFT" << std::endl;}
                    else if (motion == dwAg.down){ prueba.v = ((motion.v - av*stept < bounds.vlim_min) ? bounds.vlim_min : motion.v - av*stept); prueba.w = motion.w; } // std::cout << "MOTION: DOWN" << std::endl;}
                    else if (motion == dwAg.up){ prueba.v = ((motion.v + av*stept > bounds.vlim_max) ? bounds.vlim_max : motion.v + av*stept); prueba.w = motion.w; } // std::cout << "MOTION: UP" << std::endl;}
                    else {prueba = motion; }// std::cout << "MOTION: OTHER" << std::endl;}

                    // andrew: experimento, comprobar si es seguro ademas de si esta en poligono
                    if(!pol.InsidePolygon(prueba)) {
                        // std::cout << "ESTRATEGIA VALLE = TRUE" << std::endl;
                        estrategia_valle = true;
                    } else {
                        // std::cout << "NO HAY VALLE" << std::endl;
                    }
                    /*
                    else{
                        //Evaluamos los commands de aceleracion máxima de la DW
                        Evaluate_DWMaxAccCommands();

                        //La velocidad que representa el movimiento optimo está ocupada
                        double minDist = 10000; double minTita = 10000;
                        Velocidad minDist_vel = vel;
                        Velocidad minTita_vel = vel;
                        //Select the velocity in the dynamic window following a criterion from the above
                        bool minDist_selected = false; bool minTita_selected = false;
                        for (int i=0; i<(int)dw.size(); i++){
                            DW_Info data = dw[i];

                            if (!data.inCollision && data.distGoal_next < minDist){
                                minDist_vel = data.vel;
                                minDist = data.distGoal_next;
                                minDist_selected = true;
                                // std::cout << "Velocidad nueva: " << data.inCollision << "; vel: " << vel.v << ", " << vel.w << std::endl;
                                //std::cin.get();
                            }

                            if (!data.inCollision && std::abs(data.titaGoal_next) < minTita){
                                minTita_vel = data.vel;
                                minTita = std::abs(data.titaGoal_next);
                                minTita_selected = true;
                                // std::cout << "Velocidad nueva: " << data.inCollision << "; vel: " << vel.v << ", " << vel.w << std::endl;
                                //std::cin.get();
                            }
                        }

                        //if (minTita_selected){
                            //La velocidad que lleva el robot tiene la misma dirección que el dir_goal
                            if ((dir_goal.w >=0 && robot.w >= 0 && robot.v > 0) || (dir_goal.w <=0 && robot.w <= 0 && robot.v > 0)) vel = minDist_vel;
                                //if (minDist_selected) vel = minDist_vel;	//Elijo el comando que más me acerca al goal
                            else vel = minTita_vel;

                            //}
                        //}
                    }
                    //*/
                }
            }else{
                // std::cout << "Inside collision band" << std::endl;
                motion = EscapeCB(originInside);

                if (pol.InsidePolygon(motion)){
                    // std::cout << "Escape motion is inside polygon" << std::endl;
                    Velocidad prueba;
                    double aw = space.GetConstraints().aw; double av = space.GetConstraints().av;
                    if (motion == velAg) prueba = motion;
                    // andrew: confusion entre limites?
                    // else if (motion == dwAg.right){ prueba.v = motion.v; prueba.w = ((motion.w + aw*stept > bounds.wmax_left) ? bounds.wmax_left : motion.w + aw*stept); // std::cout << "MOTION: RIGHT" << std::endl;}
                    // else if (motion == dwAg.left){ prueba.v = motion.v; prueba.w = ((motion.w - aw*stept < bounds.wmax_right) ? bounds.wmax_right : motion.w - aw*stept); // std::cout << "MOTION: LEFT" << std::endl;}
                    else if (motion == dwAg.right){ prueba.v = motion.v; prueba.w = ((motion.w + aw*stept > maxAngularVelocity(motion.v)) ? maxAngularVelocity(motion.v) : motion.w + aw*stept); } // std::cout << "MOTION: RIGHT, for V=" << motion.v << " max angular velocity: " << maxAngularVelocity(motion.v) << std::endl;}
                    else if (motion == dwAg.left){ prueba.v = motion.v; prueba.w = ((motion.w - aw*stept < -maxAngularVelocity(motion.v)) ? -maxAngularVelocity(motion.v) : motion.w - aw*stept); } // std::cout << "MOTION: LEFT" << std::endl;}
                    else if (motion == dwAg.down){ prueba.v = ((motion.v - av*stept < bounds.vlim_min) ? bounds.vlim_min : motion.v - av*stept); prueba.w = motion.w; } // std::cout << "MOTION: DOWN" << std::endl;}
                    else if (motion == dwAg.up){ prueba.v = ((motion.v + av*stept > bounds.vlim_max) ? bounds.vlim_max : motion.v + av*stept); prueba.w = motion.w; } // std::cout << "MOTION: UP" << std::endl;}
                    else {prueba = motion;} // std::cout << "MOTION: OTHER" << std::endl;}

                    if(!pol.InsidePolygon(prueba)) {
                        // std::cout << "ESTRATEGIA VALLE = TRUE" << std::endl;
                        estrategia_valle = true;
                    } else {
                        // std::cout << "NO HAY VALLE" << std::endl;
                    }
                }
            }
        }

        NewDirGoal(false);

        // andrew

        if (!pol.InsidePolygon(motion) && IsSafeVelocity(motion)){
            // std::cout << "New motion is not inside polygon and safe " << motion.v << std::endl;
            //Apply command
            //if (estrategia_frenado) estrategia_frenado = false;
        }else if (!pol.InsidePolygon(motion) && !IsSafeVelocity(motion)){

            // std::cout << "New motion is not inside polygon and isn't safe" << std::endl;
            // andrew
            if (false && pol.InsidePolygon(velAg)){
                // std::cout << "New motion is not inside polygon but not safe either, keeping new motion " << std::endl;
                //(16/09/2016) Si no está haciendo estrategia de valle, entonces asegurar que coge una velocidad en un hueco seguro (aunque por aceleración no sería posible)
                //if (!estrategia_valle) vel = SelectVelInSpace();

                //if (estrategia_frenado) vel = down;
                //Aplicar comando, seguir con las estrategias
            }else{
                //No aplicar nuevo comando, mantener el mismo
                // andrew: mirar mas estrategias al evadir
                if (IsSafeVelocity(velAg)){
                    // std::cout << "Same velocity is safe, keeping old command" << std::endl;
                        if (IsSafeVelocity(dwAg.up)) {
                            motion = dwAg.up;
                        } else if (IsSafeVelocity(dwAg.left)) {
                            motion = dwAg.left;
                        } else if (IsSafeVelocity(dwAg.right)) {
                            motion = dwAg.right;
                        } else if (IsSafeVelocity(dwAg.down)) {
                            motion = dwAg.down;
                        } else {
                            motion = velAg;
                        }

                    // std::cout << "New motion is not inside polygon but not safe either, same motion is safe so not applying new motion " << std::endl;
                    // std::cout << "Safe velocity new" << std::endl;
                    //estrategia_frenado = false;
                }else{
                    // std::cout << "New motion is not inside polygon but not safe either, same motion is not safe so calculating new motion " << std::endl;
                    // std::cout << "Not safe velocity new" << std::endl;
                    //vel = down;	//o cualquier otro comando que sea seguro
                    //estrategia_frenado = true;

                    //Evaluamos los commands de aceleracion máxima de la DW
                    Evaluate_DWMaxAccCommands();
                    // std::cout << "Evaluating max accelerating commands" << std::endl;
                    //La velocidad que representa el movimiento optimo está ocupada
                    double minDist = 10000; double minTita = 10000;
                    Velocidad minDist_vel, minTita_vel;
                    // Select the velocity in the dynamic window following a criterion from the above
                    bool selected = false;
                    for (int i=0; i<(int)dw.size(); i++){
                        DW_Info data = dw[i];

                        if (!data.inCollision && data.distGoal_next < minDist){
                            minDist_vel = data.vel;
                            minDist = data.distGoal_next;
                            selected = true;
                        }

                        if (!data.inCollision && std::abs(data.titaGoal_next) < minTita){
                            minTita_vel = data.vel;
                            minTita = std::abs(data.titaGoal_next);
                            selected = true;
                        }
                    }

                    //Puede darse el caso que todas las velocidad sean prohibidas: definir una estrategia de evasión hacia velocidades libres lo más próximas al robot
                    if (!selected){
                        // std::cout << "Didnt find velocity" << std::endl;
                        //(16/09/2016) Asegurar que coge una velocidad en un hueco seguro (aunque por aceleración no sería posible) en lugar de frenar o salir de BC
                        //vel = SelectVelInSpace();
                        ///*
                        if (!originInside){
                            motion = dwAg.down;
                            //estrategia_frenado = true;
                        }else{
                            //vel = vel_ant;
                            // andrew
                            //dir_goal = ComputeNewVel(velAg);
                            // std::cout << "ESCAPANDO DEL BC!! " << std::endl;
                            motion = Estrategia_escaparBC();
                        }//*/
                    }else{
                        // std::cout << "Found a velocity" << std::endl;
                        //La velocidad que lleva el robot tiene la misma dirección que el dir_goal
                        if ((dir_goal.w >=0 && velAg.w >= 0) || (dir_goal.w <=0 && velAg.w <= 0)) motion = minDist_vel;
                        else motion = minTita_vel;
                    }
                }
            }
        }else if (pol.InsidePolygon(motion)){ // && !IsSafeVelocity(vel)){
            // std::cout << "New motion is inside polygon" << std::endl;
            //if (estrategia_frenado) estrategia_frenado = false;
            // std::cout << "Inside polygon" << std::endl;
            // Diego quitado estrategia valle
            if (estrategia_valle && false){
                // TODO: WHEN DOES ESTRATEGIA VALLE = TRUE? andrew
                // std::cout << "Applying valley strategy" << std::endl;
                //Apply command
            }else{
                // std::cout << "Not applying valley strategy" << std::endl;
                if (!pol.InsidePolygon(velAg)){
                    // Andrew, mirar velocidades proximas?
                    // std::cout << "Current velocity is not inside polygon, maintaining this motion" << std::endl;
                    // std::cout << "Not inside polygon new velocity" << std::endl;
                    // motion = FreeMotion();
                    // andrew ????
                    // if (!pol.InsidePolygon(calculateVelocities(dir_goal, dwAg))) {
                    //     motion = calculateVelocities(dir_goal, dwAg);
                    // } else {
                        motion = velAg;
                    // }
                }else{
                    // std::cout << "Current velocity is inside polygon, computing new motion" << std::endl;
                    // std::cout << "Inside polygon new velocity" << std::endl;
                    //Evaluamos los commands de aceleracion máxima de la DW
                    Evaluate_DWMaxAccCommands();

                    //La velocidad que representa el movimiento optimo está ocupada
                    double minDist = 10000; double minTita = 10000;
                    Velocidad minDist_vel, minTita_vel;
                    //Select the velocity in the dynamic window following a criterion from the above
                    bool selected = false;
                    for (int i=0; i<(int)dw.size(); i++){
                        DW_Info data = dw[i];
                        // if (!data.inCollision) {
                        //     std::cout << "NO collision" << std::endl;
                        // }
                        // std::cout << "ANDREW ANDREW: " << IsSafeVelocity(dwAg.left) << std::endl;
                        if (!data.inCollision && data.distGoal_next < minDist){
                            // std::cout << "Decision 1" << std::endl;
                            minDist_vel = data.vel;
                            minDist = data.distGoal_next;
                            selected = true;
                        }

                        if (!data.inCollision && std::abs(data.titaGoal_next) < minTita){
                            // std::cout << "Decision 2" << std::endl;
                            minTita_vel = data.vel;
                            minTita = std::abs(data.titaGoal_next);
                            selected = true;
                        }
                    }
                    //Puede darse el caso que todas las velocidad sean prohibidas: definir una estrategia de evasión hacia velocidades libres lo más próximas al robot
                    if (!selected){
                        // std::cout << "Al velocities are prohibited, applying evasion strategy" << std::endl;
                        //(16/09/2016) Si no está haciendo estrategia de valle, entonces asegurar que coge una velocidad en un hueco seguro (aunque por aceleración no sería posible)
                        //vel = SelectVelInSpace();
                        ///*
                        // andrew -> remove true ||
                        // Diego set always to true
                        // if (originInside){
                        if (true){
                            if (IsSafeVelocity(velAg)) {
                                // std::cout << "Origin is inside and its safe, going down..." << std::endl;
                                motion = dwAg.down;
                                // if (IsSafeVelocity(calculateVelocities(dir_goal, dwAg))) {
                                //     motion = calculateVelocities(dir_goal, dwAg);
                                // } else {
                                //     motion = dwAg.down;
                                // }
                            } else {
                                // std::cout << "Origin is inside and its safe, chosing best way to move..." << std::endl;
                                if (IsSafeVelocity(dwAg.up)) {
                                    // std::cout << "Up is safe" << std::endl;
                                    motion = dwAg.up;
                                // andrew
                                // } else if (IsSafeVelocity(calculateVelocities(dir_goal, dwAg))) {
                                //     // std::cout << "New calculated velocity is safe" << std::endl;
                                //     motion = calculateVelocities(dir_goal, dwAg);
                                } else if (dir_goal.w < 0 && IsSafeVelocity(dwAg.left)) {
                                    // std::cout << "Goal is left and left is safe" << std::endl;
                                    motion = dwAg.left;
                                } else if (dir_goal.w >= 0 && IsSafeVelocity(dwAg.right)) {
                                    // std::cout << "Goal is right and right is safe" << std::endl;
                                    motion = dwAg.right;
                                } else if (IsSafeVelocity(dwAg.left)) {
                                    // std::cout << "Left is safe" << std::endl;
                                    motion = dwAg.left;
                                } else if (IsSafeVelocity(dwAg.right)) {
                                    // std::cout << "Right is safe" << std::endl;
                                    motion = dwAg.right;
                                } else {
                                    // andrew, mejora, coger la mas segura?
                                    // std::cout << "None are safe, slowing down" << std::endl;
                                    motion = dwAg.down;
                                }
                            }
                            //estrategia_frenado = true;
                        }else{
                            // andrew
                            //dir_goal = ComputeNewVel(velAg);
                            //vel = Estrategia_escaparBC();
                            //5/10/2016: que siga el goal directamente
                            Velocidad ini;
                            Velocidad fin(dir_goal.v, dir_goal.w);
                            DW_Range dwRange;
                            if (IntersectRangeDW_Goal(dwRange)){    //if (IntersectionDinGoal(ini, fin)){

                                //DW_Range dwRange = DW_VelocityRange();	//Computes the range of velocities inside the DW that belongs to dir_goal radio
                                motion = dwRange.ini;

                            }else{
                                if (velAg.w > dir_goal.w){
                                    motion = dwAg.left;
                                }else if (velAg.w < dir_goal.w){
                                    motion = dwAg.right;
                                }else{
                                    if (velAg.v > dir_goal.v){
                                        motion = dwAg.down;
                                    }else{
                                        if (velAg.v < dir_goal.v) motion = dwAg.up;
                                        else motion = VelDinGoal(dir_goal, space.GetBounds());
                                    }
                                }
                            }
                        }
                    }else{
                        //La velocidad que lleva el robot tiene la misma dirección que el dir_goal
                        if ((dir_goal.w >=0 && velAg.w >= 0) || (dir_goal.w <=0 && velAg.w <= 0)) {
                            // std::cout << "Applying minDist_vel " << minDist_vel.w << std::endl;
                            motion = minDist_vel;
                        }
                        else {
                            // std::cout << "Applying minTita_vel " << minTita_vel.w << std::endl;
                            //andrew
                            motion = minTita_vel;
                            
                        }
                    }
                }
            }
        }// else: (InsidePolygon(vel) && IsSafeVelocity(vel)) -> se podria aplicar el siguiente comando, que no es libre,
        //y a continuación frenar sin colisionar; pero no lo contemplamos en nuestro algoritmo
    }

    /*
    if (IsSafeVelocity(motion)) safe_ant = true;
    else safe_ant = false;
    vel_ant = motion;
    */
    // cout << "Dir goal at the end of compute motion: " << atan2(dir_goal.v, dir_goal.w) << endl;
    this->space.SetDirGoal(dir_goal);
    commands.clear();
    return true;
}

void Polygon::InsertCommands(const std::vector<dovt> &data){
    std::vector<Command> commandData;
    for (auto it = data.begin(); it != data.end(); ++it){
        auto data = (*it);
        std::vector<Command> d = data.GetCommands();
        d.push_back(Command());
        commandData.insert(commandData.end(), d.begin(), d.end());
    }
    this->InsertCommands(commandData);
}

void Polygon::InsertCommandsAhead(const std::vector<dovtAhead> &commands){

    std::vector<double> mxray, myray;
    int indice_ini = 0;
    Par lim;
    Velocidad vel;
/*
    if (!commands.empty()){
        lim.ini = 0;
        for (int m = 0; m<commands[0].infAhead.size(); m++){
            std::vector<std::pair<Root,Comando>> infAheadK = infAhead[m];
            //std::pair<double, double> point = std::make_pair(infAheadK[m].first.x, infAheadK[m].first.y);
            std::tuple<double, double, double> point = std::make_tuple(commands[0].infAhead[m].first.x, commands[0].infAhead[m].first.y, commands[0].infAhead[m]second.t);
            roots.push_back(point);
            //std::pair<double, double> vel = std::make_pair(infAheadK[m].second.vel.w, infAheadK[m].second.vel.v);
            vel.w = commands[0].infAhead[m].sec.second.vel.w, commands[0].infAhead[m].second.vel.v, commands[0].infAhead[m].second.t);
            commands.push_back(vel);
            points.push_back(vel);
        }
        rootGP.push_back(roots);
        commandGP.push_back(commands);

        vel.w = commands[0].infAhead[0].inf.vel.w; vel.v = commands[0].inf.vel.v;
        points.push_back(vel);
        mxray.push_back(commands[0].inf.vel.w); myray.push_back(commands[0].inf.vel.v);
        int i = 1;
        while (i < (int)commands.size()){
            if (commands[i].objeto != 0){
                if (commands[i].inf.vel.v != commands[i-1].inf.vel.v ||
                    commands[i].inf.vel.w != commands[i-1].inf.vel.w){
                    vel.w = commands[i].inf.vel.w; vel.v = commands[i].inf.vel.v;
                    points.push_back(vel);
                    mxray.push_back(commands[i].inf.vel.w); myray.push_back(commands[i].inf.vel.v);
                }
                i++;
            }else{
                int eltos = i-1;
                vel.w = commands[eltos].sup.vel.w; vel.v = commands[eltos].sup.vel.v;
                points.push_back(vel);
                mxray.push_back(commands[eltos].sup.vel.w); myray.push_back(commands[eltos].sup.vel.v);
                for (int j=eltos-1; j>=indice_ini; j--){
                    //the superior bounds are different
                    if (commands[j].objeto != 0){
                        if (commands[j].sup.vel.v != commands[j+1].sup.vel.v ||
                            commands[j].sup.vel.w != commands[j+1].sup.vel.w){
                            vel.w = commands[j].sup.vel.w; vel.v = commands[j].sup.vel.v;
                            points.push_back(vel);
                            mxray.push_back(commands[j].sup.vel.w); myray.push_back(commands[j].sup.vel.v);
                        }
                    }
                }
                lim.fin = (int)points.size();
                lim_poligono.push_back(lim);
                lim.ini = (int)points.size();
                i++;
                if (i < (int)commands.size()){
                    vel.w = commands[i].inf.vel.w; vel.v = commands[i].inf.vel.v;
                    points.push_back(vel);
                    mxray.push_back(commands[i].inf.vel.w); myray.push_back(commands[i].inf.vel.v);
                    indice_ini = i;
                    i++;
                }
            }
        }
    }
*/
}

void Polygon::InsertCommands(const std::vector<Command> &commands){

    std::vector<double> mxray, myray;
    int indice_ini = 0;
    Par lim;
    Velocidad vel;

    if (!commands.empty()){
        lim.ini = 0;
        vel.w = commands[0].inf.vel.w; vel.v = commands[0].inf.vel.v;
        points.push_back(vel);
        mxray.push_back(commands[0].inf.vel.w); myray.push_back(commands[0].inf.vel.v);
        int i = 1;
        while (i < (int)commands.size()){
            if (commands[i].objeto != 0){
                if (commands[i].inf.vel.v != commands[i-1].inf.vel.v ||
                    commands[i].inf.vel.w != commands[i-1].inf.vel.w){
                    vel.w = commands[i].inf.vel.w; vel.v = commands[i].inf.vel.v;
                    points.push_back(vel);
                    mxray.push_back(commands[i].inf.vel.w); myray.push_back(commands[i].inf.vel.v);
                }
                i++;
            }else{
                int eltos = i-1;
                vel.w = commands[eltos].sup.vel.w; vel.v = commands[eltos].sup.vel.v;
                points.push_back(vel);
                mxray.push_back(commands[eltos].sup.vel.w); myray.push_back(commands[eltos].sup.vel.v);
                for (int j=eltos-1; j>=indice_ini; j--){
                    //the superior bounds are different
                    if (commands[j].objeto != 0){
                        if (commands[j].sup.vel.v != commands[j+1].sup.vel.v ||
                            commands[j].sup.vel.w != commands[j+1].sup.vel.w){
                            vel.w = commands[j].sup.vel.w; vel.v = commands[j].sup.vel.v;
                            points.push_back(vel);
                            mxray.push_back(commands[j].sup.vel.w); myray.push_back(commands[j].sup.vel.v);
                        }
                    }
                }
                lim.fin = (int)points.size();
                lim_poligono.push_back(lim);
                lim.ini = (int)points.size();
                i++;
                if (i < (int)commands.size()){
                    vel.w = commands[i].inf.vel.w; vel.v = commands[i].inf.vel.v;
                    points.push_back(vel);
                    mxray.push_back(commands[i].inf.vel.w); myray.push_back(commands[i].inf.vel.v);
                    indice_ini = i;
                    i++;
                }
            }
        }
    }
}

bool Polygon::InsidePolygon(Velocidad vel) {

    bool inside = false;
    double edge_error = 1e-7;
    int k = 0;

    while (!inside && k<(int)lim_poligono.size()){

        int j = lim_poligono[k].fin-1;
        for (int i=0; i<(lim_poligono[k].fin-lim_poligono[k].ini); i++){

            if ((points[lim_poligono[k].ini + i].v > vel.v) != (points[j].v > vel.v)){

                double b = (points[j].w - points[lim_poligono[k].ini + i].w)*(vel.v - points[lim_poligono[k].ini + i].v)/(points[j].v - points[lim_poligono[k].ini + i].v) + points[lim_poligono[k].ini + i].w;
                if (std::abs(vel.w - b) <= edge_error){
                    return true;	//ON_BOUNDARY
                }
                if (vel.w < b){
                    inside = !inside;
                }
            }else if ((edge_error>=std::abs(vel.v-points[lim_poligono[k].ini + i].v)) && (edge_error >= std::abs(vel.v-points[j].v))){
                if (vel.w - points[lim_poligono[k].ini + i].w <= edge_error && points[j].w - vel.w <= edge_error){
                    return true;	//ON_BOUNDARY
                }
            }
            j=lim_poligono[k].ini + i;
        }
        k++;
    }
    return inside;
}

int Polygon::Size() {
    return (int)lim_poligono.size();
}

std::vector<Velocidad> Polygon::GetPoints(){
    return points;
}

std::vector<Par> Polygon::GetLimPol() {
    return lim_poligono;
}

//function added for the recirpocal case
Velocidad Strategies::ComputeClosest(Velocidad goalTarget){

    Velocidad closest = goalTarget;

    commands.resize(comm_max*obst_max);
    commands = space.GetCommands();
    limites_DOV = space.GetBoundsDOV();

    //Construct the polygon(s) with the different velocity points
    pol.InsertCommands(commands);

    dir_goal = space.GetGoal().dirGoal;

    Velocidad velAg = space.GetAgent();
    boundsVS bounds = space.GetBounds();

    AnalyseCommands();

    ComputeSpaces();	//classifies the hollows in space_outDOV, space_inDOV y valles_z2 (03/05/2016)

    const double gridsize = 100;
    double    bestdist = DBL_MAX;
    double bestTheta;
    const double angGoal = std::atan2(dir_goal.v, dir_goal.w);
    bool found = false;

    //Compute the closest velocity to the goal within angular velocities in same direction as goalTarget
    for (int i=0; i<(int)space_outDOV.size(); i++){

        Space sp = space_outDOV[i];
        double dang = sp.width/100;
        double tita = std::atan2(goalTarget.v, goalTarget.w);
        //find the closest velocity to the goal in this space
        for (double ang = sp.ang.ini; ang <= sp.ang.fin; ang+=dang){
            const double dist = std::abs(ang - angGoal);
            Velocidad velCandidate = ComputeVel(ang, bounds);
            if (dist < bestdist &&
                    (goalTarget.w <= 0 && velCandidate.w <= 0 || goalTarget.w >= 0 && velCandidate.w >= 0)){ //velocities of same sign
                bestdist = dist;
                bestTheta = ang;
                found = true;
            }
        }
    }
    if (found){
        closest = ComputeVel(bestTheta, bounds);
    }

    return closest;
}

PosibleVelocidad Strategies::findClosestReachable(Velocidad target, bool safe) {
    const Velocidad vel  = space.GetAgent();
    const Velocidad goal = target;


    // Size of reachable window
    const double dw = space.GetConstraints().aw * stept;
    const double dv = space.GetConstraints().av * stept;

    Velocidad bestvel;
    double    bestdist = DBL_MAX;
    bool      found    = false;

    const double vstep = dw/10.0; // Resolution of grid over reachable window

    boundsVS bounds = space.GetBounds();
    double maxw_v = bounds.InsideBoundsW(bounds.ComputeMaxW_V(vel.v));
    double maxv_w = bounds.InsideBoundsV(bounds.ComputeMaxV_W(vel.w));

    double wini = (std::abs(vel.w - dw) <= maxw_v ? vel.w - dw : copysign(maxw_v, vel.w - dw));
    double wfin = (std::abs(vel.w + dw) <= maxw_v ? vel.w + dw : copysign(maxw_v, vel.w + dw));
    double vini = bounds.InsideBoundsV(vel.v - dv) <= maxv_w ? bounds.InsideBoundsV(vel.v - dv) : maxv_w;
    double vfin = vel.v + dv <= maxv_w ? vel.v + dv : maxv_w;
    //for (double w = vel.w - dw; w <= vel.w + dw; w+=vstep)
        //for (double v = vel.v - dv; v <= vel.v + dv; v+=vstep) {
    for (double w = wini; w <= wfin; w+=vstep) {
        for (double v = vini; v <= vfin; v+=vstep) {
            const double dist = sqrt(pow(goal.v - v, 2) + pow(goal.w - w, 2));
            if (space.VelReachable(stept, {v, w}) && dist < bestdist && (!safe || !pol.InsidePolygon({v, w}))) {
                found = true;
                bestdist = dist;
                bestvel = {v, w};
            }
        }
    }

    return { found, bestvel };
}

PosibleVelocidad Strategies::findClosestSafe() {

    boundsVS bounds = space.GetBounds();

    const Velocidad vel  = space.GetAgent();
    const double gridsize = 100;

    Velocidad bestvel;
    double    bestdist = DBL_MAX;
    bool      found    = false;

    for (double w = bounds.wmax_right; w < bounds.wmax_left; w += (fabs(bounds.wmax_right - bounds.wmax_left)/gridsize)){
        double maxv_w = bounds.InsideBoundsV(bounds.ComputeMaxV_W(w));
        for (double v = 0; v < maxv_w; v += (bounds.vlim_max/gridsize)) //for (double v = 0; v < bounds.vlim_max; v += (bounds.vlim_max/gridsize))
            if (!pol.InsidePolygon({v, w})) {
                const double dist = sqrt(pow(vel.v - v, 2) + pow(vel.w - w, 2));
                if (dist < bestdist) {
                    found    = true;
                    bestdist = dist;
                    bestvel  = {v, w};
                }
            }
    }

    return { found, bestvel };
}

Velocidad Strategies::Greedy() {

    Velocidad target;
    Tpf goalAg;
    transfor_inversa_p(goal.x, goal.y, &posAg, &goalAg);
    target = FollowGoal(goalAg);

    //Construct the polygon(s) with the different velocity points
    //std::vector<Command> commands;
    commands.resize(comm_max*obst_max);
    commands = space.GetCommands();
    pol.InsertCommands(commands);   //ConstructPolygon();

    PosibleVelocidad best = findClosestReachable(target, true);

    if (best.valid) {// Found a valid safe vel
        return best.vel;
    } else { // Look for the closest safe vel
        best = findClosestSafe();
        if (best.valid) {
            // std::cout << "FOUND ESCAPE VEL: " << best.vel.v << ", " << best.vel.w << std::endl;
            best = findClosestReachable(best.vel, false); // Replace target vel with closest reachable vel
            if (!best.valid)
                //throw std::runtime_error("No valid vel towards escape");
            return best.vel;
        }
        else { // No existing escape vel, emergency stop
            best = findClosestReachable({0, 0}, false);
            if (!best.valid)
                //throw std::runtime_error("No valid vel towards zero");
            // std::cout << "EMERGENCY STOP" << std::endl;
            return best.vel;
        }
    }

}

std::vector<int> Strategies::goalDirInDW(std::vector<Velocidad>& possibleVelocities){
    Velocidad velAg = space.GetAgent();
    boundsVS bounds = space.GetBounds();
    dwAg = DW(velAg, space.GetConstraints(), bounds, stept, false);
    DW_Range range;
    dir_goal = space.GetDirGoal();
    IntersectRangeDW_Goal(range);
    std::vector<int> freeVelocities;
    freeVelocities.clear();
    possibleVelocities.push_back(dwAg.equal);
    possibleVelocities.push_back(dwAg.down);
    possibleVelocities.push_back(dwAg.left);
    possibleVelocities.push_back(dwAg.up);
    possibleVelocities.push_back(dwAg.right);
    if (range.num > 0) {
        Velocidad toGoal;
        toGoal.v = (range.ini.v + range.fin.v)/2;
        if (range.ini.w + range.fin.w != 0) {
            toGoal.w = (range.ini.w + range.fin.w)/2;
        } else {
            toGoal.w = 0;
        }
        if ((toGoal.v >= dwAg.down.v && toGoal.v <= dwAg.up.v && toGoal.w >= dwAg.left.w && toGoal.w <= dwAg.right.w) &&
             ((dwAg.equal.v == 0 && toGoal.v == 0) || toGoal.v > 0)) {
                freeVelocities.push_back(5);
                possibleVelocities.push_back(toGoal);
                // std::cout << "Equal is free" << std::endl;
        }
        else{
            possibleVelocities.push_back(dwAg.equal);
        }
        if ((range.ini.v >= dwAg.down.v && range.ini.v <= dwAg.up.v && range.ini.w >= dwAg.left.w && range.ini.w <= dwAg.right.w) &&
            ((range.ini.v == 0 && dwAg.equal.v == 0) || range.ini.v > 0)) {
                freeVelocities.push_back(6);
                possibleVelocities.push_back(range.ini);
                // std::cout << "Up is free" << std::endl;
        }
        else{
            possibleVelocities.push_back(dwAg.equal);
        }
        if ((range.fin.v >= dwAg.down.v && range.fin.v <= dwAg.up.v && range.fin.w >= dwAg.left.w && range.fin.w <= dwAg.right.w) &&
            ((range.fin.v == 0 && dwAg.equal.v == 0) || range.fin.v > 0)) {
                freeVelocities.push_back(7);
                possibleVelocities.push_back(range.fin);
                // std::cout << "Down is free" << std::endl;
        }
        else{
            possibleVelocities.push_back(dwAg.equal);
        }
        return freeVelocities;
    }
    else{
        possibleVelocities.push_back(dwAg.equal);
        possibleVelocities.push_back(dwAg.equal);
        possibleVelocities.push_back(dwAg.equal);
        return freeVelocities;
    }
}

std::vector<std::pair<int, Velocidad>> Strategies::getSafeVelocities(int agId, Tsc posR, Tpf posG, std::vector<std::unique_ptr<Agent>> *agentsIn, bool originInside, Velocidad &motion, bool &freeActions,
bool learning) {
    // cout << "Get safe velocities" << endl;
    dir_goal = space.GetGoal().dirGoal;
    if (!learning){
        NewDirGoal(false);
        space.SetDirGoal(dir_goal);
    } 
    commands.resize(comm_max*obst_max);
    commands = space.GetCommands();
    limites_DOV = space.GetBoundsDOV();
    posAg = posR;
    goal = posG;
    agents = agentsIn;
    idAg = agId;

    //Construct the polygon(s) with the different velocity points
    pol.InsertCommands(commands);

    //Goal with respect to the agent
    Tpf goalAg;
    transfor_inversa_p(goal.x, goal.y, &posR, &goalAg);
    dir_goal = FollowGoal(goalAg);

    Velocidad velAg = space.GetAgent();
    boundsVS bounds = space.GetBounds();


    AnalyseCommands();

    ComputeSpaces();
    
    dwAg = DW(velAg, space.GetConstraints(), bounds, stept, true);
    // DIEGO
    // double kv = 0.25;
    // double v_goal = sqrt(goalAg.x*goalAg.x+goalAg.y+goalAg.y)*kv;

    // if (v_goal<=dwAg.up.v && v_goal>=dwAg.down.v ){
    //     dwAg.up.v = v_goal;
    // }
    
    std::vector<std::pair<int, Velocidad>> freeVelocities = std::vector<std::pair<int, Velocidad>>();
    if(!pol.InsidePolygon(dwAg.equal)) {
        freeVelocities.push_back(std::make_pair(0,dwAg.equal));
    }
    if(!pol.InsidePolygon(dwAg.down) && dwAg.down.v >=0) {
        freeVelocities.push_back(std::make_pair(1,dwAg.down));
    }
    if(!pol.InsidePolygon(dwAg.left) && dwAg.left.w > -1) {
        freeVelocities.push_back(std::make_pair(2, dwAg.left));
    }
    if(!pol.InsidePolygon(dwAg.up) && dwAg.up.v >= 0) {
        freeVelocities.push_back(std::make_pair(3, dwAg.up));
    }
    if(!pol.InsidePolygon(dwAg.right) && dwAg.right.w > -1) {
        freeVelocities.push_back(std::make_pair(4, dwAg.right));
    }
    DW_Range range;
    IntersectRangeDW_Goal(range);
    if (range.num > 0) {
        Velocidad toGoal;
        toGoal.v = (range.ini.v + range.fin.v)/2;
        if (range.ini.w + range.fin.w != 0) {
            toGoal.w = (range.ini.w + range.fin.w)/2;
        } else {
            toGoal.w = 0;
        }
        if (!pol.InsidePolygon(toGoal)) {
            if (toGoal.v >= dwAg.down.v && toGoal.v <= dwAg.up.v && toGoal.w >= dwAg.left.w && toGoal.w <= dwAg.right.w) {
                if ((dwAg.equal.v == 0 && toGoal.v == 0) || toGoal.v > 0) {
                    freeVelocities.push_back(std::make_pair(5, toGoal));
                    // std::cout << "Equal is free" << std::endl;
                } 
            }
        }
        if (!pol.InsidePolygon(range.ini)) {
            if (range.ini.v >= dwAg.down.v && range.ini.v <= dwAg.up.v && range.ini.w >= dwAg.left.w && range.ini.w <= dwAg.right.w) {
                if ((range.ini.v == 0 && dwAg.equal.v == 0) || range.ini.v > 0) {
                    freeVelocities.push_back(std::make_pair(6, range.ini));
                    // std::cout << "Up is free" << std::endl;
                }
            }
        }
        if (!pol.InsidePolygon(range.fin)) {
            if (range.fin.v >= dwAg.down.v && range.fin.v <= dwAg.up.v && range.fin.w >= dwAg.left.w && range.fin.w <= dwAg.right.w) {
                if ((range.fin.v == 0 && dwAg.equal.v == 0) || range.fin.v > 0) {
                    freeVelocities.push_back(std::make_pair(7, range.fin));
                    // std::cout << "Down is free" << std::endl;
                }
            }
        }
    }

    if (freeVelocities.empty()) {
        freeActions = false;
        if(dwAg.down.v >= 0)
            freeVelocities.push_back(std::make_pair(1, dwAg.down));
        if (dwAg.left.w > -5 )
            freeVelocities.push_back(std::make_pair(2, dwAg.left));
        if (dwAg.right.w > -5)
            freeVelocities.push_back(std::make_pair(4, dwAg.right));
        if (dwAg.up.v <= 1.5) {
            freeVelocities.push_back(std::make_pair(3, dwAg.up));
        }
    } else {
        freeActions = true;
    }

    return freeVelocities;
}

int Strategies::getCurrentZone() {
    if (0 <= atan2(dwAg.equal.v, dwAg.equal.w) && atan2(dwAg.equal.v, dwAg.equal.w) < atan2(space.GetBounds().vlim_max, space.GetBounds().wmax_left))
        return 1;
    
    if (atan2(space.GetBounds().vlim_max, space.GetBounds().wmax_left) <= atan2(dwAg.equal.v, dwAg.equal.w) &&
                  atan2(dwAg.equal.v, dwAg.equal.w) <= atan2(space.GetBounds().vlim_max, space.GetBounds().wmax_right))
        return 2;

    else return 3;

}

bool Strategies::isDirGoalFree() {
    DW_Range range;
    IntersectRangeDW_Goal(range);
    if (range.num > 0) {
        Velocidad toGoal;
        toGoal.v = (range.ini.v + range.fin.v)/2;
        if (range.ini.w + range.fin.w != 0) {
            toGoal.w = (range.ini.w + range.fin.w)/2;
        } else {
            toGoal.w = 0;
        }
        if (!pol.InsidePolygon(toGoal)) {
            if (toGoal.v >= dwAg.down.v && toGoal.v <= dwAg.up.v && toGoal.w >= dwAg.left.w && toGoal.w <= dwAg.right.w) {
                if ((dwAg.equal.v == 0 && toGoal.v == 0) || toGoal.v > 0) {
                    return true;
                } 
            }
        }
        if (!pol.InsidePolygon(range.ini)) {
            if (range.ini.v >= dwAg.down.v && range.ini.v <= dwAg.up.v && range.ini.w >= dwAg.left.w && range.ini.w <= dwAg.right.w) {
                if ((range.ini.v == 0 && dwAg.equal.v == 0) || range.ini.v > 0) {
                    return true;
                }
            }
        }
        if (!pol.InsidePolygon(range.fin)) {
            if (range.fin.v >= dwAg.down.v && range.fin.v <= dwAg.up.v && range.fin.w >= dwAg.left.w && range.fin.w <= dwAg.right.w) {
                if ((range.fin.v == 0 && dwAg.equal.v == 0) || range.fin.v > 0) {
                    return true;
                }
            }
        }
        return false;
    } else {
        return false;
    }
}