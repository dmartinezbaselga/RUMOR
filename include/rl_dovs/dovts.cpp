//
// Created by maite14 on 8/02/17.
//

//#include <eigen3/Eigen/src/Core/util/XprHelper.h>
#include <stdio.h>
#include <ostream>

#include <eigen3/Eigen/Dense>

#include "dovts.h"

const double maxCost = 10e4;

/*
DOVTS& DOVTS::operator= (const DOVTS& stateIn){

    stept = stateIn.stept;
    sizew = stateIn.sizew;
    sizev = stateIn.sizev;
    bounds = stateIn.bounds;
    int n = stateIn.vts.data3d.size();
    vts = map3d<layerVW>(n);
    for (unsigned i = 0; i < n; i++) {
        layerVW layer = stateIn.vts.data3d[i];
        vts.data3d.push_back(layer);
    }

    const goalVS gIn = stateIn.GetGoal();
    //this->SetGoal(gIn);
    //velAgent = stateIn.velAgent;
    //diffConstraints = stateIn.diffConstraints;
}
*/

Velocidad DOVTS::GetVelocity(keyWV key){
//returns the velocity associated to the cell at key

    assert(key.first >= 0 && key.first <= sizew && key.second >= 0 && key.second <= sizev);

    double stepw = std::abs(bounds.wmax_right - bounds.wmax_left)/sizew;
    double stepv = bounds.vlim_max/sizev;

    return Velocidad(key.second*stepv, bounds.wmax_right + key.first*stepw);
}

double DOVTS::GetTime(unsigned k){
//returns the time associated to the layer k

    assert(k>=0 && k<=vts.data3d.size());

    return k*stept;
}

bool DOVTS::IsValidWV(Velocidad vel){

    bool exp1 = (vel.v >= bounds.vlim_min && vel.v <= bounds.vlim_max);
    bool exp2 = (bounds.wmax_right <= vel.w && vel.w <= bounds.wmax_left);

    if (!(exp1 && exp2)) std::cout << "Velocidad no válida: " << vel.v << ", " << vel.w << std::endl;
    return (exp1 && exp2);
}

bool DOVTS::IsValidT(double t){

    return t>=0 && t <= stept*vts.data3d.size();
}

keyWV DOVTS::GetKeyWV(Velocidad vel) {
    //returns the key(i,j) which identifies the indexes in the velocity map;
    //assuming that the size of the velocity map is the same through the layers, then
    //each velocity (v,w) will have an only key valid for all the layers in DOVTS

    assert(IsValidWV(vel));

    double stepw = std::abs(bounds.wmax_right - bounds.wmax_left)/sizew;
    double stepv = bounds.vlim_max/sizev;

    //unsigned i = (std::abs(bounds.wmax_right - vel.w) == stepw*sizew) ? std::floor(std::abs(bounds.wmax_right - vel.w) / stepw) - 1 : std::floor(std::abs(bounds.wmax_right - vel.w) / stepw);  // w dimension
    //unsigned j = (vel.v == stepv*sizev) ? std::floor(vel.v / stepv) - 1 : std::floor(vel.v / stepv); // v dimension

    //unsigned i = std::floor(std::abs(bounds.wmax_right - vel.w) / stepw);   // w dimension
    //unsigned j = std::floor(vel.v / stepv); // v dimension

    //w.v is the center of the cell
    unsigned i = std::floor(std::abs(bounds.wmax_right - vel.w) / stepw)  +  std::floor(std::fmod(std::abs(bounds.wmax_right - vel.w), stepw) /  (stepw/2));  // w dimension
    unsigned j = std::floor(vel.v / stepv) +  std::floor(std::fmod(vel.v, stepv) /  (stepv/2)); // v dimension

    return {std::make_pair(i, j)};
}

unsigned DOVTS::GetKeyT(double t){
    //returns the layer of the map in the time dimension

    assert(IsValidT(t));

    return (t == stept*vts.data3d.size() ? (unsigned) std::floor(t/stept) - 1 : (unsigned) std::floor(t/stept));
}

bool DOVTS::InsertKey(keyWV k_wv, unsigned k_t, cellVW cell, unsigned long nCells){
//nCells: number of cells 'k_wv' downwards 'k_t' to be inserted

    bool ok = vts.data3d[k_t].mapVW.insert({k_wv, cell}).second;
    //propagate the cell downwards
    int k = k_t - 1; int i = 0;
    while (i<nCells && k>=0 && k<vts.data3d.size()){
        vts.data3d[(unsigned)k].mapVW.insert({k_wv, cell});
        i++; k--;
    }

    return ok;
}

bool DOVTS::GetKey(const Velocidad vel, const double t, std::pair<keyWV, unsigned>& key) {

    double time = t;

    keyWV k_vw = GetKeyWV(vel);
    if (time > stept*vts.data3d.size()) {
        //time = stept *vts.data3d.size();   //so that the planner knows this velocity is also occupied, and can only plan in the time horizon given
        return false;
    }
    unsigned k_t = GetKeyT(time);
    key.operator=(std::make_pair(k_vw, k_t));

    return true;
}

bool DOVTS::IsCellFree(std::pair<keyWV, unsigned> key){

    if (!vts.data3d.empty()){
        layerVW layer = vts.data3d[key.second];
        auto search = layer.mapVW.find(key.first);
        if (search != layer.mapVW.end()){
            return search->second.free;
        }
        return true;
    }

    return false;   //error: el mapa no tiene datos
}

bool DOVTS::IsCellFree(Velocidad vel, double t){

    std::pair<keyWV, unsigned> key;
    GetKey(vel, t, key);

    return IsCellFree(key);
}

void DOVTS::InsertVTS(const map3d<layerVW> &space) {

    //assume the input space and this->space have same discretization size in the grid
    for (unsigned i=0; i< (unsigned)space.data3d.size(); i++){  //time layers
        layerVW input = space.data3d[i];
        if (!input.mapVW.empty()){
            layerVW result = vts.data3d[i];
            map2d< keyWV, cellVW >::iterator it;
            for (it = input.mapVW.begin(); it != input.mapVW.end(); it++){
                keyWV key = (*it).first;
                cellVW cell = (*it).second;

                result.mapVW.insert({key, cell});
            }
            vts.data3d[i] = result;
        }
    }
}

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

void WriteCube(unsigned w, unsigned v, unsigned t, cellVW cell, int delta, const char *name);


void DOVTS::GenerateCellLine(const Command command, unsigned long nCells) {

    double stepw = std::abs(bounds.wmax_right - bounds.wmax_left) / sizew;
    double stepv = bounds.vlim_max / sizev;

    //Generate the line between the boundsVS- 3D line
    Eigen::Vector3d psup(command.sup.vel.w, command.sup.vel.v, command.sup.t);
    Eigen::Vector3d pinf(command.inf.vel.w, command.inf.vel.v, command.inf.t);
    Eigen::Vector3d director = psup - pinf; //from inf to sup
    //double dstep = std::sqrt(stepw*stepw + stepv*stepv + stept*stept);
    double dstep = std::min(stepw, std::min(stepv, stept));
    double d = std::sqrt(
            (psup[0] - pinf[0]) * (psup[0] - pinf[0]) + (psup[1] - pinf[1]) * (psup[1] - pinf[1]) +
            (psup[2] - pinf[2]) * (psup[2] - pinf[2]));
    //d = d/director.norm();
    double n = std::ceil(d / dstep);
    dstep = 1 / n;
    for (int j = 1; j < n; j++) {
        double x = j * dstep * director[0] + pinf[0];
        double y = j * dstep * director[1] + pinf[1];
        double z = j * dstep * director[2] + pinf[2];

        std::pair<keyWV, unsigned> key;
        if (GetKey(Velocidad(y, x), z, key)){
            cellVW cell;    //TODO: initialize the variable
            InsertKey(key.first, key.second, cell, nCells);
            /*
            auto search = vts.data3d[key.second].mapVW.find(key.first);
            cellVW cell = search->second;
            WriteCube(key.first.first, key.first.second, key.second, cell, name);
            fpoints << std::abs(boundsVS.wmax_right - x) / stepw << "\t" << y / stepv << "\t"
                    << (z / stept > vts.data3d.size() ? vts.data3d.size() : z / stept) << std::endl;
            //g << "splot 'bounds.dat' u 1:2:3 w l lc 010 notitle, 'points.dat' u 1:2:3 w lp lc 003\n";
            //g.flush();
            */
        }
    }
}

void SwapPoints(std::pair<keyWV, unsigned>& keyIni, std::pair<keyWV, unsigned>& keyEnd){

    std::pair<keyWV, unsigned> keySup = keyEnd;
    std::pair<keyWV, unsigned> keyInf = keyIni;

    int dw = keySup.first.first - keyInf.first.first;   //w
    int dv = keySup.first.second - keyInf.first.second; //v
    int dt = keySup.second - keyInf.second; //t
    if (std::abs(dv) < std::abs(dw) && std::abs(dt) < std::abs(dw)) {
        if (keySup.first.first < keyInf.first.first) {
            std::pair<keyWV, unsigned> kAux = keyInf;
            keyInf = keySup;
            keySup = kAux;
        }
    } else if (std::abs(dw) < std::abs(dv) && std::abs(dt) < std::abs(dv)) {
        if (keySup.first.second < keyInf.first.second) {
            std::pair<keyWV, unsigned> kAux = keyInf;
            keyInf = keySup;
            keySup = kAux;
        }
    } else {
        if (keySup.second < keyInf.second) {
            std::pair<keyWV, unsigned> kAux = keyInf;
            keyInf = keySup;
            keySup = kAux;
        }
    }

    keyIni = keyInf;
    keyEnd = keySup;

}

std::vector<std::pair<keyWV, unsigned>>
DOVTS::BresenLine3d(const std::pair<keyWV, unsigned> keyIni, const std::pair<keyWV, unsigned> keyEnd, unsigned long nCells, Gnuplot& g) {

    std::vector<std::pair<keyWV, unsigned>> points;

    cellVW cell;
    cell.free = false; //cell.data = nullptr;
    cell.currentCost = maxCost; //10000;
    cell.previousCost = cell.currentCost;

    std::pair<keyWV, unsigned> keySup, keyInf;

    keySup = keyEnd; keyInf = keyIni;
    //Bresenham algorithm
    int dw = keySup.first.first - keyInf.first.first;   //w
    int dv = keySup.first.second - keyInf.first.second; //v
    int dt = keySup.second - keyInf.second; //t
    int vi = 1; int wi = 1; int ti = 1;
    if (dv < 0){ dv = -dv;   vi = -1; }
    if (dw < 0){ dw = -dw; wi = -1; }
    if (dt < 0){ dt = -dt; ti = -1; }
    if (std::abs(dv) < std::abs(dw) && std::abs(dt) < std::abs(dw)){
        int D1 = 2*dv - dw;
        int D2 = 2*dt - dw;
        unsigned v = keyInf.first.second;
        unsigned t = keyInf.second;
        for (unsigned w = keyInf.first.first; w <= keySup.first.first; w++) {
            if (InsertKey({w,v}, t, cell, nCells))  //map2d.mapVW.insert({{w, v}, cell});
                points.push_back({{w,v}, t});   //the data is not in the grid yet

            if (D1 > 0){
                v += vi;
                D1 -= 2*dw;
            }
            if (D2 > 0){
                t += ti;
                D2 -= 2*dw;
            }
            D1 += 2*dv;
            D2 += 2*dt;
        }
    }else if (std::abs(dw) < std::abs(dv) && std::abs(dt) < std::abs(dv)){
        int D1 = 2*dw - dv;
        int D2 = 2*dt - dv;
        unsigned w = keyInf.first.first;
        unsigned t = keyInf.second;
        for (unsigned v = keyInf.first.second; v <= keySup.first.second; v++) {
            if (InsertKey({w,v}, t, cell, nCells))  //map2d.mapVW.insert({{w, v}, cell});
                points.push_back({{w,v}, t});

            if (D1 > 0){
                w += wi;
                D1 -= 2*dv;
            }
            if (D2 > 0){
                t += ti;
                D2 -= 2*dv;
            }
            D1 += 2*dw;
            D2 += 2*dt;
        }
    }else{
        int D1 = 2*dw - dt;
        int D2 = 2*dv - dt;
        unsigned w = keyInf.first.first;
        unsigned v = keyInf.first.second;
        for (unsigned t = keyInf.second; t <= keySup.second; t++) {
            if (InsertKey({w,v}, t, cell, nCells))  //map2d.mapVW.insert({{w, v}, cell});
                points.push_back({{w,v}, t});

            if (D1 > 0){
                w += wi;
                D1 -= 2*dt;
            }
            if (D2 > 0){
                v += vi;
                D2 -= 2*dt;
            }
            D1 += 2*dw;
            D2 += 2*dv;
        }
    }

    return points;
}

void DOVTS::BresenLine(const Command c, unsigned long nCells, Gnuplot& g) {

    cellVW cell;
    cell.free = false; //cell.data = nullptr;
    cell.currentCost = maxCost; //10000;
    cell.previousCost = cell.currentCost;
    std::pair<keyWV, unsigned> keySup, keyInf;

    //layerVW map2d(vts.data3d[0].size_w, vts.data3d[0].size_v);
    layerVW map2d = vts.data3d[0];
    if (GetKey(c.sup.vel, c.sup.t, keySup)) {   //if the command is forbidden beyond the time horizon, then it is not translated to the grid
        //map2d.mapVW.insert({keySup.first, cell}); //InsertKey(keySup.first, keySup.second, cell, nCells); //InsertElement(c.sup.vel, c.sup.t);


        //Lower bound
        if (GetKey(c.inf.vel, c.inf.t, keyInf)) {
            //if (keyInf.second != keySup.second || keyInf.first != keySup.first) { //check if the forbidden velocity lies in a different cell

                //Bresenham algorithm
                int dw = keySup.first.first - keyInf.first.first;   //w
                int dv = keySup.first.second - keyInf.first.second; //v
                if (std::abs(dv) < std::abs(dw)){
                    if (keySup.first.first < keyInf.first.first){
                        std::pair<keyWV, unsigned> kAux = keyInf;
                        keyInf = keySup;
                        keySup = kAux;
                    }
                }else{
                    if (keySup.first.second < keyInf.first.second){
                        std::pair<keyWV, unsigned> kAux = keyInf;
                        keyInf = keySup;
                        keySup = kAux;
                    }
                }

                dw = keySup.first.first - keyInf.first.first;   //w
                dv = keySup.first.second - keyInf.first.second; //v
                if (dw == 0) {   //vertical lines
                    unsigned w = keyInf.first.first;
                    for (unsigned v = keyInf.first.second; v <= keySup.first.second; v++) {
                        map2d.mapVW.insert({{w, v}, cell});
                    }
                }else if (dv == 0){ //Horizontal line
                    unsigned v = keyInf.first.second;
                    for (unsigned w = keyInf.first.first; w <= keySup.first.first; w++) {
                        map2d.mapVW.insert({{w, v}, cell});
                    }
                }else{
                    if (std::abs(dv/dw) < 1){   //w increments by 1 each step
                        int vi = 1;
                        if (dv < 0){
                            dv = -dv;
                            vi = -1;
                        }
                        int D = 2*dv - dw;
                        unsigned v = keyInf.first.second;
                        for (unsigned w = keyInf.first.first; w <= keySup.first.first; w++) {
                            map2d.mapVW.insert({{w, v}, cell});
                            if (D > 0){
                                v += vi;
                                D -= 2*dw;
                            }
                            D += 2*dv;
                        }
                    }else{
                        int wi = 1;
                        if (dw < 0){
                            dw = -dw;
                            wi = -1;
                        }
                        int D = 2*dw - dv;
                        unsigned w = keyInf.first.first;
                        for (unsigned v = keyInf.first.second; v <= keySup.first.second; v++) {
                            map2d.mapVW.insert({{w, v}, cell});
                            if (D > 0){
                                w += wi;
                                D -= 2*dv;
                            }
                            D += 2*dw;
                        }
                    }
                }
            //}
        }
    }
    vts.data3d[0] = map2d;
}



void DOVTS::FillCells(const Command c, unsigned long nCells, Gnuplot& g) {
//nCells: number of cells to be filled downwards

    //To store the cell points
    std::ofstream fpoints("points.dat", std::ios_base::app);
    //std::ofstream fpointsC("pointsComando.dat", std::ios_base::trunc);

    //To store the cubes
    //const char *name = "bounds.dat";
    //const char *nameC = "boundsComando.dat";
    //std::ofstream fnameC("boundsComando.dat", std::ios_base::trunc);

    cellVW cell;
    cell.free = false; //cell.data = nullptr;
    cell.currentCost = maxCost; //10000;
    cell.previousCost = cell.currentCost;
    std::pair<keyWV, unsigned> keySup, keyInf;
    //Upper bound; the time associated to the upper bound is less than the time associated to the lower bound
    if (GetKey(c.sup.vel, c.sup.t, keySup)) {
        InsertKey(keySup.first, keySup.second, cell, nCells); //InsertElement(c.sup.vel, c.sup.t);
        fpoints << c.sup.vel.w << "\t" << c.sup.vel.v << "\t" << c.sup.t << std::endl;

        //Lower bound
        if (GetKey(c.inf.vel, c.inf.t, keyInf)) {
            if (keyInf.second != keySup.second || keyInf.first != keySup.first) //check if the forbidden velocity lies in a different cell
                InsertKey(keyInf.first, keyInf.second, cell, nCells); //InsertElement(c.inf.vel, c.inf.t);
            else
                return;
            fpoints << c.inf.vel.w << "\t" << c.inf.vel.v << "\t" << c.inf.t << std::endl;
        }

        //GenerateCellLine(c, nCells);
        //Generate the line between the boundsVS- 3D line
        double stepw = std::abs(bounds.wmax_right - bounds.wmax_left) / sizew;
        double stepv = bounds.vlim_max / sizev;
        Eigen::Vector3d psup(c.sup.vel.w, c.sup.vel.v, c.sup.t);
        Eigen::Vector3d pinf(c.inf.vel.w, c.inf.vel.v, c.inf.t);
        Eigen::Vector3d director = psup - pinf; //from inf to sup
        //double dstep = std::sqrt(stepw*stepw + stepv*stepv + stept*stept);
        double dstep = std::min(stepw, std::min(stepv, stept));
        double d = std::sqrt(
                (psup[0] - pinf[0]) * (psup[0] - pinf[0]) + (psup[1] - pinf[1]) * (psup[1] - pinf[1]) +
                (psup[2] - pinf[2]) * (psup[2] - pinf[2]));
        //d = d/director.norm();
        double n = std::ceil(d / dstep);
        dstep = 1 / n;
        for (int j = 1; j < n; j++) {
            double x = j * dstep * director[0] + pinf[0];
            double y = j * dstep * director[1] + pinf[1];
            double z = j * dstep * director[2] + pinf[2];

            std::pair<keyWV, unsigned> key;
            if (GetKey(Velocidad(y, x), z, key)){
                InsertKey(key.first, key.second, cell, nCells);
                fpoints << x << "\t" << y << "\t" << z << std::endl;
            }
        }
    }

    /*
    g << "set xrange [0:" << sizew << "]\n" << "set yrange [0:" << sizev << "]\n" << "set zrange [0:" << vts.data3d.size() << "]\n";
    g << "set size ratio -1\n";
    //g << "set view equal xy\n";
     */
    //Write data in log file
    /*
    auto search = vts.data3d[keySup.second].mapVW.find(keySup.first);
    cellVW cell = search->second;
    WriteCube(keySup.first.first, keySup.first.second, keySup.second, cell, name);
    WriteCube(keySup.first.first, keySup.first.second, keySup.second, cell, nameC);
    fpoints << std::abs(boundsVS.wmax_right - c.sup.vel.w) / stepw << "\t" << c.sup.vel.v / stepv << "\t"
            << (c.sup.t / stept > vts.data3d.size() ? vts.data3d.size() : c.sup.t / stept) << std::endl;
    fpointsC << std::abs(boundsVS.wmax_right - c.sup.vel.w) / stepw << "\t" << c.sup.vel.v / stepv << "\t"
             << (c.sup.t / stept > vts.data3d.size() ? vts.data3d.size() : c.sup.t / stept) << std::endl;
    */
    //fpoints.close();

    //fname.close();
    //fpointsC.close();

    fpoints.close();
}

void DOVTS::FillMap2(const std::vector<Command>& dov, unsigned long n){
//n: number of cells to be filled downwards for a forbidden command
    Gnuplot g;

    bool primero = true;
    Eigen::Vector3d previous_psup, previous_pinf;
    std::vector<std::pair<keyWV, unsigned>> prev_points;
    for (unsigned i=0; i<(unsigned)dov.size(); i++){
        Command c = dov[i];
        if (c.objeto != 0){

            std::pair<keyWV, unsigned> keySup, keyInf;
            if (GetKey(c.sup.vel, c.sup.t, keySup)) {   //if the command is forbidden beyond the time horizon, then it is not translated to the grid
                //map2d.mapVW.insert({keySup.first, cell}); //InsertKey(keySup.first, keySup.second, cell, nCells); //InsertElement(c.sup.vel, c.sup.t);

                //Lower bound
                if (GetKey(c.inf.vel, c.inf.t > stept * vts.data3d.size() ? stept * vts.data3d.size() : c.inf.t, keyInf)) {
                    //if (GetKey(c.inf.vel, c.inf.t, keyInf)) {
                    //if (keyInf.second != keySup.second || keyInf.first != keySup.first) { //check if the forbidden velocity lies in a different cell


                    FillCells(c, n, g);

                    ///*
                    //Generate the line between the boundsVS- 3D line
                    Eigen::Vector3d psup(c.sup.vel.w, c.sup.vel.v, c.sup.t);
                    Eigen::Vector3d pinf(c.inf.vel.w, c.inf.vel.v, c.inf.t);
                    if (!primero) {

                        //check for interpolation between limit points in the dov
                        Eigen::Vector3d dir_sup = previous_psup - psup; //from psup (current one) to previous_psup
                        Eigen::Vector3d dir_inf = previous_pinf - pinf; //from pinf (current one) to previous_pinf

                        double dsup = std::sqrt(
                                std::pow(previous_psup[0] - psup[0], 2) + std::pow(previous_psup[1] - psup[1], 2) +
                                std::pow(previous_psup[2] - psup[2], 2));
                        double dinf = std::sqrt(
                                std::pow(previous_pinf[0] - pinf[0], 2) + std::pow(previous_pinf[1] - pinf[1], 2) +
                                std::pow(previous_pinf[2] - pinf[2], 2));

                        double stepw = std::abs(bounds.wmax_right - bounds.wmax_left) / sizew;
                        double stepv = bounds.vlim_max / sizev;

                        int n = 0;
                        double step_sup = stept;
                        double step_inf = stept;
                        if (dsup > dinf) {
                            //n = std::ceil(dsup/stept);
                            n = std::ceil(dsup / stepv);
                            step_inf = dinf / n;
                        } else {
                            //n = std::ceil(dinf/stept);
                            n = std::ceil(dinf / stepv);
                            step_sup = dsup / n;
                        }
                        //if (n<=4){
                        //step_sup = stept;
                        //step_inf = stept;
                        //n = 4;
                        //}else{
                        //    std::cout << "Paramos" << std::endl;
                        //}

                        //if (dinf > 1)//{
                        //std::cout << "Paramos" << std::endl;


                        if (n > 1) { //Interpolate radius

                            //step_sup = stept; step_inf = stept; n = 4;
                            step_sup = stepv;
                            step_inf = stepv;
                            n = 8;

                            for (int j = 1; j < n; j++) {
                                double xsup = j * step_sup * dir_sup[0] + psup[0];
                                double ysup = j * step_sup * dir_sup[1] + psup[1];
                                double zsup = j * step_sup * dir_sup[2] + psup[2];

                                double xinf = j * step_inf * dir_inf[0] + pinf[0];
                                double yinf = j * step_inf * dir_inf[1] + pinf[1];
                                double zinf = j * step_inf * dir_inf[2] + pinf[2];

                                Comando sup(Velocidad(ysup, xsup), zsup);
                                Comando inf(Velocidad(yinf, xinf), zinf);

                                FillCells(Command(sup, inf, 1), n, g);

                            }
                        }
                        //}
                    } else {
                        primero = false;
                    }

                    previous_psup = psup;
                    previous_pinf = pinf;
                    //*/
                }
            }

        } else{
            std::ofstream fpoints("points.dat", std::ios_base::app);
            fpoints << std::endl;
            fpoints.close();
        }
    }

    /*
    g << "set xlabel \"Angular Velocity\"\n";
    g << "set ylabel \"Linear Velocity\"\n";

    std::ofstream f("dataBresen.dat", std::ios_base::trunc);
    f.close();

    int nlayer = 0;
    for (auto layer = vts.data3d.begin(); layer != vts.data3d.end(); ++layer){
        for (auto it = (*layer).mapVW.begin(); it != (*layer).mapVW.end(); ++it) {
            cellVW cell = (*it).second;
            keyWV key = (*it).first;

            if (!cell.free) {
                unsigned j = key.first; unsigned k = key.second;
                WriteCube(j, k, nlayer, cell, delta, "dataBresen.dat");

            }
        }
        nlayer++;
    }
    g << "splot 'dataBresen.dat' u 1:2:3 w pm3d\n";
    //g << "plot 'dataBresen.dat' u 1:2 w l fill\n";
    g.flush();
    */

}

void DOVTS::FillMap(const std::vector<Command>& dov, unsigned long n){
//n: number of cells to be filled downwards for a forbidden command
    Gnuplot g, gIt;
    const char* name = "dovt_forbidden.dat";
    const char* nameIt = "dovt_forbiddenIt.dat";

    std::ofstream file(name, std::ios_base::trunc);
    file.close();

    file.open(nameIt, std::ios_base::trunc);
    file.close();

    bool primero = true;
    Eigen::Vector3d previous_psup, previous_pinf;
    std::vector<std::pair<keyWV, unsigned>> prev_points;
    for (unsigned i=0; i<(unsigned)dov.size(); i++){
        Command c = dov[i];
        if (c.objeto != 0){

            std::pair<keyWV, unsigned> keySup, keyInf;
            if (GetKey(c.sup.vel, c.sup.t, keySup)) {   //if the command is forbidden beyond the time horizon, then it is not translated to the grid
                //map2d.mapVW.insert({keySup.first, cell}); //InsertKey(keySup.first, keySup.second, cell, nCells); //InsertElement(c.sup.vel, c.sup.t);

                //Lower bound
                if (GetKey(c.inf.vel, c.inf.t > stept * vts.data3d.size() ? stept * vts.data3d.size() : c.inf.t, keyInf)) {
                //if (GetKey(c.inf.vel, c.inf.t, keyInf)) {
                    //if (keyInf.second != keySup.second || keyInf.first != keySup.first) { //check if the forbidden velocity lies in a different cell

                    //Check which point goes first
                    SwapPoints(keyInf, keySup);
                    std::vector<std::pair<keyWV, unsigned>> points;
                    //points.resize(stept*sizev*sizew);
                    points = BresenLine3d(keyInf, keySup, n, g);
                    //BresenLine(c, n, g);
                    /*
                    for (int l = 0; l < (int)points.size(); l++){
                        std::pair<keyWV, unsigned> p = points[l];
                        cellVW cell; cell.currentCost = 0;
                        WriteCube(p.first.first, p.first.second, p.second, cell, delta, name);
                        WriteCube(p.first.first, p.first.second, p.second, cell, delta, nameIt);
                    }
                    g << "splot 'dovt_forbidden.dat' u 1:2:3 w pm3d\n";
                    g.flush();
                    gIt << "splot 'dovt_forbiddenIt.dat' u 1:2:3 w pm3d\n";
                    gIt.flush();
                    */

                    std::ofstream file(nameIt, std::ios_base::trunc);
                    file.close();

                    ///*
                    if (!primero) {
                        //Compare the cells added for consecutive Bresenham lines
                        int m = (int)points.size() < (int)prev_points.size() ? (int)points.size() : (int)prev_points.size();
                        for (int i = 0; i<m; i++){
                            std::pair<keyWV, unsigned> p, prev_p;
                            p = points[i]; prev_p = prev_points[i];
                            SwapPoints(prev_p, p);
                            std::vector<std::pair<keyWV, unsigned>> inter_points;
                            inter_points = BresenLine3d(prev_p, p, n, g);
                            //if (inter_points.empty()) break;
/*
                            for (int l = 0; l < (int)inter_points.size(); l++){
                                std::pair<keyWV, unsigned> p = points[l];
                                cellVW cell; cell.currentCost = 0;
                                WriteCube(p.first.first, p.first.second, p.second, cell, delta, name);
                                WriteCube(p.first.first, p.first.second, p.second, cell, delta, nameIt);
                            }

                            g << "splot 'dovt_forbidden.dat' u 1:2:3 w pm3d\n";
                            g.flush();
                            gIt << "splot 'dovt_forbiddenIt.dat' u 1:2:3 w pm3d\n";
                            gIt.flush();

                            std::ofstream file(nameIt, std::ios_base::trunc);
                            file.close();
*/
                        }

                    }else
                        primero = false;

                    prev_points = points;
                    //*/
                }
            }

        } else{
            std::ofstream fpoints("points.dat", std::ios_base::app);
            fpoints << std::endl;
            fpoints.close();
        }
    }

    /*
    g << "set xlabel \"Angular Velocity\"\n";
    g << "set ylabel \"Linear Velocity\"\n";

    std::ofstream f("dataBresen.dat", std::ios_base::trunc);
    f.close();

    int nlayer = 0;
    for (auto layer = vts.data3d.begin(); layer != vts.data3d.end(); ++layer){
        for (auto it = (*layer).mapVW.begin(); it != (*layer).mapVW.end(); ++it) {
            cellVW cell = (*it).second;
            keyWV key = (*it).first;

            if (!cell.free) {
                unsigned j = key.first; unsigned k = key.second;
                WriteCube(j, k, nlayer, cell, delta, "dataBresen.dat");

            }
        }
        nlayer++;
    }
    g << "splot 'dataBresen.dat' u 1:2:3 w pm3d\n";
    //g << "plot 'dataBresen.dat' u 1:2 w l fill\n";
    g.flush();
    */

}

void DOVTS::InsertDOVT(const std::vector<Command>& dovt, unsigned long numC){

    std::string nameFile = "timeFillMap2.dat";
    std::ofstream fdata(nameFile, std::ios::app);

    //Add velocity and time information of obstacles
    //if (obstacles){
        //unsigned long n = space3d.GetSizeT();  //number of cells to be filled for a forbidden command downwards
        unsigned long n = numC;
        //if (o.GetConsider()){
            //if (o.GetW() != 0){ //non-linear obstacles
            //    FillMap(o.GetListaComandosId(), n);    //3D-aproximation version
            //}else{  //linear obstacles
                //space3d.InsertDOVT(o.GetDOVT());    //3D-exact version

                clock_t start = clock();
                FillMap(dovt, n);   //3D-aproximation version
                clock_t finish = clock();

                //std::ofstream fdata("timeFillMap.dat", std::ios::app);
                fdata << ((double)(finish-start))/CLOCKS_PER_SEC << std::endl;
                //time += ((double)(finish-start))/CLOCKS_PER_SEC;

                //space3d.FillMap(o.GetListaComandosId());    //3D-aproximation version
                //insideBC = o.GetEnZona();   //store if the robot is inside the collision band of the obstacle
            //}
        //}

    //}

    fdata.close();
}


//void DOVTS::InsertDOVTAhead(std::vector<std::vector<std::vector<std::pair<Root, Comando>>>> dovtAhead, unsigned long nCells) {
void DOVTS::InsertDOVTAhead(const std::vector<std::vector<std::vector<std::pair<Root, Comando>>>>& dovtAhead, const std::vector<Comando>& dovt, int lookAhead, unsigned long nCells){
    //Insert the values into the grid

    /*
    for (auto it = dovtAhead.begin(); it != dovtAhead.end(); ++it){
        std::vector<std::vector<std::pair<Root, Comando>>> pointAhead = *it;
        for (int k = 0; k<(int)pointAhead.size(); k++){
            int n = k+2;
            std::vector<std::pair<Root, Comando>> data = pointAhead[k];
            for (auto itData = data.begin(); itData != data.end(); ++itData){
                Comando c = itData->second;
                cellVW cell;
                cell.free = false;
                cell.currentCost = maxCost/n;
                cell.previousCost = cell.currentCost;
                std::pair<keyWV, unsigned> key;
                if (GetKey(c.vel, c.t, key)) {
                    auto search = vts.data3d[key.second].mapVW.find(key.first);
                    if (search == vts.data3d[key.second].mapVW.end()) {   //the cell is free and has not been added to the layer
                        InsertKey(key.first, key.second, cell, nCells);
                        //fpoints << c.sup.vel.w << "\t" << c.sup.vel.v << "\t" << c.sup.t << std::endl;
                    }
                } else
                    break;
            }
        }
    }
    //*/

    ///*
    Gnuplot g;

    std::list<Comando> toExplore; int ind = 0;
    std::vector<Comando> stored;
    stored.assign(dovt.begin(), dovt.end());
    for (int k = 0 ; k < lookAhead; k++){

        toExplore.assign(stored.begin(), stored.end());
        stored.clear();

        Command previousC;
        for (auto it = dovtAhead.begin(); it != dovtAhead.end(); ++it){

            std::vector<std::pair<Root, Comando>> data = (*it)[k];
            if (!data.empty()){
                std::sort(data.begin(), data.end(), [](const std::pair<Root, Comando>& cl, const std::pair<Root, Comando>& cr){
                    return (std::atan2(cl.second.vel.v, cl.second.vel.w) < std::atan2(cr.second.vel.v, cr.second.vel.w));
                });

                Comando c1 = toExplore.front(); toExplore.pop_front();
                Comando c2 = data.front().second;
                ///*
                int n = k+2;
                cellVW cell;
                cell.free = false;
                cell.currentCost = maxCost/n;
                cell.previousCost = cell.currentCost;
                std::pair<keyWV, unsigned> key;
                if (GetKey(c2.vel, c2.t, key)) {
                    auto search = vts.data3d[key.second].mapVW.find(key.first);
                    if (search == vts.data3d[key.second].mapVW.end()) {   //the cell is free and has not been added to the layer
                        InsertKey(key.first, key.second, cell, nCells);
                        //fpoints << c.sup.vel.w << "\t" << c.sup.vel.v << "\t" << c.sup.t << std::endl;
                    }
                } else
                    break;
                //*/

                ///*
                FillCells(Command(c1, c2, 1), nCells, g);

                stored .push_back(c2);

                if (it != dovtAhead.begin()){
                    //Compute Commands in between c1 and c2 to plot the cells
                    CompleteLines(Command(c1, c2, 1), previousC, nCells, g);

                }
                previousC = Command(c1, c2, 1);
            }
        }
    }
    //*/
}


void DOVTS::UpdateCell(const keyWV keyVel, const unsigned k, cellVW &cell){

    double cost = 0;
    int    neighbors = 0;

    for (int dw=-1; dw <= 1; dw++)
        for (int dv=-1; dv <= 1; dv++) {
            if (dv != 0 && dw != 0)
                continue;

            int dt;
            dw == 0 && dv ==0 ? dt = 1 : dt = 0;

            if (k+dt >= vts.data3d.size())
                continue;

            keyWV newKey = std::make_pair(keyVel.first + dw, keyVel.second + dv);
            auto &layer = vts.data3d[k+dt].mapVW;
            const auto search = layer.find(newKey);

            if (search != layer.end()) {
                cost += search->second.previousCost;
                neighbors++;
            }
        }

    if (cost >= maxCost) //10000)
        cell.currentCost = cost / 10;
    else
        cell.currentCost = cost / neighbors;
}

void DOVTS::FillCosts(){

    double vSize = sizev + delta;
    double wSize = sizew + delta;
    //Relleno la estructura con celdas libres
    for (unsigned k=0; k<(unsigned) vts.data3d.size(); k++) {
        layerVW layer = vts.data3d[k];
        for (unsigned j=0; j<vSize; j++){
            for (unsigned i=0; i<wSize; i++) {
                keyWV keyVel = std::make_pair(i, j);

                auto search = layer.mapVW.find(keyVel);
                if (search == layer.mapVW.end()){   //the cell is free and has not been added to the layer

                    cellVW cell;
                    cell.free = true; cell.currentCost = 0; cell.previousCost = cell.currentCost;
                    layer.mapVW.insert({keyVel, cell}).second;
                }
            }
        }
        vts.data3d[k] = layer;
    }


    const char* name = "dovt_forbidden.dat";
    std::ofstream forbidden(name, std::ios::trunc); forbidden.close();  //clear the content of the file

    //The costs of the cells are initialized to 0 (free) or maxCost (10000) (forbidden)
    int its = 0;
    while (its++ < 1){

        //Update cell's cost with the current cost
        for (unsigned k=0; k<(unsigned) vts.data3d.size(); k++) {
            for (unsigned j = 0; j < vSize; j++) {
                for (unsigned i = 0; i < wSize; i++) {

                    keyWV keyVel = std::make_pair(i, j);

                    //equivalente a vts.data3d[k].mapVW[keyVel].previousCost  = data3d[k].mapVW[keyVel].currentCost
                    auto search = vts.data3d[k].mapVW.find(keyVel);
                    search->second.previousCost = search->second.currentCost;

                }
            }
        }

        //Update current cost depending on the cost of the neighbours
        for (unsigned k=0; k<(unsigned) vts.data3d.size(); k++){
            for (unsigned j=0; j<vSize; j++){
                for (unsigned i=0; i<wSize; i++){

                    keyWV keyVel = std::make_pair(i,j);

                    auto search = vts.data3d[k].mapVW.find(keyVel);
                    if (search != vts.data3d[k].mapVW.end()){
                        cellVW &cell = search->second;
                        if (cell.free) {
                            if (cell.currentCost == 0){
                                UpdateCell(keyVel, k, cell);
                            }
                        }
                        if (cell.currentCost > 0)
                            WriteCube(i, j, k, cell, delta, name);
                    }
                }
            }
        }
    }
}

/*
const std::vector<Velocidad>& GenerateVelSeq(Velocidad vini, Velocidad vfin, constraints dw){

    //To generate the sequence of velocities between two velocities
    std::vector<Velocidad> seq;

    double aw = dw.aw; double t = dw.t;
    double radio = vfin.v / vfin.w;
    double vi, wi;

    seq.push_back(vini);
    wi = vfin.w - aw*t; vi = wi*radio;
    while (vi > 0) {
        seq.push_back(Velocidad(vi,wi));
        wi = wi - aw*t; vi = wi*radio;
    }
    seq.push_back(Velocidad(0,0));


    //To associate time for the sequence of velocities
    std::vector<Velocidad>::reverse_iterator it;
    for (it = seq.rbegin(); it != seq.rend(); it--){
        Velocidad vel = (*it);
        if (!space3d.IsValidWV(vel) || !space3d.IsValidT(t)) break;
        space3d.InsertElement(vel, t);
        t += t;
    }
}
//*/

const map3d<layerVW>& DOVTS::GetVTSpace() {

    return vts;
}

unsigned DOVTS::GetSizeV() {
    return sizev;
}

unsigned DOVTS::GetSizeW() {
    return sizew;
}

void DOVTS::ClearVTSpace(){

    for (unsigned i = 0; i<(unsigned)vts.data3d.size(); i++){
        layerVW layer = vts.data3d[i];
        //layer.mapVW.clear();
        vts.data3d[i].mapVW.clear();
    }
}


void WriteCube(unsigned w, unsigned v, unsigned t, cellVW cell, int delta, const char *name) {
//w, v is the center of the cell
    double i = w; double j = v; double k = t;
    i -= delta/2.0; j -= delta/2.0;
    ///*
    std::ofstream file(name, std::ios_base::app);
    int cost = cell.currentCost;
    //store the points for the squares in (w,v) plane in a file
    file << i << "\t" << j << "\t" << k << "\t" << cost << "\n";
    file << i << "\t" << j << "\t" << k + delta << "\t" << cost << "\n";
    file << i << "\t" << j + delta << "\t" << k + delta << "\t" << cost << "\n";
    file << i << "\t" << j + delta << "\t" << k << "\t" << cost << "\n";
    file << i << "\t" << j << "\t" << k << "\t" << cost << "\n\n";

    //store the points for the cube (t,w,v) in a file
    file << i + delta << "\t" << j << "\t" << k << "\t" << cost << "\n";
    file << i + delta << "\t" << j << "\t" << k + delta << "\t" << cost << "\n";
    file << i + delta << "\t" << j + delta << "\t" << k + delta << "\t" << cost << "\n";
    file << i + delta << "\t" << j + delta << "\t" << k << "\t" << cost << "\n";
    file << i + delta << "\t" << j << "\t" << k << "\t" << cost << "\n\n";

    file << i << "\t" << j << "\t" << k << "\t" << cost << "\n";
    file << i + delta << "\t" << j << "\t" << k << "\t" << cost << "\n";
    file << i + delta << "\t" << j + delta << "\t" << k << "\t" << cost << "\n";
    file << i << "\t" << j + delta << "\t" << k << "\t" << cost << "\n";
    file << i << "\t" << j << "\t" << k << "\t" << cost << "\n\n";

    file << i << "\t" << j << "\t" << k + delta << "\t" << cost << "\n";
    file << i + delta << "\t" << j << "\t" << k + delta << "\t" << cost << "\n";
    file << i + delta << "\t" << j + delta << "\t" << k + delta << "\t" << cost << "\n";
    file << i << "\t" << j + delta << "\t" << k + delta << "\t" << cost << "\n";
    file << i << "\t" << j << "\t" << k + delta << "\t" << cost << "\n\n";

    file << "\n";
    file.close();
    //*/
}


void DOVTS::Plot(Gnuplot &gp, const Velocidad followGoal, const double t) {
//Plot the DOVTS with the dynamic obstacles

    //gp.clear();
    //set/unset parametric????

    //gp << "set nokey\n set hidden3d\n";
    //gp << "set size ratio -1\n";

    //gp << "set view 0,0\n";
    //gp << "set terminal x11 replotonresize\n"; //  size 900,700
    //gp << "set view 0,90\n";
    /*
    gp << "set nokey \n";
    gp << "set hidden3d\n";
    gp << "set xrange [" << vs->GetWMaxRight() << ":" << vs->GetWMaxLeft() << "]\n";
    gp << "set yrange [" << 0 << ":" << vs->GetVMax() << "]\n";
    gp << "set zrange [0:" << stept*vts.data3d.size() << "]\n";
    gp << "set ticslevel 0\n";
    //*/

    gp << "set xlabel \"Angular Velocity\"\n";
    gp << "set ylabel \"Linear Velocity\"\n";
    gp << "set zlabel \"Time\"\n";

    bool layout;
    std::vector<std::tuple<double, double, double>> pts;

    const char* name = "dovt_forbidden.dat";
    std::ofstream forbidden(name, std::ios::trunc); forbidden.close();  //clear the content of the file

    int nforbidden = 0;
    for (unsigned i = 0; i < (unsigned) vts.data3d.size(); i++) {
        layerVW layer = vts.data3d[i];
        map2d< keyWV, cellVW >::iterator it;
        for (it = layer.mapVW.begin(); it != layer.mapVW.end(); ++it){
            cellVW cell = (*it).second;
            keyWV key = (*it).first;

            if (!cell.free){
                unsigned j = key.first; unsigned k = key.second;

                //store the points for the squares in (w,v) plane in a file
                WriteCube(j, k, i, cell, delta, name);

                pts.push_back(std::make_tuple(j,k,i)); //pts.push_back(std::make_tuple(i,j,k));
                pts.push_back(std::make_tuple(j,k+1,i));//pts.push_back(std::make_tuple(i,j,k+1));
                pts.push_back(std::make_tuple(j+1,k+1,i));//pts.push_back(std::make_tuple(i,j+1,k+1));
                pts.push_back(std::make_tuple(j+1,k, i));//pts.push_back(std::make_tuple(i,j+1,k));
                pts.push_back(std::make_tuple(j,k, i));//pts.push_back(std::make_tuple(i,j,k));
                //points << "\n";

                pts.push_back(std::make_tuple(j,k,i+1)); //pts.push_back(std::make_tuple(i+1,j,k));
                pts.push_back(std::make_tuple(j,k+1,i+1)); //pts.push_back(std::make_tuple(i+1,j,k+1));
                pts.push_back(std::make_tuple(j+1,k+1,i+1)); //pts.push_back(std::make_tuple(i+1,j+1,k+1));
                pts.push_back(std::make_tuple(j+1,k,i+1)); //pts.push_back(std::make_tuple(i+1,j+1,k));
                pts.push_back(std::make_tuple(j,k,i+1)); //pts.push_back(std::make_tuple(i+1,j,k));
                //points << "\n";

                pts.push_back(std::make_tuple(j,k,i)); //pts.push_back(std::make_tuple(i,j,k));
                pts.push_back(std::make_tuple(j,k,i+1)); //pts.push_back(std::make_tuple(i+1,j,k));
                pts.push_back(std::make_tuple(j+1,k,i+1)); //pts.push_back(std::make_tuple(i+1,j+1,k));
                pts.push_back(std::make_tuple(j+1,k,i)); //pts.push_back(std::make_tuple(i,j+1,k));
                pts.push_back(std::make_tuple(j,k,i)); //pts.push_back(std::make_tuple(i,j,k));
                //points << "\n";

                pts.push_back(std::make_tuple(j,k+1,i)); //pts.push_back(std::make_tuple(i,j,k+1));
                pts.push_back(std::make_tuple(j,k+1,i+1)); //pts.push_back(std::make_tuple(i+1,j,k+1));
                pts.push_back(std::make_tuple(j+1,k+1,i+1)); //pts.push_back(std::make_tuple(i+1,j+1,k+1));
                pts.push_back(std::make_tuple(j+1,k+1,i)); //pts.push_back(std::make_tuple(i,j+1,k+1));
                pts.push_back(std::make_tuple(j,k+1,i)); //pts.push_back(std::make_tuple(i,j,k+1));

                nforbidden++;
            }
        }
    }

    //goal information
    std::pair<keyWV, unsigned> key, key1, keyR;
    GetKey(this->GetGoal().commandGoal, vts.data3d.size()-1 * stept, key);
    auto search = vts.data3d[key.second].mapVW.find(key.first);
    cellVW cell = search->second;

    GetKey(followGoal, t, key1);

    double wSize = sizew + delta/2.0;
    double vSize = sizev + delta/2.0;
    //gp << "set view equal xy\n";
    gp << "set xrange [" << -delta/2.0 << ":" << wSize << "]\n";
    gp << "set yrange [" << -delta/2.0 << ":" << vSize << "]\n";
    gp << "set zrange [0:" << vts.data3d.size() << "]\n";
    //gp << "set xtics " << -delta/2.0 << "," << 1 << "\nset ytics " << -delta/2.0 << "," << 1 << "\n";
    gp << "set xtics " << 0 << "," << 1 << "\nset ytics " << 0 << "," << 1 << "\n";
    gp << "set mxtics " << 2 << "\nset mytics " << 2 << "\n";
    gp << "set grid mxtics mytics\n";
    gp << "set size ratio -1\n";

    //gp << "splot 'dovt_forbidden.dat' u 1:2:3 w lp, '' u 4:5:6 w lp\n";
    gp << "goalw = " << key.first.first << "\n";
    gp << "goalv = " << key.first.second << "\n";
    gp << "goalt = " << key.second << "\n";
    gp << "followGoalw = " << key1.first.first << "\n";
    gp << "followGoalv = " << key1.first.second << "\n";
    gp << "followGoalt = " << key1.second << "\n";
    //gp << "print goalw\n"; gp << "print goalv\n"; gp << "print followGoalw\n"; gp << "print followGoalv\n"; gp << "print followGoalt\n";
    //gp << "set label at goalw,goalv,goalt \"\" point pointtype 7 pointsize 2 lc 002\n"; //green
    gp << "set label at followGoalw,followGoalv,followGoalt \"\" point pointtype 7 pointsize 2 lc 003\n"; //blue

    GetKey(GetAgent(), 0.0, keyR);
    gp << "robotw = " << keyR.first.first << "\n";
    gp << "robotv = " << keyR.first.second << "\n";
    //gp << "set label at robotw,robotv \"R\" point pointtype 7 pointsize 2 lc 004\n";
    gp << "set label at robotw,robotv \"R\" boxed lc 004\n";

    std::ofstream frob("infoRobotGoal.dat", std::ios::trunc);
    frob << key.first.first << "\t" << key.first.second << "\t" << key.second << "\t" << keyR.first.first << "\t" << keyR.first.second << std::endl;
    frob.close();


    /*
    for (unsigned i = 0; i < (unsigned) vts.data3d.size(); i++) {
        layerVW layer = vts.data3d[i];
        map2d<keyWV, cellVW>::iterator it;
        for (it = layer.mapVW.begin(); it != layer.mapVW.end(); ++it) {
            cellVW cell = (*it).second;
            keyWV key = (*it).first;

            gp << "keyw = " << key.first << "\nkeyv = " << key.second << "\nkeyt = " << i << "\n";
            gp << "set label at keyw,keyv,keyt \"" << cell.currentCost << "\" lc 004\n";
        }
    }
    */


    //gp << "splot 'dovt_forbidden.dat' u 1:2:3 w l, 'pathAstar.dat' u 1:2:3 w lp lc 003, 'pathAstar2.dat' u 1:2:3 w lp lc 004\n"; //, '<echo $goalw $goalv $goalt'\n";
    gp << "splot 'dovt_forbidden.dat' u 1:2:3 w pm3d, 'pathAstar.dat' u 1:2:3 w lp lc 003\n"; //, 'pathAstar2.dat' u 1:2:3 w lp lc 004\n"; //, '<echo $goalw $goalv $goalt'\n";
    //gp << "plot 'dovt_forbidden.dat' u 1:2 w l, 'pathAstar.dat' u 1:2 w lp lc 003\n";
    //gp << "replot\n";
    gp.flush();
    gp << "unset label\n";
    gp << "unset xtics\nunset ytics\nunset mxtics\nunset mytics\n";
    gp << "unset grid\n";

    /*
    //dynamic window for the robot
    std::vector<std::tuple<double, double, double>> dw;
    dw.push_back(std::make_tuple(robot_w + aw * t, robot_v, 0));
    dw.push_back(std::make_tuple(robot_w, robot_v + av * t, 0));
    dw.push_back(std::make_tuple(robot_w - aw * t, robot_v, 0));
    dw.push_back(std::make_tuple(robot_w, robot_v - av * t, 0));
    dw.push_back(std::make_tuple(robot_w + aw * t, robot_v, 0));

    gp << "splot '-' with lines lc 002\n";
    gp.send1d(dw);
    */

    ///* //Plot the whole 3d space
    //gp << "set dgrid3d\n";
    //gp << "splot 'DOVTS.dat' u 2:3:1 w lp lc 010\n";
    //gp << "set pm3d  \n";
    //gp << "set style data pm3d \n";
    //gp << "set pm3d depthorder hidden3d\n";
    //gp << "set pm3d implicit\n";
    //gp << "unset hidden3d\n";
    //gp << "n = " << n << "\n";
    //gp << "splot for[in=0:n-1] 'DOVTS.dat' i in u 2:3:1 w l notitle\n";
    //gp << "splot 'DOVTS.dat' u 2:3:1:4 w l\n";
    //gp << "splot for[in=0:4:(n-1)*4] 'DOVTS.dat' i in:in+3 u 2:3:1:($1) with pm3d\n";
    //gp.flush();

    //gp << "unset pm3d \n";
    //*/

    /* //Plot only the forbidden data
    gp << "set pm3d depthorder hidden3d\n";
    gp << "set pm3d implicit\n";
    gp << "unset hidden3d\n";
    //gp << "set style data pm3d \n";
    //gp << "set hidden3d \n";
    gp << "n = " << n << "\n";
    gp << "splot 'dovt_forbidden.dat' u 2:3:1:4 w l notitle\n";
    gp << "unset pm3d \n";
    gp.flush();

    gp << "set dgrid3d gauss \n";
    gp << "n = " << n << "\n";
    //gp << "splot for[in=0:n-1] 'DOVTS.dat' i in u 2:3:1 w l notitle\n";
    gp << "splot 'dovt_forbidden.dat' u 2:3:1 w l notitle\n";
    gp << "unset dgrid3d \n";

    gp.flush();
    //*/

    //2ª forma plot
    //gp << "set pm3d\n";
    //gp << "set dgrid3d\n";
    //gp << "set style data pm3d\n";
    //gp << "splot '-' with lp lc 010 notitle\n";
    //gp.send1d(pts);
    //gp.flush();
    //gp << "unset pm3d \n";

    //gp.send1d(pts);
    //gp << "unset pm3d\n";
    //gp << "unset multiplot\n";

}

/*
void DOVTS::PlotPlan(Gnuplot &gp, std::list<Velocidad>& plan) {

    //std::list<Velocidad> lista = plan;
    std::list<Velocidad>::iterator it; // = lista.begin();

    const char* name = "plan.dat";
    std::ofstream forbidden(name, std::ios::trunc); forbidden.close();  //clear the content of the file

    std::vector<std::tuple<double, double, double>> pts;

    unsigned n = 1;
    for (it = plan.begin(); it != plan.end(); ++it){
        Velocidad vel = (*it);
        std::pair<keyWV, unsigned> key;
        GetKey(vel, n*stept, key);

        pts.push_back(std::make_tuple(key.first.first, key.first.second, key.second));
        n++;
    }

    //gp << "set view equal xyz\n";
    gp << "reread; replot; splot '-' with lp lc 010 notitle\n";
    gp.send1d(pts);
}
*/

bool init = false;

void DOVTS::PlotDislin() {

    if (!init) {
        metafl("XWIN");
        scrmod("revers");
        //winsiz(550, 340);
        winsiz(900, 700);
        //window(425,0,400,325);
        disini();
        nochek(); // Don't warn a
        init = true;
    }
    //selwin(id);
    //nochek();	//nochek is used to suppress the listing of points that lie outside of the axis scaling
    name("Angular velocity","x");
    name("Linear Velocity","y");
    name("Time","z");

    graf3d(0, sizew, 0, 1, 0, sizev, 0, 1, 0, vts.data3d.size(), 0, 5);
    //dash();
    //grid(1, 1);

    ///*
    double tfb[40][10][10], vfb[10], wfb[10], zfb[40];
    int nforbidden = 0;

    for (unsigned j=0; j < sizew; j++) wfb[j] = j;
    for (unsigned j=0; j < sizev; j++) vfb[j] = j;
    for (unsigned j=0; j < (unsigned) vts.data3d.size(); j++) zfb[j] = j;

    for (unsigned i = 0; i < (unsigned) vts.data3d.size(); i++) {
        layerVW layer = vts.data3d[i];
        map2d< keyWV, cellVW >::iterator it;
        for (it = layer.mapVW.begin(); it != layer.mapVW.end(); ++it){
            cellVW cell = (*it).second;
            keyWV key = (*it).first;

            unsigned j = key.first; unsigned k = key.second;
            tfb[i][j][k] = 1;
        }
    }


    curv4d ((float *)wfb, (float *)vfb, (float *)zfb, (float *)tfb, nforbidden);
    //surshd ((float *)wfb, 10, (float *)vfb, 10, (float *) tfb);

    selwin(1);
    //disfin();

    //*/
}


double DOVTS::GetTh() {
    return  stept*vts.data3d.size();
}

unsigned long DOVTS::GetSizeT() {
    return vts.data3d.size();
}

void DOVTS::CompleteLines(Command currentC, Command previousC, unsigned long nCells, Gnuplot &g) {
    //Generate the line between the boundsVS- 3D line

    Eigen::Vector3d psup(currentC.sup.vel.w, currentC.sup.vel.v, currentC.sup.t);
    Eigen::Vector3d pinf(currentC.inf.vel.w, currentC.inf.vel.v, currentC.inf.t);
    Eigen::Vector3d previous_psup(previousC.sup.vel.w, previousC.sup.vel.v, previousC.sup.t);
    Eigen::Vector3d previous_pinf(previousC.inf.vel.w, previousC.inf.vel.v, previousC.inf.t);

    //check for interpolation between limit points in the dov
    Eigen::Vector3d dir_sup = previous_psup - psup; //from psup (current one) to previous_psup
    Eigen::Vector3d dir_inf = previous_pinf - pinf; //from pinf (current one) to previous_pinf

    double dsup = std::sqrt(std::pow(previous_psup[0] - psup[0], 2) + std::pow(previous_psup[1] - psup[1], 2) + std::pow(previous_psup[2] - psup[2], 2));
    double dinf = std::sqrt(std::pow(previous_pinf[0] - pinf[0], 2) + std::pow(previous_pinf[1] - pinf[1], 2) + std::pow(previous_pinf[2] - pinf[2], 2));

    double stepw = std::abs(bounds.wmax_right - bounds.wmax_left)/sizew;
    double stepv = bounds.vlim_max/sizev;

    int n = 0; double step_sup = stept; double step_inf = stept;
    if (dsup > dinf){
        //n = std::ceil(dsup/stept);
        n = std::ceil(dsup/stepv);
        step_inf = dinf/n;
    }
    else{
        //n = std::ceil(dinf/stept);
        n = std::ceil(dinf/stepv);
        step_sup = dsup/n;
    }

    if (n > 1){ //Interpolate radius

        //step_sup = stept; step_inf = stept; n = 4;
        step_sup = stepv; step_inf = stepv; //n = 8;

        for (int j = 0; j<n ; j++){
            double xsup = j*step_sup*dir_sup[0] + psup[0];
            double ysup = j*step_sup*dir_sup[1] + psup[1];
            double zsup = j*step_sup*dir_sup[2] + psup[2];

            double xinf = j*step_inf*dir_inf[0] + pinf[0];
            double yinf = j*step_inf*dir_inf[1] + pinf[1];
            double zinf = j*step_inf*dir_inf[2] + pinf[2];

            //TODO: CHANGE THE ALGORITHM TO PLOT 3D LINE THROUGH THE GRID
            if (zsup > 0 && zinf <0 ) {

                Comando sup(Velocidad(ysup, xsup), zsup);
                Comando inf(Velocidad(yinf, xinf), zinf);

                FillCells(Command(sup, inf, 1), nCells, g);
            }
        }
    }
}

const layerVW & DOVTS::GetLayer(unsigned k) {

    if (k >= vts.data3d.size())
        return layerVW();

    return vts.data3d[k];
}
