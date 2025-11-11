#include <cmath>
#include <math.h>
#include "Agent.h"
#include "calcul.h"
#include "utilidades.h"
#include <algorithm>
#include <iostream>
#include <fstream>
#include <assert.h>

#include "graficas.h"
#include "config.h"
#include "TData.h"

//#include <CGAL/Interval_nt.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>


//static double NormalizarPI(double ang){
//    return (ang+(2*M_PI)*std::floor((M_PI-ang)/(2*M_PI)));
//}

double offset = 1e-3*0.25/2;

double NormalisePI(double d){
//Transforms the value into +/-pi range

    while (d > M_PI) d -= 2*M_PI;
    while (d < -M_PI) d += 2*M_PI;

    if (d>=-offset && d<=offset) return 0.0;
    else if (d>=M_PI-offset && d<=M_PI+offset) return M_PI;
    else if (d>=-(M_PI+offset) && d<=-(M_PI-offset)) return -M_PI;
    else if (d>=M_PI_2-offset && d<=M_PI_2+offset) return M_PI_2;
    else if (d>=-(M_PI_2+offset) && d<=-(M_PI_2-offset)) return -M_PI_2;

    return d;
}

Agent::Agent(int i, double x, double y, double tita, double v0, double w0, double r, bool newActive, bool graph, 
bool segment, double distance_segment, Tpf first_point, Tpf last_point){
    active = newActive;
	id = i;
	loc.x = x; loc.y = y; loc.tita = tita;
	v = v0; w = w0; apparent_radius = r; real_radius = r;
    av = 0; aw = 0;
    this->v0 = v0;
    this->w0 = w0;
    this->graph = graph;
    this->segment = segment;
    this->distance_segment = distance_segment;
    this->first_point = first_point;
    this->last_point = last_point;

	//AddTraza(loc);

    //consider = true;
}

Agent::~Agent(){}

void Agent::Update(double t) {
//Update position of the agent (differential-drive agent)

    if (std::abs(w) < 1e-5){
        loc.x = loc.x + v*std::cos(loc.tita)*t;
        loc.y = loc.y + v*std::sin(loc.tita)*t;
    }
    else{	//w != 0
        loc.x = loc.x - (v/w)*std::sin(loc.tita) + (v/w)*std::sin(loc.tita+w*t);
        loc.y = loc.y + (v/w)*std::cos(loc.tita) - (v/w)*std::cos(loc.tita+w*t);
        loc.tita = NormalisePI(loc.tita + w*t);
    }
}

int Agent::GetId() {
    return id;
}

double Agent::GetV() {
    return v;
}

double Agent::GetW() {
    return w;
}

Tsc Agent::GetLocalization(){

    return loc;
}

double Agent::GetRealRadius() {
    return real_radius;
}

void Agent::SetVelocity(Velocidad vel) {

    v = vel.v;
    w = vel.w;
}

void Agent::SetAcceleration(double av, double aw) {
    this->av = av;
    this->aw = aw;
}

void Agent::SetLocalization(Tsc l){

    loc = l;
}

double Agent::GetAV() {
    return av;
}

double Agent::GetAW() {
    return aw;
}

double Agent::getV0() {
    return v0;
}

double Agent::getW0() {
    return w0;
}

double Agent::getDistanceSegment(){
    return distance_segment;
}

bool Agent::isSegment(){
    return segment;
}

Tpf Agent::getFirstPoint(){
    return first_point;
}

Tpf Agent::getLastPoint(){
    return last_point;
}

void Agent::printAgent() {
    if (this->active) std::cout << "Active agent" << std::endl;
    else std::cout << "Passive agent" << std::endl;
    std::cout << "-------------" << std::endl;
    std::cout << "radio: " << this->GetRealRadius() << std::endl;
    std::cout << "x: " <<this->GetLocalization().x << std::endl;
    std::cout << "y: " << this->GetLocalization().y << std::endl;
    std::cout << "theta: " << this->GetLocalization().tita << std::endl;
    std::cout << "v0: " << this->getV0() << std::endl;
    std::cout << "w0: " << this->getW0() << std::endl;
    std::cout << "av: " << this->GetAV() << std::endl;
    std::cout << "aw: " << this->GetAW() << std::endl << std::endl;

}
