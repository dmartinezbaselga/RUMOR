//
// Created by maite14 on 3/02/17.
// Data types fot the definition of DOVTS
//

#ifndef VS_DATADOVTS_H
#define VS_DATADOVTS_H

#include <unordered_map>
#include <boost/functional/hash.hpp>

template <typename T1, typename T2, typename TD>
struct cell2d{

    struct id {T1 id_1d; T2 id_2d;}; //value for the cell, representative value for data which corresponds to the cell

    bool free = true;  //by default, the cell is free; false if it is occupied by an obstacle (modified during navigation)

    double currentCost = 0;
    double previousCost = 0;

    TD data;
};

//template <typename key, typename T>
template <typename key, typename T>
using map2d = std::unordered_map<key, T, boost::hash<key> >;

template <typename T>
struct map3d{   //velocity and time map

    //size of the vector counts the number of time steps to reach the time horizon which is used for storing information and planning
    std::vector<T> data3d;    //vector de capas de datos de tipo T;

    map3d(){};
    map3d(unsigned nd){ //el tamaño del vector es fijo, en función del tiempo horizonte disponible para planificar y el time step entre iteraciones
        data3d.reserve(nd);
    }
    ~map3d(){
        data3d.clear();
    };
};

#endif //VS_DATADOVTS_H
