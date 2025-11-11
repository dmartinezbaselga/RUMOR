//
// Created by maite14 on 7/12/17.
//

#ifndef VS_STRATEGIES_H
#define VS_STRATEGIES_H

#include "dovs.h"
#include "utilidades.h"


#include <CGAL/Cartesian.h>
#include <CGAL/MP_Float.h>
#include <CGAL/Quotient.h>
#include <CGAL/Arr_segment_traits_2.h>
#include <CGAL/Sweep_line_2_algorithms.h>


//#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Polygon_2_algorithms.h>
#include <CGAL/enum.h>

#include <CGAL/Polygon_2.h>

#include <CGAL/Simple_cartesian.h>

//definiciones para funciones que calculan intersecciones de poligonos con poligonos o puntos
//typedef CGAL::Quotient<CGAL::MP_Float>                  NT;
//typedef CGAL::Simple_cartesian<NT>                             Kernel;
typedef CGAL::Simple_cartesian<double>					Kernel;
//typedef Kernel::Point_2                                 Point_2;
//typedef CGAL::Arr_segment_traits_2<Kernel>              Traits_2;
//typedef Traits_2::Curve_2                               Segment_2;

//definiciones para funcion que calcula si un punto cae dentro, fuera o en el limite de un poligono
typedef CGAL::Exact_predicates_exact_constructions_kernel	K;
typedef K::Point_2 Point;
typedef K::Point_2 Point_2;
typedef CGAL::Arr_segment_traits_2<K>            	Traits_2;
typedef Traits_2::Curve_2                    		Segment_2;


typedef struct {
    int ini, fin;
}Par;

class Polygon{

    std::vector<Velocidad> points;
    std::vector<Par> lim_poligono;

public:
    Polygon(){};
    ~Polygon(){
        points.clear();
        lim_poligono.clear();
    }

    void InsertCommands(const std::vector<dovt>& data);
    void InsertCommands(const std::vector<Command>& commands);
    int Size();
    std::vector<Velocidad> GetPoints();
    std::vector<Par> GetLimPol();
    bool InsidePolygon(Velocidad vel);

    void InsertCommandsAhead(const std::vector<dovtAhead> &commands);
};

typedef struct{
    double ini, fin;
}ParAng;

typedef struct{
    Velocidad ini, fin;
}ParVel;

typedef struct{
    ParAng ang;	//angular bounds
    ParVel vel;	//velocity bounds
    Velocidad depth;	//profundidad del hueco
    double width;	//angular size = ang.fin - ang.ini
}Space;

typedef struct{
    int ini, fin, piso;
}infoValle;

class Strategies {

    double stept;
    Polygon pol;
    std::vector<std::unique_ptr<Agent>> *agents;
    int idAg;
    Tsc posAg;  //agent position in world coordinates
    Tpf goal;   //goal in world coordinates
    Velocidad dir_goal;

    //Heuristic variables for planning
    double wmax_steering, ang_left, ang_right, wlim_z2, valley_depth, cte_goal_valley, vorientacion, worientacion, waling_lim;

    //Variables obtained from analizing free and occupied zones in DOVS
    std::vector<Par> zonaOcupados;	//para almacenar zonas(ini, fin) con comandos de velocidad ocupados
    std::vector<ParAng> zonaLibres;	//almacena zonas libres de colisión, que son los ángulos de los radios que son libres
    std::vector<ParVel> velLibres;	//velocidades sup. correspondientes a los limites de las zonas libres
    std::vector<infoValle> valles_left, valles_z2, valles_right;
    std::vector<std::pair<Command, Command> > limites_DOV;	//Comandos que delimitan los diferentes DOV en el VS
    
    std::vector<Space> space_inDOV, space_outDOV;	//spaces inside and outside the bounds of the DOV

    std::vector<std::pair<double, double>> walign;	//pares de velocidad walign (left,right)

    std::vector<std::pair<std::pair<Velocidad, Velocidad>, std::pair<double, double> >> canal;	//pares de velocidad canal (left,right) y el tiempo

    bool hay_comandos_z1 = false; bool hay_comandos_z2 = false; bool hay_comandos_z3 = false;
    bool zona1 = false; bool zona2 = false; bool zona3 = false;

    // andrew
    float maxAngularVelocity(float linearVelocity);
    //andrew
    bool brake = false;

    bool IntersectRangeDW_Goal(DW_Range& range);
    DW_Range DW_VelocityRange();
    bool MaxFreeVelocityInRange(DW_Range points, Velocidad &best_vel);

    bool IsVelInZone2(Velocidad vel);
    void ComputeSpaces();
    void AnalyseCommands();

    bool IntersectionDinGoal(Velocidad ini, Velocidad fin);
    bool IsSafeVelocity(Velocidad vel); //, std::vector<std::unique_ptr<Agent>> *agents);
    bool IsVelocityFree(Velocidad vel);
    bool IsVelInGoalSpace(Velocidad vel, Velocidad dir_goal);

    bool NewDirGoal(bool noObstacles);
    bool ReachableValley(Command cini, Command cpiso, Command cfin);
    bool ReachableSpace(Space sp);

    bool InValleyZ2(Velocidad vel, infoValle& valleOut);
    bool InSpaceDOV(Velocidad vel, Space& spOut);
    Tsc PredictPosition_Kstepts(double v, double w, int k);
    Tsc PredictPosition(double v, double w);
    Velocidad ComputeNewVel(Velocidad vel);
    bool IsDirGoalRadiusFree();
    void Evaluate_DWMaxAccCommands();
    bool InSpaceInsideDOV(Velocidad vel);
    bool NewDirGoalOutsideDOV();


    Velocidad FollowGoal(Tpf goal);

    //Strategies
    Velocidad ReachGoal();
    Velocidad Estrategia_escaparBC();
    Velocidad EscapeCB(bool originInside);
    Velocidad OrientacionGoalCanal(Velocidad canal);
    Velocidad ComputeVelGoalRobotEnZona();
    Velocidad AlineacionWObstacle(double walign);
    Velocidad ObjSinValleZ2ConHueco(double walign);
    Velocidad UnderDirGoal();
    Velocidad ObjSinValleSinHueco();
    Velocidad DetrasObjeto(double ang);
    Velocidad NoHayVallesZ2(double ang_ini, double ang_fin);
    Velocidad PassingBefore(Velocidad piso);
    Velocidad SelectStrategy();
    Velocidad AlineacionGoal2();
    Velocidad AlineacionGoal();
    Velocidad SeguirGoal();
    Velocidad FreeMotion(bool noObstacles);

    //Greedy
    PosibleVelocidad findClosestReachable(Velocidad target, bool safe);
    PosibleVelocidad findClosestSafe();

public:
    DOVS space;
    DW dwAg;

    Strategies(double time_step):stept(time_step){};
    Strategies(double time_step, boundsVS bounds):stept(time_step){
        space = DOVS(bounds);
    };
    ~Strategies(){};

    void SetHeuristicValues(double wmax_steering, double al, double ar, double wz2, double waling, double valley, double goal_valley, double vorient, double worient);
    //Computes the next velocity to be applied by the agent
    bool ComputeMotion(int agId, Tsc posR, Tpf posG, std::vector<std::unique_ptr<Agent>> *agents, bool originInside, Velocidad& motion);

    // andrew: reinforced learning
    std::vector<std::pair<int, Velocidad>> getSafeVelocities(int agId, Tsc posR, Tpf posG, std::vector<std::unique_ptr<Agent>> *agents, bool originInside, Velocidad& motion, bool &freeActions,
    bool learning = false);
    int getCurrentZone();
    bool isDirGoalFree();
    std::vector<int> goalDirInDW(std::vector<Velocidad>& possibleVelocities);

    Velocidad Greedy();

    //function added for the reciprocal case, to look for a new goal closer to the real one in the velocity zone within same direction
    Velocidad ComputeClosest(Velocidad goalTarget);

    const Velocidad GetDirGoal(){ return  dir_goal;}

};


#endif //VS_STRATEGIES_H
