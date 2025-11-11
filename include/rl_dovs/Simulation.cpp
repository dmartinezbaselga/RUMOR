/*
 * Simulation.cpp
 * Author: Maite and Andrew
 */

#include <chrono>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <thread>
#include <string>
#include <cmath>
#include <cfloat>
#include "config.h"
#include "Simulation.h"
#include "graficas.h"
#include "utilidades.h"
#include <dislin.h>
#include <string.h>
#include "LinearAgent.h"
#include <eigen3/Eigen/Dense>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <math.h>
#include <mutex>

std::string const dis_lib("dislin");

int num_obs;
double xmin, xmax, ymin, ymax, titamin, titamax, vmin, vmax, wmin, wmax, num_times;
double lim_xmin, lim_xmax, lim_ymin, lim_ymax;
int duplicar = 0; //to store if we want to duplicate the obstacles so that there is the same density in the scenario
//std::vector<Obstaculo> estaticos;
std::vector<std::vector<double>> segmentos_estaticos;
std::vector<Tsc> localizaciones;
std::vector<double> velocidades;
std::vector<double> velocidades_w;
std::vector<double> distantObs;   //distancia a la que hay que generar los obstaculos cada vez que salen del marco
std::vector<Tpf> pointRange;  //punto en el rango de los limites del entorno de trabajo con el que hay que comparar
bool new_goal;
int scenario, iteracion;
//Fichero de log
bool logSim = true;
std::ofstream flog;
char name_fich[50] = "logs/logReciprocal.dat";
//std::strchr(name_fich, nscen+1);
//std::ofstream flog(name_fich);
std::string namelog, namelogFillCells;

bool initialized;
bool in_collision;

char path[150];
char path_init[50] = "new_vs_data/new_log/";

char fich_tgs[50]="log_tangentes";
std::ofstream ftgs(fich_tgs);

char fich[100] = "data/density_DOV";
std::ofstream fdensity(fich);

bool primero;
std::vector<Gnuplot*> gnuplotGp;


std::vector<std::shared_ptr<Agent>> agentsBackUp;

// Variables for scenario creation.
bool Simulation::quit = false;
bool Simulation::insertActive = true;
bool Simulation::insertRandom = false;
int Simulation::passives = 0;
bool Simulation::finished = false;
bool Simulation::start = false;
int Simulation::display_xmax = 0;
int Simulation::display_xmin = 0;
int Simulation::display_ymin = 0;
int Simulation::display_ymax = 0;
std::mutex mu;

std::random_device rd{};
std::mt19937 Simulation::generator{rd()};
std::uniform_real_distribution<double> Simulation::distributionX;
std::uniform_real_distribution<double> Simulation::distributionY;
std::uniform_real_distribution<double> Simulation::distributionTita;
std::uniform_real_distribution<double> Simulation::distributionEpsilon;
std::uniform_real_distribution<double> Simulation::distributionV;
std::uniform_real_distribution<double> Simulation::distributionW;
std::uniform_real_distribution<double> Simulation::distributionFixed;


std::vector<std::unique_ptr<Agent>> Simulation::agents;
double Simulation::min_dist;	//minimum distance to goal

//bool CollisionObs(Obstaculo& obs, double radio_robot);

Simulation::Simulation()//: generator( std::random_device()()) {
{

    time_step = 0;
    min_dist = 0.3; //min_dist = 0.7; //min_dist = 0.2; //TO_DO: pasarlo como parametro


    primero = true;
}

Simulation::~Simulation(){
    agents.clear();
}


bool CollisionObs(Tsc ag1, Tsc ag2, double securityDist){

    double distancia = Distancia(ag1.x - ag2.x, ag1.y - ag2.y);

    if (distancia - securityDist < 0.0)
        return true;
    else
        return false;

}

/**
 * Writes current scenario to "scenario".json
 */ 
void Simulation::writeScenarioJSON(std::string scenario) {
    std::string json = "{\n", type;
    json += "\t\"agents\": [\n";
    for (int i=0; i<(int)agents.size(); i++){
        type = agents[i]->active ? "active" : "passive";
        json += "\t\t{\n";
        json += "\t\t\t\"type\": \"" + type + "\",\n";
        json += "\t\t\t\"radio\": " + std::to_string(agents[i]->GetRealRadius()) + ",\n";
        json += "\t\t\t\"x\": " + std::to_string(agents[i]->GetLocalization().x)+ ",\n";
        json += "\t\t\t\"y\": " + std::to_string(agents[i]->GetLocalization().y)+ ",\n";
        json += "\t\t\t\"theta\": " + std::to_string(agents[i]->GetLocalization().tita)+ ",\n";
        json += "\t\t\t\"v0\": " + std::to_string(agents[i]->getV0())+ ",\n";
        json += "\t\t\t\"w0\": " + std::to_string(agents[i]->getW0());
        if (agents[i]->active) {
            ActiveAgent *active = dynamic_cast<ActiveAgent*>(agents[i].get());
            json += ",\n";
            json += "\t\t\t\"av\": " + std::to_string(active->GetAV()) + ",\n";
            json += "\t\t\t\"aw\": " + std::to_string(active->GetAW()) + ",\n";
            json += "\t\t\t\"goal\": {\n";
            json += "\t\t\t\t\"x\": " + std::to_string(active->GetFirstGoal().x) + ",\n";
            json += "\t\t\t\t\"y\": " + std::to_string(active->GetFirstGoal().y) + "\n";
            json += "\t\t\t}\n";
        } else {
            json += "\n";
        }
        if (i == (int)agents.size()  -1) json += "\t\t}\n";
        else json += "\t\t},\n";
    }
    json += "\t]\n";
    json += "}";
    std::ofstream of;
    std::cout << "Writting scenario to " << scenario << ".json..." << std::endl;
    of.open("./escenarios/" + scenario + ".json");
    of << json;
    of.close();
    std::cout << "Done!" << std::endl << std::endl;
}

bool Simulation::isNumber(char c) {
    return !((c >= 48 && c <= 57) || c == '-' || c == '.');
}

/**
 * Read scenario from "name".json
 */
void Simulation::readScenarioJSON(std::string name) {
    std::string line;
    std::ifstream file("./escenarios/" + name + ".json");
    if (file.is_open()) {
        std::cout << "Reading scenario " << name << " from file " << name << ".json" << std::endl;
        if (file.peek() != EOF) file >> line;
        while (file.peek() != EOF) {
            file >> line;
            boost::trim(line);
            if (line == "\"agents\":") {
                bool active;
                double radio, x, y, theta, v0, w0, av, aw, x_goal, y_goal, titamin_ini = -3.1416,
                       titamax_ini = 3.1416;
                int i = 0;
                file >> line >> line;
                boost::trim(line);
                while (line != "]") {
                    if (line == "{") {
                        //std::cout << "New agent!" << std::endl;
                        file >> line >> line;
                        boost::trim(line);
                        active = line == "\"active\"," ? true: false;
                        file >> line >> line;
                        boost::trim_if(line, &isNumber);
                        radio = ::atof(line.c_str());
                        file >> line >> line;
                        boost::trim_if(line, &isNumber);
                        x = ::atof(line.c_str());
                        file >> line >> line;
                        boost::trim_if(line, &isNumber);
                        y = ::atof(line.c_str());
                        file >> line >> line;
                        boost::trim_if(line, &isNumber);
                        theta = ::atof(line.c_str());
                        file >> line >> line;
                        boost::trim_if(line, &isNumber);
                        v0 = ::atof(line.c_str());
                        file >> line >> line;
                        boost::trim_if(line, &isNumber);
                        w0 = ::atof(line.c_str());
                        if (active) {
                            file >> line >> line;
                            boost::trim_if(line, &isNumber);
                            av = ::atof(line.c_str());
                            file >> line >> line;
                            boost::trim_if(line, &isNumber);
                            aw = ::atof(line.c_str());
                            file >> line >> line;
                            file >> line >> line;
                            boost::trim_if(line, &isNumber);
                            x_goal = ::atof(line.c_str());
                            file >> line >> line;
                            boost::trim_if(line, &isNumber);
                            y_goal = ::atof(line.c_str());
                            std::unique_ptr<ActiveAgent> ag 
                                    {new ActiveAgent(i, x, y, theta, v0, w0, av, aw,radio)};
                            ag->AddGoal(x_goal, y_goal);
                            if (logSim){ ag->SetNameLog(name_fich); ag->UpdateLog(); }
                            ag->SetBoundsVS(time_step);
                            agents.push_back(std::move(ag));
                            file >> line;
                        } else {
                            std::unique_ptr<PassiveAgent> pag 
                                    {new PassiveAgent(i, x, y, theta, v0, w0, radio)};
                            pag->SetBoundsVS(time_step);
                            agents.push_back(std::move(pag));
                        }

                        titamin = titamin_ini; titamax = titamax_ini;

                        if (y > ymax/3) {
                            titamax =  -0.7;
                        } else if (y < 0){
                            titamin = 0.7;
                            titamax = 1.57 + 0.7;
                        }

                        titamin = titamin_ini; titamax = titamax_ini;
                        file >> line;
                        i++;
                    } else {
                        std::cerr << "There are no agents in this scenario" << std::endl;
                    }
                    file >> line;
                    boost::trim(line);
                }
            }
        }
        std::cout << "Closing file " << name << ".json" << std::endl;
        file.close();
        std::cout << "Done!" << std::endl << std::endl;
    } else {
        std::cerr << "File " << name << ".json could not be opened" << std::endl;
    }
}

/**
 * Calibrates screen for graphical menu
 */
void Simulation::calibrate() {
    double x, y, tita, v, w, radioAgent;
    radioAgent = 0.3;
    int num_ag = 2;
    double dist_minGoal = 4;
    x = (1.0*std::ceil(std::sqrt((num_ag*M_PI*radioAgent*radioAgent)/(0.04))) < dist_minGoal 
            ? dist_minGoal : 1.5*std::ceil(std::sqrt((num_ag*M_PI*radioAgent*radioAgent)/(0.04))));
    y = x;

    Inicializar();
    SeleccionaVentana(1);
    LimpiaVentana();
    errmod("all", "off");

    double xmin = -10; double xmax = -xmin; double ymin = -10; double ymax = -ymin;
    lim_xmin = xmin; lim_xmax = xmax; lim_ymin = ymin; lim_ymax = ymax;
    DibujaEjes(1, lim_xmin, lim_xmax, lim_xmin, 2, lim_ymin, lim_ymax, lim_ymin, 2);
    TerminaEjes();
    int *x_in = new int(), *y_in = new int();

    std::cout << "Calibrating screen..." << std::endl;
    std::cout << "Touch x=" << lim_xmin << std::endl;
    csrpt1(x_in, y_in);
    display_xmin = *x_in;
    std::cout << "Touch x=" << lim_xmax << std::endl;
    csrpt1(x_in, y_in);
    display_xmax = *x_in;
    std::cout << "Touch y=" << lim_ymin << std::endl;
    csrpt1(x_in, y_in);
    display_ymin = *y_in;
    std::cout << "Touch y=" << lim_ymax << std::endl;
    csrpt1(x_in, y_in);
    display_ymax = *y_in;
    std::cout << "Done calibrating screen!" << std::endl << std::endl;
    std::cout << "Save values to file? [y/n]: ";
    std::string option;
    std::cin >> option;
    if (option == "y") {
        std::ofstream of;
        of.open("data/calibration.txt");
        of << display_xmax << std::endl;
        of << display_xmin << std::endl;
        of << display_ymax << std::endl;
        of << display_ymin << std::endl;
        of.close();
    }
}

/**
 * Console menu for scenario creation
 */
void Simulation::menuCreateScenario() {

    std::cout << "Launching menu..." << std::endl;
    std::string option, type;
    do {
        type = insertActive ? "active": "passive";
        std::cout << std::endl << "SCENARIO CREATOR" << std::endl;
        std::cout << "1- Insert new active agent" << std::endl;
        std::cout << "2- Insert new passive agent" << std::endl;
        std::cout << "3- Insert random passive agents" << std::endl;
        std::cout << "4- Insert random active agent" << std::endl;
        std::cout << "q- Quit scenario creator" << std::endl;
        std::cin >> option;
        std::cout << std::endl;
        if (option != "q") {
            if (option == "3") {
                std::cout << "Number of agents: ";
                std::cin >> option;
                insertActive = false;
                insertRandom = true;
                passives = std::atoi(option.c_str());
                start = true;
                while (!finished) {
                    sleep(1);
                }
                finished = false;
            } else {
                insertRandom = false;
                passives = 1;
                insertActive = option == "1" || option == "4" ? true : false;
                insertRandom = option == "4" ? true : false;
                start = true;
                while (!finished) {
                    sleep(1);
                }
                finished = false;
            }
        }
    } while (option != "q");
    mu.lock();
    Simulation::quit = true;
    mu.unlock();
    std::cout << "Exiting menu..." << std::endl;
}

/**
 * Translate coordinates with calibrated values
 */
void Simulation::translateCoordinates(int *x_in, int *y_in, double *x_out, double *y_out) {
    *x_out = lim_xmin + ((float) (*x_in-display_xmin)/((display_xmax-display_xmin)
            /(lim_xmax-lim_xmin))) + 1;
    *y_out = lim_ymin + ((float) ((2750-*y_in)-(2750-display_ymin)))/((display_ymin-display_ymax)
            /(lim_ymax-lim_ymin));
}

/**
 * Scenario creator
 */
void Simulation::createScenario(bool graph) {

    // Variable declarations
    double v0, w0, a_v = 0.15, a_w = 1.0477, time_step = 0.25, x, y, tita, v,
           w, radioAgent = 0.3, dist_minGoal = 4, realX, realY,
           titamin_ini = -3.1416, titamax_ini = 3.1416, vObs = 0.00, xObs,
           yObs, xGoal, yGoal, titaObs;
    int num_ag = 2, numInserted = 0;
    double vMax = 0.7, wMax = 3.14;

    std::string name, thetaInit, thetaDefault, vInit, vDefault, wInit, wDefault;

    // Variable initialization
    thetaDefault = "0.0";
    vDefault = "0.20";
    wDefault = "0.0";

    x = (1.0*std::ceil(std::sqrt((num_ag*M_PI*radioAgent*radioAgent)/(0.04))) < dist_minGoal 
            ? dist_minGoal : 1.5*std::ceil(std::sqrt((num_ag*M_PI*radioAgent*radioAgent)/(0.04))));
    y = x;

    double xmin = -10; double xmax = -xmin; double ymin = -10; double ymax = -ymin;
    lim_xmin = xmin; lim_xmax = xmax; lim_ymin = ymin; lim_ymax = ymax;
    if (graph) {
        Inicializar();
        SeleccionaVentana(1);
        LimpiaVentana();
        errmod("all", "off");
        DibujaEjes(1, lim_xmin, lim_xmax, lim_xmin, 2, lim_ymin, lim_ymax, lim_ymin, 2);
        TerminaEjes();  //necesario para poder dibujar en las otras ventanas los ejes;
                        //ponerlo solo cuando vaya a llamar otra vez a graf porque si no dara error
    }

    int *x_in = new int(), *y_in = new int();
    int i = 0;
    while (!quit) {
        while (!start) {
            if (quit) break;
            sleep(0.25);
        }
        if (quit) continue;
        name = insertActive ? "active": "passive";
        if (!insertRandom) {
            std::cout << "Insert starting position of " << name << " agent" << std::endl;
            csrpt1(x_in, y_in);
            if (quit) continue;
            translateCoordinates(x_in, y_in, &xObs, &yObs);
        } else {
                if (insertActive) {
                    std::uniform_real_distribution<double>::param_type parX(xmin, xmax);
                    distributionX.param(parX);
                    xObs = distributionX(generator);
                    std::uniform_real_distribution<double>::param_type parY(ymin, ymax);
                    distributionY.param(parY);
                    yObs = distributionY(generator);
                } else {
                    xObs = xmin;
                    while(xObs >= xmin && xObs <= xmin+4) {
                        std::uniform_real_distribution<double>::param_type parX(xmin-2, xmax+2);
                        distributionX.param(parX);
                        xObs = distributionX(generator);
                    }
                    yObs = ymin;
                    while(yObs >= ymin && yObs <= ymin+4) {
                        std::uniform_real_distribution<double>::param_type parY(ymin-2, ymax+2);
                        distributionY.param(parY);
                        yObs = distributionY(generator);
                    }
                }
        }

        titamin = titamin_ini; titamax = titamax_ini;

        if (yObs > ymax/3) {
            titamax =  -0.7;
        } else if (yObs < 0){
            titamin = 0.7;
            titamax = 1.57 + 0.7;
        }

        titamin = titamin_ini; titamax = titamax_ini;
        xGoal , yGoal;
        if (insertActive) {
            if (!insertRandom) {
                std::cout << "Insert goal" << std::endl;
                csrpt1(x_in, y_in);
                translateCoordinates(x_in, y_in, &xGoal, &yGoal);
            } else {
                std::uniform_real_distribution<double>::param_type parX(xmin, xmax);
                distributionX.param(parX);
                xGoal = distributionX(generator);
                std::uniform_real_distribution<double>::param_type parY(ymin, ymax);
                distributionY.param(parY);
                yGoal = distributionY(generator);
            }

            //Compute goals at a certain distance of the agent
            if ((xObs - xGoal)*(xObs - xGoal) + (yObs - yGoal)*(yObs - yGoal) < dist_minGoal*dist_minGoal) {
                continue;
            }

            auto it = agents.begin();
            for ( it=agents.begin(); it!=agents.end(); ++it) {
                // andrew: added more safety
                if (CollisionObs((*it)->GetLocalization(), Tsc(xObs,yObs,0), 
                    (radioAgent)*4* config::safety_factor)) {
                    break;
                }
                Tpf goal;
                (*it)->GetCurrentGoal(goal);
                if ((goal.x - xGoal)*(goal.x - xGoal) + (goal.y - yGoal)*(goal.y - yGoal) 
                     < (2*radioAgent+min_dist)*(2*radioAgent+min_dist)) {

                    break;
                }
            }
            if (it != agents.end()) {
                continue;
            }
        }
        if (!insertRandom) {
            std::cout << std::endl << "Insert 'd' for default values or 'r' for random values" 
                      << std::endl;
            std::cout << "->Insert initial theta [" << thetaDefault << "]: ";
            std::cin >> thetaInit;
            thetaInit = thetaInit == "d" ? thetaDefault: thetaInit;
        }
        if (thetaInit == "r" || insertRandom) {
            std::uniform_real_distribution<double>::param_type parTita(titamin, titamax);
            distributionTita.param(parTita);
            titaObs = distributionTita(generator);
        } else {
            titaObs = ::atof(thetaInit.c_str());
        }
        if (!insertRandom) {
            std::cout << "->Insert initial lineal velocity [" << vDefault << "]: ";
            std::cin >> vInit;
            vInit = vInit == "d" ? vDefault: vInit;
        }
        if (vInit == "r" || insertRandom) {
            std::uniform_real_distribution<double>::param_type parV(vMax*0.20, vMax*0.80);
            distributionV.param(parV);
            v0 = distributionV(generator);
        } else {
            v0 = ::atof(vInit.c_str());
        }
        if (!insertRandom) {
            std::cout << "->Insert initial angular velocity [" << wDefault << "]: ";
            std::cin >> wInit;
            wInit = wInit == "d" ? wDefault: wInit;
        }
        if (wInit == "r" || insertRandom) {
            std::uniform_real_distribution<double>::param_type parW(0, 1);
            distributionW.param(parW);
            if (distributionW(generator) <= 0.5) {
                std::uniform_real_distribution<double>::param_type parW(-wMax*0.2, wMax*0.2);
                distributionW.param(parW);
                w0 = distributionW(generator);
            } else {
                w0 = 0;
            }
        } else {
            w0 = ::atof(wInit.c_str());
        }

        if (insertActive) {
            std::unique_ptr<ActiveAgent> ag 
                    {new ActiveAgent(i, xObs, yObs, titaObs, v0, w0, a_v, a_w, radioAgent, graph)};
            ag->AddGoal(xGoal, yGoal);
            if (logSim){ ag->SetNameLog(name_fich); ag->UpdateLog(); }
            ag->SetBoundsVS(time_step);
            agents.push_back(std::move(ag));
        } else {
            std::unique_ptr<PassiveAgent> pag 
                    {new PassiveAgent(i, xObs, yObs, titaObs, v0, w0, radioAgent)};
            pag->SetBoundsVS(time_step);
            agents.push_back(std::move(pag));
        }
        std::cout << "Done inserting " << name << " agent!" << std::endl;
        if (graph)
            PlotWorldEnvironmentStatic();
        numInserted++;
        if (numInserted == passives) {
            start = false;
            finished = true;
            numInserted = 0;
        }
        i++;

    }
    titamin = -3.1416, titamax = 3.1416;
    std::cout << "Exiting scenario creator..." << std::endl;

}

void Simulation::SetEnvironment(bool graph, std::string library, bool debug) {
    this->graph = graph;
    this->library = dis_lib.c_str();
    this->debug= debug;
    time_step = 0.25;
    //los primeros 4 parametros son los limites de velocidad para el VS
    //los 3 siguientes son las restricciones de velocidad para el robot
    //f >> vmin_adm; f >> vmax_adm; f >> wright_adm; f >> wleft_adm; f >> v_max; f >> w_max_left; f >> w_max_right;
    //bounds = boundsVS(vmin_adm, v_max, w_max_left, w_max_right);
    std::string option;
    video = false;
    std::cout << std::endl << "Record saved simulation to video? [y/n]: ";
    std::cin >> option;
    if (option == "y") {
        video = true;
        boost::filesystem::create_directory("output");
    }
    if (this->graph && video)
        this->graph = false;
    if (!video) {
        std::cout << std::endl << "Load scenario from file? [y/n]: ";
        std::cin >> option;
    } else option == "y";
    if (option == "n") {
        if (!video) {
            std::cout << std::endl << "Recalibrate window? [y/n]: ";
            std::cin >> option;
            if (option == "y") calibrate();
            else {
                std::cout << "Reading calibration data from file..." << std::endl;
                std::ifstream calibration_file("data/calibration.txt");
                calibration_file >> display_xmax;
                calibration_file >> display_xmin;
                calibration_file >> display_ymax;
                calibration_file >> display_ymin;
                std::cout << "Done!" << std::endl << std::endl;
                calibration_file.close();
            }
        }
        std::thread menu(Simulation::menuCreateScenario);
        std::thread scenarioCreator(Simulation::createScenario, graph);
        menu.join();
        scenarioCreator.join();
        std::cout << std::endl << "Save scenario to file? [y/n]: ";
        std::cin >> option;
        if (option == "y") {
            std::string scenario;
            std::cout << "Name of scenario: ";
            std::cin >> scenario;
            writeScenarioJSON(scenario);
        }
    } else {
        lim_xmin = -10; lim_xmax = 10; lim_ymin = -10; lim_ymax = 10;
        titamin = -3.1416, titamax = 3.1416;
        std::cout << "Name of scenario to load: ";
        std::cin >> option;
        readScenarioJSON(option);
        if (!video && graph) {
            Inicializar();
            SeleccionaVentana(1);
            LimpiaVentana();
            errmod("all", "off");
            DibujaEjes(1, lim_xmin, lim_xmax, lim_xmin, 2, lim_ymin, lim_ymax, lim_ymin, 2);
            TerminaEjes();
        }
    }

    // andrew: leerlo de algun lado?
    vmax = 1.5;

    for (int i=0; i<(int)agents.size(); i++)
        if (agents[i]->active) {
            Tpf goalAg;
            (agents[i])->GetCurrentGoal(goalAg);
            int goalX = goalAg.x, goalY = goalAg.y;
            Tsc loc = (agents[i])->GetLocalization();
            transfor_inversa_p(goalX, goalY, &loc, &goalAg);
        }

    int num_ag = agents.size();
    //std::cout << "Num agents: " << num_ag << std::endl;
    scenario = 0; iteracion = 0;

    if (graph){
        if (!video) Inicializar(false);
        PlotWorldEnvironment(video);
        if (std::strcmp(this->library, dis_lib.c_str()) == 00 && scenario == 0){
            if (!video) Inicializar(video); //Inicializar el entorno grafico
            int numVentana = 0;
            for (int i = 0; i<num_ag; i++) {
                // agents[i]->printAgent();
                if (agents[i]->active and !video) {
                    NuevaVentana(numVentana+2);
                    numVentana++;
                }
            }
        }

        if (logSim) {
            if (scenario > 0){
                flog.open(name_fich, std::ios::app);
                flog << "\n\n\n";
            }else
                flog.open(name_fich, std::ios::trunc);

            flog << scenario << "\t" << iteracion << "\t";
            flog.close();
        }

        PlotWorldEnvironment(video);
    }
    std::cout << "Leaving enviroment setup" << std::endl << std::endl;
}

void Simulation::createRandomScenario(int actives, int passives, double fixedPorcentage) {
    
    // Variable declarations.
    double v0, w0, a_v = 0.15, a_w = 1.0477, time_step = 0.25, x, y, tita, v,
           w, radioAgent = 0.3, dist_minGoal = 1.5, realX, realY,
           titamin_ini = -3.1416, titamax_ini = 3.1416, vObs = 0.00, xObs,
           yObs, xGoal, yGoal, titaObs;
    int num_ag = 2, numInserted = 0;
    double vMax = 0.7, wMax = 3.14;

    bool fixed = false;
    std::uniform_real_distribution<double>::param_type parFixed(0, 1);
    distributionFixed.param(parFixed);
    if (distributionFixed(generator) < fixedPorcentage) {
        fixed = true;
    }

    std::string name, thetaInit, thetaDefault, vInit, vDefault, wInit, wDefault;

    // Initializing variables.
    thetaDefault = "0.0";
    vDefault = "0.20";
    wDefault = "0.0";

    x = (1.0*std::ceil(std::sqrt((num_ag*M_PI*radioAgent*radioAgent)/(0.04))) < dist_minGoal 
         ? dist_minGoal : 1.5*std::ceil(std::sqrt((num_ag*M_PI*radioAgent*radioAgent)/(0.04))));
    y = x;

    double xmin = -10; double xmax = -xmin; double ymin = -10; double ymax = -ymin;
    lim_xmin = xmin; lim_xmax = xmax; lim_ymin = ymin; lim_ymax = ymax;
    if (graph) {
        Inicializar();
        SeleccionaVentana(1);
        LimpiaVentana();
        errmod("all", "off");
        DibujaEjes(1, lim_xmin, lim_xmax, lim_xmin, 2, lim_ymin, lim_ymax, lim_ymin, 2);
        TerminaEjes();                     
    }

    int *x_in = new int(), *y_in = new int();
    int i = 0;
    insertActive = true;
    while (passives >= 0) {

        // TODO: only difficult scenarios?
        if (insertActive) {
            std::uniform_real_distribution<double>::param_type parX(xmin, xmax);
            distributionX.param(parX);
            xObs = distributionX(generator);
            std::uniform_real_distribution<double>::param_type parY(ymin, ymax);
            distributionY.param(parY);
            yObs = distributionY(generator);
            // xObs = xmin+2;
            // yObs = ymin+2;
        } else {
            std::uniform_real_distribution<double>::param_type parX(xmin-2, xmax+2);
            distributionX.param(parX);
            xObs = distributionX(generator);
            std::uniform_real_distribution<double>::param_type parY(ymin-2, ymax+2);
            distributionY.param(parY);
            yObs = distributionY(generator);
        }


        titamin = titamin_ini; titamax = titamax_ini;

        if (yObs > ymax/3) {
            titamax =  -0.7;
        } else if (yObs < 0){
            titamin = 0.7;
            titamax = 1.57 + 0.7;
        }

        titamin = titamin_ini; titamax = titamax_ini;
        xGoal , yGoal;
        if (insertActive) {

            std::uniform_real_distribution<double>::param_type parX(xmin, xmax);
            distributionX.param(parX);
            xGoal = distributionX(generator);
            std::uniform_real_distribution<double>::param_type parY(ymin, ymax);
            distributionY.param(parY);
            yGoal = distributionY(generator);


            //Compute goals at a certain distance of the agent
            if ((xObs - xGoal)*(xObs - xGoal) + (yObs - yGoal)*(yObs - yGoal) 
                 < dist_minGoal*dist_minGoal) {
                continue;
            }
        }

        auto it = agents.begin();
        for ( it=agents.begin(); it!=agents.end(); ++it) {
            // andrew: added more safety
            if (CollisionObs((*it)->GetLocalization(), Tsc(xObs,yObs,0), 
                (radioAgent)*3* config::safety_factor)) {
                break;
            }

            if ((*it)->active) {
                Tpf goal;
                (*it)->GetCurrentGoal(goal);
                if (CollisionObs(Tsc(goal.x, goal.y, 0), Tsc(xObs,yObs,0), 
                    (radioAgent)*10* config::safety_factor)) {
                    break;
                }
            }

        }
        if (it != agents.end()) {
            continue;
        }

        std::uniform_real_distribution<double>::param_type parTita(titamin, titamax);
        distributionTita.param(parTita);
        titaObs = distributionTita(generator);
        if (!insertActive) {
            std::uniform_real_distribution<double>::param_type parV(vMax*0.20, vMax*0.80);
            distributionV.param(parV);
            v0 = distributionV(generator);
            if (fixed)
                v0 = 0;
        } else {
            std::uniform_real_distribution<double>::param_type parV(0, vMax);
            distributionV.param(parV);
            v0 = distributionV(generator);
        }

        if (insertActive) {
            std::uniform_real_distribution<double>::param_type parW(-0.5, 0.5);
            distributionW.param(parW);
            w0 = distributionW(generator);
            w = 0;
        } else {

            std::uniform_real_distribution<double>::param_type parW(0, 1);
            distributionW.param(parW);

            if (distributionW(generator) <= 0.5) {
                std::uniform_real_distribution<double>::param_type parW(-wMax*0.1, wMax*0.1);
                distributionW.param(parW);
                w0 = distributionW(generator);
            } else {
                w0 = 0;
            }
        }

        if (insertActive) {
            std::unique_ptr<ActiveAgent> ag 
                    {new ActiveAgent(i, xObs, yObs, titaObs, v0, w0, a_v, a_w, radioAgent, graph)};
            ag->AddGoal(xGoal, yGoal);
            if (logSim){ ag->SetNameLog(name_fich); ag->UpdateLog(); }
            ag->SetBoundsVS(time_step);
            agents.push_back(std::move(ag));
        } else {
            std::unique_ptr<PassiveAgent> pag 
                    {new PassiveAgent(i, xObs, yObs, titaObs, v0, w0, radioAgent)};
            pag->SetBoundsVS(time_step);
            agents.push_back(std::move(pag));
        }

        if (graph)
            PlotWorldEnvironmentStatic();

        i++;
        if (i >= actives) {
            insertActive = false;
            passives--;
        }


    }
    titamin = -3.1416, titamax = 3.1416;
    std::cout << "Exiting scenario creator..." << std::endl;
}

/**
 * Set environment for RL
 */
void Simulation::SetEnvironmentRL(bool graph, std::string library, bool debug, bool video, 
                                  std::string nomFich, int actives, int passives, double fixed) {
    this->graph = graph;
    this->library = dis_lib.c_str();
    this->debug= debug;
    if (video) {
            this->graph = false;
            boost::filesystem::create_directory("output");
    }
    this->video = video;
    time_step = 0.25;

    // Initialize variables
    distLastState = std::vector<double>();
	distCurrentState = std::vector<double>();
	thetaLastState = std::vector<double>();
	thetaCurrentState = std::vector<double>();
	distObsLastState = std::vector<double>();
	distObsCurrentState = std::vector<double>();
	vLastState = std::vector<double>();
	vCurrentState = std::vector<double>();
    wCurrentState = std::vector<double>();
    wLastState = std::vector<double>();
	currentClosestAgent = std::vector<int>();
	lastClosestAgent = std::vector<int>();
	thetaObsCurrentState = std::vector<double>();
	thetaObsLastState = std::vector<double>();
	relThetaCurrentState = std::vector<double>();
	relThetaLastState = std::vector<double>();
	lastCollision = std::vector<int>();
	currentCollision = std::vector<int>();
	lastCollisionAgent = std::vector<int>();
	currentCollisionAgent = std::vector<int>();
	thetaObsOriginal = std::vector<double>();
	freeActions = std::vector<bool>();

    lim_xmin = -10; lim_xmax = 10; lim_ymin = -10; lim_ymax = 10;
    titamin = -3.1416, titamax = 3.1416;

    if (nomFich == "") {
        createRandomScenario(actives, passives, fixed);
    } else {
        readScenarioJSON(nomFich);
    }

    if (!video && graph) {
        Inicializar(); 
        LimpiaVentana();
        errmod("all", "off");
        DibujaEjes(1, lim_xmin, lim_xmax, lim_xmin, 2, lim_ymin, lim_ymax, lim_ymin, 2);
        TerminaEjes();
    }

    vmax = 1.5;

    for (int i=0; i<(int)agents.size(); i++)
        if (agents[i]->active) {
            Tpf goalAg;
            (agents[i])->GetCurrentGoal(goalAg);
            int goalX = goalAg.x, goalY = goalAg.y;
            Tsc loc = (agents[i])->GetLocalization();
            transfor_inversa_p(goalX, goalY, &loc, &goalAg);
            distLastState.push_back(0);
            distCurrentState.push_back(0);
            thetaLastState.push_back(0);
            thetaCurrentState.push_back(0);
            distObsLastState.push_back(0);
            distObsCurrentState.push_back(0);
            vLastState.push_back(0);
            vCurrentState.push_back(0);
            wCurrentState.push_back(0);
            wLastState.push_back(0);
            currentClosestAgent.push_back(0);
            lastClosestAgent.push_back(0);
            thetaObsCurrentState.push_back(0);
            thetaObsLastState.push_back(0);
            relThetaCurrentState.push_back(0);
            relThetaLastState.push_back(0);
            lastCollision.push_back(0);
            currentCollision.push_back(0);
            lastCollisionAgent.push_back(0);
            currentCollisionAgent.push_back(0);
            thetaObsOriginal.push_back(0);
            freeActions.push_back(true);
        }

    int num_ag = agents.size();
    scenario = 0; iteracion = 0;
    if (graph){
        if (!video) Inicializar(false);
        PlotWorldEnvironment(video);
        if (std::strcmp(this->library, dis_lib.c_str()) == 00 && scenario == 0){
            if (!video) Inicializar(video); 
            int numVentana = 0;
            for (int i = 0; i<num_ag; i++) {
                if (agents[i]->active and !video) {
                    NuevaVentana(numVentana+2);
                    numVentana++;
                }
            }
        }

        if (logSim) {
            if (scenario > 0){
                flog.open(name_fich, std::ios::app);
                flog << "\n\n\n";
            }else
                flog.open(name_fich, std::ios::trunc);

            flog << scenario << "\t" << iteracion << "\t";
            flog.close();
        }

        PlotWorldEnvironment(video);
    }
    std::cout << "Leaving enviroment setup" << std::endl << std::endl;
}

void Simulation::SetEnvironment(bool g, const char *lib, const char *filename, int &num_scen, int itScen) {
//SETS THE ENVIRONMENT, global position for agents and their goals
///*
    scenario = itScen; iteracion = 0;
    if (logSim) {
        if (itScen > 0){
            flog.open(name_fich, std::ios::app);
            flog << "\n\n\n";
        } else
            flog.open(name_fich, std::ios::trunc);

        flog << scenario << "\t" << iteracion << "\t";
        flog.close();
    }

    graph = g; library = lib;

    std::ifstream f(filename);
    double a_v, a_w;

    f >> time_step; f >> a_v; f >> a_w; //acceleration constraints are the same for all the robots

    //los primeros 4 parametros son los limites de velocidad para el VS
    //los 3 siguientes son las restricciones de velocidad para el robot
    //f >> vmin_adm; f >> vmax_adm; f >> wright_adm; f >> wleft_adm; f >> v_max; f >> w_max_left; f >> w_max_right;
    //bounds = boundsVS(vmin_adm, v_max, w_max_left, w_max_right);

    double x, y, tita, v, w, radioAgent;
    radioAgent = 0.3;   //radioAgent = 1e-4; //minimum radius for correct calculations: 1e-4
    int num_ag = 0;
    f >> num_ag;
    if (std::strcmp(filename, "simulacionesObstaculosMultiCirculo.txt") == 0) {
        double radioCir = 0;
        f >> radioCir;  //radio de la cirunferencia a formar por los agentes
        f >> v; w = 0;
        //if (itScen == 1 || itScen == 9)
        //if (num_scen <= 1)
            f >> num_scen;
        double div = (num_scen > 1 ? num_scen : num_ag/2);
        //double offset = M_PI/div;
        double offset = (2*radioCir)/(num_ag/2);
        int id = 0;
        for (int i=0; i<num_ag/2; i++){
            double tita = M_PI - itScen*offset;
            //double x1 = radioCir * std::cos(tita); //-radioCir + i*offset;
            //double y1 = radioCir * std::sin(tita); //std::sqrt(radioCir*radioCir - x1*x1);
            double x1 = -radioCir + i*offset;
            double y1 = std::sqrt(radioCir*radioCir - x1*x1);

            if (std::abs(y1) < 1e-5){ //una solucion
                double x2 = -x1;
                double y2 = -y1;
                double tita1 = std::atan2(y2-y1,x2-x1);
                double tita2 = std::atan2(y1-y2,x1-x2);

                std::unique_ptr<ActiveAgent> ag {new ActiveAgent(id, x1, y1, tita1, v, w, a_v, a_w, radioAgent)};
                //ag->AddGoal(x2, y2);
                ag->AddGoal(x2, y2+0.2);
                if (logSim){ ag->SetNameLog(name_fich); ag->UpdateLog(); }
                ag->SetBoundsVS(time_step);
                agents.push_back(std::move(ag));
                id++;

                ag = std::unique_ptr<ActiveAgent> {new ActiveAgent(id, x2, y2, tita2, v, w, a_v, a_w, radioAgent)};
                //ag->AddGoal(x1, y1);
                ag->AddGoal(x1+0.3, y1-0.4);
                if (logSim){ ag->SetNameLog(name_fich); ag->UpdateLog(); }
                ag->SetBoundsVS(time_step);
                agents.push_back(std::move(ag));
                id++;
            }
            else{
                double tita1 = std::atan2(-y1-y1,-x1-x1); //goal = (-x1,-y1) Pto = (x1,y1)
                double tita2 = std::atan2(y1+y1,-x1-x1); //goal = (-x1,y1) Pto = (x1,-y1)

                std::unique_ptr<ActiveAgent> ag {new ActiveAgent(id, x1, y1, tita1, v, w, a_v, a_w, radioAgent)};
                //ag->AddGoal(-x1, -y1);
                ag->AddGoal(-x1+0.6,-y1-0.7);
                if (logSim){ ag->SetNameLog(name_fich); ag->UpdateLog(); }
                ag->SetBoundsVS(time_step);
                agents.push_back(std::move(ag));
                id++;

                //ag = std::unique_ptr<ActiveAgent> {new ActiveAgent(id, 0, 0, 0, v, w, a_v, a_w, radioAgent)};
                //ag->AddGoal(2*radioCir,0);
                ag = std::unique_ptr<ActiveAgent> {new ActiveAgent(id, x1, -y1, tita2, v, w, a_v, a_w, radioAgent)};
                //ag->AddGoal(-x1, y1);
                ag->AddGoal(-x1-0.31,y1+0.1);
                if (logSim){ ag->SetNameLog(name_fich); ag->UpdateLog(); }
                ag->SetBoundsVS(time_step);
                agents.push_back(std::move(ag));
                id++;
            }
        }
        f >> lim_xmin; f >> lim_xmax; f >> lim_ymin; f >> lim_ymax;
    }
    else if (std::strcmp(filename, "simulaciones.txt") == 0) {

        double dist_minGoal = 8;
        double x = (1.0*std::ceil(std::sqrt((num_ag*M_PI*radioAgent*radioAgent)/(0.04))) < dist_minGoal ? dist_minGoal : 1.5*std::ceil(std::sqrt((num_ag*M_PI*radioAgent*radioAgent)/(0.04))));
        double y = x;

        double xmin = 0; double xmax = x; double ymin = 0; double ymax = y;

        std::uniform_real_distribution<double>::param_type parX(xmin, xmax);
        std::uniform_real_distribution<double>::param_type parY(ymin, ymax);
        distributionX.param(parX); distributionY.param(parY);

        double titamin_ini = -3.1416;
        double titamax_ini = 3.1416;
        double vObs = 0.00;

        titamin = titamin_ini; titamax = titamax_ini;

        int i=0;
        while (i<num_ag){
            double xObs = distributionX(generator);
            double yObs = distributionY(generator);

            if (yObs > ymax/3){
                titamax =  -0.7;
            }else if (yObs < 0){
                titamin = 0.7;
                titamax = 1.57 + 0.7;
            }
            std::uniform_real_distribution<double>::param_type parTita1(titamin, titamax);
            distributionTita.param(parTita1);
            double titaObs = distributionTita(generator);
            titamin = titamin_ini; titamax = titamax_ini;
            //titaObs = 0;

            std::unique_ptr<ActiveAgent> ag {new ActiveAgent(i, xObs, yObs, titaObs, vObs, 0, a_v, a_w, radioAgent)};
            double xGoal = distributionX(generator);
            double yGoal = distributionY(generator);
            //Compute goals at a certain distance of the agent
            if ((xObs - xGoal)*(xObs - xGoal) + (yObs - yGoal)*(yObs - yGoal) < dist_minGoal*dist_minGoal) continue;

            auto it = agents.begin();
            for ( it=agents.begin(); it!=agents.end(); ++it){
                if (CollisionObs((*it)->GetLocalization(), Tsc(xObs,yObs,0), (radioAgent+radioAgent)* config::safety_factor))
                    break;
                Tpf goal;
                (*it)->GetCurrentGoal(goal);
                if ((goal.x - xGoal)*(goal.x - xGoal) + (goal.y - yGoal)*(goal.y - yGoal) < (2*radioAgent+min_dist)*(2*radioAgent+min_dist))
                    break;
            }
            if (it != agents.end()) continue;

            ag->AddGoal(xGoal, yGoal);
            if (logSim){ ag->SetNameLog(name_fich); ag->UpdateLog(); }
            ag->SetBoundsVS(time_step);
            agents.push_back(std::move(ag));
            i++;
        }
        lim_xmin = xmin; lim_xmax = xmax; lim_ymin = ymin; lim_ymax = ymax;
    }
    else{
        for (int i = 0; i < num_ag; i++){
            f >> x; f >> y; f >> tita; f >> v; f >> w; //f >> radioRob;

            std::unique_ptr<ActiveAgent> ag {new ActiveAgent(i, x, y, tita, v, w, a_v, a_w, radioAgent)};
            //Add goals for the agent
            int n; f >> n; //number of goals
            for (int i=0; i<n; i++){
                double xGoal, yGoal;
                f >> xGoal; f >> yGoal;;
                ag->AddGoal(xGoal, yGoal);
            }
            if (logSim){ ag->SetNameLog(name_fich); ag->UpdateLog(); }
            ag->SetBoundsVS(time_step);
            agents.push_back(std::move(ag));
        }
        f >> lim_xmin; f >> lim_xmax; f >> lim_ymin; f >> lim_ymax;
    }

    //(9/02/2016): leo limites para replicar los obstaculos (que siempre haya el mismo numero de obstaculos moviendose en el entorno de navegacion)

    //Escribimos en el log los valores iniciales
    //namelog = std::string("log_") + filename;
    //flog.open(namelog, std::ios::trunc);
    //flog << scenario << "\t" << iteracion << "\t" << goals[current_goal].GetLocalizacion().x << "\t" <<
    //     goals[current_goal].GetLocalizacion().y << "\t" << robot.GetLocalizacion().x << "\t" << robot.GetLocalizacion().y <<
    //     "\t" << robot.GetLocalizacion().tita << "\t" << robot.GetV() << "\t" << robot.GetW() << "\t";

    if (std::strcmp(filename, "simulacionesObstaculos.txt") == 0){
        int n; f >> n;
        int k = 2;  //Se incluyen tambien los obstaculos que aparecen en el lado contrario dentro del marco de trabajo; y los estaticos
        localizaciones.resize(k*n + 2*2*((lim_ymax-lim_ymin)/(config::radioEstatico+config::radioRob)+1) + 2*2*((lim_xmax-lim_xmin)/(config::radioEstatico+config::radioRob)+1));
        velocidades.resize(k*n + 2*2*((lim_ymax-lim_ymin)/(config::radioEstatico+config::radioRob)+1) + 2*2*((lim_xmax-lim_xmin)/(config::radioEstatico+config::radioRob)+1));
        velocidades_w.resize(k*n + 2*2*((lim_ymax-lim_ymin)/(config::radioEstatico+config::radioRob)+1) + 2*2*((lim_xmax-lim_xmin)/(config::radioEstatico+config::radioRob)+1));
        //distantObs.resize(k*num_obs + 2*std::ceil((lim_ymax-lim_ymin)/0.6) + 2*std::ceil((lim_xmax-lim_xmin)/0.6));
        //pointRange.resize(k*num_obs + 2*std::ceil((lim_ymax-lim_ymin)/0.6) + 2*std::ceil((lim_xmax-lim_xmin)/0.6));

        for (int i=0; i<n; i++){
            double xObs, yObs, titaObs, vObs, wObs, radioObs, av, aw;
            f >> xObs; f >> yObs; f >> titaObs; f >> vObs; f >> wObs; // f >> radioObs;
            av = 0; aw = 0; //Passive agents (linear and non-linear) move with constant velocity

            std::unique_ptr<PassiveAgent> pAg {new PassiveAgent(i, x, y, tita, v, w, radioAgent)};
            agents.push_back(std::move(pAg));
            localizaciones[i].x = xObs; localizaciones[i].y = yObs; localizaciones[i].tita = titaObs;
            velocidades[i] = vObs; velocidades_w[i] = wObs;

            if (vObs > 0){
                //flog << pAg->GetLocalization().x << "\t" << pAg->GetLocalization().y << "\t" << pAg->GetLocalization().tita << "\t" << vObs << "\t" << wObs << "\t";
            }
        }
        //Añadimos obstaculos estaticos a lo largo del escenario para que el robot se mantenga dentro de los limites de velocidad
        double id_number = n;
        GenerateStaticObstacles(id_number);
    }
    f.close();

    //There should be metrics for each agent
    //WriteMetricsLog(0);
    //*/

    Inicializar();
    PlotWorldEnvironment();

    if (graph == 1){
        if (std::strcmp(library, dis_lib.c_str()) == 00 && itScen == 0){
            Inicializar(); //Inicializar el entorno grafico
            int numVentana = 0;
            for (int i = 0; i<num_ag; i++) {
                NuevaVentana(numVentana+2);
                numVentana++;
            }
        }

        PlotWorldEnvironment();
    }
}

void Simulation::WriteMetricsLog(double cost, const char* name){
/*
    //MeTRICAS PARA LA EVALUACIoN DE LA NAVEGACIoN
    //Almacenamos en el log la orientacion del goal con respecto al robot,
    //la menor de todas las distancias a los obstaculos (distancia del robot con respecto a los obstaculos, como medida de distancia de seguridad),
    //velocidad lineal y angular inicial del robot;
    //25-04-2017: coste del camino
    double d = 10000;
    for (int i=0; i<(int)obstacles.size(); i++){
        Obstaculo obs = obstacles[i];
        if (obs.GetV() > 0){
            double radio_seg = robot.GetRadius() + obs.GetApparentRadius();
            double distancia = Distancia(obs.GetLocRobot().x, obs.GetLocRobot().y);
            //std::cout << "Distancia: " << distancia << ", radio: " << radio_seg << std::endl;
            if (distancia - radio_seg < d) d = distancia - radio_seg;
        }
    }
    flog << atan2(goals[current_goal].GetLocRobot().y, goals[current_goal].GetLocRobot().x) << "\t" << d << "\t";
    flog << robot.GetV() << "\t" << robot.GetW() << "\t" << cost << std::endl;

    //std::ofstream fdata("timeComputeDOVT.dat", std::ios::app); fdata << std::endl; fdata.close();
    //fdata.open("timeAstar.dat", std::ios::app); fdata << std::endl; fdata.close();
    //fdata.open("timeFillMap.dat", std::ios::app); fdata << std::endl; fdata.close();
*/

    std::ofstream f(name, std::ios_base::app);
    for (int i=0; i<(int)agents.size(); i++) {
        //Goal with respect to agent's position
        Tpf goal; Tsc loc = agents[i]->GetLocalization();
        if (agents[i]->GetCurrentGoal(goal)){
            transfor_inversa_p(goal.x, goal.y, &loc, &goal);
            f << agents[i]->GetW() << "\t" << std::atan2(goal.y, goal.x) << "\t";
        }
    }
    f << std::endl;
    f.close();
}

void Simulation::GenerateStaticObstacles(int id_number){ //, double radioRob) {
    //Añadimos obstaculos estaticos a lo largo del escenario para que el robot se mantenga dentro de los limites de velocidad
    double delta = 0;
    //const double radioEstatico = radioRob * 1.0;//16.0;
    const double velEstatico = 0.0;
    /*
    int k = 1;
    for (int i=0; i<k*((int)((lim_xmax-lim_xmin)/(config::radioEstatico+config::radioRob))+1); i++){
        double x = lim_xmin + delta/k;
        Obstaculo obs(id_number, x, lim_ymin - config::radioEstatico, 0, velEstatico, 0, config::radioEstatico);
        obs.Localizacion(robot.GetLocalizacion());
        obstacles.push_back(obs);
        localizaciones[id_number].x = x; localizaciones[id_number].y = lim_ymin; localizaciones[id_number].tita = 0;
        velocidades[id_number] = 0; velocidades_w[id_number] = 0;
        id_number++;

        Obstaculo obs1(id_number, x, lim_ymax + config::radioEstatico, 0, velEstatico, 0, config::radioEstatico);
        obs1.Localizacion(robot.GetLocalizacion());
        obstacles.push_back(obs1);
        localizaciones[id_number].x = x; localizaciones[id_number].y = lim_ymax; localizaciones[id_number].tita = 0;
        velocidades[id_number] = 0; velocidades_w[id_number] = 0;
        id_number++;

        delta = (i+1)*(config::radioEstatico+config::radioRob);
    }

    delta = 0;
    for (int i=0; i<k*((int)((lim_ymax-lim_ymin)/(config::radioEstatico+config::radioRob))+1); i++){
        double y = lim_ymin + delta/k;
        Obstaculo obs(id_number, lim_xmin - config::radioEstatico, y, 0, velEstatico, 0, config::radioEstatico);
        obs.Localizacion(robot.GetLocalizacion());
        obstacles.push_back(obs);
        localizaciones[id_number].x = lim_xmin; localizaciones[id_number].y = y; localizaciones[id_number].tita = 0; velocidades[id_number] = 0; velocidades_w[id_number] = 0;
        id_number++;

        Obstaculo obs1(id_number, lim_xmax + config::radioEstatico, y, 0, velEstatico, 0, config::radioEstatico);
        obs1.Localizacion(robot.GetLocalizacion());
        obstacles.push_back(obs1);
        localizaciones[id_number].x = lim_xmax; localizaciones[id_number].y = y; localizaciones[id_number].tita = 0; velocidades[id_number] = 0; velocidades_w[id_number] = 0;
        id_number++;

        delta = (i+1)*(config::radioEstatico+config::radioRob);
    }
    //*/
}

void Simulation::PlotWorldEnvironment(bool video) {
    //Workspace graph
    if (std::strcmp(this->library, dis_lib.c_str()) == 0){ //dislin graph

        if (video) {
            setfil("output/we.png");
            filopt( "NONE", "SEPARATOR");
            filopt("LONG", "NUMBER");
            filopt("6", "DIGITS");
            Inicializar(true);
        }
        //dibujo inicial del entorno
        if (!video) SeleccionaVentana(1);
        LimpiaVentana();
        errmod("all", "off");
        //DibujaEjes(1, -2, 18,-2, 2, -5, 15,-5, 2);    //DibujaEjes(1, -4, 20,-4, 2, -4, 20,-4, 2);
        //DibujaEjes(1, -10, 30, -5, 5, -10, 30,-5, 5);
        DibujaEjes(1, lim_xmin, lim_xmax, lim_xmin, 2, lim_ymin, lim_ymax, lim_ymin, 2);


        /*
        rline(lim_xmin, lim_ymin, lim_xmax, lim_ymin);
        rline(lim_xmax, lim_ymin, lim_xmax, lim_ymax);
        rline(lim_xmax, lim_ymax, lim_xmin, lim_ymax);
        rline(lim_xmin, lim_ymax, lim_xmin, lim_ymin);

        double radio = robot.GetRadius();
        rline(lim_xmin+radio, lim_ymin+radio, lim_xmax-radio, lim_ymin+radio);
        rline(lim_xmax-radio, lim_ymin+radio, lim_xmax-radio, lim_ymax-radio);
        rline(lim_xmax-radio, lim_ymax-radio, lim_xmin+radio, lim_ymax-radio);
        rline(lim_xmin+radio, lim_ymax-radio, lim_xmin+radio, lim_ymin+radio);
        //*/

        /*//segmentos estaticos
        for (int i=0; i<(int)segmentos_estaticos.size(); i++){
            std::vector<double> segmento = segmentos_estaticos[i];
            if (segmento.size() == 4) rline(segmento[0], segmento[1], segmento[2], segmento[3]);
        }
        //*/
        std::string it = "Iteracion " + std::to_string(iteracion);
        rlmess(it.c_str(), 2, 9);
        for (int i=0; i<(int)agents.size(); i++){
            //std::cout << "Plotting agent " << i << std::endl;
            Tsc loc = agents[i]->GetLocalization();
            //if consider...
            if (agents[i]->active)
                DibujaObsInicial(1, loc.x, loc.y, agents[i]->GetRealRadius(), 0.1, "blue");
            else DibujaObsInicial(1, loc.x, loc.y, agents[i]->GetRealRadius(), 0.1, "green");
            //DibujaObsInicial(1, o.GetLocalizacion().x, o.GetLocalizacion().y, o.GetApparentRadius(), 0.1, "red");

            if (agents[i]->GetV() != 0) {
                height(20);
                std::string eti = " " + std::to_string(agents[i]->GetId());
                height(38);
                rlmess(eti.c_str(), loc.x, loc.y);

                double vx = agents[i]->GetV() * std::cos(loc.tita);
                double vy = agents[i]->GetV() * std::sin(loc.tita);
                rlvec(loc.x, loc.y, loc.x + vx, loc.y + vy, 2101);
            }

            //Plot the current goal followed by the agent
            Tpf goal;
            if (agents[i]->GetCurrentGoal(goal)) {
                std::string eti = "g" + std::to_string(agents[i]->GetId());
                DibujaGoal(1, goal, "red", eti.c_str());
            }
        }

        int *x = new int(), *y = new int();
        TerminaEjes();  //necesario para poder dibujar en las otras ventanas los ejes;
                    //ponerlo solo cuando vaya a llamar otra vez a graf porque si no dara error
        if (video) disfin();
    }//else: gnuplot
}


void Simulation::PlotWorldEnvironmentStatic() {

    //dibujo inicial del entorno
    SeleccionaVentana(1);
    LimpiaVentana();
    errmod("all", "off");
    DibujaEjes(1, lim_xmin, lim_xmax, lim_xmin, 2, lim_ymin, lim_ymax, lim_ymin, 2);

    for (int i=0; i<(int)agents.size(); i++){
        //std::cout << "Plotting agent " << i << std::endl;
        Tsc loc = agents[i]->GetLocalization();
        //if consider...
        if (agents[i]->active) DibujaObsInicial(1, loc.x, loc.y, agents[i]->GetRealRadius(), 0.1, "blue");
        else DibujaObsInicial(1, loc.x, loc.y, agents[i]->GetRealRadius(), 0.1, "green");
        //DibujaObsInicial(1, o.GetLocalizacion().x, o.GetLocalizacion().y, o.GetApparentRadius(), 0.1, "red");

        if (agents[i]->GetV() != 0) {
            height(20);
            std::string eti = " " + std::to_string(agents[i]->GetId());
            height(38);
            rlmess(eti.c_str(), loc.x, loc.y);

            double vx = agents[i]->GetV() * std::cos(loc.tita);
            double vy = agents[i]->GetV() * std::sin(loc.tita);
            rlvec(loc.x, loc.y, loc.x + vx, loc.y + vy, 2101);
        }

        //Plot the current goal followed by the agent
        Tpf goal;
        if (agents[i]->GetCurrentGoal(goal)) {
            std::string eti = "g" + std::to_string(agents[i]->GetId());
            DibujaGoal(1, goal, "red", eti.c_str());
        }
    }

    int *x = new int(), *y = new int();
    TerminaEjes();  //necesario para poder dibujar en las otras ventanas los ejes;
                //ponerlo solo cuando vaya a llamar otra vez a graf porque si no dara error

}

bool Simulation::IsFinished(){
//The simulation finishes when the agents have reached their respective goals

    bool finished = true;

    int i = 0;
    while (finished && i < (int)agents.size()){
        if (agents[i]->active) {
            finished = finished && (agents[i]->IsFinished(min_dist));
        }
        i++;
    }
    //andrew
    if (iteracion > 500) {
        finished = true;
    }
    // if (finished) std::cout << "Iteraciones: " << iteracion << std::endl;

    return finished;
}

bool Simulation::UpdateCycle(bool &unreachable) {
//Update movement of the agents

    iteracion++;

    //Check if the agents cannot reach their goals
    flog.open(name_fich, std::ios::app);
    if (iteracion > 500){
        unreachable = true;
        //flog << std::endl <<  "GOAL UNREACHABLE" << std::endl;
        return false;
    }else
        flog << std::endl << scenario << "\t" << iteracion << "\t";
    flog.close();

#pragma omp parallel for
    for (int i=0; i<(int)agents.size(); i++) {
        agents[i]->Update(time_step);
        //andrew
        if (agents[i]->active)
            agents[i]->WriteMetricsLog(&agents);
    }
        /*//Para los pasivos (dentro del Update???)
        //INI(23/02/2016): Vector pointRange esta definido solo para varias simulaciones, no cuando ejecutamos un caso concreto (simulacionesObstaculos.txt)
        if ((*it).GetV() > 0){
            //Solo los obstaculos dinamicos pueden salirse del marco del escenario
            if (pointRange.size() > 0){
                //INI(6/02/2016): Compruebo si el obstaculo se ha salido de los limites del entorno de trabajo para reinicializarlo en el extremo opuesto
                Tsc sist;
                sist = (*it).GetLocalizacion(); //sist.x = (*it).GetLocalizacion().x; sist.y = (*it).GetLocalizacion().y; sist.tita = (*it).GetLocalizacion().tita;
                Tpf sol;
                transfor_inversa_p(pointRange[(*it).GetId()].x, pointRange[(*it).GetId()].y, &sist, &sol);
                //std::cout << "Localizacion del obstaculo: " << sist.x << ", " << sist.y << ", " << sist.tita << std::endl;
                if (sol.x < 0){  //El punto de referencia del marco de trabajo queda por detras del obstaculo
                    Tpf ptObs;
                    transfor_directa_p(-2*distantObs[(*it).GetId()], 0, &sist, &ptObs);
                    Tsc loc_obs; loc_obs.x = ptObs.x; loc_obs.y = ptObs.y; loc_obs.tita = sist.tita;
                    (*it).SetLocalizacion(loc_obs);
                    //std::cout << "Distancia: " << distantObs[(*it).GetId()] << std::endl;
                    //std::cout << "Localizacion obstaculo: " << loc_obs.x << ", " << loc_obs.y << ", " << loc_obs.tita << ", " << atan2(ptObs.y, ptObs.x) << std::endl;
                    //std::cout << "Velocidad: " << (*it).GetV() << ", " << (*it).GetW() << std::endl;
                    (*it).Localizacion(robot.GetLocalizacion());
                    (*it).CalculaModDir(robot.GetLocalizacion(), time_step);
                }
                //FIN(6/02/2016)
            }
        }
        //FIN(23/02/2016)
        */

        /*Model environment
        double radioDOV = ((*it).GetRealRadio() + robot.GetRadius())*config::safety_factor;
        if ((*it).GetW() > 0) (*it).CalculaBandaColisionNL(radioDOV, time_step);
        else (*it).CalculaBandaColision(radioDOV);

        bool consider = ConsiderObstacle(&robot, &(*it), goals[current_goal].GetLocalizacion(), time_step);
        consider = true;

        //if ((*it).GetLocRobot().x*(*it).GetLocRobot().x + (*it).GetLocRobot().y*(*it).GetLocRobot().y > 6*6) consider = false;     //para la simulacion del escenario
        if ((*it).GetW() > 0) consider = true;
        (*it).SetConsider(consider);
        */

    //PlotWorldEnvironment();
    if (this->graph || this->video){
        PlotWorldEnvironment(video);
    }

    return !CheckCollision();  //if there is a collision, it returns false
}

double Distance (Tsc loc1, Tsc loc2){    //Euclidean distance between positions

    double dx = loc2.x - loc1.x;
    double dy = loc2.y - loc1.y;

    return (std::sqrt( dx*dx + dy*dy));
}

bool CollisionAgent(Tsc locAg, Tsc locAgComp, double secRadius){

    return (Distance(locAg, locAgComp) - secRadius < 0.0);
}

bool Simulation::CheckCollision(){
//There is collision when the distance between any pair of agents is less than the inflated apparent_radius (in the workspace),
//or when the agent lies inside the square that inscribes the inflated obstacle (in agent-centric representation of the workspace)
    std::ofstream flog;
    flog.open(name_fich, std::ios::app);
    for (int i=0; i<(int)agents.size(); i++){
        if (agents[i]->active) {
            std::unique_ptr<Agent>& ag = agents[i];

            // andrew
            // for (int j=i+1; j<(int)agents.size(); j++){
            for (int j=0; j<(int)agents.size(); j++){
                if (i != j) {
                    std::unique_ptr<Agent>& agComp = agents[j];

                    if (CollisionAgent(ag->GetLocalization(), agComp->GetLocalization(), ag->GetRealRadius() + agComp->GetRealRadius())){
                        //flog << std::endl << "COLISION";
                        return true;
                    }
                }
            }
        }
    }
    flog.close();
    return false;

    //Case where the agent lies in the square of another agent
    for (int i=0; i<(int)agents.size(); i++){
        std::unique_ptr<Agent>& ag = agents[i];

        for (int j=i+1; j<(int)agents.size(); j++) {
            std::unique_ptr<Agent>& agComp = agents[j];

            Tpf v1, v2, v3;

            //v1.x = -obs.GetMas45().x; v1.y = -obs.GetMas45().y;
            //v2.x = obs.GetMenos45().x - obs.GetMas45().x; v2.y = obs.GetMenos45().y - obs.GetMas45().y;
            //v3.x = obs.GetMas135().x - obs.GetMas45().x; v3.y = obs.GetMas135().y - obs.GetMas45().y;

            double v1v2 = v1.x * v2.x + v1.y * v2.y;
            double v2v2 = v2.x * v2.x + v2.y * v2.y;
            double v1v3 = v1.x * v3.x + v1.y * v3.y;
            double v3v3 = v3.x * v3.x + v3.y * v3.y;

            //if ((0 < v1v2 && v1v2 + 0.24 < v2v2) && (0 < v1v3 && v1v3 + 0.24 < v3v3)){
            //if ((0 < v1v2 && v1v2 < v2v2) && (0 < v1v3 && v1v3 < v3v3)){
            //if ((0.05 < v1v2 && v1v2 < v2v2) && (0.05 < v1v3 && v1v3 < v3v3)){
            if ((0.07 < v1v2 && v1v2 + 0.24 < v2v2) && (0.07 < v1v3 && v1v3 + 0.24 < v3v3)) {

                double d = Distance(ag->GetLocalization(), agComp->GetLocalization()) - ag->GetRealRadius() +
                           agComp->GetRealRadius();

                //flog << atan2(goals[current_goal].GetLocRobot().y, goals[current_goal].GetLocRobot().x) << "\t" << d << "\t";
                //flog << robot.GetV() << "\t" << robot.GetW();

                //flog << std::endl << "COLISION, DENTRO CUADRADO";
                return true;
            }
        }
    }

    return false;
}

void Simulation::ModelEnvironment(const int lookAhead, const int th, const bool accConst) {
//Model dynamic information: passive and active agents

#pragma omp parallel for
    for (int i=0; i<(int)agents.size(); i++){
        //if (!agents[i]->IsFinished(min_dist)) agents[i]->ModelAgents(&agents, time_step, lookAhead, th, accConst);
        if (!agents[i]->IsFinished(min_dist) && agents[i]->active) agents[i]->ModelAgentsFusion(&agents, time_step, lookAhead, th, accConst);
    }
}

void Simulation::ComputeMotion(int lookAhead, const int algorithm) {
    //PARAMETERS FOR PLANNING
    std::ifstream fp("data/planner3d_data.txt");
    double th; unsigned n, m; unsigned long numC;
    fp >> th;   //time horizon
    fp >> n; fp >> m; //dovt size
    fp >> numC;  //number of cells to filled downwards

    assert(th > 0 && n > 0 && m > 0 && numC >= 0);

    std::vector<Velocidad> motion; motion.resize((int)agents.size());

#pragma omp parallel for
     for (int i=0; i<(int)agents.size(); i++){
        //std::cout << "Agent " << i << "  Active: " << agents[i]->active << std::endl;
        if (agents[i]->active) {
            if (agents[i]->IsFinished(min_dist)) {
                motion[i] = Velocidad(0,0);
            }
            else{
                //Trial computation for sharing avoidance of collision
                //motion[i] = agents[i]->MotionShared(&agents, time_step, bounds, lookAhead);
                switch (algorithm) {
                    case 0: //Greedy (2D planning)
                        motion[i] = agents[i]->MotionGreedy(time_step, &agents, video);
                        break;
                    case 1: //Strategies (2D planning)
                        motion[i] = agents[i]->MotionStrategy(time_step, &agents, video, iteracion, graph, debug);
                        break;
                    case 2: //A* (3D planning)
                        motion[i] = agents[i]->MotionPlan(time_step, th, n, m, numC, lookAhead);
                        break;
                }
            }
        } else {
            motion[i] = Velocidad(agents[i]->getV0(),agents[i]->getW0());
        }
         //break;   //So that only first agent takes decisions
/*
        SeleccionaVentana(agents[i]->GetId()+2);
        double w = 0;  double v = 0;
        w = agents[i]->GetW(); v = agents[i]->GetV();
        Selecciona(w, v, boundsVS.wmax_left, boundsVS.wmax_right, boundsVS.vlim_max, boundsVS.vlim_min);
        motion = Velocidad(v,w);
*/
        //agents[i]->SetVelocity(motion);

        //TerminaEjes();
    }

    //if (primero){

    //return;
#pragma omp parallel for
        for (int i=0; i<(int)agents.size(); i++){
            //if (agents[i]->GetId() == 1)
                agents[i]->SetVelocity(motion[i]);
            //break;   //So that only first agent takes decisions
        }
    //    primero = false;
    //}
}

void Simulation::AddAgent(std::unique_ptr<Agent> ag) {

    agents.push_back(std::move(ag));
}

Agent& Simulation::GetAgent(const int id) {
//id: identifier for one agent
    for (auto it = agents.begin(); it != agents.end(); ++it){
        if ((*it)->GetId() == id) return (**it);
    }
}

Velocidad FollowGoal(Tpf goal, goalVS g, boundsVS bounds){
    //Decide the goal to be reached

    double min_dist = 2;
    double dist = std::sqrt(std::pow(goal.x, 2) + std::pow(goal.y, 2));

    if (bounds.IsInsideBounds(g.velGoal)){
        return g.velGoal;
    }else{
        if (dist < min_dist)
            return g.commandGoal;
        else
            return g.dirGoal;
    }
}

bool IsVelReachable(double step, const Velocidad &current, const Velocidad & vel, boundsVS bounds, constraints c);

bool deviateCB(const Tsc loc, const Line traj, const double offset, Tsc &raiz){    //Se desvian si las BC se cruzan y los agentes se mueven en direccion hacia el punto de corte
//traj: trajectoria relativa entre los agentes
//loc: localizacion relativa
    //double offset = 0.6; //radio de los agentes
    int sol; bool deviate = false;
    SolDosRectas(traj, Line(0,1,0), raiz, sol);
    if (sol > 0){
        if (raiz.x + offset >= 0){
            if (loc.y >= 0 && std::sin(loc.tita) < 0) deviate = true;
            else if (loc.y < 0 && std::sin(loc.tita) > 0) deviate = true;
        }
        else{  //check wrt axis 'y'
            sol = 0; raiz = Tsc();
            SolDosRectas(traj, Line(1,0,0), raiz, sol);
            if (sol > 0){
                if (std::abs(raiz.y) - offset <= 0){
                    if (loc.y >= 0 && std::sin(loc.tita) < 0) deviate = true;
                    else if (loc.y < 0 && std::sin(loc.tita) > 0) deviate = true;
                }
            }//else if (std::abs(loc.x) <= 0.6) deviate = true;
        }
    }else{
        if (std::abs(loc.y) <= offset && loc.x + offset >= 0){
            //Compute the intersecting point wrt axis 'y'
            sol = 0; raiz = Tsc();
            SolDosRectas(traj, Line(1,0,0), raiz, sol);
            deviate = true;
        }
    }
    return deviate;
}

Velocidad ComputeVelGoal(const Tpf goal, Tsc sist, boundsVS bounds, double time_step){

    Tpf goalAg;
    transfor_inversa_p(goal.x, goal.y, &sist, &goalAg);

    VS space(bounds);
    space.InsertGoal(goalAg, time_step);

    return FollowGoal(goalAg, space.GetGoal(), bounds);
}

std::pair<Velocidad, int> ComputeTarget(Agent& agi, Agent& agj, const Tsc raiz, const boundsVS bounds){

    Velocidad vTarget;
    int idj = -1;
    int idi = -1;

    LinearAgent trajectory(agi.GetLocalization(), agj.GetLocalization(),
                           agi.GetRealRadius() + agj.GetRealRadius(), agj.GetV(),
                           agj.GetId());

    //Compute which agent should make the biggest effort
    double di = std::sqrt(std::pow(raiz.x, 2) + std::pow(raiz.y, 2));
    double dj = std::sqrt(std::pow(trajectory.GetLocalization().x - raiz.x, 2) +
                          std::pow(trajectory.GetLocalization().y - raiz.y, 2));

    if (agi.GetV() > 0) di = di / agi.GetV();
    else di = DBL_MAX;
    if (agj.GetV() > 0) dj = dj / agj.GetV();
    else dj = DBL_MAX;
    Velocidad currenti, currentj;
    Command cfirst, clast, cinf;
    Command cinfIt = agi.GetStraightLineCommand(agj.GetId());
    Command cinfItInside = agj.GetStraightLineCommand(agi.GetId());
    //double av = (*it)->GetAV(); double aw = (*it)->GetAW();
    if (di < dj) {   //Agent j should make the effort
        idj = agj.GetId();
        idi = agi.GetId();
        currentj = {agj.GetV(), agj.GetW()};
        currenti = {agi.GetV(), agi.GetW()};
        trajectory = LinearAgent(agj.GetLocalization(), agi.GetLocalization(),
                                 agj.GetRealRadius() + agi.GetRealRadius(),
                                 agi.GetV(), agi.GetId());
        cfirst = agj.GetFirstCommand(idi);
        clast = agj.GetLastCommand(idi);
        cinf = agj.GetStraightLineCommand(idi);
    } else {
        idj = agi.GetId();
        idi = agj.GetId();
        currentj = {agi.GetV(), agi.GetW()};
        currenti = {agj.GetV(), agj.GetW()};
        trajectory = LinearAgent(agi.GetLocalization(), agj.GetLocalization(),
                                 agi.GetRealRadius() + agj.GetRealRadius(), agj.GetV(),
                                 agj.GetId());
        cfirst = agi.GetFirstCommand(idi);
        clast = agi.GetLastCommand(idi);
        cinf = agi.GetStraightLineCommand(idi);
    }

    if (!trajectory.GetOriginInside()) {
        if (std::sin(trajectory.GetLocalization().tita) > 0) {
            //Agent i is moving from right to left (towards values of positive 'y') relative to agent j
            //Then agent j should move to the right
            vTarget = clast.inf.vel; //OR clast.sup.vel OR clast.inf.vel + (clast.sup.vel - clast.inf.vel)/2
        } else {
            //Agent i is moving from left to right (towards values of negative 'y') relative to agent j
            //Then agent j should move to the right
            vTarget = cfirst.inf.vel;
        }
    } else {
        if (trajectory.GetLocalization().y >= 0) {
            vTarget = {bounds.vlim_max, clast.sup.vel.w};
        } else {
            vTarget = {bounds.vlim_max, cfirst.sup.vel.w};
        }
    }

    return std::make_pair(vTarget, idj);
}

Velocidad ComputeVel(double ang, boundsVS bounds);
double AngDisplacement(const Root point, const double rcandidate, const unsigned k);
std::pair<Eigen::Matrix<Velocidad, 4, 4>, int>
ComputeMotionReciprocal(std::vector<std::unique_ptr<Agent>> *agents, boundsVS bounds, double time_step, double min_dist,
                        const bool rca, const int algorithm) {

    Eigen::Matrix<double,10,10> utilidad;
    Eigen::Matrix<Velocidad,4,4> motion;
    int motionRec = -1;
    if ((int)agents->size() > 1) {
        int row = 0;
        std::shared_ptr<ActiveAgent> agi, AgJ;
        for (auto it = agents->begin(); it != agents->end(); ++it) {    //agent i
            for (auto itInside = agents->begin(); itInside != agents->end(); ++itInside) {  //agent j
                if ((*itInside)->GetId() != (*it)->GetId()) {
                    //Compute the distance from agents' position to the point of intersection of the CB
                    LinearAgent trajectory((*it)->GetLocalization(), (*itInside)->GetLocalization(),
                                           (*it)->GetRealRadius() + (*itInside)->GetRealRadius(), (*itInside)->GetV(),
                                           (*itInside)->GetId());
                    Tsc raiz;
                    double offset = (*itInside)->GetRealRadius() + (*it)->GetRealRadius();
                    bool collaborate = rca && deviateCB(trajectory.GetLocalization(), trajectory.GetTrajectory(), offset, raiz) && !(*itInside)->IsFinished(min_dist);
                    if (collaborate){

                        std::pair<Velocidad, int> target = ComputeTarget((**it), (**itInside), raiz, bounds);

                        switch (algorithm){
                            case 0: //Greedy
                                if (target.second == (*itInside)->GetId()) {
                                    motion(row, target.second) = (*itInside)->MotionReciprocal(target.first, time_step, bounds, "ra",0); //the effort
                                    motion(row, (*it)->GetId()) = (*it)->MotionReciprocal({bounds.vlim_max, target.first.w}, time_step, bounds,"co", 0); //the effort
                                    //motion(row, (*it)->GetId()) = (*it)->MotionReciprocal(vTarget, time_step, bounds,"co", 0); //the effort
                                } else {
                                    motion(row, target.second) = (*it)->MotionReciprocal(target.first, time_step, bounds, "ra", 0); //the effort
                                    motion(row, (*it)->GetId()) = (*itInside)->MotionReciprocal({bounds.vlim_max, target.first.w}, time_step, bounds, "co", 0); //collaborative
                                    //motion(row, (*it)->GetId()) = (*itInside)->MotionReciprocal(vTarget, time_step, bounds, "co", 0); //collaborative
                                }
                                break;
                            case 1: //Astar
                                if (target.second == (*itInside)->GetId()){
                                    //motion(row, target.second) = (**itInside).MotionReciprocalDOVTS(target.first, time_step, bounds, "ra"); //the effort
                                    //motion(row, (*it)->GetId()) = (*it)->MotionReciprocalDOVTS({bounds.vlim_max, target.first.w}, time_step, bounds, "co");  //collaborative
                                }else{
                                    //motion(row, target.second) = (*it)->MotionReciprocalDOVTS(target.first, time_step, bounds, "ra");   //the effort
                                    //motion(row, (*it)->GetId()) = (*itInside)->MotionReciprocalDOVTS({bounds.vlim_max, target.first.w}, time_step, bounds, "co"); //collaborative
                                }
                                break;
                            case 2: //Greedy + VS correction of goal
                                if (target.second == (*itInside)->GetId()) {
                                    motion(row, target.second) = (*itInside)->MotionReciprocal(target.first, time_step, bounds, "ra", 0); //the effort
                                    //motion(row, (*it)->GetId()) = (*it)->MotionReciprocal({bounds.vlim_max, vTarget.w}, time_step, bounds, "co", 0); //the effort
                                    motion(row, (*it)->GetId()) = (*it)->MotionReciprocalDOVS({bounds.vlim_max, target.first.w}, time_step, bounds, "co", 0, agents);  //collaborative
                                } else {
                                    motion(row, target.second) = (*it)->MotionReciprocal(target.first, time_step, bounds, "ra", 0); //the effort
                                    //motion(row, (*it)->GetId()) = (*itInside)->MotionReciprocal({bounds.vlim_max, vTarget.w}, time_step, bounds, "co", 0); //collaborative
                                    motion(row, (*it)->GetId()) = (*itInside)->MotionReciprocalDOVS({bounds.vlim_max, target.first.w}, time_step, bounds, "ra", 0, agents);   //the effort
                                }
                        }
                        if (motionRec == -1) motionRec = row;
                    } else {
                        Tpf goal;
                        (*itInside)->GetCurrentGoal(goal);
                        if ((*itInside)->IsFinished(min_dist)) motion(row, (*itInside)->GetId()) = Velocidad();
                        else {
                            Velocidad vTarget = ComputeVelGoal(goal, (*itInside)->GetLocalization(), bounds, time_step);
                            switch (algorithm) {
                                case 0 : default:
                                    motion(row, (*itInside)->GetId()) = (*itInside)->MotionReciprocal(vTarget,time_step, bounds,"ac",0); //no reciprocal avoiding strategy
                                    break;
                                //case 1:
                                    //motion(row, (*itInside)->GetId()) = (*itInside)->MotionReciprocalDOVTS(vTarget,time_step,bounds,"ac");
                            }
                        }

                        if ((*it)->IsFinished(min_dist)) motion(row, (*it)->GetId()) = Velocidad();
                        else {
                            (*it)->GetCurrentGoal(goal);
                            Velocidad vTarget = ComputeVelGoal(goal, (*it)->GetLocalization(), bounds, time_step);
                            switch (algorithm){
                                case 0 : default:
                                    motion(row, (*it)->GetId()) = (*it)->MotionReciprocal(vTarget, time_step, bounds, "ac",0); //no reciprocal avoiding strategy
                                    break;
                                //case 1:
                                    //motion(row, (*it)->GetId()) = (*it)->MotionReciprocalDOVTS(vTarget, time_step, bounds,"ac"); //no reciprocal avoiding strategy
                            }

                        }
                        if (motionRec == -1) motionRec = row;
                    }
                }
            }
            row++;
        }
    }else{
        auto it = agents->begin();
        Tpf goal; (*it)->GetCurrentGoal(goal);
        Velocidad vTarget = ComputeVelGoal(goal, (*it)->GetLocalization(), bounds, time_step);
        switch (algorithm) {
            case 0 : default:
                motion(0, (*it)->GetId()) = (*it)->MotionReciprocal(vTarget, time_step, bounds, "ac", 0); //no reciprocal avoiding strategy
                break;
            //case 1:
                //motion(0, (*it)->GetId()) = (*it)->MotionReciprocalDOVTS(vTarget, time_step, bounds, "ac");
        }
        motionRec = 0;
    }

    return std::make_pair(motion, motionRec);
}

std::pair<Eigen::Matrix<Velocidad, 10, 10>, int>
SelectVelocity(std::vector<std::unique_ptr<Agent>> *agents, boundsVS bounds, double time_step, double min_dist,
                        const bool rca, const int algorithm) {

    Eigen::Matrix<Velocidad,10,10> motion;
    int row = 0;

    for (auto it = agents->begin(); it != agents->end(); ++it) {
        (*it)->PlotVSData(bounds, time_step);
    }

    for (auto it = agents->begin(); it != agents->end(); ++it) {
        motion(row, (*it)->GetId()) = (*it)->SelectVelocity(bounds, time_step);
    }

    return std::make_pair(motion, row);
};

double StepsToReachVelocity(const Velocidad current, const Velocidad goal, const constraints c){

    Velocidad follow = goal;

    double t = std::ceil(std::abs(follow.w - current.w)/c.aw);
    if (goal.v > current.v) t += std::ceil(std::abs(follow.v - current.v)/c.av);

    /*
    //TODO: Considering the radius of circunference the agent is following
    if (goal.w != 0){
        follow.w = current.v / (goal.v/goal.w); follow.v = current.v;
        double t = std::ceil(std::abs(follow.w - current.w)/c.aw);
        double d = std::sqrt(std::pow(follow.w - goal.w,2) + std::pow(follow.v - goal.v,2));
    }
    */

    return t;
}

std::pair<Eigen::Matrix<Velocidad, 10, 10>, int>
ComputeMotionReciprocal2(std::vector<std::unique_ptr<Agent>> *agents, boundsVS bounds, double time_step, double min_dist,
                         const bool rca, const int algorithm) {
//Function to implement a decision algorithm based on a utility function
    std::ifstream fp("data/planner3d_data.txt");
    double th; unsigned n, m; unsigned long numC;
    fp >> th;   //time horizon
    fp >> n; fp >> m; //dovt size
    fp >> numC;  //number of cells to fill downwards

    assert(th > 0 && n > 0 && m > 0 && numC >= 0);

    Eigen::Matrix<double,10,10> utilidad = Eigen::Matrix<double,10,10>::Zero(); std::vector<double> total;
    Eigen::Matrix<Comando,10,10> minVel, velObj, infVel;
    Eigen::Matrix<double,10,10> tReachV = Eigen::Matrix<double,10,10>::Zero();
    Eigen::Matrix<Velocidad,10,10> motion;
    int motionRec = 0; int idMax = -1; double utilMax = 1e5;
    if ((int)agents->size() > 1) {
        int row = 0;
        std::shared_ptr<ActiveAgent> agi, AgJ;
        for (auto it = agents->begin(); it != agents->end(); ++it) {    //agent i
            //(*it)->PlotVSData(bounds, time_step);
            double totalU = 0;
            for (auto itInside = agents->begin(); itInside != agents->end(); ++itInside) {  //agent j
                if ((*itInside)->GetId() != (*it)->GetId()) {
                    //Compute the distance from agents' position to the point of intersection of the CB
                    LinearAgent trajectory((*it)->GetLocalization(), (*itInside)->GetLocalization(),
                                           (*it)->GetRealRadius() + (*itInside)->GetRealRadius(), (*itInside)->GetV(),
                                           (*itInside)->GetId());
                    Tsc raiz;
                    double offset = (*itInside)->GetRealRadius() + (*it)->GetRealRadius();
                    bool collaborate =
                            rca && deviateCB(trajectory.GetLocalization(), trajectory.GetTrajectory(), offset, raiz) &&
                            !(*itInside)->IsFinished(min_dist);
                    if (collaborate) {

                        Tpf goal;
                        (*it)->GetCurrentGoal(goal);

                        VS space(bounds);
                        Tpf goalAg;
                        Tsc loc = (*it)->GetLocalization();
                        transfor_inversa_p(goal.x, goal.y, &loc, &goalAg);
                        space.InsertGoal(goalAg, time_step);
                        goalVS gVS = space.GetGoal();
                        Velocidad gV = FollowGoal(goalAg, gVS, bounds);

                        Command cinf = (*it)->GetStraightLineCommand((*itInside)->GetId());
                        Command cmin = (*it)->GetMinVelocityCommand((*itInside)->GetId());  //Agij (valley)

                        double f1 = std::abs(gV.w - (*it)->GetW()) / std::abs(bounds.wmax_right - bounds.wmax_left);
                        double f2 = 0;
                        if (!cinf.operator==(Command())) {
                            if (cmin.operator==(
                                    Command(Comando(Velocidad(bounds.vlim_max, 0), 0), Comando(Velocidad(0, 0), 0), 1,
                                            cmin.id)))
                                cmin = cinf;
                            //time to reach the minimum velocity to escape velocity and pass before the agent j
                            //f2 = cmin.sup.t - time_step * StepsToReachVelocity({(*it)->GetV(), (*it)->GetW()}, cmin.sup.vel, constraints((*it)->GetAV(), (*it)->GetAW()));
                            //f2 /= (bounds.vlim_max / (*it)->GetAV());
                            double tMax = time_step * StepsToReachVelocity({0, bounds.wmax_right},
                                                                           {bounds.vlim_max, bounds.wmax_left},
                                                                           constraints((*it)->GetAV(), (*it)->GetAW()));
                            //(std::ceil(std::abs(bounds.wmax_left - bounds.wmax_right)/(*it)->GetAW()) + std::ceil(std::abs(bounds.vlim_max-0)/(*it)->GetAV()));
                            f2 = time_step * StepsToReachVelocity({(*it)->GetV(), (*it)->GetW()}, cmin.sup.vel,
                                                                  constraints((*it)->GetAV(), (*it)->GetAW()));
                            if (cmin.sup.t - f2 < 0)
                                f2 = 1e5;   //There is no enough time to reach the velocity before the collision occurs
                            else {
                                //f2 /= tMax;

                                Command cminAgji = (*itInside)->GetMinVelocityCommand((*it)->GetId());
                                cinf = (*itInside)->GetStraightLineCommand((*it)->GetId());
                                double flow = cminAgji.sup.t;
                                f2 += flow;
                                f2 = flow;
                                if (flow > 10) continue;
                            }
                            //TODO: if f2 above a threshold, do not consider for collaboration
                            //f2 /= tMax;
                            minVel((*it)->GetId(), (*itInside)->GetId()) = cmin.sup;
                            infVel((*it)->GetId(), (*itInside)->GetId()) = cinf.sup;
                            tReachV((*it)->GetId(), (*itInside)->GetId()) = time_step * StepsToReachVelocity(
                                    {(*it)->GetV(), (*it)->GetW()}, cmin.sup.vel,
                                    constraints((*it)->GetAV(), (*it)->GetAW()));
                        } else
                            std::cout << "COMMAND INF NULL" << std::endl;
                        double f3 = (bounds.vlim_max - (*it)->GetV()) / bounds.vlim_max;

                        f1 *= 0.3; f2 *= 0.6; f3 *= 0.1;
                        f1 = 0; f3 = 0;
                        utilidad((*it)->GetId(), (*itInside)->GetId()) = (f1 + f2 + f3 != 0 ? 1 / (f1 + f2 + f3) : 0);

                    } else {
                        std::cout << "Agents DO NOT COLLABORATE" << std::endl;
                    }
                }
            }
        }

        //Matrix with the objective velocities which allow to pass before or after an agent
        Eigen::Matrix<int, 10, 10> action = Eigen::Matrix<int, 10, 10>::Zero();
        for (int i = 0; i < (int) agents->size(); i++) {
            for (int j = 0; j < (int) agents->size(); j++) {
                if (i != j) {
                    double uij = utilidad(i, j); double uji = utilidad(j, i);
                    if (uij > uji) {
                        velObj(i, j) = minVel(i, j);
                        action(i, j) = 1;
                    }else if (uij == uji){    //Symmetrical case
                        //Give priority to the agent on the right

                        Agent &agi = (*agents->at(i));
                        Agent &agj = (*agents->at(j));

                        LinearAgent trajectory(agi.GetLocalization(), agj.GetLocalization(),
                                               agi.GetRealRadius() + agj.GetRealRadius(), agj.GetV(),
                                               agj.GetId());

                        //if (!trajectory.GetOriginInside()) {
                            if (std::sin(trajectory.GetLocalization().tita) > 0) {
                                //Agent j is moving from right to left (towards values of positive 'y') relative to agent i
                                //Then agent i should move to the right to give way to agent j
                                velObj(i, j) = agi.GetLastCommand(agj.GetId()).inf;
                                utilidad(i,j) -= 1e-3;
                                velObj(j, i) = minVel(j, i);
                            } else {
                                //Agent j is moving from left to right (towards values of negative 'y') relative to agent i
                                //Then agent i has priority to move over agent j => its utility is decreased an epsilon
                                velObj(i, j) = minVel(i, j);
                                utilidad(j,i) -= 1e-3;
                            }
                        //}
                    }
                }
            }
        }

        for (int i = 0; i < (int) agents->size(); i++) {
            for (int j = 0; j < (int) agents->size(); j++) {
                if (i != j) {
                    if (utilidad(i, j) < utilidad(j, i)) {
                        Comando c = velObj(j, i);

                        Agent &agi = (*agents->at(i));
                        Agent &agj = (*agents->at(j));

                        LinearAgent trajectory(agi.GetLocalization(), agj.GetLocalization(),
                                               agi.GetRealRadius() + agj.GetRealRadius(), agj.GetV(),
                                               agj.GetId());

                        if (!trajectory.GetOriginInside()) {
                            if (std::sin(trajectory.GetLocalization().tita) > 0) {
                                //Agent j is moving from right to left (towards values of positive 'y') relative to agent i
                                //Then agent j should move to the right
                                velObj(i, j) = agi.GetLastCommand(agj.GetId()).inf;
                            } else {
                                //Agent j is moving from left to right (towards values of negative 'y') relative to agent i
                                //Then agent j should move to the right
                                velObj(i, j) = agi.GetFirstCommand(agj.GetId()).inf;
                            }
                        } else {
                            if (trajectory.GetLocalization().y >= 0) {
                                Comando c = agi.GetFirstCommand(agj.GetId()).inf;
                                double cw = c.vel.w > 0 ? 0 : c.vel.w;
                                velObj(i, j) = {{bounds.vlim_max, cw}, c.t};
                            } else {
                                Comando c = agi.GetFirstCommand(agj.GetId()).inf;
                                double cw = c.vel.w < 0 ? 0 : c.vel.w;
                                velObj(i, j) = {{bounds.vlim_max, cw}, c.t};
                            }
                        }
                        if (velObj(i,j).vel.v > agi.GetV()) velObj(i,j) = {{agi.GetV(), velObj(i,j).vel.w}, velObj(i,j).t};
                    }
                }
            }
        }

        //TODO: Generate clusters of agents for consideration during the reciprocal avoidance collision
        //std::vector<std::vector<std::pair<int, int>>> clusters;
        //Look for best utility
        double bestUtil = 0; bool finished = false;
        double maxUtil = 1e5;
        while (!finished){
            bestUtil = 0; int agiId = -1; int agjId = -1;
            for (auto it = agents->begin(); it != agents->end(); ++it) {
                for (auto itAgj = agents->begin(); itAgj != agents->end(); ++itAgj) {
                    int i = (*it)->GetId(); int j = (*itAgj)->GetId();
                    if (i != j) {
                        if (utilidad(i,j) < maxUtil && utilidad(i,j) > bestUtil && utilidad(j,i) > 0) {   //To consider the case when agent i already passed agent j
                            bestUtil = utilidad(i,j);
                            agiId = i;
                            agjId = j;
                        }
                    }
                }
            }

            if (agiId == -1 || agjId == -1) break;

            //std::vector<std::pair<int,int>> cluster;
            std::pair<int,int> peer = std::make_pair(agiId, agjId);
            //cluster.push_back(peer);
            double minUtil = utilidad(agjId,agiId);
            //look for another pair which has to be resolved in time between peer(agiId, agjId)
            for (auto it = agents->begin(); it != agents->end(); ++it) {
                for (auto itAgj = agents->begin(); itAgj != agents->end(); ++itAgj) {
                    int i = (*it)->GetId(); int j = (*itAgj)->GetId();
                    if (i != j) {
                        if (utilidad(i,j) < bestUtil && utilidad(i,j) > minUtil && utilidad(i,j) > 0){  //To consider the case when agent i already passed agent j
                            //if (std::find(cluster.begin(), cluster.end(), peer) == cluster.end()){
                            //    peer = (utilidad(i,j) > utilidad(j,i) ? std::make_pair(i,j) : std::make_pair(j,i));
                            //    cluster.push_back(peer);
                            //}
                            minUtil = std::min(minUtil, std::min(utilidad(i,j), utilidad(j,i)));
                        }
                    }
                }
            }
            //clusters.push_back(cluster);
            maxUtil = minUtil;
        }

        //...

        //Compute the next motion to reach the objective velocity
        std::vector<std::pair<Comando, int>> velTarget;
        for (auto it = agents->begin(); it != agents->end(); ++it){
            int i = (*it)->GetId();
            for (auto itAgj = agents->begin(); itAgj != agents->end(); ++itAgj) {
                int j = (*itAgj)->GetId();
                if (i != j){
                    if (!(velObj(i,j).vel == Velocidad())) velTarget.push_back(std::make_pair(velObj(i,j),action(i,j)));
                    else{
                        if ((*it)->IsFinished(min_dist)) motion(0, i) = Velocidad();
                        else {
                            Tpf goal;
                            (*it)->GetCurrentGoal(goal);
                            velTarget.push_back(std::make_pair(Comando(ComputeVelGoal(goal, (*it)->GetLocalization(), bounds, time_step), th), 0));
                        }
                    }
                }
            }
            std::sort(velTarget.begin(), velTarget.end(), [](const std::pair<Comando, int> &l, const std::pair<Comando, int> &r){return l.first.t < r.first.t;});

            switch (algorithm) {
                case 0 :
                default:
                    {
                    Comando vTarget;
                    if (!velTarget.empty()) vTarget = velTarget[0].first;
                        //velTarget.pop_front();
                        if (bestUtil > 0)
                            motion(0, (*it)->GetId()) = (*it)->MotionReciprocal2(vTarget.vel, time_step, bounds, "ac", 0, -1, -1); //no reciprocal avoiding strategy
                        else
                            motion(0, (*it)->GetId()) = (*it)->MotionReciprocal(vTarget.vel, time_step, bounds, "ac", 0); //no reciprocal avoiding strategy
                    }
                    break;
                case 1:
                    motion(0, (*it)->GetId()) = (*it)->MotionReciprocalDOVTS(std::list<std::pair<Comando, int>>(velTarget.begin(), velTarget.end()), time_step, bounds, "ac");
            }
            velTarget.clear();
        }

    }else{
        auto it = agents->begin();
        Tpf goal; (*it)->GetCurrentGoal(goal);
        Velocidad vTarget = ComputeVelGoal(goal, (*it)->GetLocalization(), bounds, time_step);
        std::list<std::pair<Comando, int>> velTarget; velTarget.push_back(std::make_pair(Comando(vTarget, th),0));
        switch (algorithm) {
            case 0 : default:
                motion(0, (*it)->GetId()) = (*it)->MotionReciprocal(vTarget, time_step, bounds, "ac", 0); //no reciprocal avoiding strategy
                break;
            case 1:
                motion(0, (*it)->GetId()) = (*it)->MotionReciprocalDOVTS(velTarget, time_step, bounds, "ac");
        }
    }

    return std::make_pair(motion, motionRec);

}

Eigen::Matrix<double,2,2> ComputeMatrix(std::vector<std::unique_ptr<Agent>> *agents);
void PlotData(std::vector<std::unique_ptr<Agent>> *agents);
void Simulation::MotionShared(const bool rca, const int algorithm) {

    //Eigen::Matrix<Velocidad, 2, 2> motion = ComputeMotionReciprocal2(&agents, bounds, time_step);
    ///*
    std::pair<Eigen::Matrix<Velocidad, 10, 10>, int> motion;
    //motion = ComputeMotionReciprocal(&agents, bounds, time_step, min_dist, rca, algorithm);
    //motion = ComputeMotionReciprocal2(&agents, bounds, time_step, min_dist, rca, algorithm);
    //motion = SelectVelocity(&agents, bounds, time_step, min_dist, rca, algorithm);

    //Plot the trajectories of the agents and its velocity vectors
    //PlotData(&agents);

    if (motion.second == -1) motion.second = 0;
    for (auto it = agents.begin(); it != agents.end(); ++it)
        if ((*it)->IsFinished(min_dist)) (*it)->SetVelocity(Velocidad());
        else (*it)->SetVelocity(motion.first(motion.second, (*it)->GetId()));

    if (logSim){
        for (auto it = agents.begin(); it != agents.end(); ++it)
            (*it)->WriteMetricsLog(&agents);
        flog.open(name_fich, std::ios::app);
        flog << std::endl;
        flog.close();
    }
    //*/

}

Eigen::Matrix<double,2,2> ComputeMatrix(std::vector<std::unique_ptr<Agent>> *agents){

    Eigen::Matrix<double, 2, 2> values;
    for (auto it = agents->begin(); it != agents->end(); ++it) {
        Eigen::Vector2d vAg1((*it)->GetV(), 0);
        for (auto itInside = agents->begin(); itInside != agents->end(); ++itInside) {
            if ((*itInside)->GetId() != (*it)->GetId()) {
                LinearAgent trajectory((*it)->GetLocalization(), (*itInside)->GetLocalization(), (*it)->GetRealRadius() + (*itInside)->GetRealRadius(), (*itInside)->GetV(), (*itInside)->GetId());
                Eigen::Vector2d vAg2(trajectory.GetVX(), trajectory.GetVY());
                values((*it)->GetId(), (*itInside)->GetId()) = vAg1.dot(vAg2);

            }else{
                values((*it)->GetId(), (*it)->GetId()) = vAg1.dot(vAg1);
            }
        }
    }
    std::cout << "[" << values(0,0) << ", " << values(0,1) << std::endl << values(1,0) << ", " << values(1,1) << "]" << std::endl;
    return values;
};

void PlotData(std::vector<std::unique_ptr<Agent>> *agents){
    //Plot the trajectories of the agents and its velocity vectors

    for (auto it = agents->begin(); it != agents->end(); ++it) {
        for (auto itInside = agents->begin(); itInside != agents->end(); ++itInside) {
            if ((*it)->GetId() != (*itInside)->GetId()){

                dynamic_cast<ActiveAgent&>(**it).PlotRelativeInfo(dynamic_cast<ActiveAgent&>(**itInside));
            }
        }
    }
}

void Simulation::createVideo() {
    if (video) {
        system("ffmpeg -r 60 -f image2 -s 1920x1080 -i output/we%06d.png -vcodec libx264 -crf 25 -pix_fmt yuv420p we.mp4");
        system("ffmpeg -r 60 -f image2 -s 1920x1080 -i output/vs%06d.png -vcodec libx264 -crf 25 -pix_fmt yuv420p vs.mp4");
        system("ffmpeg -i we.mp4 -i vs.mp4 -filter_complex hstack output.mp4");
        system("rm we.mp4");
        system("rm vs.mp4");
        boost::filesystem::remove_all("output");
    }
}

/**
 * Returns the posible actions for the active agents
 */
std::vector<std::vector<std::pair<int, Velocidad>>> Simulation::getActions(int lookAhead, 
                                                                           int &zone) {

    //PARAMETERS FOR PLANNING
    std::ifstream fp("data/planner3d_data.txt");
    double th; unsigned n, m; unsigned long numC;
    fp >> th;   //time horizon
    fp >> n; fp >> m; //dovt size
    fp >> numC;  //number of cells to filled downwards

    assert(th > 0 && n > 0 && m > 0 && numC >= 0);

    std::vector<Velocidad> motion; motion.resize((int)agents.size());
    std::vector<std::pair<int, Velocidad>> actions;
    std::pair<int, Velocidad> velEnd;
    velEnd.first = 0;
    velEnd.second = Velocidad(0, 0);
    int id = 0;

    bool fr;

    std::vector<std::vector<std::pair<int, Velocidad>>> totalActions 
            = std::vector<std::vector<std::pair<int, Velocidad>>>();

    // Select possible velocities for the different type of agents
    for (int i=0; i<(int)agents.size(); i++){
        if (agents[i]->active) {
            if (agents[i]->IsFinished(min_dist)) {
                motion[i] = Velocidad(0,0);
                actions.push_back(velEnd);
                totalActions.push_back(actions);
            }
            else{
                totalActions.push_back(agents[i]->MotionStrategyRL(time_step, &agents, 
                        fr, this->video, iteracion, graph, debug));
            }
        } else {
            id++;
            motion[i] = Velocidad(agents[i]->getV0(),agents[i]->getW0());
        }
    }

    // Set velocities for passive and finished agents
#pragma omp parallel for
    for (int i=0; i<(int)agents.size(); i++){
        if (!agents[i]->active) {
            agents[i]->SetVelocity(motion[i]);
        }
    }

    return totalActions;
}

/**
 * Returns the states of the different active agents
 */
std::vector<State> Simulation::getStates(int zone, bool freeDirToGoal) {
    
    int id = 0;
    std::vector<State> states = std::vector<State>();
    int found = 0;
    
    for (auto it = agents.begin(); it != agents.end(); ++it) {
        if ((*it)->active) {
            // Update last distance
            this->distLastState[id] = this->distCurrentState[id];

            // Update last theta
            this->thetaLastState[id] = this->thetaCurrentState[id];

            // Update last distance to obstacle
            this->distObsLastState[id] = this->distObsCurrentState[id];

            // Update last velocities
            this->vLastState[id] = this->vCurrentState[id];
            this->wLastState[id] = this->wCurrentState[id];
            // Update ID of last closest agent
            this->lastClosestAgent[id] = this->currentClosestAgent[id];

            // Update orientation of last closest agent
            this->thetaObsLastState[id] = this->thetaObsCurrentState[id];
            this->relThetaLastState[id] = this->relThetaCurrentState[id];

            // Update last distances and orientations to CB points
            this->lastCollision[id] = this->currentCollision[id];

            double d = 10000;
            Tpf goalAg;
            double goalX, goalY;
            double radio_seg;
            PosibleVelocidad pVel;
            double v, w, vObs;
            Tsc posObs;
            Tsc loc;
            double thetaObs = 0;
            int idx;
            found = 0;
            for (auto it = agents.begin(); it != agents.end(); ++it){
                if ((*it)->active) {
                    found++;
                    if (found-1 == id) {
                        for (auto it2 = agents.begin(); it2 != agents.end(); ++it2){
                            if ((*it2)->GetId() != (*it)->GetId()){
                                radio_seg = (*it)->GetRealRadius() + (*it2)->GetRealRadius();
                                double distancia = Distancia((*it)->GetLocalization().x 
                                    - (*it2)->GetLocalization().x, (*it)->GetLocalization().y 
                                    - (*it2)->GetLocalization().y);
                                if (distancia - radio_seg < d) {
                                    d = distancia - radio_seg;
                                    this->currentClosestAgent[id] = (*it2)->GetId();
                                    posObs = (*it2)->GetLocalization();
                                    vObs = (*it2)->GetV();
                                    thetaObs = posObs.tita;
                                    idx = (*it2)->GetId();
                                }
                            }
                        }
                        (*it)->GetCurrentGoal(goalAg);
                        goalX = goalAg.x; goalY = goalAg.y;
                        loc = (*it)->GetLocalization();
                        transfor_inversa_p(goalX, goalY, &loc, &goalAg);

                        v = (*it)->GetV();
                        w = (*it)->GetW();

                        pVel = dynamic_cast<ActiveAgent*>((*it).get())->findClosestSafeVelocity();
                    }
                }
            }

            // Update current distance
            this->distCurrentState[id] = sqrt(goalAg.x*goalAg.x + goalAg.y*goalAg.y);

            // Update current theta
            this->thetaCurrentState[id] = atan2(goalAg.y, goalAg.x);

            // Update current distance to obstacle
            this->distObsCurrentState[id] = d;

            // Update current velocities
            this->vCurrentState[id] = v;
            this->wCurrentState[id] = w;

            int dGoal; 
            //if (this->distCurrentState[id] < 1) {
            //    dGoal = 0; // Very close
            //} else if (this->distCurrentState[id] < 2) {
            //    dGoal = 5; // Very close
            //} else if (this->distCurrentState[id] < 4) {
            if (this->distCurrentState[id] < 4) {
                dGoal = 1; // Close
            } else if (this->distCurrentState[id] < 8) {
                dGoal = 2; // Far
            } else if (this->distCurrentState[id] < 40) {
                dGoal = 3; // Very far
            } else {
                dGoal = 4; // Too far, terminate episode
            }

            int thGoal;
            if (atan2(goalAg.y, goalAg.x) < 0.1 && atan2(goalAg.y, goalAg.x) > -0.1) {
                thGoal = 0;
            } else if (atan2(goalAg.y, goalAg.x) > 2*M_PI/3 ) {
                thGoal = 1;
            } else if (atan2(goalAg.y, goalAg.x) < -2*M_PI/3) {
                thGoal = 2;
            } else if (atan2(goalAg.y, goalAg.x) > 0) {
                thGoal = 3; // Esta hacia la izquierda
            } else {
                thGoal = 4; // Esta hacia la derecha
            }

            int vel = 0;
            if (v > 1) {
                vel = 4;
            } else if (v > 0.75) {
                vel = 3;
            } else if (v > 0.5) {
                vel = 5;
            // } else if (v > 0.25) {
            //     vel = 2;
            } else if (v > 0.1) {
                vel = 1;
            } else {
                vel = 0;
            }

            int angular = 0;
            if (w > 0.5) {
                angular = 0;
            } else if (w > 0.1) {
                angular = 1;
            } else if (w <= 0.1 && w >= -0.1) {
                angular = 2;
            } else if (w > -0.5) {
                angular = 3;
            } else {
                angular = 4;
            }

            int freeVelocities;
            // if (this->freeActions[id]) {
            if (true) {
                if (freeDirToGoal) {
                    freeVelocities = 1;
                } else {
                    freeVelocities = 0;
                }
            } else {
                freeVelocities = 2;
            }

            int closestFreeVelocity;
            if (freeVelocities == 2) {
                if (pVel.valid) {
                    if (pVel.vel.v > v) {
                        closestFreeVelocity = 2; // Closest free velocity is above
                    } else if (pVel.vel.v < v) {
                        closestFreeVelocity = 4; // Closest free velocity is under
                    } else if (pVel.vel.w < w) {
                        closestFreeVelocity = 1; // Closest free velocity is to the left
                    } else {
                        closestFreeVelocity = 3; // Closest free velocity is to the right
                    }
                } else {
                    closestFreeVelocity = 0; // There are no free velocities
                }
            } else {
                closestFreeVelocity = 5;
            }

            float theta = loc.tita;
            std::vector<double> currentDirection {cos(theta), sin(theta)};
            std::vector<double> obsDirection {posObs.x - loc.x,posObs.y - loc.y};
            float dot = currentDirection[0] * obsDirection[0] + currentDirection[1]*obsDirection[1];
            float det = currentDirection[0]*obsDirection[1] - currentDirection[1]*obsDirection[0];
            float angle = atan2(det, dot);
            this->thetaObsCurrentState[id] = angle;
            this->relThetaCurrentState[id] =  atan2(sin(thetaObs-theta), cos(thetaObs-theta));

            int collision;

            // Going towards collision
            collision = this->lastCollision[id];

            int collisionDistance = 3;

            int thObs;
            if (collision > 0 ) {
                if (angle > 2*M_PI/3) {
                    thObs = 1;
                } else if (angle > M_PI/2) {
                    thObs = 2;
                } else if (angle > M_PI/3) {
                    thObs = 3;
                } else if (angle > 0) {
                    thObs = 4;
                } else if (angle > -M_PI/3) {
                    thObs = 5;
                } else if (angle > -M_PI/2) {
                    thObs = 6;
                } else if (angle > -2*M_PI/3) {
                    thObs = 7;
                } else {
                    thObs = 8;
                }
            } else {
                thObs = 0;
            }

            if ( (collision == 1) && this->currentClosestAgent[id] != this->lastClosestAgent[id] 
                && this->distObsCurrentState[id] < collisionDistance ) {
                std::cout << "New agent" << std::endl;
                this->currentCollision[id] = 0;
                this->lastCollision[id] = 0;
                collision = 0;
            }

            if (this->lastCollision[id] == 0 && this->distObsCurrentState[id] < collisionDistance 
                && this->distCurrentState[id] > 2 && (angle <= M_PI/3 && angle > -M_PI/3) ) {
                    std::cout << "New danger: agent " << this->currentClosestAgent[id] << std::endl;
                    std::cout << "THObs: " << thObs << std::endl;
                    this->lastCollisionAgent[id] = this->currentCollisionAgent[id];
                    this->currentCollisionAgent[id] = this->currentClosestAgent[id];
                    collision = 1;
                    this->currentCollision[id] = 1;
                    this->thetaObsOriginal[id] = angle;
            }


            if (this->currentCollision[id] == 1 && this->lastCollision[id] == 1 
                && ( (thObs != 4 && thObs != 5) 
                || this->currentClosestAgent[id] != this->lastClosestAgent[id]) ) {
                std::cout << "Evaded obstacle" << std::endl;
                collision = 0;
                this->currentCollision[id] = 0;
            }

            int relativeTh;
            if (collision > 0) {
                if (this->relThetaCurrentState[id]  > 2*M_PI/3) {
                    relativeTh = 0;
                } else if (this->relThetaCurrentState[id]  < -2*M_PI/3) {
                    relativeTh = 1;
                } else if (this->relThetaCurrentState[id]  > M_PI/3) {
                    relativeTh = 2;
                } else if (this->relThetaCurrentState[id]  < -M_PI/3) {
                    relativeTh = 3;
                } else if (this->relThetaCurrentState[id] > 0) {
                    relativeTh = 4;
                } else {
                    relativeTh = 5;
                }
            } else {
                relativeTh = 0;
            }

            int relativeVel;
            if (collision > 0) {
                if (vObs > 1) {
                    relativeVel = 0;
                } else if (vObs > 0.5) {
                    relativeVel = 1;
                } else if (vObs > 0.0) {
                    relativeVel = 2;
                } else {
                    relativeVel = 3;
                }
            } else {
                relativeVel = 0;
            }

            int dObs;
            if (collision == 1) {
                dObs = 1;
            } else {
                dObs = 0;
            }
            
            id++;
            states.push_back(State(vel, angular, closestFreeVelocity, dGoal, thGoal, 
                             relativeTh, relativeVel, thObs));
        }
    }

    return states;
}

std::vector<double> Simulation::getRewards(std::vector<State> lastStates, 
                                           std::vector<bool> chosenDirToGoal) {
    double rDistGoal, rThetaGoal, rDistObs, rTimeStep, rAcceleration, rVelocity, rThetaObs;
    double kDistGoal, kThetaGoal, kDistObs, kAcceleration, kVelocity, kThetaObs;
    std::vector<double> rewards = std::vector<double>();
    for (int k=0; k < lastStates.size(); k++) {
        // Reward for orientation to goal
        if (this->distCurrentState[k] < 1) {
            rThetaGoal = 0;
        } else {
            rThetaGoal = -(abs(this->thetaCurrentState[k]) - abs(this->thetaLastState[k]));
        }

        if (chosenDirToGoal[k] && this->thetaCurrentState[k] < M_PI/3 
            && this->thetaCurrentState[k] > -M_PI/3 )
            rThetaGoal += 0.3;
        // Reward for distance to goal
        if (this->distCurrentState[k] >= 1) {
            rDistGoal = -(this->distCurrentState[k] - this->distLastState[k]);
        } else {
            rDistGoal = 0;
        }
        // Penalization for time-step
        rTimeStep = -0.1;
        bool invalidate = false;

        if (this->currentCollision[k] == 1) {
            invalidate = true;
            rAcceleration = -5;
        } else if (this->distCurrentState[k]> 8 && ((this->thetaCurrentState[k] < M_PI/3 
                   && this->thetaCurrentState[k] > -M_PI/3) || chosenDirToGoal[k]) 
                   && this->vCurrentState[k] <= 1) {
            rAcceleration = (this->vCurrentState[k]-this->vLastState[k]);
        } else if (this->distCurrentState[k]> 8 && (this->thetaCurrentState[k] >= M_PI/3 
                   || this->thetaCurrentState[k] <= -M_PI/3) && this->vCurrentState[k] > 1) {
            rAcceleration = -(this->vCurrentState[k]-this->vLastState[k]);
        } else if ( this->distCurrentState[k] < 1) {
            rAcceleration = -(this->vCurrentState[k]-this->vLastState[k]);
        } else if (this->distCurrentState[k] < 2 && this->vCurrentState[k] > 0.5) {
            rAcceleration = -(this->vCurrentState[k]-this->vLastState[k]);
        } else if (this->distCurrentState[k] < 8 && this->vCurrentState[k] > 1) {
            rAcceleration = -(this->vCurrentState[k]-this->vLastState[k]);
        } else if (this->vCurrentState[k] < 0.5) {
            rAcceleration = (this->vCurrentState[k]-this->vLastState[k]);
        } else {
            rAcceleration = -5;
            invalidate = true;
        }


        if (rAcceleration > 0.01) {
            rAcceleration = 0.20;
        } else if (rAcceleration < 0.01) {
            rAcceleration = -0.20;
        } else {
            rAcceleration = -0.1;
        }

        if (invalidate) {
            rAcceleration = 0;
        }



        double rObs;
        if (this->currentCollision[k] > 0 && this->thetaObsCurrentState[k] < M_PI/3 
            && this->thetaObsCurrentState[k] > -M_PI/3) {
            rObs = abs(this->thetaObsCurrentState[k] - this->thetaObsOriginal[k]);
        } else {
            rObs = 0;
        }

        if (this->currentCollision[k] > 0)
            rObs -= 0.3;

        if (this->currentCollision[k] > 0 && this->thetaObsCurrentState[k] < M_PI/3 
            && this->thetaObsCurrentState[k] > -M_PI/3) {
            // rThetaGoal = 0;
            rDistGoal = 0;
        }

        double rAcc = 0;
        if ( (this->vCurrentState[k] != this->vLastState[k]) ) {
            rAcc = -0.3;
        }

        kDistGoal = 0.75; kThetaGoal = 2; kDistObs = 0; kAcceleration = 4; kVelocity = 0; 
        kThetaObs = 0;
        rewards.push_back(kDistGoal*rDistGoal + kThetaGoal*rThetaGoal + kAcceleration*rAcceleration 
                          + 4*rObs + rAcc);
    }
    return rewards;
}

void Simulation::executeActions(std::vector<Velocidad> actions) {
    int id = 0;
    for (int i=0; i < (int) agents.size(); i++) {
        if (agents[i]->active) {
            agents[i]->SetVelocity(actions[id]);
            id++;
        }
    }
}

void Simulation::writeQTable(std::map<State, std::array<double, 8>> qTable, std::string fileName) {
    std::map<State, std::array<double, 8>>::iterator it;
    std::ofstream of;
    std::cout << "Writting qTable to " << fileName << std::endl;
    of.open(fileName);
    for (it = qTable.begin(); it != qTable.end(); it++) {
        of << it->first.getCode() << "\t" << it->second[0] << "\t" << it->second[1] << "\t" 
           << it->second[2] << "\t" << it->second[3] << "\t" << it->second[4] << "\t" 
           << it->second[5]<< "\t" << it->second[6]<< "\t" << it->second[7] << std::endl;
    }
    of.close();
    std::cout << "Done!" << std::endl;
}

void Simulation::readQTable(std::map<State, std::array<double, 8>> &qTable, std::string fileName) {
    qTable.clear();
    std::string value;
    std::ifstream f(fileName);
    if (f.is_open()) {
        std::cout << "Reading qTable from " << fileName << std::endl;
        while (f.peek() != EOF) {
            f >> value;
            State newState(std::atol(value.c_str()));
            std::array<double, 8> newValues;
            for (int i=0; i < 8; i++) {
                f >> value;
                newValues[i] = std::atof(value.c_str());
            }

            qTable.insert(std::pair<State, std::array<double, 8>>(newState, newValues));
        }
        f.close();
        std::cout << "Done!" << std::endl;
    } else {
        std::cerr << "Could not open file " << fileName << std::endl;
    }

}