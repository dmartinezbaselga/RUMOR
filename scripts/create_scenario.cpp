#include <string>
#include <vector>
#include <memory>
#include <iostream>
#include <fstream>
#include <cmath>
#include <atomic>
#include <random>
#include <unistd.h>
#include <thread>

#include "../include/rl_dovs/graficas.h"
#include "../include/rl_dovs/config.h"

using namespace std;





class Agent {
    private:
        int id;
        double real_radius;
        double apparent_radius;
        Tsc loc, goal;    //initial localization (world coordinates)
        double v,w;
        double av, aw;
        double v0, w0;
        bool graph;
        bool active;
    public:
        Agent(int i, double x, double y, double tita, double v0, double w0, double av, double aw, double r, bool newActive){
            active = newActive;
            id = i;
            loc.x = x; loc.y = y; loc.tita = tita;
            v = v0; w = w0; apparent_radius = r; real_radius = r;
            this->av = av; 
            this->aw = aw;
            this->v0 = v0;
            this->w0 = w0;
        }
        void AddGoal(double xGoal, double yGoal){
            goal.x = xGoal;
            goal.y = yGoal;
        }
        Tsc GetFirstGoal(){ 
            return goal;
        }
        bool GetCurrentGoal(Tsc& g){ 
            g = goal;
            return active;
        }
        int GetId(){
            return id;
        }
        double GetV(){
            return v;
        }
        double GetW(){
            return w;
        }
        Tsc GetLocalization(){
            return loc;
        }
        double GetRealRadius(){
            return real_radius;
        }

        double GetAV(){
            return av;
        }
        double GetAW(){
            return aw;
        }
        double getV0(){
            return v0;
        }
        double getW0(){
            return w0;
        }
        bool isActive(){
            return active;
        }
};

std::vector<std::unique_ptr<Agent>> agents;
double lim_xmin, lim_xmax, lim_ymin, lim_ymax;
int display_xmax = 0;
int display_xmin = 0;
int display_ymin = 0;
int display_ymax = 0;
bool insertActive = true;
bool insertRandom = false;
int passives = 0;
atomic<bool> finished (false);
atomic<bool> quit (false);
atomic<bool> start (false);

std::random_device rd{};
std::mt19937 generator{rd()};
static std::uniform_real_distribution<double> distributionX;
static std::uniform_real_distribution<double> distributionY;
static std::uniform_real_distribution<double> distributionTita;
static std::uniform_real_distribution<double> distributionV;
static std::uniform_real_distribution<double> distributionW;
static std::uniform_real_distribution<double> distributionFixed;


double Distancia (double x, double y){

	return (std::sqrt(x*x+y*y));
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
void writeScenarioJSON(std::string scenario) {
    std::string json = "{\n";
    string type;
    json += "\t\"agents\": [\n";
    for (int i=0; i<(int)agents.size(); i++){
        type = agents[i]->isActive() ? "active" : "passive";
        json += "\t\t{\n";
        json += "\t\t\t\"type\": \"" + type + "\",\n";
        json += "\t\t\t\"radio\": " + std::to_string(agents[i]->GetRealRadius()) + ",\n";
        json += "\t\t\t\"x\": " + std::to_string(agents[i]->GetLocalization().x)+ ",\n";
        json += "\t\t\t\"y\": " + std::to_string(agents[i]->GetLocalization().y)+ ",\n";
        json += "\t\t\t\"theta\": " + std::to_string(agents[i]->GetLocalization().tita)+ ",\n";
        json += "\t\t\t\"v0\": " + std::to_string(agents[i]->getV0())+ ",\n";
        json += "\t\t\t\"w0\": " + std::to_string(agents[i]->getW0());
        if (agents[i]->isActive()) {
            json += ",\n";
            json += "\t\t\t\"av\": " + std::to_string(agents[i]->GetAV()) + ",\n";
            json += "\t\t\t\"aw\": " + std::to_string(agents[i]->GetAW()) + ",\n";
            json += "\t\t\t\"goal\": {\n";
            json += "\t\t\t\t\"x\": " + std::to_string(agents[i]->GetFirstGoal().x) + ",\n";
            json += "\t\t\t\t\"y\": " + std::to_string(agents[i]->GetFirstGoal().y) + "\n";
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

void calibrate() {
    Inicializar();
    SeleccionaVentana(1);
    LimpiaVentana();
    errmod("all", "off");

    double xmin = -10; double xmax = -xmin; double ymin = -10; double ymax = -ymin;
    lim_xmin = xmin; lim_xmax = xmax; lim_ymin = ymin; lim_ymax = ymax;
    DibujaEjes(lim_xmin, lim_xmax, lim_xmin, 2, lim_ymin, lim_ymax, lim_ymin, 2);
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
    std::ofstream of;
    of.open("data/calibration.txt");
    of << display_xmax << std::endl;
    of << display_xmin << std::endl;
    of << display_ymax << std::endl;
    of << display_ymin << std::endl;
    of.close();
}

void calibrate_window(){
    string option;
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

/**
 * Console menu for scenario creation
 */
void menuCreateScenario() {

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
                    usleep(1000000);
                }
                finished = false;
            } else {
                insertRandom = false;
                passives = 1;
                insertActive = option == "1" || option == "4" ? true : false;
                insertRandom = option == "4" ? true : false;
                start = true;
                while (!finished) {
                    usleep(1000000);
                }
                finished = false;
            }
        }
    } while (option != "q");
    quit = true;
    std::cout << "Exiting menu..." << std::endl;
}

/**
 * Translate coordinates with calibrated values
 */
void translateCoordinates(int *x_in, int *y_in, double *x_out, double *y_out) {
    *x_out = lim_xmin + ((float) (*x_in-display_xmin)/((display_xmax-display_xmin)
            /(lim_xmax-lim_xmin)));
    *y_out = lim_ymin + ((float) ((2750-*y_in)-(2750-display_ymin)))/((display_ymin-display_ymax)
            /(lim_ymax-lim_ymin));
}


void PlotWorldEnvironmentStatic() {

    //dibujo inicial del entorno
    SeleccionaVentana(1);
    LimpiaVentana();
    errmod("all", "off");
    DibujaEjes(lim_xmin, lim_xmax, lim_xmin, 2, lim_ymin, lim_ymax, lim_ymin, 2);

    for (int i=0; i<(int)agents.size(); i++){
        //std::cout << "Plotting agent " << i << std::endl;
        Tsc loc = agents[i]->GetLocalization();
        //if consider...
        if (agents[i]->isActive()) DibujaObsInicial(loc.x, loc.y, agents[i]->GetRealRadius(), 0.1, "blue");
        else DibujaObsInicial(loc.x, loc.y, agents[i]->GetRealRadius(), 0.1, "green");
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
        Tsc goal;
        if (agents[i]->GetCurrentGoal(goal)) {
            std::string eti = "g" + std::to_string(agents[i]->GetId());
            DibujaGoal(goal, "red", eti.c_str());
        }
    }

    TerminaEjes();  //necesario para poder dibujar en las otras ventanas los ejes;
                //ponerlo solo cuando vaya a llamar otra vez a graf porque si no dara error

}

/**
 * Scenario creator
 */
void createScenario(bool graph) {

    // Variable declarations
    double v0, w0, a_v = 0.15, a_w = 1.0477, radioAgent = 0.3, dist_minGoal = 4,
           titamin_ini = -3.1416, titamax_ini = 3.1416, xObs,
           yObs, xGoal, yGoal, titaObs, min_dist = 0.3;
    int numInserted = 0;
    double vMax = 0.7, wMax = 3.14;

    std::string name, thetaInit, thetaDefault, vInit, vDefault, wInit, wDefault;

    // Variable initialization
    thetaDefault = "0.0";
    vDefault = "0.20";
    wDefault = "0.0";

    double xmin = -10; double xmax = -xmin; double ymin = -10; double ymax = -ymin;
    double titamin, titamax;
    lim_xmin = xmin; lim_xmax = xmax; lim_ymin = ymin; lim_ymax = ymax;
    if (graph) {
        Inicializar();
        SeleccionaVentana(1);
        LimpiaVentana();
        errmod("all", "off");
        DibujaEjes(lim_xmin, lim_xmax, lim_xmin, 2, lim_ymin, lim_ymax, lim_ymin, 2);
        TerminaEjes();  //necesario para poder dibujar en las otras ventanas los ejes;
                        //ponerlo solo cuando vaya a llamar otra vez a graf porque si no dara error
    }

    int *x_in = new int(), *y_in = new int();
    int i = 0;
    while (!quit) {
        while (!start) {
            if (quit) break;
            usleep(250000);
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

        titamin = titamin_ini; 
        titamax = titamax_ini;

        if (yObs > ymax/3) {
            titamax =  -0.7;
        } else if (yObs < 0){
            titamin = 0.7;
            titamax = 1.57 + 0.7;
        }

        titamin = titamin_ini; titamax = titamax_ini;
        //xGoal , yGoal;
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
                //andrew: added more safety
                if (CollisionObs((*it)->GetLocalization(), Tsc(xObs,yObs,0), 
                    (radioAgent)*4* config::safety_factor)) {
                    break;
                }
                Tsc goal;
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
            std::uniform_real_distribution<double>::param_type parV(vMax*0.20, vMax*1.200);
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
                std::uniform_real_distribution<double>::param_type parW(-wMax*0.5, wMax*0.5);
                distributionW.param(parW);
                w0 = distributionW(generator);
            } else {
                w0 = 0;
            }
        } else {
            w0 = ::atof(wInit.c_str());
        }

        if (insertActive) {
            std::unique_ptr<Agent> ag 
                    {new Agent(i, xObs, yObs, titaObs, v0, w0, a_v, a_w, radioAgent, true)};
            ag->AddGoal(xGoal, yGoal);
            //if (logSim){ ag->SetNameLog(name_fich); ag->UpdateLog(); }
            //ag->SetBoundsVS(time_step);
            agents.push_back(std::move(ag));
        } else {
            std::unique_ptr<Agent> pag 
                    {new Agent(i, xObs, yObs, titaObs, v0, w0, 0, 0, radioAgent, false)};
            //pag->SetBoundsVS(time_step);
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


void usage(int argc, char* argv[], string& scenario){
    if(argc != 2){
        cerr << "Usage: ./create_scenario.sh scenario" << endl;
        exit(1);
    }
    else{
        scenario = argv[1];
    }
}

int main(int argc, char *argv[]){
    string scenario;
    usage(argc, argv, scenario);
    calibrate_window();
    std::thread menu(menuCreateScenario);
    std::thread scenarioCreator(createScenario, true);
    menu.join();
    scenarioCreator.join();
    writeScenarioJSON(scenario);
}