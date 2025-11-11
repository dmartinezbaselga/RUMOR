#include <iostream>
#include <fstream>
#include <random>

struct Tpf{
    double x,y;
    Tpf () : x(0), y(0) {}
    Tpf ( const double& x0, const double& y0) : x (x0), y(y0) {};
};

double Distancia (double x, double y){

	return (std::sqrt(x*x+y*y));
}

bool CollisionObs(Tpf ag1, Tpf ag2, double securityDist){

    double distancia = Distancia(ag1.x - ag2.x, ag1.y - ag2.y);

    if (distancia - securityDist < 0.0)
        return true;
    else
        return false;
}

std::random_device rd{};
std::mt19937 generator{rd()};
std::uniform_real_distribution<double> distributionX;
std::uniform_real_distribution<double> distributionY;
std::uniform_real_distribution<double> distributionTita;
std::uniform_real_distribution<double> distributionV;
std::uniform_real_distribution<double> distributionW;
std::uniform_real_distribution<double> distributionFixed;
const double xmin = -4; 
const double xmax = 4; 
const double ymin = -4; 
const double ymax = 4;
double vMax = 1, wMax = 1.0472;

void getPosition (double& x, double& y, double& theta, std::vector<Tpf>& agents){
    bool done = false;

    while(!done){
        done = true;
        //x
        std::uniform_real_distribution<double>::param_type parX(xmin+1, xmax-1);
        distributionX.param(parX);
        x = distributionX(generator);
        //y
        std::uniform_real_distribution<double>::param_type parY(ymin+1, ymax-1);
        distributionY.param(parY);
        y = distributionY(generator);

        for (auto it=agents.begin(); it!=agents.end(); ++it){
            if (CollisionObs(*it, Tpf(x, y), (0.2)*3* 1.1)) {
            done = false;
            }
        }
    }
    //Theta
    double titamin_ini = -3.1416, titamax_ini = 3.1416;
    double titamin = titamin_ini; 
    double titamax = titamax_ini;

    if (y > ymax/3) {
        titamax =  -0.7;
    } else if (y < 0){
        titamin = 0.7;
        titamax = 1.57 + 0.7;
    }

    titamin = titamin_ini; titamax = titamax_ini;
    std::uniform_real_distribution<double>::param_type parTita(titamin, titamax);
    distributionTita.param(parTita);
    theta = distributionTita(generator);
}

void getGoal (double& xg, double& yg, const double x, const double y){
    bool done = false;
    double dist_minGoal = 4;
    while(!done){
        done = true;
        std::uniform_real_distribution<double>::param_type parX(xmin+1, xmax-1);
        distributionX.param(parX);
        xg = distributionX(generator);
        std::uniform_real_distribution<double>::param_type parY(ymin+1, ymax-1);
        distributionY.param(parY);
        yg = distributionY(generator);
        if ((x - xg)*(x - xg) + (y - yg)*(y - yg) < dist_minGoal*dist_minGoal) {
            done = false;
        }
    }
}

void getVelocities (double& vx, double& vth){
    //Velocity
    std::uniform_real_distribution<double>::param_type parV(vMax*0.20, vMax*1.20);
    distributionV.param(parV);
    vx = distributionV(generator);

    //Angular velocity
    std::uniform_real_distribution<double>::param_type parW(-wMax*0.5, wMax*0.5);
    distributionW.param(parW);
    vth = distributionW(generator);
}

int main()
{
    using namespace std;

    for (int n_actives = 1; n_actives <= 5; n_actives+=10){
        for (int n_pasives = 1; n_pasives <= 15; n_pasives+=1){
            for (int episodes = 1; episodes<=200; episodes++){
                ofstream ofs(to_string(n_actives) + "-" + to_string(n_pasives) + "/" + to_string(episodes) + ".dovs", ios::binary);
                std::vector<Tpf> agents_new;
                for (int idx = 0; idx < n_actives; idx++){
                    double x_new, y_new, theta_new, x_goal_new, y_goal_new;
                    getPosition(x_new, y_new, theta_new, agents_new);
                    theta_new = 0.0;
                    getGoal(x_goal_new, y_goal_new, x_new, y_new);
                    ofs.write(reinterpret_cast<char*>(&x_new), sizeof(x_new));
                    ofs.write(reinterpret_cast<char*>(&y_new), sizeof(y_new));
                    ofs.write(reinterpret_cast<char*>(&theta_new), sizeof(theta_new));
                    ofs.write(reinterpret_cast<char*>(&x_goal_new), sizeof(x_goal_new));
                    ofs.write(reinterpret_cast<char*>(&y_goal_new), sizeof(y_goal_new)); 
                    agents_new.push_back(Tpf(x_new, y_new));               
                }
                for (int idx = 0; idx < n_pasives; idx++){
                    double x_new, y_new, theta_new, v_new, w_new;
                    getPosition(x_new, y_new, theta_new, agents_new);
                    getVelocities(v_new, w_new);
                    ofs.write(reinterpret_cast<char*>(&x_new), sizeof(x_new));
                    ofs.write(reinterpret_cast<char*>(&y_new), sizeof(y_new));
                    ofs.write(reinterpret_cast<char*>(&theta_new), sizeof(theta_new));
                    ofs.write(reinterpret_cast<char*>(&v_new), sizeof(v_new));
                    ofs.write(reinterpret_cast<char*>(&w_new), sizeof(w_new));  
                    agents_new.push_back(Tpf(x_new, y_new));
                }
                ofs.close();
            }
        }
    }
    return 0;
}