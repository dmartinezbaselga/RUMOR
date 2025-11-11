#include <iostream>
#include <fstream>
#include <random>
#include <boost/algorithm/string.hpp>
#include <vector>
//#include <boost/filesystem.hpp>

using namespace std;

std::random_device rd{};
std::mt19937 generator{rd()};
static std::uniform_real_distribution<double> distributionX;
static std::uniform_real_distribution<double> distributionY;
static std::uniform_real_distribution<double> distributionTita;
static std::uniform_real_distribution<double> distributionV;
static std::uniform_real_distribution<double> distributionW;
static std::uniform_real_distribution<double> distributionFixed;

// const double xmin = -4; 
// const double xmax = 4; 
// const double ymin = -4; 
// const double ymax = 4;
double xmin = -4; 
double xmax = 4; 
double ymin = -4; 
double ymax = 4;
double vMax = 0.7, wMax = 3.14;
double av = 0.3, aw = av/(vMax/wMax);

const int N_COLORS = 8;
string colors[] = {"red", "green", "blue", "white", "yellow", "grey", "orange", "brown"};
int iteracion;

string targets_world = "";

class Tsc{
    public:
        double x;
        double y;
        Tsc(){}
        Tsc (double xa, double ya){
            x = xa;
            y = ya;
        }
};

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

void getPosition (double& x, double& y, double& theta, vector<Tsc>& agents){
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
            if (CollisionObs(*it, Tsc(x, y), (0.2)*3* 1.1)  || abs(abs(x) - 2) < 0.5 || abs(abs(y) - 2) < 0.5) {
            done = false;
            }
        }
    }
    agents.push_back(Tsc(x, y));
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
        if ((x - xg)*(x - xg) + (y - yg)*(y - yg) < dist_minGoal*dist_minGoal||
                abs(abs(xg) - 2) < 0.5 || abs(abs(yg) - 2) < 0.5) {
            done = false;
        }
    }
}

bool isNumber(char c) {
    return !((c >= 48 && c <= 57) || c == '-' || c == '.');
}

void writeAMCL(ofstream& f, double x, double y, double theta, int id, bool two_lasers){
    f << "<node name=\"amcl\" pkg=\"amcl\" type=\"amcl\" output=\"screen\">" << endl;

    if (two_lasers){
	    f << "<remap from=\"scan\" to=\"ranger_0/base_scan\"/>" << endl;	   
    }
    else{
        f << "<remap from=\"scan\" to=\"ranger_0/base_scan\"/>" << endl;
    }
    f << "<param name=\"use_map_topic\"             value=\"true\"/>" << endl;
	f << "<param name=\"odom_frame_id\" value=\"/robot_" << id << "/odom\" />" << endl;
	f << "<param name=\"base_frame_id\" value=\"/robot_" << id << "/base_link\" />" << endl;
    f << "<param name=\"global_frame_id\" value=\"/map\" />" << endl;
    f << "<remap from=\"static_map\" to=\"/static_map\"/>" << endl;
    f << "<remap from=\"map\" to=\"/map\"/>" << endl;
	f << "<!-- Initial position -->" << endl;
	f << "<param name=\"initial_pose_x\" value=\"" << x << "\" />" << endl;
    f << "<param name=\"initial_pose_y\" value=\"" << y << "\" />" << endl;
    f << "<param name=\"initial_pose_a\" value=\"" << theta << "\" />" << endl;
	
	f << "<!-- amcl specific -->" << endl;
	f << "<param name=\"odom_model_type\"           value=\"diff\"/>" << endl;
	f << "<param name=\"gui_publish_rate\"          value=\"10.0\"/>" << endl;
	f << "<param name=\"laser_max_beams\"           value=\"60\"/>" << endl;
	f << "<param name=\"laser_max_range\"           value=\"12.0\"/>" << endl;
	f << "<param name=\"min_particles\"             value=\"500\"/>" << endl;
	f << "<param name=\"max_particles\"             value=\"2000\"/>" << endl;
	f << "<param name=\"kld_err\"                   value=\"0.05\"/>" << endl;
	f << "<param name=\"kld_z\"                     value=\"0.99\"/>" << endl;
	f << "<param name=\"odom_alpha1\"               value=\"0.2\"/>" << endl;
	f << "<param name=\"odom_alpha2\"               value=\"0.2\"/>" << endl;
	f << "<!-- translation std dev, m -->" << endl;
	f << "<param name=\"odom_alpha3\"               value=\"0.2\"/>" << endl;
	f << "<param name=\"odom_alpha4\"               value=\"0.2\"/>" << endl;
    f << "<param name=\"odom_alpha5\"               value=\"0.1\"/>" << endl;
    f << "<param name=\"laser_z_hit\"               value=\"0.5\"/>" << endl;
	f << "<param name=\"laser_z_short\"             value=\"0.05\"/>" << endl;
	f << "<param name=\"laser_z_max\"               value=\"0.05\"/>" << endl;
	f << "<param name=\"laser_z_rand\"              value=\"0.5\"/>" << endl;
	f << "<param name=\"laser_sigma_hit\"           value=\"0.2\"/>" << endl;
	f << "<param name=\"laser_lambda_short\"        value=\"0.1\"/>" << endl;
	f << "<param name=\"laser_model_type\"          value=\"likelihood_field\"/>" << endl;
	f << "<param name=\"laser_likelihood_max_dist\" value=\"2.0\"/>" << endl;
	f << "<param name=\"update_min_d\"              value=\"0.1\"/>" << endl;
	f << "<param name=\"update_min_a\"              value=\"0.2\"/>" << endl; 
	f << "<param name=\"resample_interval\"         value=\"1\"/>" << endl; 
	f << "<param name=\"transform_tolerance\"       value=\"1.0\"/>" << endl;
	f << "<param name=\"recovery_alpha_slow\"       value=\"0.0001\"/>" << endl;
	f << "<param name=\"recovery_alpha_fast\"       value=\"0.1\"/>" << endl;
    f << "</node>" << endl;

}

void writeObstacleTracker(ofstream& f, int id, bool two_lasers){
    if (two_lasers){
        f << "<node name=\"scans_merger\" pkg=\"obstacle_detector\" type=\"scans_merger_node\" output=\"screen\">" << endl;
        f << "<param name=\"active\"            value=\"true\"/>" << endl;
        f << "<param name=\"publish_scan\"      value=\"true\"/>" << endl;
        f << "<param name=\"publish_pcl\"       value=\"false\"/>" << endl;

        f << "<param name=\"ranges_num\"        value=\"1000\"/>" << endl;

        f << "<param name=\"min_scanner_range\" value=\"0.03\"/>" << endl;
        f << "<param name=\"max_scanner_range\" value=\"30.0\"/>" << endl;

        f << "<param name=\"min_x_range\"       value=\"-30.0\"/>" << endl;
        f << "<param name=\"max_x_range\"       value=\"30.0\"/>" << endl;
        f << "<param name=\"min_y_range\"       value=\"-30.0\"/>" << endl;
        f << "<param name=\"max_y_range\"       value=\"30.0\"/>" << endl;

        f << "<param name=\"fixed_frame_id\"   value=\"/map\"/>" << endl;
        f << "<param name=\"target_frame_id\"   value=\"robot_" << id << "/odom\"/>" << endl;

        f << "<remap from=\"front_scan\" to=\"ranger_0/base_scan\"/>" << endl;
        f << "<remap from=\"rear_scan\" to=\"ranger_1/base_scan\"/>" << endl;
        f << "</node>" << endl;
    }
    f << "<node name=\"obstacle_extractor\" pkg=\"obstacle_detector\" type=\"obstacle_extractor_node\">" << endl;
    if (!two_lasers){
        f << "<remap from=\"scan\" to=\"ranger_0/base_scan\"/>" << endl;
    }
    f << "<param name=\"active\"               value=\"true\"/>" << endl;
    f << "<param name=\"use_scan\"             value=\"true\"/>" << endl;
    f << "<param name=\"use_pcl\"              value=\"false\"/>" << endl;

    f << "<param name=\"use_split_and_merge\"    value=\"false\"/>" << endl;
    f << "<param name=\"circles_from_visibles\"  value=\"true\"/>" << endl;
    f << "<param name=\"discard_converted_segments\" value=\"true\"/>" << endl;
    f << "<param name=\"transform_coordinates\"  value=\"true\"/>" << endl;

    f << "<param name=\"min_group_points\"     value=\"5\"/>" << endl;

    f << "<param name=\"max_group_distance\"   value=\"0.1\"/>" << endl;
    f << "<param name=\"distance_proportion\"  value=\"0.00628\"/>" << endl;
    f << "<param name=\"max_split_distance\"   value=\"0.2\"/>" << endl;
    f << "<param name=\"max_merge_separation\" value=\"0.2\"/>" << endl;
    f << "<param name=\"max_merge_spread\"     value=\"0.2\"/>" << endl;
    f << "<param name=\"max_circle_radius\"    value=\"0.6\"/>" << endl;
    f << "<param name=\"radius_enlargement\"   value=\"0.3\"/>" << endl;

    f << "<param name=\"frame_id\"             value=\"/map\"/>" << endl;
    f << "</node>" << endl;

    f << "<node name=\"obstacle_tracker\" pkg=\"obstacle_detector\" type=\"obstacle_tracker_node\" output=\"screen\">" << endl;
    f << "<param name=\"active\"                  value=\"true\"/>" << endl;

    f << "<param name=\"loop_rate\"               value=\"25.0\"/>" << endl;
    f << "<param name=\"tracking_duration\"       value=\"2.0\"/>" << endl;
    f << "<param name=\"min_correspondence_cost\" value=\"0.6\"/>" << endl;
    f << "<param name=\"std_correspondence_dev\"  value=\"0.15\"/>" << endl;
    f << "<param name=\"process_variance\"        value=\"0.1\"/>" << endl;  
    f << "<param name=\"process_rate_variance\"   value=\"0.1\"/>" << endl;  
    f << "<param name=\"measurement_variance\"    value=\"1.0\"/>" << endl; 

    f << "<param name=\"frame_id\"                value=\"/map\"/>" << endl;
    f << "<param name=\"algorithm\"               value=\"1\"/>" << endl;

    f << "<remap from=\"tracked_obstacles\" to=\"obstacles\"/>" << endl;
    f << "</node>" << endl;

}

void insertAgent (int i, double vx, double vth, double x, double y, double theta, 
ofstream& f_launch_out, ofstream& f_world_out, bool track_metric, bool record_steps, bool collaborative, bool orca_agents, double xg = 0, double yg = 0, bool active = false,
double av = av, double aw = aw, bool learning = false, bool deep_q_learning = false, bool readQTable = false, bool test = false,
double alpha = 0, double gamma = 0, double epsilon = 0, string fileQTable = "", bool graph = false, bool two_lasers = false, 
bool no_training = false, int nActives = -1, int nPasives = -1, int n_episodes = 1, double epsilon_discount = 0, bool visualize = false,
bool random_scenario = true, bool move_base = false, bool navrep = false, bool crowdnav = false, bool curricular_obstacles = false, 
bool static_obstacles = false, bool random_n_obstacles = false, bool cadrl = false, string crowdnav_policy = "", bool curricular_goal = false, 
bool global_scenario = false, bool rllib = false, string rllib_policy = "", bool use_crowdnav_actions = false, 
bool lidar_observation = false, bool sac = false){
    if (active || !orca_agents){
        f_launch_out << "<group ns=\"robot_" << i << "\">" << endl;
        if (active && i == 0 && move_base){
            f_launch_out << "<include file = \"$(find rl_dovs)/launch/move_base.launch\"/>" << endl;
        }
        string type = active? "active_agent_multi" : "pasive_agent";
        if (active && i == 0 && deep_q_learning){
            type = "active_agent_dqn";
        }
        else if (active && i == 0 && move_base){
            type = "active_agent_move_base";
        }
        f_launch_out << "<node name=\"robot_" << i
        << "\" pkg=\"rl_dovs\" type=\"" << type << "\" output=\"screen\" clear_params=\"true\">" << endl;
        // f_launch_out << "<remap from=\"cmd_vel\" to=\"/robot_" << i << "/cmd_vel\" />" << endl;
        // f_launch_out << "<remap from=\"/base_pose_ground_truth\" to=\"/robot_" << i << "/base_pose_ground_truth\" />" << endl;
        // f_launch_out << "<remap from=\"/near_robot_positions\" to=\"/robot_" << i << "/near_robot_positions\" />" << endl;
        // f_launch_out << "<remap from=\"/shutdown_agent\" to=\"/robot_" << i << "/shutdown_agent\" />" << endl;
        f_launch_out << "<param name = \"v_x\" value = \"" << vx << "\" />" << endl;
        f_launch_out << "<param name = \"v_th\" value = \"" << vth << "\" />" << endl;
        string boolean = track_metric && i == 0? "true" : "false";
        f_launch_out << "<param name = \"track_metric\" value = \"" << boolean << "\" />" << endl;
        boolean = collaborative ? "true" : "false";
        f_launch_out << "<param name = \"collaborative\" value = \"" << boolean << "\" />" << endl;
        boolean = record_steps? "true" : "false";
        f_launch_out << "<param name = \"record_steps\" value = \"" << boolean << "\" />" << endl;
        f_launch_out << "<param name = \"id\" value = \"" << i << "\" />" << endl;
        f_launch_out << "<param name = \"x\" value = \"" << x << "\" />" << endl;
        f_launch_out << "<param name = \"y\" value = \"" << y << "\" />" << endl;
        if (active){
            if (iteracion == 0 && learning && i == 0){
                if (readQTable){
                    cout << "COPY FILE" << endl;
                    std::ifstream srce("../data/" + fileQTable, std::ios::binary ) ;
                    std::ofstream dest( "../data/outQ_" + to_string(i) + ".txt", std::ios::binary ) ;
                    dest << srce.rdbuf() ;
                }
                else{
                    std::ofstream dest( "../data/outQ_" + to_string(i) + ".txt");
                    dest << endl;
                }
            }
            if (i == 0 && graph){
                f_launch_out << "<param name = \"graph\" value = \"true\" />" << endl;
            }
            f_launch_out << "<param name = \"theta\" value = \"" << theta << "\" />" << endl;
            f_launch_out << "<param name = \"x_goal\" value = \"" << xg << "\" />" << endl;
            f_launch_out << "<param name = \"y_goal\" value = \"" << yg << "\" />" << endl;
            f_launch_out << "<param name = \"av\" value = \"" << av << "\" />" << endl;
            f_launch_out << "<param name = \"aw\" value = \"" << aw << "\" />" << endl;
            boolean = learning && i == 0? "true" : "false";
            f_launch_out << "<param name = \"learning\" value = \"" << boolean << "\" />" << endl;
            boolean = use_crowdnav_actions ? "true" : "false";
            f_launch_out << "<param name = \"use_crowdnav_actions\" value = \"" << boolean << "\" />" << endl;
            boolean = orca_agents && i == 0? "true" : "false";
            f_launch_out << "<param name = \"orca_agents\" value = \"" << boolean << "\" />" << endl;
            boolean = global_scenario? "true" : "false";
            f_launch_out << "<param name = \"global_scenario\" value = \"" << boolean << "\" />" << endl;
            boolean = lidar_observation? "true" : "false";
            f_launch_out << "<param name = \"lidar_observation\" value = \"" << boolean << "\" />" << endl;
            boolean = sac? "true" : "false";
            f_launch_out << "<param name = \"sac\" value = \"" << boolean << "\" />" << endl;
            boolean = visualize && i == 0? "true" : "false";
            f_launch_out << "<param name = \"visualize\" value = \"" << boolean << "\" />" << endl;
            boolean = no_training? "true" : "false";
            f_launch_out << "<param name = \"no_training\" value = \"" << boolean << "\" />" << endl;
            boolean = deep_q_learning && i == 0? "true" : "false";
            f_launch_out << "<param name = \"deep_q_learning\" value = \"" << boolean << "\" />" << endl;
            boolean = curricular_obstacles && i == 0? "true" : "false";
            f_launch_out << "<param name = \"curricular_obstacles\" value = \"" << boolean << "\" />" << endl;
            boolean = curricular_goal && i == 0? "true" : "false";
            f_launch_out << "<param name = \"curricular_goal\" value = \"" << boolean << "\" />" << endl;
            boolean = static_obstacles && i == 0? "true" : "false";
            f_launch_out << "<param name = \"static_obstacles\" value = \"" << boolean << "\" />" << endl;  
            boolean = random_n_obstacles && i == 0? "true" : "false";
            f_launch_out << "<param name = \"random_n_obstacles\" value = \"" << boolean << "\" />" << endl;
            boolean = navrep && i == 0? "true" : "false";
            f_launch_out << "<param name = \"lidar_state\" value = \"" << boolean << "\" />" << endl;
            boolean = crowdnav? "true" : "false";
            f_launch_out << "<param name = \"crowdnav\" value = \"" << boolean << "\" />" << endl;
            f_launch_out << "<param name = \"crowdnav_policy\" value = \"" << crowdnav_policy << "\" />" << endl;
            boolean = rllib? "true" : "false";
            f_launch_out << "<param name = \"rllib\" value = \"" << boolean << "\" />" << endl;
            f_launch_out << "<param name = \"rllib_policy\" value = \"" << rllib_policy << "\" />" << endl;
            // boolean = readQTable && i == 0 ? "true" : "false";        
            boolean = learning && i == 0 ? "true" : "false";
            f_launch_out << "<param name = \"readQTable\" value = \"" << boolean << "\" />" << endl;
            boolean = two_lasers && i == 0 ? "true" : "false";
            f_launch_out << "<param name = \"two_lasers\" value = \"" << boolean << "\" />" << endl;
            boolean = test? "true" : "false";
            f_launch_out << "<param name = \"test\" value = \"" << boolean << "\" />" << endl;
            boolean = cadrl? "true" : "false";
            f_launch_out << "<param name = \"cadrl\" value = \"" << boolean << "\" />" << endl;
            f_launch_out << "<param name = \"alpha\" value = \"" << alpha << "\" />" << endl;
            f_launch_out << "<param name = \"gamma\" value = \"" << gamma << "\" />" << endl;
            f_launch_out << "<param name = \"epsilon\" value = \"" << epsilon << "\" />" << endl;
            f_launch_out << "<param name = \"epsilon_discount\" value = \"" << epsilon_discount << "\" />" << endl;    
            if (deep_q_learning){
                f_launch_out << "<param name = \"fileQTable\" value = \"" << fileQTable << "\" />" << endl;
            }    
            else{
                f_launch_out << "<param name = \"fileQTable\" value = \"outQ_" << i << ".txt\" />" << endl;
            }
            f_launch_out << "<param name = \"q_table_name\" value = \"" << fileQTable << "\" />" << endl;
            f_launch_out << "<param name = \"nActives\" value = \"" << nActives << "\" />" << endl; 
            f_launch_out << "<param name = \"nPasives\" value = \"" << nPasives << "\" />" << endl; 
            f_launch_out << "<param name = \"n_episodes\" value = \"" << n_episodes << "\" />" << endl; 
            boolean = random_scenario ? "true" : "false";
            f_launch_out << "<param name = \"random_scenario\" value = \"" << boolean << "\" />" << endl;
        }
        f_launch_out << "</node>" << endl;
    }
    if (active){
        string color = colors[i%N_COLORS];
        if (two_lasers){
            f_world_out << "erratic2( pose [ "; 
        }
        else{
            f_world_out << "erratic( pose [ "; 
        }
        f_world_out << x << " " << y << " 0.000 "<< theta*180/M_PI << " ] name \"robot_" << i <<"\" color \"" << color << "\")" << endl;
        writeAMCL(f_launch_out, x, y, theta, i, two_lasers);
        if (!collaborative){
            writeObstacleTracker(f_launch_out, i, two_lasers);
        }
        targets_world +=  "target( pose [ " + to_string(xg) + " " + to_string(yg) + " 0.000 0.000 ] name \"target_" +
        to_string(i) +"\" color \"" + color + "\")\n";
    }
    else if (orca_agents){
        string color = "black";
        f_world_out << "erratic_passive( pose [ " << x << " " << y << " 0.000 "<< theta*180/M_PI << " ] name \"orca_" <<
        i <<"\" color \"" << color << "\")" << endl;        
    }
    else{
        string color = "black";
        f_world_out << "erratic_passive( pose [ " << x << " " << y << " 0.000 "<< theta*180/M_PI << " ] name \"robot_" <<
        i <<"\" color \"" << color << "\")" << endl;
    }
    if (active || !orca_agents){
        f_launch_out << "</group>" << endl;
    }
}

/**
 * Read scenario from "name".json
 */
int readScenarioJSON(std::string name, ofstream& f_launch_out, ofstream& f_world_out, string& index_actives,
bool learning, bool deep_q_learning, bool readQTable, bool test, double alpha, double gamma, double epsilon, string fileQTable, bool graph, bool track_metric,
bool record_steps, bool two_lasers, bool no_training, int n_episodes, double epsilon_discount, bool visualize, bool move_base,
bool collaborative, bool navrep, bool crowdnav, bool curricular_obstacles, bool static_obstacles, bool random_n_obstacles, bool cadrl, string crowdnav_policy, 
bool curricular_goal, bool global_scenario, bool rllib, string rllib_policy, bool orca_agents, bool use_crowdnav_actions,
bool lidar_observation, bool sac) {
    std::string line;
    std::ifstream file("./escenarios/" + name + ".json");
    int nAgents = 0, nActives = 0;
    std::vector<bool> active_vec;
    std::vector<double> radio_vec, x_vec, y_vec, theta_vec, vx_vec, vth_vec, av_vec, aw_vec, x_goal_vec, y_goal_vec;
    if (file.is_open()) {
        std::cout << "Reading scenario " << name << " from file " << name << ".json" << std::endl;
        if (file.peek() != EOF) file >> line;
        while (file.peek() != EOF) {
            file >> line;
            boost::trim(line);
            if (line == "\"agents\":") {
                bool active;
                double radio, x, y, theta, vx, vth, av, aw, x_goal, y_goal;
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
                        vx = ::atof(line.c_str());
                        file >> line >> line;
                        boost::trim_if(line, &isNumber);
                        vth = ::atof(line.c_str());
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
                            // insertAgent(i, vx, vth, x, y, theta, f_launch_out, f_world_out, track_metric, record_steps, x_goal, y_goal, true, av, aw,
                            // learning, deep_q_learning, readQTable, test, alpha, gamma, epsilon, fileQTable, graph, two_lasers, no_training, -1, -1, 
                            // n_episodes, epsilon_discount, visualize, false);
                            active_vec.push_back(true);
                            radio_vec.push_back(radio);
                            x_vec.push_back(x);
                            y_vec.push_back(y);
                            theta_vec.push_back(theta);
                            vx_vec.push_back(vx);
                            vth_vec.push_back(vth);
                            av_vec.push_back(av);
                            aw_vec.push_back(aw);
                            x_goal_vec.push_back(x_goal);
                            y_goal_vec.push_back(y_goal);
                            index_actives += to_string(i) + " ";
                            file >> line;
                            nActives++;
                        } else {
                            cout << radio << active << endl;
                            active_vec.push_back(false);
                            radio_vec.push_back(radio);
                            x_vec.push_back(x);
                            y_vec.push_back(y);
                            theta_vec.push_back(theta);
                            vx_vec.push_back(vx);
                            vth_vec.push_back(vth);
                            av_vec.push_back(av);
                            aw_vec.push_back(aw);
                            x_goal_vec.push_back(x_goal);
                            y_goal_vec.push_back(y_goal);
                            // insertAgent(i, vx, vth, x, y, theta, f_launch_out, f_world_out, track_metric, record_steps);
                        }
                        file >> line;
                        i++;
                        nAgents++;
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
        for (unsigned int i_agents = 0; i_agents < active_vec.size(); i_agents++){
            if (active_vec[i_agents]){
                insertAgent(i_agents, vx_vec[i_agents], vth_vec[i_agents], x_vec[i_agents], y_vec[i_agents], theta_vec[i_agents], f_launch_out, f_world_out, track_metric, record_steps, 
                            collaborative, orca_agents, x_goal_vec[i_agents], y_goal_vec[i_agents], true, av_vec[i_agents], aw_vec[i_agents],
                            learning, deep_q_learning, readQTable, test, alpha, gamma, epsilon, fileQTable, graph, two_lasers, no_training, nActives, nAgents-nActives, 
                            n_episodes, epsilon_discount, visualize, false, move_base, navrep, crowdnav, curricular_obstacles, static_obstacles, random_n_obstacles,
                            cadrl, crowdnav_policy, curricular_goal, global_scenario, rllib, rllib_policy, use_crowdnav_actions, lidar_observation, sac);
            }
            else{
                insertAgent(i_agents, vx_vec[i_agents], vth_vec[i_agents], x_vec[i_agents], y_vec[i_agents], theta_vec[i_agents], f_launch_out, f_world_out, track_metric, record_steps,
                collaborative, orca_agents);
            }
        }
        std::cout << "Done!" << std::endl << std::endl;
    } else {
        std::cerr << "File " << name << ".json could not be opened" << std::endl;
        exit(1);
    }
    return nAgents;
}

int randomScenario(int nActives, int nPasives, ofstream& f_launch_out, ofstream& f_world_out, string &index_actives, 
bool learning, bool deep_q_learning, bool readQTable, bool test, double alpha, double gamma, double epsilon, string fileQTable, bool graph, 
bool track_metric, bool record_steps, bool two_lasers, bool no_training, int n_episodes, double epsilon_discount, bool visualize, 
bool move_base, bool collaborative, bool navrep, bool crowdnav, bool curricular_obstacles, bool static_obstacles, bool random_n_obstacles, 
bool cadrl, string crowdnav_policy, bool curricular_goal, bool global_scenario, bool rllib, string rllib_policy, bool orca_agents, 
bool use_crowdnav_actions, bool lidar_observation, bool sac){
    vector<Tsc> agents_pos;
    int i;
    for (i = 0; i < nActives; i++){	
        double vx, vth, x, y, theta, xg, yg;
        getVelocities(vx, vth);
        getPosition(x, y, theta, agents_pos);
        getGoal(xg, yg, x, y);
        insertAgent(i, vx, vth, x, y, theta, f_launch_out, f_world_out, track_metric, record_steps, collaborative, orca_agents, xg, yg, true, 
        av, aw, learning, deep_q_learning, readQTable, test, alpha, gamma, epsilon, fileQTable, graph, two_lasers,
        no_training, nActives, nPasives, n_episodes, epsilon_discount, visualize, true, move_base, navrep, crowdnav, 
        curricular_obstacles, static_obstacles, random_n_obstacles, cadrl, crowdnav_policy, curricular_goal, global_scenario, rllib, rllib_policy, 
        use_crowdnav_actions, lidar_observation, sac);
        index_actives += to_string(i) + " ";
    }

    for (; i < nPasives+nActives; i++){	
        double vx, vth, x, y, theta;
        getVelocities(vx, vth);
        getPosition(x, y, theta, agents_pos);
        insertAgent(i, vx, vth, x, y, theta, f_launch_out, f_world_out, track_metric, record_steps, collaborative, orca_agents);
    }
    return nPasives + nActives;
}

void usage(int argc, char* argv[], int& nActives, int& nPasives, string& scenario, bool& learning, bool& deep_q_learning,
bool& readQTable, bool& test, double& alpha, double& gamma, double& epsilon, string& fileQTable,
double& epsilon_discount, bool& graph, bool& track_metric, bool& record_steps, bool& two_lasers, 
bool& training_world, bool& decaying_alpha, bool& no_training, bool& visualize, bool& move_base, bool& collaborative,
bool& navrep, bool& crowdnav, bool& cadrl, bool& r2d2, string& crowdnav_policy, bool& curricular_obstacles,
bool& restore_epsilon, bool& static_obstacles, bool& random_n_obstacles, bool& rllib, bool& new_checkpoint, bool& curricular_goal,
string& rllib_algorithm, bool& global_scenario, bool& orca_agents, bool& use_crowdnav_actions, bool& easy_map,
bool& lidar_observation, bool& sac)
{
    int i = 3;
    bool error = i >= argc;
    while (!error && i < argc){
        if(string(argv[i]) == "-r" && argc > i+2){
            i++;
            srand (time(NULL));
            if (string(argv[i])== "r"){
                nPasives = rand() % 20 + 1;
            }
            else{
                nPasives = atoi(argv[i]);
            }
            i++;
            if (string(argv[i]) =="r"){
                nActives = rand() % 20 + 1;
            }
            else{
                nActives = atoi(argv[i]);
            }
            printf("actives: %d, pasives: %d\n\n\n\n", nActives, nPasives);
            i++;
        }
        else if (string(argv[i]) == "-s" && argc > i+1){
            i++;
            nPasives = -1;
            scenario = argv[i];
            i++;
        }
        else if (string(argv[i]) == "-orca_agents"){
            orca_agents = true;
            i++;
        }
        else if (string(argv[i]) == "-use_crowdnav_actions"){
            use_crowdnav_actions = true;
            i++;
        }
        else if (string(argv[i]) == "-l"){
            learning = true;
            i++;
        }
        else if (string(argv[i]) == "-global_scenario"){
            global_scenario = true;
            i++;
        }
        else if (string(argv[i]) == "-new_checkpoint"){
            new_checkpoint = true;
            i++;
        }
        else if (string(argv[i]) == "-lidar_observation"){
            lidar_observation = true;
            i++;
        }
        else if (string(argv[i]) == "-sac"){
            sac = true;
            i++;
        }
        else if (string(argv[i]) == "-dqn"){
            deep_q_learning = true;
            i++;
        }
        else if (string(argv[i]) == "-graph"){
            graph = true;
            i++;
        }
        else if (string(argv[i]) == "-easy_map"){
            easy_map = true;
            i++;
        }
        else if (string(argv[i]) == "-two_lasers"){
            two_lasers = true;
            i++;
        }
        else if (string(argv[i]) == "-track_metric"){
            track_metric = true;
            i++;
        }
        else if (string(argv[i]) == "-record_steps"){
            record_steps = true;
            i++;
        }
        else if (string(argv[i]) == "-curricular_obstacles"){
            curricular_obstacles = true;
            i++;
        }
        else if (string(argv[i]) == "-curricular_goal"){
            curricular_goal = true;
            i++;
        }
        else if (string(argv[i]) == "-restore_epsilon"){
            restore_epsilon = true;
            i++;
        }
        else if (string(argv[i]) == "-static_obstacles"){
            static_obstacles = true;
            i++;
        }
        else if (string(argv[i]) == "-random_n_obstacles"){
            random_n_obstacles = true;
            i++;
        }
        else if (string(argv[i]) == "-training_world"){
            training_world = true;
            i++;
        }
        else if (string(argv[i]) == "-decaying_alpha"){
            decaying_alpha = true;
            i++;
        }
        else if (string(argv[i]) == "-visualize"){
            i++;
            visualize = true;
        }
        else if (string(argv[i]) == "-qTable" && argc > i+1){
            i++;
            fileQTable = string(argv[i]);
            readQTable = true;
            i++;
        }
        else if (string(argv[i]) == "-dqn_weights" && argc > i+1){
            i++;
            fileQTable = string(argv[i]);
            readQTable = true;
            i++;
        }
        else if (string(argv[i]) == "-t"){
            test = true;
            i++;
        }
        else if (string(argv[i]) == "-collaborative"){
            collaborative = true;
            i++;
        }
        else if (string(argv[i]) == "-no_training"){
            no_training = true;
            i++;
        }          
        else if (string(argv[i]) == "-navrep"){
            navrep = true;
            deep_q_learning = true;
            i++;
        }                 
        else if (string(argv[i]) == "-r2d2"){
            r2d2 = true;
            deep_q_learning = true;
            i++;
        }           
        else if (string(argv[i]) == "-rllib" && argc > i+1){
            i++;
            rllib = true;
            deep_q_learning = true;
            rllib_algorithm = string(argv[i]);
            i++;
        }           
        else if (string(argv[i]) == "-crowdnav" && argc > i+1){
            i++;
            crowdnav = true;
            deep_q_learning = true;
            crowdnav_policy = string(argv[i]);
            i++;
        }     
        else if (string(argv[i]) == "-cadrl"){
            cadrl = true;
            crowdnav = true;
            deep_q_learning = true;
            i++;
        }     
        else if (string(argv[i]) == "-move_base"){
            move_base = true;
            i++;
        }
        else if (string(argv[i]) == "-a" && argc > i+1){
            i++;
            alpha = atof(argv[i]);
            i++;
        }
        else if (string(argv[i]) == "-g" && argc > i+1){
            i++;
            gamma = atof(argv[i]);
            i++;
        }
        else if (string(argv[i]) == "-e" && argc > i+1){
            i++;
            epsilon = atof(argv[i]);
            i++;
        }
        else if (string(argv[i]) == "-d" && argc > i+1){
            i++;
            epsilon_discount = atof(argv[i]);
            i++;
        }
        else{
            error = true;
        }
    }
    if (error){
        cerr << "Usage: ./launch.sh number_chunks episodes_per_chunk [-r n_pasives n_actives | -s scenario | -l | -dqn | ";
        cerr <<  "-qTable <file> | -t | -a <alpha> | -g <gamma> | -e <epsilon> | -d <epsilon_discount> | -track_metric | ";
        cerr << "-two_lasers | -record_steps | -decaying_alpha | -dqn_weights <file> | -no_training | -visualize ]" << endl;
        exit(1);
    }
}

int main(int argc, char *argv[]){
    int nActives, nPasives, nRobots, episodes, chunks;
    bool learning = false, readQTable = false, test = false, graph = false, track_metric = false, record_steps = false, 
            two_lasers = false, training_world = false, decaying_alpha = false, deep_q_learning = false, 
            visualize = true, no_training = false, move_base = false, collaborative = false, navrep = false, crowdnav = false,
            cadrl = false, r2d2 = false, curricular_obstacles = false, restore_epsilon = false, static_obstacles = false, 
            random_n_obstacles = false, rllib = false, new_checkpoint = false, curricular_goal = false, global_scenario = false, 
            orca_agents = false, use_crowdnav_actions = false, easy_world = false, lidar_observation = false, sac = false;
    double alpha = 0.01, gamma = 0.99, epsilon = 0.1, epsilon_discount = 0.0025;
    string scenario, fileQTable = "qTable.txt", crowdnav_policy, rllib_algorithm;
    usage(argc-1, argv, nActives, nPasives, scenario, learning, deep_q_learning, readQTable, test, alpha, gamma, epsilon, 
    fileQTable, epsilon_discount, graph, track_metric, record_steps, two_lasers, training_world, decaying_alpha, no_training, 
    visualize, move_base, collaborative, navrep, crowdnav, cadrl, r2d2, crowdnav_policy, curricular_obstacles, restore_epsilon, 
    static_obstacles, random_n_obstacles, rllib, new_checkpoint, curricular_goal, rllib_algorithm, global_scenario, orca_agents,
    use_crowdnav_actions, easy_world, lidar_observation, sac);
    iteracion = atoi(argv[argc-1])-1;
    chunks = atoi(argv[1]);
    episodes = atoi(argv[2]);
    // epsilon = max(epsilon - iteracion*epsilon_discount, 0.05);
    // if (decaying_alpha){
    //     alpha = 1.0 / (alpha + iteracion);
    // }
    ifstream f_launch_in("../launch/multi_agents_template.launch");
    if (!f_launch_in.is_open()){
        cerr << "Template file not found at ../launch/multi_agents_template.launch" << endl;
        return -1;
    }
    ifstream f_world_in;
    if (navrep){
        f_world_in.open("../worlds/basic_template_navrep.world");
    } 
    else if (easy_world){
        f_world_in.open("../worlds/basic_template_easy.world");
    }
    else if (training_world){
        f_world_in.open("../worlds/basic_template_training.world");
    }
    else if (global_scenario){
        // f_world_in.open("../worlds/basic_template_global.world");
        f_world_in.open("../worlds/basic_template_global.world");
    }
    else{
        f_world_in.open("../worlds/basic_template.world");
    }
    if (!f_world_in.is_open()){
        cerr << "Template file not found at ../worlds/basic_template.world or ../worlds/basic_template_training.world" << endl;
        return -1;
    }

    ifstream f_pub_pos_in("../launch/publish_positions_template.launch");
    if (!f_world_in.is_open()){
        cerr << "Template file not found at ../launch/publish_positions_template.launch" << endl;
        return -1;
    }

    ifstream f_stage_in("../launch/stage_template.launch");


    ofstream f_launch_out("../launch/multi_agents.launch");
    ofstream f_world_out("../worlds/basic.world");
    ofstream f_pub_pos_out("../launch/publish_positions.launch");
    ofstream f_stage_out("../launch/stage.launch");

    f_launch_out << f_launch_in.rdbuf();
    f_world_out << f_world_in.rdbuf();
    f_pub_pos_out << f_pub_pos_in.rdbuf();
    f_stage_out << f_stage_in.rdbuf();
    string index_actives = "";
    int episodes_simulator = episodes;
    if (deep_q_learning && !no_training){
        episodes_simulator = 1000000;
    }
    if (global_scenario){
        nActives = max(1, int(ceil(nActives*(iteracion+1)/double(chunks))));
        nPasives = max(1, int(ceil(nPasives*(iteracion+1)/double(chunks))));
        xmin = -6;
        xmax = 6;
        ymin = -6;
        ymax = 6;
        printf("2->actives: %d, pasives: %d\n\n\n\n", nActives, nPasives);
        if (nPasives == -1){
            nRobots = readScenarioJSON(scenario, f_launch_out, f_world_out, index_actives,
            learning, deep_q_learning, readQTable, test, alpha, gamma, epsilon, fileQTable, graph, track_metric, record_steps, 
            two_lasers, no_training, episodes_simulator, epsilon_discount, visualize, move_base, collaborative, navrep, crowdnav,
            curricular_obstacles, static_obstacles, random_n_obstacles, cadrl, crowdnav_policy, curricular_goal, global_scenario,
            rllib, rllib_algorithm, orca_agents, use_crowdnav_actions, lidar_observation, sac);
        }
        else{
            nRobots = randomScenario(nActives, nPasives, f_launch_out, f_world_out, index_actives,
            learning, deep_q_learning, readQTable, test, alpha, gamma, epsilon, fileQTable, graph, track_metric, record_steps, 
            two_lasers, no_training, episodes_simulator, epsilon_discount, visualize, move_base, collaborative, navrep, crowdnav,
            curricular_obstacles, static_obstacles, random_n_obstacles, cadrl, crowdnav_policy, curricular_goal, global_scenario,
            rllib, rllib_algorithm, orca_agents, use_crowdnav_actions, lidar_observation, sac);
        }
    }
    else if (nPasives == -1){
        nRobots = readScenarioJSON(scenario, f_launch_out, f_world_out, index_actives, learning, deep_q_learning,
        readQTable, test, alpha, gamma, epsilon, fileQTable, graph, track_metric, record_steps, two_lasers, no_training, episodes_simulator, 
        epsilon_discount, visualize, move_base, collaborative, navrep, crowdnav, curricular_obstacles, static_obstacles, random_n_obstacles, 
        cadrl, crowdnav_policy, curricular_goal, global_scenario, rllib, rllib_algorithm, orca_agents, use_crowdnav_actions, lidar_observation, sac);
    }
    else{
        nActives = max(1, int(ceil(nActives*(iteracion+1)/double(chunks))));
        nPasives = max(1, int(ceil(nPasives*(iteracion+1)/double(chunks))));
        printf("2->actives: %d, pasives: %d\n\n\n\n", nActives, nPasives);
        // WHATCH OUT!! Last two arguments are only for global scenarios
        nRobots = randomScenario(nActives, nPasives, f_launch_out, f_world_out, index_actives,
        learning, deep_q_learning, readQTable, test, alpha, gamma, epsilon, fileQTable, graph, track_metric, record_steps, 
        two_lasers, no_training, episodes_simulator, epsilon_discount, visualize, move_base, collaborative, navrep, crowdnav,
        curricular_obstacles, static_obstacles, random_n_obstacles, cadrl, crowdnav_policy, curricular_goal,
        global_scenario, rllib, rllib_algorithm, orca_agents, use_crowdnav_actions, lidar_observation, sac);
    }
    f_launch_out << "</launch>" << endl;

    f_world_out << targets_world;

    f_pub_pos_out << "<param name = \"nRobots\" value = \"" << nRobots << "\" />" << endl;
    f_pub_pos_out << "<param name = \"actives\" value = \"" << index_actives << "\" />" << endl;
    f_pub_pos_out << "</node>" << endl << "</launch>" << endl;

    // if (deep_q_learning){
    //     ofstream f_deep_q_learning_server("../launch/dqn_server.launch");
    //     f_deep_q_learning_server << "<?xml version=\"1.0\"?>" << endl; 
    //     f_deep_q_learning_server << "<launch>" << endl;
    //     f_deep_q_learning_server << "<param name=\"/use_sim_time\" value=\"true\"/>" << endl;
    //     // if (no_training){
    //     if (navrep){
    //         f_deep_q_learning_server << "<node name=\"dqn_server\" pkg=\"rl_dovs\" type=\"navrep_server.py\" output=\"screen\">" << endl;
    //     }     
    //     else if (cadrl){
    //         f_deep_q_learning_server << "<node name=\"dqn_server\" pkg=\"cadrl_ros\" type=\"cadrl_node_gym.py\" output=\"screen\">" << endl;
    //     }   
    //     else if (crowdnav){
    //         if (crowdnav_policy == "-esa"){
    //             f_deep_q_learning_server << "<node name=\"dqn_server\" pkg=\"esa\" type=\"crowdnav_server.py\" output=\"screen\" " << 
    //             "args=\"--policy=esa --dynamic_num=15 --use_latest\">" << endl;
    //         }
    //         else if (crowdnav_policy == "-tsrl"){
    //             f_deep_q_learning_server << "<node name=\"dqn_server\" pkg=\"rl_dovs\" type=\"tsrl_server.py\" output=\"screen\">" << endl;
    //         }
    //         else{
    //             f_deep_q_learning_server << "<node name=\"dqn_server\" pkg=\"rl_dovs\" type=\"crowdnav_server.py\" output=\"screen\">" << endl;
    //         }
    //         f_deep_q_learning_server << "<param name=\"policy\"         value=\"" << crowdnav_policy << "\"/>" << endl;
    //     }
    //     else if (r2d2){
    //         f_deep_q_learning_server << "<node name=\"dqn_server\" pkg=\"rl_dovs\" type=\"acme_r2d2_server.py\" output=\"screen\">" << endl;
    //     }        
    //     else if (rllib){
    //         f_deep_q_learning_server << "<node name=\"dqn_server\" pkg=\"rl_dovs\" type=\"rllib_server.py\" output=\"screen\">" << endl;
    //         f_deep_q_learning_server << "<param name=\"rllib_algorithm\"         value=\"" << rllib_algorithm << "\"/>" << endl;
    //     }
    //     else{
    //         f_deep_q_learning_server << "<node name=\"dqn_server\" pkg=\"rl_dovs\" type=\"acme_rl_server.py\" output=\"screen\">" << endl;
    //     }
    //     f_deep_q_learning_server << "<param name=\"action_size\"               value=\"8\"/>" << endl;
    //     f_deep_q_learning_server << "<param name=\"state_size\"               value=\"408\"/>" << endl;
    //     f_deep_q_learning_server << "<param name=\"epsilon\"               value=\""<< epsilon <<"\"/>" << endl;
    //     f_deep_q_learning_server << "<param name=\"epsilon_discount\"               value=\""<< epsilon_discount <<"\"/>" << endl;
    //     f_deep_q_learning_server << "<param name=\"gamma\"               value=\""<< gamma <<"\"/>" << endl;
    //     f_deep_q_learning_server << "<param name=\"alpha\"               value=\""<< alpha <<"\"/>" << endl;
    //     f_deep_q_learning_server << "<param name=\"weights_file\"      value=\"" << fileQTable << "\"/>" << endl;
    //     f_deep_q_learning_server << "<param name=\"n_episodes\"         value=\"" << episodes << "\"/>" << endl;
    //     if (readQTable){
    //         f_deep_q_learning_server << "<param name=\"read_weights\"      value=\"true\"/>" << endl;
    //     }
    //     else{
    //         f_deep_q_learning_server << "<param name=\"read_weights\"      value=\"false\"/>" << endl;
    //     }
    //     string boolean = no_training? "true" : "false";
    //     f_deep_q_learning_server << "<param name = \"no_training\" value = \"" << boolean << "\" />" << endl;
    //     boolean = use_crowdnav_actions? "true" : "false";
    //     f_deep_q_learning_server << "<param name = \"use_crowdnav_actions\" value = \"" << boolean << "\" />" << endl;
    //     boolean = new_checkpoint? "true" : "false";
    //     f_deep_q_learning_server << "<param name = \"new_checkpoint\" value = \"" << boolean << "\" />" << endl;
    //     boolean = restore_epsilon? "true" : "false";
    //     f_deep_q_learning_server << "<param name = \"restore_epsilon\" value = \"" << boolean << "\" />" << endl;
    //     boolean = lidar_observation? "true" : "false";
    //     f_deep_q_learning_server << "<param name = \"lidar_observation\" value = \"" << boolean << "\" />" << endl;
    //     boolean = sac? "true" : "false";
    //     f_deep_q_learning_server << "<param name = \"sac\" value = \"" << boolean << "\" />" << endl;
    //     f_deep_q_learning_server << "</node>" << endl;
    //     f_deep_q_learning_server << "</launch>" << endl;


    // }
    string boolean = collaborative? "true" : "false";
    f_stage_out << "<param name = \"publish_info\" value = \"" << boolean << "\" />" << endl;
    f_stage_out <<  "<param name=\"orca_robots\" value=\"" << nPasives << "\"/>" << endl;
    f_stage_out <<  "</node>" << endl;
    if (easy_world){
        f_stage_out << "<node name=\"map_server\" pkg=\"map_server\" type=\"map_server\" args=\"$(find rl_dovs)/worlds/simple_easy.yaml\" required=\"true\"/>" << endl;
    }
    else{
        f_stage_out << "<node name=\"map_server\" pkg=\"map_server\" type=\"map_server\" args=\"$(find rl_dovs)/worlds/simple.yaml\" required=\"true\"/>" << endl;
    }
    // f_stage_out << "<node name=\"rviz\" pkg=\"rviz\" type=\"rviz\" args=\"-d $(find rl_dovs)/rviz/amcl_pose.rviz\"/>" << endl;
    f_stage_out << "</launch>" << endl;

    return 0;
}