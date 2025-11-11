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

const double xmin = -10; 
const double xmax = 10; 
const double ymin = -10; 
const double ymax = 10;
double vMax = 0.7, wMax = 3.14;

const int N_METHODS = 3;
const int N_COLORS = 8;
string colors[] = {"red", "green", "blue", "white", "yellow", "grey", "orange", "brown"};
int iteracion;

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
            if (CollisionObs(*it, Tsc(x, y), (0.2)*3* 1.1)) {
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
        if ((x - xg)*(x - xg) + (y - yg)*(y - yg) < dist_minGoal*dist_minGoal) {
            done = false;
        }
    }
}

bool isNumber(char c) {
    return !((c >= 48 && c <= 57) || c == '-' || c == '.');
}

void writeAMCL(shared_ptr<ofstream>& f, double x, double y, double theta, int id, bool two_lasers){
    *f << "<node name=\"amcl\" pkg=\"amcl\" type=\"amcl\" output=\"screen\">" << endl;

    if (two_lasers){
	    *f << "<remap from=\"scan\" to=\"base_scan_0\"/>" << endl;	   
    }
    else{
        *f << "<remap from=\"scan\" to=\"base_scan\"/>" << endl;
    }

	*f << "<param name=\"odom_frame_id\" value=\"/robot_" << id << "/odom\" />" << endl;
	*f << "<param name=\"base_frame_id\" value=\"/robot_" << id << "/base_link\" />" << endl;
    *f << "<param name=\"global_frame_id\" value=\"/map\" />" << endl;
    *f << "<remap from=\"static_map\" to=\"/static_map\"/>" << endl;
	*f << "<!-- Initial position -->" << endl;
	*f << "<param name=\"initial_pose_x\" value=\"" << x << "\" />" << endl;
    *f << "<param name=\"initial_pose_y\" value=\"" << y << "\" />" << endl;
    *f << "<param name=\"initial_pose_a\" value=\"" << theta << "\" />" << endl;
	
	*f << "<!-- amcl specific -->" << endl;
	*f << "<param name=\"odom_model_type\"           value=\"diff\"/>" << endl;
	*f << "<param name=\"gui_publish_rate\"          value=\"10.0\"/>" << endl;
	*f << "<param name=\"laser_max_beams\"           value=\"60\"/>" << endl;
	*f << "<param name=\"laser_max_range\"           value=\"12.0\"/>" << endl;
	*f << "<param name=\"min_particles\"             value=\"500\"/>" << endl;
	*f << "<param name=\"max_particles\"             value=\"2000\"/>" << endl;
	*f << "<param name=\"kld_err\"                   value=\"0.05\"/>" << endl;
	*f << "<param name=\"kld_z\"                     value=\"0.99\"/>" << endl;
	*f << "<param name=\"odom_alpha1\"               value=\"0.2\"/>" << endl;
	*f << "<param name=\"odom_alpha2\"               value=\"0.2\"/>" << endl;
	*f << "<!-- translation std dev, m -->" << endl;
	*f << "<param name=\"odom_alpha3\"               value=\"0.2\"/>" << endl;
	*f << "<param name=\"odom_alpha4\"               value=\"0.2\"/>" << endl;
    *f << "<param name=\"odom_alpha5\"               value=\"0.1\"/>" << endl;
    *f << "<param name=\"laser_z_hit\"               value=\"0.5\"/>" << endl;
	*f << "<param name=\"laser_z_short\"             value=\"0.05\"/>" << endl;
	*f << "<param name=\"laser_z_max\"               value=\"0.05\"/>" << endl;
	*f << "<param name=\"laser_z_rand\"              value=\"0.5\"/>" << endl;
	*f << "<param name=\"laser_sigma_hit\"           value=\"0.2\"/>" << endl;
	*f << "<param name=\"laser_lambda_short\"        value=\"0.1\"/>" << endl;
	*f << "<param name=\"laser_model_type\"          value=\"likelihood_field\"/>" << endl;
	*f << "<param name=\"laser_likelihood_max_dist\" value=\"2.0\"/>" << endl;
	*f << "<param name=\"update_min_d\"              value=\"0.1\"/>" << endl;
	*f << "<param name=\"update_min_a\"              value=\"0.2\"/>" << endl; 
	*f << "<param name=\"resample_interval\"         value=\"1\"/>" << endl; 
	*f << "<param name=\"transform_tolerance\"       value=\"1.0\"/>" << endl;
	*f << "<param name=\"recovery_alpha_slow\"       value=\"0.0001\"/>" << endl;
	*f << "<param name=\"recovery_alpha_fast\"       value=\"0.1\"/>" << endl;
    *f << "</node>" << endl;

}

void writeObstacleTracker(shared_ptr<ofstream>& f, int id, bool two_lasers){
    if (two_lasers){
        *f << "<node name=\"scans_merger\" pkg=\"rl_dovs\" type=\"scans_merger_node\" output=\"screen\">" << endl;
        *f << "<param name=\"active\"            value=\"true\"/>" << endl;
        *f << "<param name=\"publish_scan\"      value=\"true\"/>" << endl;
        *f << "<param name=\"publish_pcl\"       value=\"false\"/>" << endl;

        *f << "<param name=\"ranges_num\"        value=\"1000\"/>" << endl;

        *f << "<param name=\"min_scanner_range\" value=\"0.03\"/>" << endl;
        *f << "<param name=\"max_scanner_range\" value=\"30.0\"/>" << endl;

        *f << "<param name=\"min_x_range\"       value=\"-30.0\"/>" << endl;
        *f << "<param name=\"max_x_range\"       value=\"30.0\"/>" << endl;
        *f << "<param name=\"min_y_range\"       value=\"-30.0\"/>" << endl;
        *f << "<param name=\"max_y_range\"       value=\"30.0\"/>" << endl;

        *f << "<param name=\"fixed_frame_id\"   value=\"/map\"/>" << endl;
        *f << "<param name=\"target_frame_id\"   value=\"robot_" << id << "/odom\"/>" << endl;

        *f << "<remap from=\"front_scan\" to=\"base_scan_0\"/>" << endl;
        *f << "<remap from=\"rear_scan\" to=\"base_scan_1\"/>" << endl;
        *f << "</node>" << endl;
    }
    *f << "<node name=\"obstacle_extractor\" pkg=\"rl_dovs\" type=\"obstacle_extractor_node\">" << endl;
    if (!two_lasers){
        *f << "<remap from=\"scan\" to=\"base_scan\"/>" << endl;
    }
    *f << "<param name=\"active\"               value=\"true\"/>" << endl;
    *f << "<param name=\"use_scan\"             value=\"true\"/>" << endl;
    *f << "<param name=\"use_pcl\"              value=\"false\"/>" << endl;

    *f << "<param name=\"use_split_and_merge\"    value=\"true\"/>" << endl;
    *f << "<param name=\"circles_from_visibles\"  value=\"true\"/>" << endl;
    *f << "<param name=\"discard_converted_segments\" value=\"true\"/>" << endl;
    *f << "<param name=\"transform_coordinates\"  value=\"true\"/>" << endl;

    *f << "<param name=\"min_group_points\"     value=\"5\"/>" << endl;

    *f << "<param name=\"max_group_distance\"   value=\"0.1\"/>" << endl;
    *f << "<param name=\"distance_proportion\"  value=\"0.00628\"/>" << endl;
    *f << "<param name=\"max_split_distance\"   value=\"0.2\"/>" << endl;
    *f << "<param name=\"max_merge_separation\" value=\"0.2\"/>" << endl;
    *f << "<param name=\"max_merge_spread\"     value=\"0.2\"/>" << endl;
    *f << "<param name=\"max_circle_radius\"    value=\"0.6\"/>" << endl;
    *f << "<param name=\"radius_enlargement\"   value=\"0.3\"/>" << endl;

    *f << "<param name=\"frame_id\"             value=\"/map\"/>" << endl;
    *f << "</node>" << endl;

    *f << "<node name=\"obstacle_tracker\" pkg=\"rl_dovs\" type=\"obstacle_tracker_node\" output=\"screen\">" << endl;
    *f << "<param name=\"active\"                  value=\"true\"/>" << endl;

    *f << "<param name=\"loop_rate\"               value=\"25.0\"/>" << endl;
    *f << "<param name=\"tracking_duration\"       value=\"2.0\"/>" << endl;
    *f << "<param name=\"min_correspondence_cost\" value=\"0.6\"/>" << endl;
    *f << "<param name=\"std_correspondence_dev\"  value=\"0.15\"/>" << endl;
    *f << "<param name=\"process_variance\"        value=\"0.1\"/>" << endl;  
    *f << "<param name=\"process_rate_variance\"   value=\"0.1\"/>" << endl;  
    *f << "<param name=\"measurement_variance\"    value=\"1.0\"/>" << endl; 

    *f << "<param name=\"frame_id\"                value=\"/map\"/>" << endl;
    *f << "<param name=\"algorithm\"               value=\"1\"/>" << endl;

    *f << "<remap from=\"tracked_obstacles\" to=\"obstacles\"/>" << endl;
    *f << "</node>" << endl;

}

void insertAgent (bool write_world, int i, double vx, double vth, double x, double y, double theta, 
shared_ptr<ofstream>& f_launch_out, ofstream& f_world_out, bool track_metric, bool record_steps, double xg = 0, double yg = 0, bool active = false,
double av = 0.15, double aw = 1.0477, bool learning = false, bool deep_q_learning = false, bool readQTable = false, bool test = false,
double alpha = 0, double gamma = 0, double epsilon = 0, string fileQTable = "", bool graph = false, bool two_lasers = false, 
bool no_training = false){
    *f_launch_out << "<group ns=\"robot_" << i << "\">" << endl;
    string type = active? "active_agent_multi" : "pasive_agent";
    *f_launch_out << "<node name=\"robot_" << i
    << "\" pkg=\"rl_dovs\" type=\"" << type << "\" output=\"screen\" clear_params=\"true\">" << endl;
    // *f_launch_out << "<remap from=\"cmd_vel\" to=\"/robot_" << i << "/cmd_vel\" />" << endl;
    // *f_launch_out << "<remap from=\"/base_pose_ground_truth\" to=\"/robot_" << i << "/base_pose_ground_truth\" />" << endl;
    // *f_launch_out << "<remap from=\"/near_robot_positions\" to=\"/robot_" << i << "/near_robot_positions\" />" << endl;
    // *f_launch_out << "<remap from=\"/shutdown_agent\" to=\"/robot_" << i << "/shutdown_agent\" />" << endl;
    *f_launch_out << "<param name = \"v_x\" value = \"" << vx << "\" />" << endl;
    *f_launch_out << "<param name = \"v_th\" value = \"" << vth << "\" />" << endl;
    string boolean = track_metric && i == 0? "true" : "false";
    *f_launch_out << "<param name = \"track_metric\" value = \"" << boolean << "\" />" << endl;
    boolean = record_steps? "true" : "false";
    *f_launch_out << "<param name = \"record_steps\" value = \"" << boolean << "\" />" << endl;
    *f_launch_out << "<param name = \"id\" value = \"" << i << "\" />" << endl;
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
            *f_launch_out << "<param name = \"graph\" value = \"true\" />" << endl;
        }
        *f_launch_out << "<param name = \"x_goal\" value = \"" << xg << "\" />" << endl;
        *f_launch_out << "<param name = \"y_goal\" value = \"" << yg << "\" />" << endl;
        *f_launch_out << "<param name = \"av\" value = \"" << av << "\" />" << endl;
        *f_launch_out << "<param name = \"aw\" value = \"" << aw << "\" />" << endl;
        boolean = learning && i == 0? "true" : "false";
        *f_launch_out << "<param name = \"learning\" value = \"" << boolean << "\" />" << endl;
        boolean = no_training? "true" : "false";
        *f_launch_out << "<param name = \"no_training\" value = \"" << boolean << "\" />" << endl;
        boolean = deep_q_learning && i == 0? "true" : "false";
        *f_launch_out << "<param name = \"deep_q_learning\" value = \"" << boolean << "\" />" << endl;
        // boolean = readQTable && i == 0 ? "true" : "false";        
        boolean = learning && i == 0 ? "true" : "false";
        *f_launch_out << "<param name = \"readQTable\" value = \"" << boolean << "\" />" << endl;
        boolean = test? "true" : "false";
        *f_launch_out << "<param name = \"test\" value = \"" << boolean << "\" />" << endl;
        *f_launch_out << "<param name = \"alpha\" value = \"" << alpha << "\" />" << endl;
        *f_launch_out << "<param name = \"gamma\" value = \"" << gamma << "\" />" << endl;
        *f_launch_out << "<param name = \"epsilon\" value = \"" << epsilon << "\" />" << endl;
        
        *f_launch_out << "<param name = \"fileQTable\" value = \"outQ_" << i << ".txt\" />" << endl;
        *f_launch_out << "<param name = \"q_table_name\" value = \"" << fileQTable << "\" />" << endl;
    }
    *f_launch_out << "</node>" << endl;

    if (active){
        if (write_world){
            string color = colors[i%N_COLORS];
            if (two_lasers){
                f_world_out << "erratic2( pose [ "; 
            }
            else{
                f_world_out << "erratic( pose [ "; 
            }
            f_world_out << x << " " << y << " 0.000 "<< theta*180/M_PI << " ] name \"robot_" << i <<"\" color \"" << color << "\")" << endl;
            f_world_out << "target( pose [ " << xg << " " << yg << " 0.000 0.000 ] name \"target_" <<
            i <<"\" color \"" << color << "\")" << endl;
        }
        writeAMCL(f_launch_out, x, y, theta, i, two_lasers);
        writeObstacleTracker(f_launch_out, i, two_lasers);
    }
    else{
        if (write_world){
            string color = "black";
            f_world_out << "erratic_passive( pose [ " << x << " " << y << " 0.000 "<< theta*180/M_PI << " ] name \"robot_" <<
            i <<"\" color \"" << color << "\")" << endl;
        }
    }
    *f_launch_out << "</group>" << endl;
}

/**
 * Read scenario from "name".json
 */
int readScenarioJSON(std::string name, std::vector<std::shared_ptr<ofstream>>& f_launch_out, ofstream& f_world_out, string& index_actives,
std::vector<bool>& learning, std::vector<bool>& deep_q_learning, bool readQTable, bool test, double alpha, double gamma, double epsilon, 
string fileQTable, bool graph, bool track_metric, bool record_steps, bool two_lasers, bool no_training) {
    std::string line;
    std::ifstream file("./escenarios/" + name + ".json");
    int nAgents = 0;
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
                            for (int method = 0; method < N_METHODS; method++){
                                insertAgent(method == 0, i, vx, vth, x, y, theta, f_launch_out[method], f_world_out, track_metric, record_steps, 
                                            x_goal, y_goal, true, av, aw, learning[method], deep_q_learning[method], readQTable, test, alpha, gamma, epsilon, 
                                            fileQTable, graph, two_lasers, no_training);
                            }
                            index_actives += to_string(i) + " ";
                            file >> line;
                        } else {
                            cout << radio << active << endl;
                            for (int method = 0; method < N_METHODS; method++){
                                insertAgent(method == 0, i, vx, vth, x, y, theta, f_launch_out[method], f_world_out, track_metric, record_steps);
                            }
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
        std::cout << "Done!" << std::endl << std::endl;
    } else {
        std::cerr << "File " << name << ".json could not be opened" << std::endl;
        exit(1);
    }
    return nAgents;
}

int randomScenario(int nActives, int nPasives, std::vector<std::shared_ptr<ofstream>>& f_launch_out, ofstream& f_world_out, string &index_actives, 
std::vector<bool>& learning, std::vector<bool>& deep_q_learning, bool readQTable, bool test, double alpha, double gamma, double epsilon, string fileQTable, bool graph, 
bool track_metric, bool record_steps, bool two_lasers, bool no_training){
    vector<Tsc> agents_pos;
    int i;
    for (i = 0; i < nActives; i++){	
        double vx, vth, x, y, theta, xg, yg;
        getVelocities(vx, vth);
        getPosition(x, y, theta, agents_pos);
        getGoal(xg, yg, x, y);
        for (int method = 0; method < N_METHODS; method++){
            insertAgent(method == 0, i, vx, vth, x, y, theta, f_launch_out[method], f_world_out, track_metric, record_steps, xg, yg, true, 
            0.15, 1.0477, learning[method], deep_q_learning[method], readQTable, test, alpha, gamma, epsilon, fileQTable, graph, two_lasers,
            no_training);
        }
        index_actives += to_string(i) + " ";
    }

    for (; i < nPasives+nActives; i++){	
        double vx, vth, x, y, theta;
        getVelocities(vx, vth);
        getPosition(x, y, theta, agents_pos);
        for (int method = 0; method < N_METHODS; method++){
            insertAgent(method==0, i, vx, vth, x, y, theta, f_launch_out[method], f_world_out, track_metric, record_steps);
        }
    }
    return nPasives + nActives;
}

void usage(int argc, char* argv[], int& nActives, int& nPasives, string& scenario,
bool& readQTable, bool& test, double& alpha, double& gamma, double& epsilon, string& fileQTable, string& fileDQNweights,
double& epsilon_discount, bool& graph, bool& track_metric, bool& record_steps, bool& two_lasers, 
bool& training_world, bool& decaying_alpha, bool& no_training)
{
    int i = 2;
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
        else if (string(argv[i]) == "-graph"){
            graph = true;
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
        else if (string(argv[i]) == "-visualize"){
            i++;
        }
        else if (string(argv[i]) == "-record_steps"){
            record_steps = true;
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
        else if (string(argv[i]) == "-no_training"){
            no_training = true;
            i++;
        }
        else if (string(argv[i]) == "-qTable" && argc > i+1){
            i++;
            fileQTable = string(argv[i]);
            readQTable = true;
            i++;
        }
        else if (string(argv[i]) == "-dqn_weights" && argc > i+1){
            i++;
            fileDQNweights = string(argv[i]);
            readQTable = true;
            i++;
        }
        else if (string(argv[i]) == "-t"){
            test = true;
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
        cerr << "Usage: ./launch.sh number_episodes [-r n_pasives n_actives | -s scenario | ";
        cerr <<  "-qTable <file> | -t | -a <alpha> | -g <gamma> | -e <epsilon> | -d <epsilon_discount> | -track_metric | ";
        cerr << "-two_lasers | -record_steps | -decaying_alpha | -dqn_weights <file> | -no_training]" << endl;
        exit(1);
    }
}

int main(int argc, char *argv[]){
    int nActives, nPasives, nRobots;
    bool readQTable = false, test = false, graph = false, track_metric = false, record_steps = false, 
            two_lasers = false, training_world = false, decaying_alpha = false, no_training = false;
    double alpha = 0.01, gamma = 0.5, epsilon = 0.1, epsilon_discount = 0.0025;
    string scenario, fileQTable = "qTable.txt", fileDQNweights = "DQN_weights.npy";
    usage(argc-1, argv, nActives, nPasives, scenario, readQTable, test, alpha, gamma, epsilon, 
    fileQTable, fileDQNweights, epsilon_discount, graph, track_metric, record_steps, two_lasers, 
    training_world, decaying_alpha, no_training);
    iteracion = atoi(argv[argc-1])-1;
    epsilon = epsilon - iteracion*epsilon_discount;
    if (decaying_alpha){
        alpha = 1.0 / (alpha + iteracion);
    }
    ifstream f_launch_in("../launch/multi_agents_template.launch");
    if (!f_launch_in.is_open()){
        cerr << "Template file not found at ../launch/multi_agents_template.launch" << endl;
        return -1;
    }
    ifstream f_world_in;
    if (training_world){
        f_world_in.open("../worlds/basic_template_training.world");
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
    std::vector<std::shared_ptr<ofstream>> f_launch_out;
    f_launch_out.push_back(make_shared<ofstream>("../launch/multi_agents_rules.launch"));
    f_launch_out.push_back(make_shared<ofstream>("../launch/multi_agents_learning.launch"));
    f_launch_out.push_back(make_shared<ofstream>("../launch/multi_agents_dqn.launch"));

    for (int method = 0; method < N_METHODS; method++){
        // *f_launch_out[method] << f_launch_in.rdbuf();
        *f_launch_out[method] << "<?xml version=\"1.0\"?>" << endl; 
        *f_launch_out[method] << "<launch>" << endl;
        *f_launch_out[method] << "<param name=\"/use_sim_time\" value=\"true\"/>" << endl;
    }
    ofstream f_world_out("../worlds/basic.world");
    ofstream f_pub_pos_out("../launch/publish_positions.launch");
    f_world_out << f_world_in.rdbuf();
    f_pub_pos_out << f_pub_pos_in.rdbuf();
    string index_actives = "";
    std::vector<bool> learning = {false, true, false};
    std::vector<bool> deep_q_learning = {false, false, true};

    if (nPasives == -1){
        nRobots = readScenarioJSON(scenario, f_launch_out, f_world_out, index_actives, learning, deep_q_learning,
        readQTable, test, alpha, gamma, epsilon, fileQTable, graph, track_metric, record_steps, two_lasers, no_training);
    }
    else{
        nRobots = randomScenario(nActives, nPasives, f_launch_out, f_world_out, index_actives,
        learning, deep_q_learning, readQTable, test, alpha, gamma, epsilon, fileQTable, graph, 
        track_metric, record_steps, two_lasers, no_training);
    }
    for (int method = 0; method < N_METHODS; method++){
        *f_launch_out[method] << "</launch>" << endl;
    }
    f_pub_pos_out << "<param name = \"nRobots\" value = \"" << nRobots << "\" />" << endl;
    f_pub_pos_out << "<param name = \"actives\" value = \"" << index_actives << "\" />" << endl;
    f_pub_pos_out << "</node>" << endl << "</launch>" << endl;

    ofstream f_deep_q_learning_server("../launch/dqn_server.launch");
    f_deep_q_learning_server << "<?xml version=\"1.0\"?>" << endl; 
    f_deep_q_learning_server << "<launch>" << endl;
    f_deep_q_learning_server << "<param name=\"/use_sim_time\" value=\"true\"/>" << endl;
    f_deep_q_learning_server << "<node name=\"dqn_server\" pkg=\"rl_dovs\" type=\"dqn_server.py\" output=\"screen\">" << endl;
    f_deep_q_learning_server << "<param name=\"action_size\"               value=\"8\"/>" << endl;
    f_deep_q_learning_server << "<param name=\"state_size\"               value=\"227\"/>" << endl;
    f_deep_q_learning_server << "<param name=\"epsilon\"               value=\""<< epsilon <<"\"/>" << endl;
    f_deep_q_learning_server << "<param name=\"epsilon_discount\"               value=\""<< epsilon_discount <<"\"/>" << endl;
    if (readQTable){
        f_deep_q_learning_server << "<param name=\"read_weights\"      value=\"true\"/>" << endl;
        f_deep_q_learning_server << "<param name=\"weights_file\"      value=\"" << fileDQNweights << "\"/>" << endl;
    }
    string boolean = no_training? "true" : "false";
    f_deep_q_learning_server << "<param name = \"no_training\" value = \"" << boolean << "\" />" << endl;
    f_deep_q_learning_server << "</node>" << endl;
    f_deep_q_learning_server << "</launch>" << endl;


    return 0;
}