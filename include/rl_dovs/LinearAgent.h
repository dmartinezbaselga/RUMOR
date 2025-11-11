//
// Created by maite16 on 18/07/17.
//


#include "TrajectoryAgent.h"

#include "dovts.h"

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#ifndef VS_LINEARAGENT_H
#define VS_LINEARAGENT_H


double NormalisePI(double value);

class LinearAgent: public TrajectoryAgent{
protected:
    std::vector<Tpf> cornerAg;   //corners of the square which inscribe the relative agent: {45.0, -45.0, 135.0, -135.0};


private:
    Line upper, lower, trajectory;

    double v_x, v_y, alfa, module;  //module and vectorial components of the velocity of the agent; alfa = loc.tita

    Velocidad velocityRef;  //module of the agent with respect the computations are being performed

    bool IsCBBehind();

    bool IntersectTrajBand(Range r);
    void IntersectionPoints(double rcandidate, double acandidate, bool& intersection, std::vector<Root>& roots);
    void TangentPoints(double rcandidate, std::vector<Root>& roots);
    bool ComputeVT_2d(const std::vector<Root> &points, bool insideBC, const bool intersection, unsigned traverse,
                          double rcandidate, double acandidate, double radio_obs, boundsVS bounds, const int th,
                          const bool accConst, Velocidad velCurrentAg, constraints acc, Command &cmdOut);

    virtual bool ConsiderRange(Range range);

public:
    LinearAgent(Tsc sistRef, Tsc sistIn, double size, double vIn, int nId, bool segment=false, double active_size = 0, Tsc sistIn2 = Tsc()):TrajectoryAgent(nId){
    // If segment, size is the value of half of the segment

    //sistRef: localization of the agent with respect the environment should be computed to
    //sistIn: localization of the agent to model (which moves with velocity vIn)
    //size: sum of the radius of each agent

        cornerAg.reserve(4);

        if (segment){
            Tsc l_first, l_last, l_center, l_upper, l_lower;
            // printf("Obstacle center in global coordinates: %f, %f\n", sistIn.x, sistIn.y);
            // std::cout << "Points received: " << sistIn.x << ", " << sistIn.y << "---" << sistIn2.x << ", " << sistIn2.y << std::endl;
            transfor_inversa_sis(sistIn, &sistRef, &l_first);
            transfor_inversa_sis(sistIn2, &sistRef, &l_last);
            // std::cout << "Points transformed: " << l_first.x << ", " << l_first.y << "---" << l_last.x << ", " << l_last.y << std::endl;
            l_center = Tsc((l_first.x + l_last.x)/2, (l_first.y + l_last.y)/2, l_first.tita);
            // printf("Sist ref: %f, %f\n", sistRef.x, sistRef.y);
            TrajectoryAgent::SetLocalization(l_center);


            /*//Compute the module of the relative velocity: vIn - vRef
            //Compute the vectorial components of the velocity vector
            double vxIn = vIn*std::cos(sistIn.tita); double vyIn = vIn*std::sin(sistIn.tita);
            double vxRef = vRef*std::cos(sistRef.tita); double vyRef = vRef*std::sin(sistRef.tita);
            //Relative velocity vector in world reference: (vxIn-vxRef)i + (vyIn-vyRef)j
            double moduleW = std::sqrt(std::pow(vxIn-vxRef,2) + std::pow(vyIn-vyRef,2));
            //Relative velocity vector in sisRef reference: (vxInRef-vRef)i + (vyInRef)j
            //Note: module is equal to moduleW
            double vxInRef = vIn*std::cos(loc.tita); double vyInRef = vIn*std::sin(loc.tita);
            double v_xRef = vxInRef-vRef; double v_yRef = vyInRef;
            double moduleRef = std::sqrt(std::pow(vxInRef-vRef,2) + std::pow(vyInRef,2));
            double alfaRef = std::atan2(v_yRef, v_xRef);
            //*/

            v_x = 0; v_y = 0;
            module = vIn; alfa = l_center.tita;
            //Equation of the linear agent's trajectory
            trajectory = Line(Tpf(l_center.x, l_center.y), l_center.tita);
            if (l_first.y > l_last.y){
                l_upper = l_first;
                l_lower = l_last;
            }
            else{
                l_upper = l_last;
                l_lower = l_first;
            }
            lower = Line(Tpf(l_lower.x, l_lower.y), l_lower.tita);
            upper = Line(Tpf(l_upper.x, l_upper.y), l_upper.tita);
            

            //Compute the square which inscribes the linear agent
            // std::cout << "Line: " << l_center.x << ", " << l_center.y << std::endl;
            // std::cout << "Active size: " << active_size << std::endl;
            if (l_center.tita >= 0 && l_center.tita <= M_PI){   
                std::vector<Tpf> cornerAux;
                std::vector<double> dist = {-active_size, active_size};
                std::vector<double> extra_length = {0, 0};
                Tsc sist = l_upper;
                // printf("Size: %f, %f, %f\n", size-active_size, l_first.x, l_first.y);
                // std::cout << "1st-2nd" << std::endl;
                for (int i = 0; i < 2; i++){
                    Tpf pto;
                    // sist.tita = NormalisePI( (i*M_PI)/180.0 + l_first.tita );
                    // printf("sist.tita = %f\n", sist.tita);
                    transfor_directa_p(extra_length[i], dist[i], &sist, &pto);
                    cornerAux.push_back(pto);    //corners of the square inscribing the agent (sistIn) with respect to sistRef, i.e., the linear agent
                    // printf("EL PUNTOOOOOOOOOOOOOOOOOO: %f, %f\n", pto.x, pto.y);
                }
                sist = l_lower;
                sist.tita = NormalisePI( M_PI + l_lower.tita );
                for (int i = 0; i < 2; i++){
                    Tpf pto;
                    // sist.tita = NormalisePI( (i*M_PI)/180.0 + l_first.tita );
                    // printf("sist.tita = %f\n", sist.tita);
                    transfor_directa_p(extra_length[i], dist[i], &sist, &pto);
                    cornerAux.push_back(pto);    //corners of the square inscribing the agent (sistIn) with respect to sistRef, i.e., the linear agent
                    // printf("EL PUNTOOOOOOOOOOOOOOOOOO: %f, %f\n", pto.x, pto.y);
                }
                cornerAg.push_back(cornerAux[0]);
                cornerAg.push_back(cornerAux[3]);
                cornerAg.push_back(cornerAux[1]);
                cornerAg.push_back(cornerAux[2]);
            }else{  //3rd and 4rd cuadrants
                // std::cout << "3rd-4rd" << std::endl;
                std::vector<double> dist = {active_size, -active_size};
                std::vector<double> extra_length = {0, 0};
                Tsc sist = l_lower;
                // printf("Size: %f, %f, %f\n", size-active_size, l_first.x, l_first.y);
                for (int i = 0; i < 2; i++){
                    Tpf pto;
                    // sist.tita = NormalisePI( (i*M_PI)/180.0 + l_first.tita );
                    // printf("sist.tita = %f\n", sist.tita);
                    transfor_directa_p(extra_length[i], dist[i],&sist, &pto);
                    cornerAg.push_back(pto);    //corners of the square inscribing the agent (sistIn) with respect to sistRef, i.e., the linear agent
                    // printf("EL PUNTOOOOOOOOOOOOOOOOOO: %f, %f\n", pto.x, pto.y);
                }
                sist = l_upper;
                sist.tita = NormalisePI( M_PI + l_upper.tita );
                for (int i = 0; i < 2; i++){
                    Tpf pto;
                    // sist.tita = NormalisePI( (i*M_PI)/180.0 + l_first.tita );
                    // printf("sist.tita = %f\n", sist.tita);
                    transfor_directa_p(extra_length[i], dist[i], &sist, &pto);
                    cornerAg.push_back(pto);    //corners of the square inscribing the agent (sistIn) with respect to sistRef, i.e., the linear agent
                    // printf("EL PUNTOOOOOOOOOOOOOOOOOO: %f, %f\n", pto.x, pto.y);
                }
            } 
            // std::cout << "Corners agent: " << std::endl;
            // for (auto p:cornerAg){
            //     std::cout << p.x << "-" << p.y << std::endl;
            // }        
        }
        else{
            Tsc l;
            transfor_inversa_sis(sistIn, &sistRef, &l);
            TrajectoryAgent::SetLocalization(l);

            /*//Compute the module of the relative velocity: vIn - vRef
            //Compute the vectorial components of the velocity vector
            double vxIn = vIn*std::cos(sistIn.tita); double vyIn = vIn*std::sin(sistIn.tita);
            double vxRef = vRef*std::cos(sistRef.tita); double vyRef = vRef*std::sin(sistRef.tita);
            //Relative velocity vector in world reference: (vxIn-vxRef)i + (vyIn-vyRef)j
            double moduleW = std::sqrt(std::pow(vxIn-vxRef,2) + std::pow(vyIn-vyRef,2));
            //Relative velocity vector in sisRef reference: (vxInRef-vRef)i + (vyInRef)j
            //Note: module is equal to moduleW
            double vxInRef = vIn*std::cos(loc.tita); double vyInRef = vIn*std::sin(loc.tita);
            double v_xRef = vxInRef-vRef; double v_yRef = vyInRef;
            double moduleRef = std::sqrt(std::pow(vxInRef-vRef,2) + std::pow(vyInRef,2));
            double alfaRef = std::atan2(v_yRef, v_xRef);
            //*/

            v_x = vIn*std::cos(l.tita); v_y = vIn*std::sin(l.tita);
            module = vIn; alfa = l.tita;
            //Equation of the linear agent's trajectory
            trajectory = Line(Tpf(l.x, l.y), l.tita);

            //The direction of the agent sistIn with respect to the agent sistRef determines which line is upper and lower for the BAND swept by the moving agent sistIn
            if ((l.tita <= 0 && l.tita >= -M_PI_2) || (l.tita >= M_PI_2 && l.tita <= M_PI)){    //the relative orientation of the agent lies in 2nd and 4th cuadrants
                upper = trajectory.Parallel(size);
                lower = trajectory.Parallel(-size);
            }else{  //1st and 3rd cuadrants
                upper = trajectory.Parallel(-size);
                lower = trajectory.Parallel(size);
            }
            //Compute the square which inscribes the linear agent
            std::vector<double> degrees = {45.0, -45.0, 135.0, -135.0}; //corners with respect to the relative orientation of the linear agent
            double x = size;
            double y = 0.0;
            Tsc sist; sist.x = l.x; sist.y = l.y;
            for (auto i:degrees){
                Tpf pto;
                sist.tita = NormalisePI( (i*M_PI)/180.0 + l.tita );
                transfor_directa_p(x, y, &sist, &pto);
                cornerAg.push_back(pto);    //corners of the square inscribing the agent (sistIn) with respect to sistRef, i.e., the linear agent
            }
        }
        // std::cout << "Corners agent: " << std::endl;
        // for (auto p:cornerAg){
        //     std::cout << p.x << "-" << p.y << std::endl;
        // }     
    }
    ~LinearAgent(){};

    bool GetOriginInside();
    void IntersectingTrajectories(std::vector<Range>& rangeTraj);

    double GetVX(){ return v_x; };
    double GetVY(){ return v_y; };
    Line GetTrajectory(){ return trajectory; };
    Line GetLineUp(){ return upper; };
    Line GetLineLow(){ return lower; };
};

#endif //VS_LINEARAGENT_H
