/*
 * ComputeDensityVS.cpp
 *
 *  Created on: Jul 21, 2016
 *      Author: maite14
 */

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

#include <cmath>

#include <CGAL/MP_Float.h>
#include <CGAL/Quotient.h>
#include <CGAL/Arr_segment_traits_2.h>
#include <CGAL/Sweep_line_2_algorithms.h>


//#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Polygon_2_algorithms.h>
#include <CGAL/enum.h>

#include <CGAL/Cartesian.h>
#include <CGAL/Polygon_2.h>

#include <CGAL/Simple_cartesian.h>

/*
//definiciones para funciones que calculan intersecciones de poligonos con poligonos o puntos
typedef CGAL::Simple_cartesian<double>					Kernel;
//definiciones para funcion que calcula si un punto cae dentro, fuera o en el limite de un poligono
typedef CGAL::Exact_predicates_exact_constructions_kernel	K;
typedef K::Point_2 Point;
typedef CGAL::Arr_segment_traits_2<K>            	Traits_2;
typedef Traits_2::Point_2							Point_2;	//point type of the polygon
typedef Traits_2::Segment_2							Segment_2;	//segment type between two points of the polygon
*/

typedef CGAL::Cartesian<double> K;
typedef K::Point_2 Point;
typedef CGAL::Polygon_2<K> Polygon;

int main (int argc, char *argv[]){

	char *path = argv[1];

	int num_scen = 40;
	int num_goals = 5;

	//std::vector<Velocidad> polygon;
	//std::vector<Segment_2> segments;

	char fich[100];
	strcpy(fich, path); strcat(fich, "density_VS");
	std::ofstream fdensity(fich);

	char log[150];
	for (int i=0; i<num_scen; i++){
		for (int j=1; j<num_goals; j++){
			std::string s = std::to_string(i);
			std::string g = std::to_string(j);
			char fichero[100]; std::string str = "escen" + s + "/goal" + g + "/vs_data";
			strcpy(log, path);	strcat(log, str.c_str());

			//std::cout << log << std::endl;

			std::ifstream f(log);
			if ((f.rdstate() & std::ifstream::failbit) != 0){
				//error al abrir el fichero
				continue;
			}
			int it = 0;
			while (!f.eof()){
				std::string line;
				std::getline(f, line);
				std::istringstream flog(line);

				double v,w,ar,ab,iz,de,steer,goalv,goalw,dirv,dirw,vmin,vmax,wright,wleft,vmax_adm,wleft_adm,wright_adm, av, aw, step,steerdir, radio_goal;
				double obj_din;
				flog >> v; flog >> w; flog >> ar; flog >> ab; flog >> iz; flog >> de; flog >> steer; flog >> goalv; flog >> goalw; flog >> dirv;
				flog >> dirw; flog >> vmin; flog >> vmax; flog >> wright; flog >> wleft; flog >> vmax_adm; flog >> wleft_adm; flog >> wright_adm; flog >> av; flog >> aw;
				flog >> step; flog >> steerdir; flog >> radio_goal; flog >> obj_din;
				std::vector<int> size_din; //std::cout << "Num obj_din:" << obj_din << ", tamaño: ";
				for (int k=0; k<obj_din; k++){
					int size = 0;
					flog >> size;	//std::cout << "\t" << size << std::endl;
					size_din.push_back(size);
				}

				//Calcular area del polígono
				char fich_data[100];
				std::string str = "escen" + s + "/goal" + g + "/obj_dinFusion_" + std::to_string(it+1) + ".dat";
				strcpy(fich_data, path); strcat(fich_data, str.c_str());
				std::ifstream fdata(fich_data);
				//if ((fdata.rdstate() & std::ifstream::failbit) != 0){
					//error al abrir el fichero
				//	continue;
				//}
				double area_total = 0;	//area total ocupada
				for (int k=0; k<(int)size_din.size(); k++){	//número de DOVs
					std::vector<Point> puntos;
					for (int n=0; n<size_din[k]; n++){	//número de puntos del DOV
						double x,y;
						fdata >> x >> y;
						Point point = Point(x,y);
						puntos.push_back(point);
					}
					//puntos.push_back(puntos[0]);
					Polygon pgn(puntos.begin(), puntos.end());
					area_total = area_total + pgn.area();
					//std::cout << "The polygon is " << (pgn.is_simple() ? "" : "not ") << "simple." << std::endl;
					//std::cout << "The polygon is " << (pgn.is_convex() ? "" : "not ") << "convex." << std::endl;
					//std::cout << "Area ocupada: " << pgn.area() << "; Area total: " << (vmax-vmin)*std::abs(wright-wleft) << std::endl;
					//std::cin.get();
				}
				//if ((int)size_din.size() > 0) area_total /= (int)size_din.size();

				std::vector<Point> vs;
				vs.push_back(Point(wright,vmin)); vs.push_back(Point(wleft,vmin)); vs.push_back(Point(wleft,vmax)); vs.push_back(Point(wright,vmax));
				Polygon pgn_vs(vs.begin(), vs.end());
				double area_vs = pgn_vs.area(); //area total del VS

				//std::cout << s << "\t" << g << "\t" << it << "\t" << area_vs << "\t" << area_total << "\t" << area_vs-area_total << std::endl;
				fdensity << s << "\t" << g << "\t" << it << "\t" << area_vs << "\t" << area_total << "\t" << area_vs-area_total << std::endl;
				it++;
				//std::cin.get();
			}
			f.close();
		}
	}
	fdensity.close();
	/*

	if (Point_2(vs->GetDinDcha(), robot.v) != Point_2 (robot.w, vs->GetDinArr())){
		Segment_2 s = Segment_2(Point_2(vs->GetDinDcha(), robot.v), Point_2 (robot.w, vs->GetDinArr()));
		segments.push_back(s);
	}
	*/

	return 0;
}



