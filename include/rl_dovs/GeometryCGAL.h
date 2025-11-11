//
// Created by maite14 on 23/01/18.
//

#ifndef VS_GEOMETRYCGAL_H
#define VS_GEOMETRYCGAL_H

#include "dovts.h"

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/convex_hull_2.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Arr_segment_traits_2.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K1;
typedef K1::Point_2 Point_2d;
typedef CGAL::Polygon_2<K1> Polygon_2d;
typedef CGAL::Arr_segment_traits_2<K1> Traits_2d;
//typedef Traits_2d::Curve_2 Segment_2d;
typedef K1::Segment_2 Segment_2d;
typedef CGAL::Line_2<K1> Line2d;
typedef K1::Intersect_2 Intersect_2d;

typedef std::vector<Point_2d> Points;

class GeometryCGAL {
    //Class to perform geometry operations with CGAL library

    std::vector<Polygon_2d> polygon;

public:

    GeometryCGAL(const std::vector<dovt> &dataDOVT){

        Points poly;
        for (auto it = dataDOVT.begin(); it != dataDOVT.end(); ++it) {
            dovt agDOVT = *it;
            for (auto itCommand = agDOVT.GetCommands().begin(); itCommand != agDOVT.GetCommands().end(); ++itCommand) {
                Point_2d p1_e2 = Point_2d(itCommand->sup.vel.w, itCommand->sup.vel.v);
                //Point_2d p2_e2 = Point_2d(itCommand->inf.vel.w, itCommand->inf.vel.v);

                poly.push_back(p1_e2);
                //poly.push_back(p2_e2);
            }
            for (auto itCommand = agDOVT.GetCommands().rbegin(); itCommand != agDOVT.GetCommands().rend(); ++itCommand){
                Point_2d p1_e2 = Point_2d(itCommand->inf.vel.w, itCommand->inf.vel.v);
                poly.push_back(p1_e2);
            }
            Polygon_2d pol = Polygon_2d(poly.begin(), poly.end());
            polygon.push_back(pol);
            if (pol.is_simple()) std::cout << "Simple";
            else std::cout << "Not simple";

        }
    };
    ~GeometryCGAL(){};

    Velocidad MinDistance(Velocidad vel);
    void ConvexHull(const std::vector<dovt> &dataDOVT, const std::vector<dovtAhead> &dataAheadDOVT);
    bool CheckIsInside(Point_2d p){
        for (auto it = polygon.begin(); it != polygon.end(); ++it){
            Polygon_2d pol = (*it);
            if (pol.is_simple())
                switch(pol.bounded_side(p)) {
                    case CGAL::ON_BOUNDED_SIDE :
                        return true;
                    case CGAL::ON_BOUNDARY:
                        break;
                    case CGAL::ON_UNBOUNDED_SIDE:
                        break;
                }
        }
        return false;
    }

};


#endif //VS_GEOMETRYCGAL_H
