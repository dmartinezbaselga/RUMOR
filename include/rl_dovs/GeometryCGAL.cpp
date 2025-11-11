//
// Created by maite14 on 23/01/18.
//

#include "GeometryCGAL.h"


#include <CGAL/Polyhedron_3.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/convex_hull_3.h>
typedef CGAL::Simple_cartesian<double> Kernel;
typedef CGAL::Polyhedron_3<Kernel>                     Polyhedron_3;
typedef Kernel::Point_3                                Point_3;
typedef CGAL::Surface_mesh<Point_3>               Surface_mesh;

#include <CGAL/IO/Geomview_stream.h>
#include <CGAL/IO/Polyhedron_geomview_ostream.h>

void GeometryCGAL::ConvexHull(const std::vector<dovt> &dataDOVT, const std::vector<dovtAhead> &dataAheadDOVT) {

    //TODO: probar polyhedron construido como una surface, .off format file

    Points points, result;
    std::vector<Points> resultCH;
    std::vector<Point_3> points3d, points3dCH;
    // define polyhedron to hold convex hull
    Polyhedron_3 poly; Surface_mesh sm;
    for (auto it = dataDOVT.begin(); it != dataDOVT.end(); ++it){
        dovt agDOVT = *it;
        auto itCommand = agDOVT.GetCommands().begin(); Point_3 p1_e1, p2_e1;
        if (!agDOVT.GetCommands().empty()){
            p1_e1 = Point_3(itCommand->inf.vel.w, itCommand->inf.vel.v, itCommand->inf.t);
            p2_e1 = Point_3(itCommand->sup.vel.w, itCommand->sup.vel.v, itCommand->sup.t);
            ++itCommand;
        }
        while (itCommand != agDOVT.GetCommands().end()){
            Point_3 p1_e2 = Point_3(itCommand->sup.vel.w, itCommand->sup.vel.v, itCommand->sup.t);
            Point_3 p2_e2 = Point_3(itCommand->inf.vel.w, itCommand->inf.vel.v, itCommand->inf.t);

            poly.make_tetrahedron(p1_e1, p2_e1, p1_e2, p2_e2);
            p1_e1 = p2_e2; p2_e1 = p1_e2;
            ++itCommand;
        }
        std::ofstream fh("polyhedron.dat", std::ios_base::out); int i = 0;
        //CGAL::set_ascii_mode( std::cout);
        CGAL::Geomview_stream gv(CGAL::Bbox_3(0,0,0, 600, 600, 600));
        for ( Polyhedron_3::Vertex_iterator v = poly.vertices_begin(); v != poly.vertices_end(); ++v){
            std::cout << v->point() << std::endl;   //print points in reverse order as inserted

            points3d.push_back(v->point());
            fh << v->point() << std::endl;
            if (i<3) i++;
            else{
                fh << std::endl;
                i = 0;
            }
        }
        gv << poly;
        fh.close();
        /*
        Gnuplot g;
        //g << "set dgrid3d\nsplot '-' w l lc 001\n";
        g << "splot 'polyhedron.dat' w pm3d\n";
        //g.send1d(points3d); //g.send1d(points);
        //g.send1d(points3dCH); //g.send1d(result);
        //g.send1d(sm.vertices());
        g.flush();
        //*/

        //resultCH.push_back(result);
    }

    /*
    for (auto it = dataAheadDOVT.begin(); it != dataAheadDOVT.end(); ++it){
        dovtAhead agDOVT = *it;
        auto itCommand = agDOVT..begin(); Point_3 p1_e1, p2_e1;
        if (!agDOVT.commands.empty()){
            p1_e1 = Point_3(itCommand->inf.vel.w, itCommand->inf.vel.v, itCommand->inf.t);
            p2_e1 = Point_3(itCommand->sup.vel.w, itCommand->sup.vel.v, itCommand->sup.t);
            ++itCommand;
        }
        while (itCommand != agDOVT.commands.end()){
            Point_3 p1_e2 = Point_3(itCommand->sup.vel.w, itCommand->sup.vel.v, itCommand->sup.t);
            Point_3 p2_e2 = Point_3(itCommand->inf.vel.w, itCommand->inf.vel.v, itCommand->inf.t);

            poly.make_tetrahedron(p1_e1, p2_e1, p1_e2, p2_e2);
            p1_e1 = p2_e2; p2_e1 = p1_e2;
            ++itCommand;
        }
        std::ofstream fh("polyhedron.dat", std::ios_base::out); int i = 0;
        //CGAL::set_ascii_mode( std::cout);
        CGAL::Geomview_stream gv(CGAL::Bbox_3(0,0,0, 600, 600, 600));
        for ( Polyhedron_3::Vertex_iterator v = poly.vertices_begin(); v != poly.vertices_end(); ++v){
            std::cout << v->point() << std::endl;   //print points in reverse order as inserted

            points3d.push_back(v->point());
            fh << v->point() << std::endl;
            if (i<3) i++;
            else{
                fh << std::endl;
                i = 0;
            }
        }
        gv << poly;
        fh.close();
        */
        /*
        Gnuplot g;
        //g << "set dgrid3d\nsplot '-' w l lc 001\n";
        g << "splot 'polyhedron.dat' w pm3d\n";
        //g.send1d(points3d); //g.send1d(points);
        //g.send1d(points3dCH); //g.send1d(result);
        //g.send1d(sm.vertices());
        g.flush();
        //*/

        //resultCH.push_back(result);
   // }

}

Velocidad GeometryCGAL::MinDistance(Velocidad vel) {

    if (polygon.empty())
        return vel;

    Point_2d velP = Point_2d(vel.w, vel.v);
    double dmin = 1e10; Segment_2d smin;
    for (auto itP = polygon.begin(); itP != polygon.end(); ++itP){
        Polygon_2d pol = (*itP);
        for (auto it = pol.edges_begin(); it != pol.edges_end(); ++it){
            Segment_2d s = (*it);
            double d = CGAL::squared_distance(velP, s);
            if (d < dmin){
                dmin = d;
                smin = s;
            }
        }
    }
    Line2d l(smin);
    Line2d lp = l.perpendicular(velP);
    CGAL::cpp11::result_of<Intersect_2d(Line2d, Line2d)>::type result = CGAL::intersection(l, lp);
    const Point_2d* p;
    if (result){
        p = boost::get<Point_2d >(&*result);
    }
    return Velocidad(p->y(), p->x());
    //return Velocidad((p->y() + velP.y())/2, (p->x() + velP.x())/2);
    //return Velocidad(pI.y, pI.y);
}
