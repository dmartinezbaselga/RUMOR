/***************************************************/
/* Last Revised: 
$Id: TData.h,v 1.1 2004/02/11 20:29:39 jminguez Exp jminguez $
*/
/***************************************************/

#include <cmath>
#include "calcul.h"
#include <vector>

#include <cassert>
#include <limits>

#ifndef TData
#define TData

/* 
   Este fichero contiene los tipos de datos utilizados por todos 
*/

#define RADIO 0.4F  /* Radio del robot */

struct Tpf{
    double x,y;
    Tpf () : x(0), y(0) {}
    Tpf ( const double& x0, const double& y0) : x (x0), y(y0) {};
};
//using Tpf = Tpf;

struct Tpfp{
	double r, t;
    Tpfp () : r(0), t(0) {}
    Tpfp ( const double& r0, const double& t0) : r(r0), t(t0) {};
};

struct Tpi{
  int x;
  int y;
};

struct Tsc{
	double x;
	double y;
	double tita;
	Tsc():x(0.0), y(0.0), tita(0.0){};
	Tsc(double posx, double posy, double postita):x(posx), y(posy), tita(postita){};
	Tsc(const Tsc &pos):x(pos.x), y(pos.y), tita(pos.tita){};
	//bool operator == (const Tsc& locIn) {	//Ascending order by angle
    //    return (this->x == locIn.x && this->y == locIn.y && this->tita == locIn.tita);
    //}
};

struct Tscan{
  int numPuntos;
  Tpf laserC[721];
  Tpfp laserP[721];
};

//typedef struct{  double A, B, C, m, b;}Recta;

struct Rango{
  double izq;
  double dch;
  int calculos;	//para determinar si la zona requiere calculos o no, un mayor análisis
  int signo;	//radios de la zona son: 0->negativo, 1->positivo
};

struct Raiz{
	double raices[4][5];
	double angulo;
	double radio;
  int num_sol;
};

/*typedef struct{
  double v;
  double w;
}Velocidad;
*/

struct Velocidad{
	double v, w;
	Velocidad(const double& velV, const double& velW): v(velV), w(velW){}
	Velocidad(): v(0.0), w(0.0){}

	//friend bool operator == (const Velocidad& l, const Velocidad& r);
	friend bool operator < (const Velocidad& l, const Velocidad& r);

	// l < r <=> r > l
    // l == r <=> !(l<r) && !(l>r)
    bool operator == (const Velocidad& vIn) {	//Ascending order by angle
        return (this->v == vIn.v && this->w == vIn.w);
    }
};

struct PosibleVelocidad {
	bool      valid;
	Velocidad vel;
};


/*typedef struct{
  Velocidad inf;
  Velocidad sup;
  int objeto;	//Indica si pertenece o no al objeto, es decir, si es de colisión
}Comando;
*/

struct Comando{
	Velocidad vel;
	double t;
	Comando(): vel(), t(0.0){}
	Comando(const Velocidad& v, const double& ts): vel(v), t(ts){}
};

/*typedef struct{
  double t1;
  double t2;
}Tiempos;
*/

typedef int Etiqueta;

//estructuras añadidas para nueva implementacion de funcion Fusion() (1-06-10)
/*typedef struct{
	int id;
	//Comando c;
	Command c;
}ComandoId;
*/

/*typedef struct{
	int id;
	Tiempos t;
}TiempoId;
*/

class Command{
public:
	Comando sup;    //minimum velocity to reach the furthest point
	Comando inf;    //maximum velocity to reach the nearest point
	int objeto;	//Indica si pertenece o no al objeto, es decir, si es de colisión
	int id; //Identificador del objeto al que pertenece
	Command(const Comando& csup, const Comando& cinf, const int& o, const int& i): sup(csup), inf(cinf), objeto(o), id(i){};
	Command(const Comando& csup, const Comando& cinf, const int& o): sup(csup), inf(cinf), objeto(o), id(-1){};
	Command(): sup(), inf(), objeto(0), id(-1){}

    bool operator== (const Command& cIn){
        return (this->sup.vel == cIn.sup.vel && this->inf.vel == cIn.inf.vel);
    }

	friend bool operator < (const Command& l, const Command& r);
};

struct Rayo{
	double radio;
	double ang;
    std::vector<bool> obs;
	std::vector<Command> c;
	//std::vector<Tiempos> t;
};

const double INF = std::numeric_limits<double>::infinity();

//valores para considerar todos los obstáculos globalmente
const unsigned long obst_max = 400; // 400; // 50;
const unsigned long comm_max = 1000; //120;

//valores para considerar un obstáculo por separado
const int din_obs  = 10; // número de objetos dinámicos por obstáculo (en función del número de raices y si se divide (BC corta por detrás del robot))
const int comm_obs = 100; // número de comandos máximo por objeto dinámico: por cada objeto dinámico hay un número máximo de zonas de arcos de circunferencia que se consideran para los cálculos

struct DW_Range{
	Velocidad ini;
	Velocidad fin;
	int num;
};

void car2pol(Tpf *in, Tpfp *out);

class Root{
public:
	double x, y, radius, angle;	//(x,y)position of the root, and the corresponding polar coordinates
	int i;
    Root(){ x = 0; y = 0; radius = 0; angle = 0; i = -1;};
    Root(const Root& rIn):x(rIn.x), y(rIn.y), radius(rIn.radius), angle(rIn.angle), i(rIn.i){};
	Root(const double& xval, const double& yval){
		x = xval; y = yval;
		Tpf in; in.x = x; in.y = y;
		Tpfp out;
		car2pol(&in, &out);
		radius = out.r; angle = out.t;
		//std::cout << "Root: " << x << ", " << y << std::endl;
		i = 0;
	}
	Root(const double& xval, const double& yval, const int& n){
		//i represents the line or circumference to which the root belongs
		x = xval; y = yval;
		Tpf in; in.x = x; in.y = y;
		Tpfp out;
		car2pol(&in, &out);
		radius = out.r; angle = out.t;
		i = n;
	}

    bool operator== (const Root& rIn){
        return (this->x == rIn.x && this->y == rIn.y);
    }

    friend bool operator < (const Root& l, const Root& r);
};

typedef std::pair<double,double> Range; //first: left value of the interval, second: right value of the interval

class Circumference{
	private:
		double xc, yc;
		double radius;
		double A, B, C; //x^2 + y^2 + Ax + By + C = 0
	public:
		Circumference(): xc(0), yc(0), radius(0), A(0), B(0), C(0){}
		Circumference(double x, double y, double rc):xc(x), yc(y), radius(rc){
			A = -2*xc; B = -2*yc;  C = xc*xc + yc*yc - radius*radius;
		}
		double x(){ return xc; }
		double y(){ return yc; }
		double r(){ return radius; }
		double cA(){ return A; }
		double cB(){ return B; }
		double cC(){ return C; }
};

class Line{

    double A, B, C;

public:
    Line():A(0.0), B(0.0), C(0.0){};

    Line(double A0, double B0, double C0):A(A0), B(B0), C(C0){};

    Line(Tpf point, double alfa){    //Given a point and angle of orientation

        double x0 = point.x; double y0 = point.y;

        //if (std::abs(std::abs(alfa) - M_PI_2) < 1e-5){  // line: x=x0 (parallel line to axis 'y')
        if (std::abs(alfa) == M_PI_2){  // line: x=x0 (parallel line to axis 'y')
            A = 1.0; B = 0.0; C = -x0;  //m=1.0; b=-x_o;    // inf. slope
        }else if (alfa == 0 || std::abs(alfa) == M_PI){ // line: y=y0 (parallel line to axis 'x')
            A = 0.0; B = 1.0; C = -y0;  //m=0.0; b=y_o;
        }else{  // line: y=m*x+b
            A = -std::tan(alfa);
            B = 1.0;
            C = -y0 + x0*std::tan(alfa);
            //m=std::tan((double)alfa); b=y_o-x_o*std::tan((double)alfa);
        }
	};

	Line(Tpf point1, Tpf point2){   //Given two points which belong to the line

        double dir;

        if (std::abs(point2.x - point1.x) <= 1e-5)  //inf slope
        // else if (point2.x - point1.x == 0)
            dir = M_PI_2;
        else if (std::abs(point2.y - point1.y) <= 1e-5)  //zero slope
        // else if (point2.y - point1.y == 0)
            dir = 0.0;
        else
            dir = std::atan2((point2.y - point1.y),(point2.x - point1.x));

        //TODO: Revisar esto....
        Line l = Line(point1, dir);
        this->A = l.A; this->B = l.B; this->C  = l.C;
	}

	double Distance(Tpf point){ //Distance from a point to a line
	    return std::abs(A*point.x + B*point.y + C)/std::sqrt(A*A + B*B);
	}

	Line Parallel(double dist){ //Computes the line parallel to this separated a distance of dist
	//dist is signed so that we distinguish upper or lower parallel lines

	    //Distance from a point to a line: d(P,r) = |A*px + B*py + C|/sqrt(A*A + B*B);
        //Solve the equation for C, given the distance is the radius of the obstacle (dist):

        assert (std::abs(A) > 1e-5 || std::abs(B) > 1e-5);  //to assure one of both is not zero

        //We define a point which belongs to line this
        double px = (std::abs(B) > 1e-5) ? 0 : -C/A;
        double py = (std::abs(B) > 1e-5) ? -C/B : 0;

        double k = dist * std::sqrt(A*A + B*B) - A*px - B*py;
        //double k2 = -dist * std::sqrt(this->A*this->A + this->B*this->B) - this->A*px - this->B*py;

        return Line(A, B, k);
	}

	Line Perpendicular(Tpf point){
		return Line(-B, A, A*point.x - B*point.y);
	}

	std::pair<double, double> TangentRadius(){ //Computes the radius of the circunference arcs which are tangent to the line, with center (0,radius)
	//The distance from the center of the circunference to the line is the radius
	//Returns a pair of values: the first one is positive, the second one is negative
	//General equation, when the circunference is centered at center: radiusTg1 = (A*center.x + B*center.y + C)/std::sqrt(A*A + B*B); radiusTg2 = (-A*center.x - B*center.y - C)/std::sqrt(A*A + B*B);

	    double radiusTg1, radiusTg2;

	    radiusTg2 = -C/(std::sqrt(A*A + B*B) + B);
	    if (A != 0){
	        radiusTg1 = C/(std::sqrt(A*A + B*B) - B);
        }else{
	        radiusTg1 = 0;
	    }

	    if (radiusTg2 > 0) return {radiusTg2, radiusTg1};
	    else return {radiusTg1, radiusTg2};
	};

	double GetA(){ return A; }
	double GetB(){ return B; }
	double GetC(){ return C; }
};

#endif