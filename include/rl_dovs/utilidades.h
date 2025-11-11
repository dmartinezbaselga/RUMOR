#include "TData.h"
#include "calcul.h"
#include <assert.h>

double Distancia (double x, double y);
void EcuacionRecta(double x_o, double y_o, double alfa, Line &r);
void SolDosRectas(Line r1, Line r2, Tsc &raiz, int &sol);
//void SolCirculoRecta(float a, float b, float c, float x, float y, float r, float raiz[][2], int& n);
void SolCirculoRecta(double a, double b, double c, double x, double y, double r, double raiz[][2], int& n);
void CalLimSup(double r, Tpf m45, Tpf m135, Tpf mns135, Tpf mns45, int& maximo, int& err);
void ObtenerComandoMaximo(double angulo, double vmax_adm, double w_max_left, double w_max_right, double &vmax, double &wmax);
void RectaDosPuntos(Tpf pto1, Tpf pto2, Line& r);
void LineTan(double radio, Line rin, double direccion, double esquinas_up[4][2], double esquinas_down[4][2],
		Tpf robot_up, Tpf robot_down, Tpf& sol, Line& rsol, int& err);
double DisPtoRecta(Line r, Tpf p);
void CalculaVelocidades(double x, double y, double r, double t, Velocidad &vel);
void DireccionMovimiento(double alfa, double vo, double xo, double yo, int& direction);
void LineTanR2(double radio, Line r, double angulo, int hacia_rob, Tpf& sol);

//Funciones para los objetos no lineales
Circumference ComputeCircumference(Tpf p0, Tpf p1, Tpf p2);
void RadioPtoTanCircunf(double xc, double yc, double r, double& r1, Tpf& p1, double& r2, Tpf& p2);
void RectaDosCircunf(Circumference c1, Circumference c2, Line& r);
void CircunfTanRecta(Circumference cobj, Line r, double robj, std::vector<std::pair<double,double>>& centers);
