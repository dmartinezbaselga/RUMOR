//#include "/usr/local/dislin/dislin.h"
#include <dislin.h>
#include <array>
//#include "/home/maite/dislin/dislin.h"
#include "TData.h"
#include "VS.h"

void Inicializar(bool video=false);
// void Terminar(int n);
void Terminar();
void NuevaVentana(int id);
void SeleccionaVentana(int id);
void LimpiaVentana();
void CierraVentana(int id);
void DibujaEjes(double xa, double xe, double xr, double xstp, double ya, double ye, double yr, double ystp);
void TerminaEjes();
// void DibujaRobotInicial(int id, Tsc rob, const char* c);
// void DibujaRobot(int id, Tsc anterior, Tsc actual, const char* c1, const char* c2);
void DibujaObsInicial(double lx, double ly, double radio, double step, const char* c);
// void DibujaObs(int id, Tsc anterior, Tsc actual, double radio, const char *c1, const char *c2);
// //void DibujaRectasNormal(double id, Tsc l, double alfa);
// void DibujaBandaColision(int id, double alfa, Tpf mas45, Tpf menos45, Tpf mas135, Tpf menos135, Line arriba, Line abajo);
// void DibujaTangentes(int id, double pos_in, double pos_out, double neg_in, double neg_out);
// void DibujaRaices(int id, Raiz raiz, double x, double y, double radio, double step, const char *c);
// void DibujaGoalRadio(int id, double radio, char *c);
void DibujaGoal(Tsc l, const char *c, const char *eti);

// void DibujaVS3(int id, float xray[100], float yray[100], float zray[100], int n,
// 		double v, double w, double a_v, double a_w, double t, double dir_goal, double vmin_adm, double vmax_adm, double wmin_adm, double wmax_adm);

// void DibujaVSG2(int id, float xray[100], float yray[100], int n, double v, double w, double a_v, double a_w, double t,
// 		double dir_goal, Velocidad vel_goal, Tsc obj, double radio_obj, Line arriba, Line abajo, double vmin_adm,
// 		double vmax_adm, double wmin_adm, double wmax_adm, double wmax_dch, double wmax_izq);
// void DibujaVSG2Varios(int id, float mxray[][100], float myray[][100], int n[], int num_obj_din, double robot_v, double robot_w, double a_v, double a_w,
// 		double t, double dir_goal, Velocidad vel_goal, Tsc obj, double radio_obj, Line arriba, Line abajo, double vmin_adm,
// 		double vmax_adm, double wmin_adm, double wmax_adm, double wmax_dch, double wmax_izq);

void DibujaVS(int id, std::vector<std::vector<double>> &mxray,
			  std::vector<std::vector<double>> &myray, std::vector<int> &n,
			  int num_obj_din, double robot_v,
		double robot_w, double dinArr, double dinAbjo, double dinIzda, double dinDcha,
		Velocidad cmdGoal, Velocidad dir_goal, Velocidad vGoal, boundsVS bounds,
		double av, double aw, double t, bool video=false, int iteracion=0, std::vector<std::vector<int>> positions = std::vector<std::vector<int>>(), bool debug = false);

// void DibujaVS3Nuevo(int id, float mxray[][100], float myray[][100], float mzray[][100],
// 		int n[], double v, double w, double a_v, double a_w, double t, double dir_goal,
// 		double vmin_adm, double vmax_adm, double wmin_adm, double wmax_adm);
void Selecciona(double& wcursor, double& vcursor, double wmax_izq, double wmax_dch, double vmax_adm, double vmin_adm);

void DibujaRecta(int id, float x[], float y[], int n, const char* c, int linw = 1);

// void DibujaVST(int id, float mxray[obst_max][comm_max], float myray[obst_max][comm_max], float mzray[obst_max][comm_max], int n[obst_max], int num_obj_din, double robot_v,
// 		double robot_w, double dinArr, double dinAbjo, double dinIzda, double dinDcha,
// 		double steer_w, Velocidad vel_goal, Velocidad dir_goal, double vmin, double vmax,
// 		double wright, double wleft, double vmax_r, double wleft_r, double wright_r,
// 		double av, double aw, double t, double steer_dir, double radio_goal);
