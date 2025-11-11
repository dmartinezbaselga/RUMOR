#include "graficas.h"
#include <cmath>
#include <iostream>
#include "utilidades.h"
#include <unistd.h>

#include <string.h>

using namespace std;

void DibujaRecta(int id, float x[], float y[], int n, const char *c, int linw){
	// reset("nochek");
	color(c);
	linwid(linw);
	//curve(x, y, n);
	rline(x[0], y[0], x[1], y[1]);

	//selwin(id);
	linwid(1);

}

void Inicializar(bool video){
	//estas dos tienen que ir antes que disini
	//setpag("da4p")
	if (video) {
		metafl("PNG");
		char tmp[256];
    getcwd(tmp, 256);
    // cout << "Current working directory: " << tmp << endl;
	} else metafl("XWIN");
	// cout << "Inicializar" << endl;

	scrmod("revers");
	//winsiz(550, 340);
	winsiz(800, 600);
	//winsiz(950, 640);
	//winsiz(485, 325);
	//window(425,0,400,325);
	disini();
	nochek(); // Don't warn about out or scale things
}

// void Terminar(int n){

// 	//for (int i=2; i<=n; i++) CierraVentana(n);
// 	disfin();
// }

void Terminar(){

	//for (int i=2; i<=n; i++) CierraVentana(n);
	disfin();
}

void NuevaVentana(int id){
	opnwin(id);
}

void SeleccionaVentana(int id){
	selwin(id);
}

void LimpiaVentana(){

	erase();
}

void CierraVentana(int id){

	clswin(id);
}

void DibujaEjes(double xa, double xe, double xr, double xstp, double ya, double ye, double yr, double ystp){

	//endgrf();
	name("X","x");
	name("Y","y");
	color("white");
	graf(xa,xe,xr,xstp,ya,ye,yr,ystp);
	//selwin(id);
	
}

void TerminaEjes(){

	endgrf();
}

// void DibujaRobotInicial(int id, Tsc l, const char* c){

// 	//selwin(id);

// 	double x[7], y[7];
// 	//double lado_coseno = 0.4*cos(l.tita);
// 	//double lado_seno = 0.4*sin(l.tita);
// 	double lado_coseno = 0.3*cos(l.tita);
// 	double lado_seno = 0.3*sin(l.tita);

// 	//selwin(id);

// 	x[0] = l.x - lado_seno - lado_coseno; y[0] = l.y - lado_seno + lado_coseno;	//trasera izquierda
// 	x[1] = l.x - lado_coseno + lado_seno; y[1] = l.y - lado_coseno - lado_seno;	//trasera derecha
	
// 	x[2] = l.x + lado_coseno + lado_seno; y[2] = l.y - lado_coseno + lado_seno;	//delantera derecha
// 	x[3] = l.x + lado_coseno - lado_seno; y[3] = l.y + lado_coseno + lado_seno;	//delantera izquierda
// 	x[4] = l.x - lado_seno - lado_coseno; y[4] = l.y - lado_seno + lado_coseno;	//trasera izquierda
// 	x[5] = l.x + lado_coseno; y[5] = l.y + lado_seno;	//frontal
// 	x[6] = l.x - lado_coseno + lado_seno; y[6] = l.y - lado_coseno - lado_seno;	//trasera derecha
	
// 	color(c);
// 	//curve(x,y,5);
// 	rline(x[0],y[0],x[1],y[1]);
// 	rline(x[1],y[1],x[2],y[2]);
// 	rline(x[2],y[2],x[3],y[3]);
// 	rline(x[3],y[3],x[0],y[0]);

// 	rline(x[0],y[0],x[5],y[5]);
// 	rline(x[5],y[5],x[1],y[1]);
	
// 	/*x[0] = l.x - lado_seno - lado_coseno; y[0] = l.y - lado_seno + lado_coseno;	//trasera izquierda
// 	x[1] = l.x + lado_coseno; y[1] = l.y + lado_seno;	//frontal
// 	x[2] = l.x - lado_coseno + lado_seno; y[2] = l.y - lado_coseno - lado_seno;	//trasera derecha
// 	curve(x,y,3);*/
// 	//cout << "Dibujo robot 2" << endl;
// 	//selwin(id);
// }

// void DibujaRobot(int id, Tsc anterior, Tsc actual, const char* c1, const char* c2){

// 	//selwin(id);
	
// 	DibujaRobotInicial(id, anterior, c1);
// 	DibujaRobotInicial(id, actual, c2);

// }

void DibujaObsInicial(double lx, double ly, double radio, double step, const char* c){

	//selwin(id);
	
	int n;

	if (step == 0.1) n=100;
	else n=6300;
	//cin.get();
	//lx=-1.5;ly=2;
	float *xray = new float[n];
	float *yray = new float[n];
	float x[2], y[2];
	double k=0.;
	for (int i=0; i<n; i++){
		xray[i] = lx + radio*cos(k);
		yray[i] = ly + radio*sin(k);
		k = k + step; 
	}
	//cout << "MiX, MiY:  " << mix << " , " << miy << endl;
	//cin.get();
	x[0] = lx; y[0] = ly;
	x[1] = lx; y[1] = ly;
	color(c);
	curve(x,y,2);
	//cout << "Dibujo obstaculo 2" << endl;
	curve(xray, yray, n);
	//curvmp(xray, yray, n);
	//rlcirc(lx,ly, radio);
	//cout << "Dibujo obstaculo 3" << endl;		
	//selwin(id);
	//cout << "Dibujo obstaculo 3" << endl;
	//	cin.get();
	//endgrf();
	delete[] xray;
	delete[] yray;
}

// void DibujaObs(int id, Tsc anterior, Tsc actual, double radio, const char* c1, const char* c2){

// 	selwin(id);
// 	//cout << "DibujaObs: Actualizo obs 1" << endl;
// 	//cout << "anterior: " << anterior.x << ", " << anterior.y << endl;
// 	//cout << "actual: " << actual.x << ", " << actual.y << endl;
// 	//cin.get();
// 	DibujaObsInicial(id, anterior.x, anterior.y, radio, 0.1, c1);
// 	DibujaObsInicial(id, actual.x, actual.y, radio, 0.1, c2);
// 	//cout << "DibujaObs: Actualizo obs 2" << endl;

// }

// void DibujaBandaColision(int id, double alfa, Tpf mas45, Tpf menos45, Tpf mas135, Tpf menos135, Line arriba, Line abajo){
	
// 	double r_mas45, r_menos45, r_mas135, r_menos135;
// 	float xray[200], yray[200];

// 	selwin(id);
// 	//dibujamos el cuadrado que inscribe al objeto
// 	nochek();	//nochek is used to suppress the listing of points that lie outside of the axis scaling
// 	color("white");
// 	rline(mas45.x, mas45.y, mas135.x, mas135.y);	//rline joins two points with a line
// 	rline(mas135.x, mas135.y, menos135.x, menos135.y);
// 	rline(menos135.x, menos135.y, menos45.x, menos45.y);
// 	rline(menos45.x, menos45.y, mas45.x, mas45.y);	

// 	selwin(id);	//selwin selects a window on the screen where the following graphics output will be sent to
// 	//cin.get();

// 	//mostramos las etiquetas de los 4 puntos correspondientes a las 4 esquinas del cuadrado
// 	color("white");
// 	height(20);	//height defines the character height
// 	rlmess("(+45) ", mas45.x, mas45.y);	//rlmess plots text
// 	rlmess("(+135) ", mas135.x, mas135.y);
// 	rlmess("(-45) ", menos45.x, menos45.y);
// 	rlmess("(-135) ", menos135.x, menos135.y);

// 	selwin(id);
// 	//cin.get();
// 	//dibujamos las rectas normales
// 	if (std::abs(alfa) == M_PI/2){

// 		for (int i=0; i<200; i++){
// 			xray[i] = -arriba.GetC();
// 			yray[i] = i-100;
// 		}
// 		color("red");	//la de arriba de color rojo
// 		curve(xray, yray, 200);
// 		for (int i=0; i<200; i++){
// 			xray[i] = -abajo.GetC();
// 			yray[i] = i-100;
// 		}
// 		color("blue");	//la de abajo de color azul
// 		curve(xray, yray, 200);
// 	}
// 	else{
// 		for (int i=0; i<200; i++){
// 			xray[i] = i-100;
// 			yray[i] = (arriba.GetA()*xray[i] + arriba.GetC()) / (-arriba.GetB());
// 		}
// 		color("red");	//la de arriba de color rojo
// 		curve(xray, yray, 200);
// 		for (int i=0; i<200; i++){
// 			xray[i] = i-100;
// 			yray[i] = (abajo.GetA()*xray[i] + abajo.GetC()) / (-abajo.GetB());
// 		}
// 		color("blue");	//la de abajo de color azul
// 		curve(xray, yray, 200);
// 		//cin.get();
// 	}
// 	selwin(id);
// 	//cin.get();

// 	r_mas45 = (pow(mas45.x,2) + pow(mas45.y,2))/(2*mas45.y);
// 	//cout << "r_mas45: " << r_mas45 << endl;
// 	r_menos45 = (pow(menos45.x,2) + pow(menos45.y,2))/(2*menos45.y);
// 	//cout << "r_menos45: " << r_menos45 << endl;
// 	r_mas135 = (pow(mas135.x,2) + pow(mas135.y,2))/(2*mas135.y);
// 	//cout << "r_mas135: " << r_mas135 << endl;
// 	r_menos135 = (pow(menos135.x,2) + pow(menos135.y,2))/(2*menos135.y);
// 	//cout << "r_menos135: " << r_menos135 << endl;

// 	//dibuja las trayectorias en arcos de circunferencia que seguiria el objeto si pasara por las 4 esquinas
// 	//DibujaObsInicial(id, 0, r_mas45, r_mas45, 0.001, "blue");
// 	//DibujaObsInicial(id, 0, r_menos45, r_menos45, 0.001, "blue");
// 	//DibujaObsInicial(id, 0, r_mas135, r_mas135, 0.001, "blue");
// 	//DibujaObsInicial(id, 0, r_menos135, r_menos135, 0.001, "blue");
	
// 	selwin(id);	
// }

// void DibujaTangentes(int id, double pos_in, double pos_out, double neg_in, double neg_out){

// 	selwin(id);
// 	nochek();
// 	// visualizacion de los radios_limites (radios tangentes)
// 	if (pos_in != 0)	//radios positivo tangente a la recta in
// 		DibujaObsInicial(id, 0, pos_in, pos_in, 0.001, "orange");
// 	if (pos_out != 0)	//radios positivo tangente a la recta out
// 		DibujaObsInicial(id, 0, pos_out, pos_out, 0.001, "orange");
// 	if (neg_in != 0)	//radios negativo tangente a la recta in
// 		DibujaObsInicial(id, 0, neg_in, neg_in, 0.001, "orange");
// 	if (neg_out != 0)	//radios negativo tangente a la recta out
// 		DibujaObsInicial(id, 0, neg_out, neg_out, 0.001, "orange");
// }

// void DibujaRaices(int id, Raiz raiz, double x, double y, double radio, double step, const char *c){

// 	//double pimedios = PI/2;
	
// 	selwin(id);
// 	//visualiza las soluciones geométricas
// 	nochek();
// 	for (int i=0; i<raiz.num_sol; i++){

// 		color("white");
// 		//hsymbl(36);	//hsymb defines the size of symbols
// 		//rlsymb(21, raiz.raices[i][0], raiz.raices[i][1]); 	//rlsymb plots a symbol where the centre is specified by user coordinates
// 		//para que salga un punto
// 		linwid(12);	//sets the line width 
// 		rline(raiz.raices[i][0], raiz.raices[i][1], raiz.raices[i][0], raiz.raices[i][1]);
// 		linwid(1);
		
// 	}
// 	//cout << " radio: " << radio << endl;
// 	if (raiz.angulo != M_PI/2) DibujaObsInicial(id, x, y, radio, step, c);
// 	selwin(id);
// }

// void DibujaGoalRadio(int id, double radio, char *c){
	
// 	selwin(id);
// 	nochek();
// 	DibujaObsInicial(id, 0, radio, radio, 0.001, c);
// 	//cout << "Radio goal: " << radio << endl;
// }

void DibujaGoal(Tsc l, const char *c, const char *eti){

	//selwin(id);

	//selwin(id);
	height(40);
	color(c);
	//hsymbl(35);	//hsymb defines the size of symbols
	rlsymb(3, l.x, l.y); 	//rlsymb plots a symbol where the centre is specified by user coordinates
	rlmess(eti, l.x, l.y);
	//selwin(id);
	//rlcirc(3,4,1);
	//rlell(6,7,2,1);

}

// /*void DibujaVS3(int id, double xray[100], double yray[100], double zray[100], int n,
// 		double v, double w, double a_v, double a_w, double t, double dir_goal, double vmin_adm, double vmax_adm,
// 		double wmin_adm, double wmax_adm){*/

// void DibujaVS3Nuevo(int id, float mxray[][100], float myray[][100], float mzray[][100], int n[], double v, double w, double a_v, double a_w, double t, double dir_goal, double 				vmin_adm, double vmax_adm, double wmin_adm, double wmax_adm){
// //muestra en la gráfica(3D) la forma del objeto dinámico

// 	int aux[100];
// 	int iret;

// 	double aux1[100][100], mwmat[100][100][100], wlevel;
// 	double xtri[100], ytri[100], ztri[100];
// 	int i1ray[100], i2ray[100], i3ray[100];
// 	int ntri;
// 	double mzmat[100];
// 	double step;
// 	double x1[4], y1[4], x2[4], y2[4];
// 	double zmat[100][100];
// 	int imat[100][100];
// 	double wmat[100][100];
// 	int nx, ny;

// 	/*for (int i=0; i<100; i++)
// 		for (int j=0; j<100; j++)
// 			for (int k=0; k<100; k++)
// 				wmat[i][j][k] = 3.0;

// 	for (int i=0; i<100; i++)
// 		zmat[i] = 3.0;
// 	*/
	
// 	/*x1[0] = xray[0]; x1[1] = xray[1];
// 	x1[2] = xray[2]; x1[3] = xray[3];
// 	y1[0] = xray[0]; y1[1] = xray[1];
// 	y1[2] = xray[2]; y1[3] = xray[3];
// 	x2[0] = xray[0]; x2[1] = xray[1];
// 	x2[2] = xray[2]; x2[3] = xray[3];
// 	y2[0] = xray[0]; y2[1] = xray[1];
// 	y2[2] = xray[2]; y2[3] = xray[3];*/
	

// 	/*step = 360./(n-1);
// 	for (int i=0; i<n; i++){
// 		for (int j=0; j<n; j++){
// 			aux1[i][j] = zray[j];
// 		}
// 	}*/
// 	cout << "DibujaVS3Nuevo" << endl;
// 	selwin(id);
// 	//nochek();	//nochek is used to suppress the listing of points that lie outside of the axis scaling
// 	name("Angular velocity","x");
// 	name("Linear Velocity","y");
// 	name("Time","z");

// 	//autres(n, n);
// 	color("white");
// 	graf3d(wmin_adm, wmax_adm, wmin_adm, 0.5, vmin_adm, vmax_adm, vmin_adm, 0.5,0.,30.,0.,10.);
// 	//graf3(wmin_adm, wmax_adm, wmin_adm, 0.5, vmin_adm, vmax_adm, vmin_adm, 0.5,0.,30.,0.,10.);
// 	//graf(wmin_adm, wmax_adm, wmin_adm, 0.5, vmin_adm, vmax_adm, vmin_adm, 0.5);
// 	//curve(x1, y1, 4);
// 	//getchar();
// 	//shdcrv(x1, y1, 4, x2, y2, 4);
	
// 	//isopts(mxray[0], n[0], myray[0], n[0], mzray[0], n[0], zmat, 300, xtri, ytri, ztri, 100, &ntri);
// 	//suriso(xtri, n[0], ytri, n[0], ztri, n[0], mzray[0], ntri);
// 	//cout << "numero triang: " << ntri << endl;
// 	//cin.get();

// 	//zscale(0, 30);
// 	//TRIANG calculates the Delaunay triangulation of an arbitrary collection of points in the plane
// 	ntri = triang (mxray[0], myray[0] , n[0], i1ray, i2ray, i3ray, 201);
// 	//crvtri (mxray[0], myray[0], mzray[0], n[0], i1ray, i2ray, i3ray, ntri);
// 	//view3d(0.7854, 0.7854, 0,"angle");
// 	surtri(mxray[0], myray[0], mzray[0], n[0], i1ray, i2ray, i3ray, ntri);

// 	//cout << "num triang: " << ntri << endl;
// 	//for (int i=0; i<ntri; i++)
// 	//	cout << i1ray[i] << ", " << i2ray[i] << ", " << i3ray[i] << endl;
	
// 	//cin.get();

// 	//crvtri(mxray[0], myray[0], mzray[0], n[0], i1ray, i2ray, i3ray, ntri);
// 	//crvmat((double *) aux1, n, n, 1, 1);
// 	//view3d(0., 1.5, 4.,"abs");
//   	//graf3d(-1.,1.,-1.,0.5,0.,1.5,0.,0.5,0.,30.,0.,10.); 

// 	/*con las lineas siguientes dibuja el contorno
// 	colray(mzray[0], aux, n[0]);
// 	mxray[0][n[0]] = mxray[0][0];
// 	myray[0][n[0]] = myray[0][0];
// 	mzray[0][n[0]] = mzray[0][0];
// 	curv3d(mxray[0], myray[0], mzray[0], n[0]+1);
// 	*/

// 	//box3d();
// 	//grid3d(1,1,"all");
	
// 	//qplclr((double *)aux, 3, 100);

//   	//color("green");
// 	//zscale(0,30);
// 	//shdmod ("3D", "symbols");

// 	//forma del objeto dinámico
//   	//curvx3(xray, yray, zray, n);

// 	//curv3d(xtri, ytri, ztri, ntri);
// 	//shdmod ("smooth", "surface"); 
//   	//iret = zbfini();
// 	//colray(zray, aux, n);
// 	//shdpat(10);
// 	//surshd(xray, n, yray, n, (double *)zmat)
// 	//curv3d(xtri, ytri, ztri, ntri);
// 	//vtx3d(xray, yray, zray, n, "curve");
// 	//surshd(xray, n, yray, n, (double *)aux1);
// 	//zbffin();
// 	//surshd(xray, n, yray, n, (double *)aux1);
// 	//shdmod("smooth","surface"); 
//   	//surfun(fun(zray),1,1,1,1);
// 	//shdpat(10);
// 	//curve3(mxray[0], myray[0], mzray[0], n[0]);
// 	//rlarea(xray, yray, n);

// 	//rlarea(xray, yray, n);
// 	//iret = zbfini();
// 	/*getmat(mxray[0], myray[0], mzray[0], n[0], (double *)zmat, nx, ny, 0.0, (int *)imat, (double *)wmat);
// 	surmat((double *)zmat, n[0], n[0], nx, ny);
// 	for (int i=0; i<nx; i++)
// 		for (int j=0; j<ny; j++)
// 			cout << zmat[i][j] << ", " << endl;

// 	cout << nx << ", " << ny << ", " << endl;
// 	cin.get();*/
// 	/*getmat(xray, yray, zray, n, zmat, n, n, 1, imat, wmat);
// 	surmat((double *)zmat, n, n, 1, 1);
// 	surshd(xray, n, yray, n, (double *)zmat);
// 	zbffin();*/
// 	//endgrf();

// 	//velocidad del robot en el espacio de 
// 	//velocidades con sus restricciones dinámicas
// 	double x[5], y[5];
// 	x[0] = w+t*a_w;
// 	y[0] = v;
// 	x[1] = w;
// 	y[1] = v+t*a_v;
// 	x[2] = w-t*a_w;
// 	y[2] = v;
// 	x[3] = w;
// 	y[3] = v-t*a_v;
// 	x[4] = w+t*a_w;
// 	y[4] = v;

// 	/*qplot(x[0],y[0],x[1],y[1]);
// 	qplot(x[1],y[1],x[2],y[2]);
// 	qplot(x[2],y[2],x[3],y[3]);
// 	qplot(x[3],y[3],x[4],y[4]);*/

// 	x[0] = w; y[0] = v;
// 	//qplsc(x[0],y[0],x[0],y[0]);

// 	//cout << "Dinamica: " << endl;
// 	//for (int i=0; i<5; i++){
// 	//	cout << "x: " << x[i] << ", y: " << y[i] << endl;
// 	//}
// 	selwin(id);
// 	endgrf();
// 	//cin.get();
// }

// void DibujaVSG2(int id, float xray[100], float yray[100], int n, double robot_v, double robot_w, double a_v, double a_w, double t,
// 		double dir_goal, Velocidad vel_goal, Tsc obj, double radio_obj, Line arriba, Line abajo, double vmin_adm,
// 		double vmax_adm, double wmin_adm, double wmax_adm, double wmax_dch, double wmax_izq){
// //muestra en la gráfica(2D) la forma del objeto dinámico

// 	int nray[2];
// 	float x[5], y[5];

// 	selwin(id);	
// 	//nochek();	//nochek is used to suppress the listing of points that lie outside of the axis scaling
// 	cout << "DibujaVSG2" << endl;
// 	name("Angular velocity","x");
// 	name("Linear Velocity","y");

// 	//para poder pintar sobre los limites de los ejes
// 	grace(1);	//grace -> defines a margin around axis systems where lines will be clipped
// 	color("white");
// 	graf(wmin_adm, wmax_adm, wmin_adm, 0.5, vmin_adm, vmax_adm, vmin_adm, 0.5);	
// 	/*//para que las lineas de grid aparezcan a rayas y no lisas
// 	nray[0] = 20;
// 	nray[1] = 15;
// 	myline(nray, 2);	//myline -> defines a global line style
// 	grid(1, 1);	//grid -> overlays a grid on an axis system
// 	nray[0] = 1;
// 	myline(nray, 1);
// 	*/
// 	dash();
// 	grid(1, 1);
// 	solid();
// 	//se delimita la zona de velocidades admisibles
// 	x[0] = wmax_dch; y[0] = 0;
// 	x[1] = wmax_dch; y[1] = vmax_adm;
// 	x[2] = wmax_izq; y[2] = vmax_adm;
// 	x[3] = wmax_izq; y[3] = 0;
// 	curve(x, y, 4);

// 	//color("yellow");
// 	setvlt("grey");	//setvlt -> selects a colour table; "grey" defines 256 grey scale colours(0->black, 255->white)
// 	setclr(100);	//setclr -> sets the foreground colour
// 	shdpat(16);
// 	//cout << "Objeto visible?: " << Distancia(obj.x, obj.y) - radio_obj << endl;
// 	if (Distancia(obj.x, obj.y) - radio_obj < 12)	// & (arriba.C/arriba.A > 0 | abajo.C/abajo.A > 0))	//CONDICION SOLO PARA SIMULACION
// 		rlarea(xray, yray, n);	//sombrea todo el area de la forma del objeto

// 	//se dibujan los rayos de cada una de las curvaturas en el VS	
// 	if (Distancia(obj.x, obj.y) - radio_obj < 12){	// & (arriba.C/arriba.A > 0 | abajo.C/abajo.A > 0)){	//CONDICION SOLO PARA SIMULACION 
// 		setclr(50);	//sets the foreground colour
// 		for (int i=0; i<n/2; i++){
// 			rline(xray[i], yray[i], xray[n-1-i], yray[n-1-i]);
// 		}
// 	}
// 	setvlt("rain");
// 	color("white");
// 	//restricciones dinámicas del robot en VS
// 	x[0] = robot_w+t*a_w; y[0] = robot_v;
// 	x[1] = robot_w; y[1] = robot_v+t*a_v;
// 	x[2] = robot_w-t*a_w; y[2] = robot_v;
// 	x[3] = robot_w; y[3] = robot_v-t*a_v;
// 	x[4] = robot_w+t*a_w; y[4] = robot_v;
// 	curve(x, y, 5);

// 	//cout << "Dinamica: " << endl;
// 	//for (int i=0; i<5; i++){
// 	//	cout << "x: " << x[i] << ", y: " << y[i] << endl;
// 	//}
// 	//posicion del robot en VS
// 	x[0] = robot_w; y[0] = robot_v;	
// 	linwid(12);	//linwid -> sets the line width
// 	rline(x[0], y[0], x[0], y[0]);
// 	linwid(1);
// 	//cout << "Robot VS: x: " << x[0] << ", y: " << y[0] << endl;

// 	//dibuja heading to goal
// 	x[0] = dir_goal; y[0] = vmin_adm;
// 	x[1] = dir_goal; y[1] = vmax_adm;
// 	color("red");
// 	linwid(4);
// 	curve(x, y, 2);
// 	//cout << "dir_goal VS:" << dir_goal << endl;
// 	linwid(1);

// 	/*R_goal en VS: mostrar el camino circular R_goal que lleva directo al goal en el VS*/
// 	//comando (v,w) que me lleva al goal
// 	x[0] = 0; y[0] = 0;
// 	x[1] = vel_goal.w; y[1] = vel_goal.v;
// 	color("magenta");
// 	curve(x, y, 2);
// 	//cout << "comando goal VS: w: " << vel_goal.w << ", v: " << vel_goal.v << endl;

// 	//mapeo del R_heading
// 	x[0] = 0; y[0] = 0;
// 	x[1] = dir_goal; y[1] = vmax_adm;
// 	color("red");
// 	curve(x, y, 2);
	
// 	//mapeamos R_media
// 	if (dir_goal > vel_goal.w){
// 		x[0] = 0; y[0] = 0;
// 		x[1] = vel_goal.w+(dir_goal-vel_goal.w)/2; y[1] = vmax_adm;
// 		color("white");
// 		curve(x, y, 2);		
// 	}else if(dir_goal == vel_goal.w){
// 		x[0] = 0; y[0] = 0;
// 		x[1] = vel_goal.w; y[1] = vmax_adm;
// 		color("white");
// 		curve(x, y, 2);		
// 	}else{
// 		x[0] = 0; y[0] = 0;
// 		x[1] = vel_goal.w-(vel_goal.w-dir_goal)/2; y[1] = vmax_adm;
// 		color("white");
// 		curve(x, y, 2);		
// 	}
// 	//cout << "R_media en VS: w->" << x[1] << ", v->" << y[1] << endl;
// 	selwin(id);
// }

// void DibujaVSG2Varios(int id, float mxray[][100], float myray[][100], int n[], int num_obj_din, double robot_v, double robot_w, double a_v, double a_w,
// 		double t, double dir_goal, Velocidad vel_goal, Tsc obj, double radio_obj, Line arriba, Line abajo, double vmin_adm,
// 		double vmax_adm, double wmin_adm, double wmax_adm, double wmax_dch, double wmax_izq){
// //muestra en la gráfica(2D) la forma del objeto dinámico

// 	int nray[2];
// 	float x[5], y[5];

// 	selwin(id);	
// 	cout << "DibujaVSG2Varios" << endl;
// 	//nochek();	//nochek is used to suppress the listing of points that lie outside of the axis scaling
// 	name("Angular velocity","x");
// 	name("Linear Velocity","y");

// 	//para poder pintar sobre los limites de los ejes
// 	grace(1);	//grace -> defines a margin around axis systems where lines will be clipped
// 	color("white");
// 	graf(wmin_adm, wmax_adm, wmin_adm, 0.5, vmin_adm, vmax_adm, vmin_adm, 0.5);	
// 	/*//para que las lineas de grid aparezcan a rayas y no lisas
// 	nray[0] = 20;
// 	nray[1] = 15;
// 	myline(nray, 2);	//myline -> defines a global line style
// 	grid(1, 1);	//grid -> overlays a grid on an axis system
// 	nray[0] = 1;
// 	myline(nray, 1);
// 	*/
// 	dash();
// 	grid(1, 1);
// 	solid();
// 	//se delimita la zona de velocidades admisibles
// 	x[0] = wmax_dch; y[0] = 0;
// 	x[1] = wmax_dch; y[1] = vmax_adm;
// 	x[2] = wmax_izq; y[2] = vmax_adm;
// 	x[3] = wmax_izq; y[3] = 0;
// 	curve(x, y, 4);

// 	//color("yellow");
// 	setvlt("grey");	//setvlt -> selects a colour table; "grey" defines 256 grey scale colours(0->black, 255->white)
// 	setclr(100);	//setclr -> sets the foreground colour
// 	shdpat(16);
// 	//cout << "Objeto visible?: " << Distancia(obj.x, obj.y) - radio_obj << endl;
// 	//cout << "objs_din: " << num_obj_din << ", num valores: " << n[num_obj_din-1] << endl;
// 	if (Distancia(obj.x, obj.y) - radio_obj < 12)	// & (arriba.C/arriba.A > 0 | abajo.C/abajo.A > 0))	//CONDICION SOLO PARA SIMULACION
// 		for (int i=0; i<num_obj_din; i++)
// 			rlarea(mxray[i], myray[i], n[i]);	//sombrea todo el area de la forma del objeto

// 	//se dibujan los rayos de cada una de las curvaturas en el VS	
// 	if (Distancia(obj.x, obj.y) - radio_obj < 12){	// & (arriba.C/arriba.A > 0 | abajo.C/abajo.A > 0)){	//CONDICION SOLO PARA SIMULACION 
// 		setclr(50);	//sets the foreground colour
// 		for (int i=0; i<num_obj_din; i++){
// 			for (int j=0; j<n[i]/2; j++){
// 				rline(mxray[i][j], myray[i][j], mxray[i][n[i]-1-j], myray[i][n[i]-1-j]);
// 			}
// 		}
// 	}
// 	setvlt("small");
// 	color("white");
// 	//restricciones dinámicas del robot en VS
// 	x[0] = robot_w+t*a_w; y[0] = robot_v;
// 	x[1] = robot_w; y[1] = robot_v+t*a_v;
// 	x[2] = robot_w-t*a_w; y[2] = robot_v;
// 	x[3] = robot_w; y[3] = robot_v-t*a_v;
// 	x[4] = robot_w+t*a_w; y[4] = robot_v;
// 	curve(x, y, 5);

// 	//cout << "Dinamica: " << endl;
// 	//for (int i=0; i<5; i++){
// 	//	cout << "x: " << x[i] << ", y: " << y[i] << endl;
// 	//}
// 	//posicion del robot en VS
// 	x[0] = robot_w; y[0] = robot_v;	
// 	linwid(12);	//linwid -> sets the line width
// 	rline(x[0], y[0], x[0], y[0]);
// 	linwid(1);
// 	//cout << "Robot VS: x: " << x[0] << ", y: " << y[0] << endl;

// 	//dibuja heading to goal
// 	x[0] = dir_goal; y[0] = vmin_adm;
// 	x[1] = dir_goal; y[1] = vmax_adm;
// 	color("red");
// 	linwid(8);
// 	curve(x, y, 2);
// 	//cout << "dir_goal VS:" << dir_goal << endl;
// 	linwid(1);

// 	/*R_goal en VS: mostrar el camino circular R_goal que lleva directo al goal en el VS*/
// 	//comando (v,w) que me lleva al goal
// 	x[0] = 0; y[0] = 0;
// 	x[1] = vel_goal.w; y[1] = vel_goal.v;
// 	color("magenta");
// 	curve(x, y, 2);
// 	//cout << "comando goal VS: w: " << vel_goal.w << ", v: " << vel_goal.v << endl;

// 	//mapeo del R_heading
// 	x[0] = 0; y[0] = 0;
// 	x[1] = dir_goal; y[1] = vmax_adm;
// 	color("red");
// 	curve(x, y, 2);
	
// 	//mapeamos R_media
// 	if (dir_goal > vel_goal.w){
// 		x[0] = 0; y[0] = 0;
// 		x[1] = vel_goal.w+(dir_goal-vel_goal.w)/2; y[1] = vmax_adm;
// 		color("white");
// 		curve(x, y, 2);		
// 	}else if(dir_goal == vel_goal.w){
// 		x[0] = 0; y[0] = 0;
// 		x[1] = vel_goal.w; y[1] = vmax_adm;
// 		color("white");
// 		curve(x, y, 2);		
// 	}else{
// 		x[0] = 0; y[0] = 0;
// 		x[1] = vel_goal.w-(vel_goal.w-dir_goal)/2; y[1] = vmax_adm;
// 		color("white");
// 		curve(x, y, 2);		
// 	}
// 	//cout << "R_media en VS: w->" << x[1] << ", v->" << y[1] << endl;
// 	selwin(id);
// }

// void DibujaVST(int id, float mxray[obst_max][comm_max], float myray[obst_max][comm_max], float mzray[obst_max][comm_max],
// 		int n[obst_max], int num_obj_din, double robot_v, double robot_w, double dinArr, double dinAbjo, double dinIzda,
// 		double dinDcha, double steer_w, Velocidad vel_goal, Velocidad dir_goal, double vmin, double vmax, double wright,
// 		double wleft, double vmax_r, double wleft_r, double wright_r,
// 		double av, double aw, double t, double steer_dir, double radio_goal){


// 	selwin(id);
// 	//nochek();	//nochek is used to suppress the listing of points that lie outside of the axis scaling
// 	cout << "DibujaVST" << endl;
// 	name("Angular velocity","x");
// 	name("Linear Velocity","y");
// 	name("Time","z");

// 	graf3d(wright, wleft, wright, 0.5, vmin, vmax, vmin, 0.5, 0, 5, 0, 1);
// 	//dash();
// 	grid(1, 1);
// 	//solid();

// 	//setvlt("grey");	//setvlt -> selects a colour table; "grey" defines 256 grey scale colours(0->black, 255->white)
// 	//setclr(100);	//setclr -> sets the foreground colour
// 	//shdpat(16);
// 	for (int i=0; i<num_obj_din; i++){
// 		//setclr(100+45*i);
// 		curv3d(mxray[i], myray[i], mzray[i], n[i]);
// 		crvqdr(mxray[i], myray[i], mzray[i], n[i]);
// 	}
// }

void DibujaVS(int id, std::vector<std::vector<double>> &mxray,
			  std::vector<std::vector<double>> &myray, std::vector<int> &n,
			  int num_obj_din, double robot_v,
		double robot_w, double dinArr, double dinAbjo, double dinIzda, double dinDcha,
		Velocidad cmdGoal, Velocidad dir_goal, Velocidad vGoal, boundsVS bounds,
		double av, double aw, double t, bool video, int iteracion, std::vector<std::vector<int>> positions, bool debug){
	//muestra en la gráfica(3D) la forma del objeto dinámico
	//dir_goal es el goal_media
	//steer_w es la direccion al goal sujeta a los limites de velocidad máximos

	// if (video) {
	// 	filopt( "NONE", "SEPARATOR");
	// 	filopt("LONG", "NUMBER");
	// 	filopt("6", "DIGITS");
	// 	setfil("output/vs.png");
	// 	cout << "Inicializa" << endl;
	// 	Inicializar(true);
	// }
	int nray[2];
	float x[5], y[5];

	//nochek();	//nochek is used to suppress the listing of points that lie outside of the axis scaling
	name("Angular velocity","x");
	name("Linear Velocity","y");

	//para poder pintar sobre los limites de los ejes
	grace(1);	//grace -> defines a margin around axis systems where lines will be clipped
	color("white");
	//graf(wleft, wright, wleft, -0.5, vmin, vmax, vmin, 0.5);
	graf(bounds.wmax_right, bounds.wmax_left, bounds.wmax_right, 0.5, bounds.vlim_min, bounds.vlim_max, bounds.vlim_min, 0.5);
	//(15/03/2016): cambio el tamaño de los ejes en función de la velocidad (v,w) que lleva al goal en el siguiente time step
	double steer_w = vGoal.w > bounds.wmax_left ? bounds.wmax_left : vGoal.w;
	steer_w = vGoal.w < 0 ? (vGoal.w <  bounds.wmax_right ?  bounds.wmax_right : vGoal.w) : (vGoal.w > bounds.wmax_left ? bounds.wmax_left : vGoal.w);
	// std::string it = "Iteration " + std::to_string(iteracion);
	std::string it = "";
	rlmess(it.c_str(), 0, 0);
	// char eti[10] = "Ag";
	// std::string idS = std::to_string((id-2));
	// strcat(eti, idS.c_str());
	// //rlmess(eti, -1, -0.25);
	htitle(33);
	titlin(it.c_str(), 1);
	title();

	dash();
	grid(1, 1);
	solid();
	//se delimita la zona de velocidades admisibles
	x[0] = bounds.wmax_right; y[0] = 0;
	x[1] = bounds.wmax_right; y[1] = bounds.vlim_max;
	x[2] = bounds.wmax_left; y[2] = bounds.vlim_max;
	x[3] = bounds.wmax_left; y[3] = 0;
	curve(x, y, 4);

	setvlt("grey");	//setvlt -> selects a colour table; "grey" defines 256 grey scale colours(0->black, 255->white)
	setclr(100);	//setclr -> sets the foreground colour
	shdpat(16); float xrayD[obst_max*comm_max]; float yrayD[obst_max*comm_max];
	for (int i=0; i<(int)mxray.size(); i++){
		setclr(100+45*i);
		std::vector<double> datax = mxray[i];
		std::vector<double> datay = myray[i];
		for (int j=0; j<(int)datax.size(); j++){
			xrayD[j] = datax[j];
			yrayD[j] = datay[j];
		}
		rlarea(xrayD, yrayD, n[i]);	//sombrea todo el area de la forma del objeto
		//se dibujan los rayos de cada una de las curvaturas en el VS
		setclr(50);	//sets the foreground colour
		for (int j=0; j<n[i]/2; j++){
			rline(xrayD[j], yrayD[j], xrayD[n[i]-1-j], yrayD[n[i]-1-j]);
		}
	}

	linwid(12);
	setvlt("small");

	color("black");
	linwid(18);
	//limitaciones en el VS para el robot, velocidades admisibles para el robot
	x[0] = bounds.wmax_right; y[0] = 0;
	x[1] = bounds.wmax_right; y[1] = bounds.vlim_max;
	x[2] = bounds.wmax_left; y[2] = bounds.vlim_max;
	x[3] = bounds.wmax_left; y[3] = 0;
	curve(x, y, 4);

	//triangle differential-drive constraints in VS
	linwid(12);
	double max_w_plot = bounds.ComputeMaxW_V(0);
	rline(0, bounds.ComputeMaxV_W(0), bounds.ComputeMaxW_V(0), bounds.ComputeMaxV_W(max_w_plot));
	rline(0, bounds.ComputeMaxV_W(0), -bounds.ComputeMaxW_V(0), bounds.ComputeMaxV_W(max_w_plot));

	color("green");
	//restricciones dinámicas del robot en VS
	if (robot_v < bounds.vlim_max){	//habria que hacer lo mismo para el resto de limites (w left y right)
		y[1] = dinArr;
	}else{
		y[1] = dinArr + av*t;
	}
	x[0] = dinDcha; y[0] = robot_v;
	x[1] = robot_w; // y[1] = dinArr;
	x[2] = dinIzda; y[2] = robot_v;
	x[3] = robot_w; y[3] = dinAbjo;
	x[4] = dinDcha; y[4] = robot_v;
	curve(x, y, 5);

	//posicion del robot en VS
	x[0] = robot_w; y[0] = robot_v;
	linwid(12);	//linwid -> sets the line width
	rline(x[0], y[0], x[0], y[0]);
	linwid(1);

	//dibuja heading to goal
	// x[0] = steer_w; y[0] = bounds.vlim_min;
	// x[1] = steer_w; y[1] = bounds.vlim_max;
	// DibujaRecta(id, x, y, 2, "red", 16);

	/*R_goal en VS: mostrar el camino circular R_goal que lleva directo al goal en el VS*/
	//comando (v,w) que me lleva al goal
	x[0] = 0; y[0] = 0;
	x[1] = cmdGoal.w; y[1] = cmdGoal.v;
	DibujaRecta(id, x, y, 2, "magenta", 12);

	//(15/03/2016) Pintamos el punto (v,w) que lleva al goal en el siguiente time step en el VS
	x[0] = vGoal.w; y[0] = vGoal.v;
	color("black");
	linwid(20);	//linwid -> sets the line width
	rline(x[0], y[0], x[0], y[0]);
	//curve(x,y,2);

	//mapeamos R_media
	x[0] = 0; y[0] = 0;
	x[1] = dir_goal.w; y[1] = dir_goal.v;
	//DibujaRecta(id, x, y, 2, "white", 12);
	DibujaRecta(id, x, y, 2, "green", 12);

	//mapeo del R_heading, steering_w
	// x[0] = 0; y[0] = 0;
	// x[1] = steer_w; y[1] = bounds.vlim_max;

	// andrew
	// int first;
	// height(50);
	// linwid(10);
	// for (int i=0; i<(int)mxray.size(); i++) {
	// 	std::vector<double> datax = mxray[i];
	// 	std::vector<double> datay = myray[i];
	// 	for (int j=0; j<(int)datax.size(); j++){
	// 		xrayD[j] = datax[j];
	// 		yrayD[j] = datay[j];
	// 	}
	// 	if (debug) {
	// 		for (int j=0; j<n[i]/2; j++) {
	// 			if (j == 0 || positions[i][j] != positions[i][j+1] || j == n[i]/2 -1) {
	// 				if (j==0) {
	// 					first = j;
	// 					color("ORANGE");
	// 					rline(xrayD[j], yrayD[j], xrayD[n[i]-1-j], yrayD[n[i]-1-j]);
	// 				} else if (j == n[i]/2 -1) {
	// 					color("GREEN");
	// 					rlnumb(positions[i][j], 0, (xrayD[j] + xrayD[first])/2, (yrayD[j] + yrayD[first])/2);
	// 					color("ORANGE");
	// 					rline(xrayD[j], yrayD[j], xrayD[n[i]-1-j], yrayD[n[i]-1-j]);
	// 					first = j + 1;
	// 				} else {
	// 					color("GREEN");
	// 					rlnumb(positions[i][j], 0, (xrayD[j] + xrayD[first])/2, (yrayD[j] + yrayD[first])/2);
	// 					color("ORANGE");
	// 					rline(xrayD[j+1], yrayD[j+1], xrayD[n[i]-1-j + 1], yrayD[n[i]-1-j + 1]);
	// 					first = j + 1;
	// 				}
	// 			}
	// 		}
	// 	}
	// }
	// linwid(1);
	// height(36);
	// cout << "disfin" << endl;
	// if (video) disfin();
}

void Selecciona(double& wcursor, double& vcursor, double wmax_adm, double wmin_adm, double vmax_adm, double vmin_adm){
	
	int nx, ny;
	// cout << "SELECCIONAAAAAAAAAAAAAAAAAAAAAAAAAAA" << endl;
	graf(wmin_adm, wmax_adm, wmin_adm, 0.5, vmin_adm, vmax_adm, vmin_adm, 0.5);
	// cout << "Hola" << endl;
	csrmod("read", "pos");
	csrpos(&nx, &ny);
	wcursor = xinvrs(nx);	//convert plot coordinates to user coordinates
	vcursor = yinvrs(ny);	//convert plot coordinates to user coordinates

	//si pulsamos fuera de la gráfica, asignamos los valores máximos o mínimos
	if (wcursor > wmax_adm)	//wmax_izq es un valor positivo
		wcursor = wmax_adm;
	if (wcursor < wmin_adm)
		wcursor = wmin_adm;
	if (vcursor > vmax_adm)
		vcursor = vmax_adm;
	if (vcursor < vmin_adm)
		vcursor = vmin_adm;
}
