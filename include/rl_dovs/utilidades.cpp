#include "TData.h"
#include <stdio.h>
#include <stdlib.h>
#include "utilidades.h"
#include "complex.h"
#include <cmath>

#include <iostream>

struct Velocidad;

double Distancia (double x, double y){

	return (std::sqrt(x*x+y*y));
}

void SolDosRectas(Line r1, Line r2, Tsc &raiz, int &sol){
/* calcula la interseccion de dos rectas; 
si son paralelas devuelve 0 en sol y 1 en caso contrario */

	if (r1.GetA()*r2.GetB()-r2.GetA()*r1.GetB() == 0.0)
		sol = 0;
	else{
		raiz.x = (r1.GetB()*r2.GetC()-r2.GetB()*r1.GetC())/(r1.GetA()*r2.GetB()-r2.GetA()*r1.GetB());
		raiz.y = (r1.GetC()*r2.GetA()-r2.GetC()*r1.GetA())/(r1.GetA()*r2.GetB()-r2.GetA()*r1.GetB());
		sol = 1;
        if (std::abs(raiz.x) < 1e-5) raiz.x = 0;
        if (std::abs(raiz.y) < 1e-5) raiz.y = 0;
	}
}

void SolCirculoRecta(double a, double b, double c, double x, double y, double r, double raiz[][2], int& n){
	// los resultados de esta funcion son:
	//	raiz[][2] contiene las raices encontradas, n es el numero de soluciones encontradas

	//Esta funcion permite calcular los puntos solucion
	//de la interseccion entre una recta (Objeto) y una circunferencia (Robot).
	//a,b,c: parametros de la ecuacion de la recta
	//x,y,r: parametro de la ecuacion geometrica de la circunferencia

	n = 0;
	if (b!=0){
		// coeficientes de la ecuacion de segundo orden:
			double A = 1 + (a/b)*(a/b);
     		double B = 2*(a/b)*(c/b) + 2*(a/b)*y - 2*x;
     		double C = x*x + (c/b)*(c/b) + 2*y*(c/b) + y*y - r*r;

		/*float A = 1.0 + ((a/b)*(a/b));
		float B = 2.0*(a/b)*(c/b) + 2.0*(a/b)*y - 2.0*x;
		//double B = 2.0*(a/b)*(r-c);
		float C = x*x + ((c/b)*(c/b)) + 2.0*y*(c/b) + y*y - r*r;
		*/

		//double C = c*c + 2.0*c*r;
		//double A = 1.0 + ((b/a)*(b/a));
		//double B = 2.0 * (((c*b)/(a*a)) - r);
		//double C = (c/a)*(c/a);

			double u = B*B;
			double v = A*C;
			double w = 4*v;

			if (std::abs((B*B)-(4*A*C)) < 1e-3){
        		raiz[0][0] = (-B/(2*A));	// despejando de la ecuacion cuadratica
        		raiz[0][1] = ((-c-a*raiz[0][0])/b);	// solucion de la ecuacion de la recta
     			//raiz[0][1] = (double)(-B/2.0*A);	// solucion de la ecuacion de la recta
     			//raiz[0][0] = (double)((-c-b*raiz[0][1])/a);	// solucion de la ecuacion de la recta
        		n = 1;	// hay 1 sola solucion
    		}
    		else if (B*B > 4*A*C){	// Dos raices!
    			//cout << "2 raices" << endl;
    			raiz[0][0] = (-B + std::sqrt(B*B-4*A*C))/(2*A);	// despejando de la ecuacion cuadratica
        		raiz[0][1] = (-c-a*raiz[0][0])/b;	// solucion de la ecuacion de la recta
				//raiz[0][1] = (double)((-B+(float)sqrtf(B*B-4.0*A*C))/(2.0*A));	// despejando de la ecuacion cuadratica
				//raiz[0][0] = (double)((-c-b*raiz[0][1])/a);	// solucion de la ecuacion de la recta

        		raiz[1][0] = (-B-std::sqrt(B*B-4*A*C))/(2*A);
        		raiz[1][1] = (-c-a*raiz[1][0])/b;
				//raiz[1][1] = (double)((-B-(float)sqrtf(B*B-4.0*A*C))/(2.0*A));
				//raiz[1][0] = (double)((-c-b*raiz[0][1])/a);	// solucion de la ecuacion de la recta
				n = 2;
			}
			//else if (B*B < 4*A*C){ //cout << "no raices" << endl;
			//	n = 0;	//Ninguna raiz, no hay solucion!!
			//}
	}
	else if (a!=0){
		// coeficientes de la ecuacion de segundo orden:
		float A = 1;
		float B = -2*y;
		float C = (c*c)/(a*a) + 2*c/a*x + x*x + y*y - r*r;

		if (std::abs((B*B)-(4*A*C)) < 1e-3){	// Solo una raiz!
			raiz[0][0] = (-c/a);
			raiz[0][1] = (-B/(2*A));
			n = 1;
		}
		else if (B*B > 4*A*C){	// Dos raices!
			raiz[0][0] = (-c/a);
			raiz[0][1] = ((-B + std::sqrt(B*B-4*A*C))/(2*A)); // despejando de la ecuacion cuadratica

			raiz[1][0] = (-c/a);
			raiz[1][1] = ((-B - std::sqrt(B*B-4*A*C))/(2*A)); // despejando de la ecuacion cuadratica
			n = 2;
		}
		//else if (B*B < 4*A*C) n = 0; // Ninguna raiz, no hay solucion!!
	}

	for (int i=0; i<n; i++){
		if (std::abs(raiz[i][0]) < 1e-5) raiz[i][0] = 0;
		if (std::abs(raiz[i][1]) < 1e-5) raiz[i][1] = 0;
	}
}

void CalLimSup(double R, Tpf m45, Tpf m135, Tpf mns135, Tpf mns45, int& maximo, int& err){
/*
 funcion que calcula el comando
 de velocidad limite superior en algunas situaciones para 
 las cuales el objeto se encuentre muy cerca del robot y 
 que se haga necesario verificar estos radios de curvatura
*/

	double m1, m2, delta_x1, delta_x2, delta_y1, delta_y2;
	double raices1[2][2], raices2[2][2];
	int n1, n2;
	double A1, B1, C1, A2, B2, C2;

	/* con las 4 esquinas en el sistema de referencia del robot calculamos
	 las 2 pendientes y encontramos las ecuaciones de las 2 rectas*/
	// pendiente m1
	if ((m45.x - mns45.x>=-0.001) & (m45.x - mns45.x<=0.001))
	    m1=0.0;
	else
	    m1=(m45.y - mns45.y)/(m45.x - mns45.x);
	
	// pendiente m2
	if ((m135.x - mns135.x>=-0.001) & (m135.x - mns135.x<=0.001))
	    m2=0.0;
	else
	    m2=(m135.y - mns135.y)/(m135.x - mns135.x);
	
	delta_x1 = m45.x - mns45.x;
	delta_x2 = m135.x - mns135.x;
	delta_y1 = m45.y - mns45.y;
	delta_y2 = m135.y - mns135.y;
	
	// recta x=0, ecuacion del eje 'y'
	if ((((m45.x<=0.001 && m45.x>=-0.001) && (mns45.x<=0.001 && mns45.x>=-0.001)) &&
		((m45.y>=0.0 && mns45.y<=0.0) || (m45.y<=0.0 && mns45.y>=0.0))) ||
		(((m135.x<=0.001 && m135.x>=-0.001) && (mns135.x<=0.001 && mns135.x>=-0.001)) &&
		((m135.y>=0.0 && mns135.y<=0.0) || (m135.y<=0.0 && mns135.y>=0.0)))){
   		err=1;
	// recta x=a, ecuacion de una recta paralela al eje 'y'
	}else if ( (delta_x1>=-0.001 && delta_x1<=0.001 && ((m45.y>=0.0 && mns45.y<=0.0) || (m45.y<=0.0 && mns45.y>=0.0))) ||
		(delta_x2>=-0.001 && delta_x2<=0.001 && ((m135.y>=0.0 && mns135.y<=0.0) || (m135.y<=0.0 && mns135.y>=0.0))) ){
   		// ecuacion de la recta
   		A1=1.0; B1=0.0; C1=-m45.x;
   		A2=1.0; B2=0.0; C2=-m135.x;
   		// se verifica la condicion de interseccion entre la recta y la curva
   		SolCirculoRecta(A1, B1, C1, 0, R, R, raices1, n1);
		SolCirculoRecta(A2, B2, C2, 0, R, R, raices2, n2);
		err = 0;
   		if (R==0.0) maximo=1;
   		else{
   			if (n1>1 || n2>1) maximo=0;
   		    else maximo=1;
   		}
	}
	// recta y=0, ecuacion del eje 'x'
	else if (((m45.y<=0.001 && m45.y>=-0.001) && (mns45.y<=0.001 && mns45.y>=-0.001) &&
			((m45.x>=0.0 && mns45.x<=0.0) || (m45.x<=0.0 && mns45.x>=0.0))) ||
			((m135.y<=0.001 && m135.y >= -0.001) && (mns135.y<=0.001 && mns135.y>=-0.001) &&
			((m135.x>=0.0 && mns135.x<=0.0) || (m135.x<=0.0 && mns135.x>=0.0)))){
		err = 1;
	// recta y=b, recta paralela al eje x en b
	}else if ( (delta_y1>=-0.001 && delta_y1<=0.001 && ((m45.x>=0.0 && mns45.x<=0.0) || (m45.x<=0.0 && mns45.x>=0.0))) ||
			(delta_y2>=-0.001 && delta_y2<=0.001 && ((m135.x>=0.0 && mns135.x<=0.0) || (m135.x<=0.0 && mns135.x>=0.0))) ){
		// ecuacion de la recta
	   	A1=0.0; B1=1.0; C1=-m45.y;
	   	A2=0.0; B2=1.0; C2=-m135.y;
	   	// se verifica la condicion de interseccion entre la recta y la curva
		SolCirculoRecta(A1, B1, C1, 0, R, R, raices1, n1);
		SolCirculoRecta(A2, B2, C2, 0, R, R, raices2, n2);
		err=0;
	   	if (R==0.0) maximo=1;
	   	else{
	   		if (n1>=1 || n2>=1) maximo=0;
	   		else maximo=1;
	   	}
	}
	// recta y=m*x, linea a traves del origen
	else if (((m1*m45.x+m45.y >= -0.001) && (m1*m45.x+m45.y<=0.001)) ||
		((m2*m135.x+m135.y>=-0.001) && (m2*m135.x+m135.y<=0.001))){
		err=1;
	// recta y=m*x+b, linea con pendiente e intersecto
	}else{	// cuando se llega a esta condicion nada se evalua porque no es necesario!
   		A1=-m1; B1=1.0; C1=m1*m45.x-m45.y;
   		A2=-m2; B2=1.0; C2=m2*m135.x-m135.y;
   		// se verifica la condicion de interseccion entre la recta y la curva
		SolCirculoRecta(A1, B1, C1, 0, R, R, raices1, n1);
		SolCirculoRecta(A2, B2, C2, 0, R, R, raices2, n2);
   		err=0;
	   	if (R==0.0) maximo=1;
	   	else{
	   		if (n1>=1 || n2>=1) maximo=0;
	   		else maximo=1; 
	   	}
	}   		
}

void ObtenerComandoMaximo(double angulo, double vmax_adm, double w_max_left, double w_max_right, double &vmax, double &wmax){
/* obtiene el comando (v,w)max asociado a un radio de curvatura (angulo de curvatura) 
que quiera evaluar el robot en un momento determinado*/

	// se consideran todas las regiones a las cuales pertenence el "angulo de curvatura" que describe el robot
	if (angulo == 0.0){
		vmax = 0.0; wmax = w_max_left;
	}
	else if (angulo == M_PI/2){
		vmax = vmax_adm; wmax = 0.0;
	}
	else if (angulo == M_PI){
		vmax = 0.0; wmax = w_max_right;
	}
	else if (0.0 < angulo && angulo < std::atan2(vmax_adm,w_max_left)){
		wmax = w_max_left; vmax = std::tan(angulo)*wmax;
	}
	else if (angulo == std::atan2(vmax_adm,w_max_left)){
		wmax = w_max_left; vmax = vmax_adm;
	}
	else if (std::atan2(vmax_adm,w_max_left) < angulo && angulo < M_PI/2){
		vmax = vmax_adm; wmax = vmax/std::tan(angulo);
	}
	else if (M_PI/2< angulo && angulo < std::atan2(vmax_adm,w_max_right)){
		vmax = vmax_adm; wmax = vmax/std::tan(angulo);
	}
	else if (angulo == std::atan2(vmax_adm,w_max_right)){
		vmax = vmax_adm; wmax = w_max_right;
	}
	else if (std::atan2(vmax_adm,w_max_right) < angulo && angulo < M_PI){
		wmax = w_max_right; vmax = std::tan(angulo)*wmax;
	}
	//cout << "comando maximo: " << vmax << ", " << wmax << endl;
}

/*void LineTan(double radio, Recta rin, double direccion, double esquinas_up[4][2], double esquinas_down[4][2],
		Tpf robot_up, Tpf robot_down, Tpf& sol, Recta& rsol, int& err){

 //función que encuentra la ecuacion de la recta tangente a la ecuacion de la circunferencia,
 //conocidos el radio R de la circunferencia y la pendiente de la recta tangente obtenida
 //de otra recta paralela(conocidos dos puntos).
 //Se quiere ademas encontrar cual de las 2 rectas solucion, esto se logra
 //evaluando cada punto solucion respecto del robot

 //direccion: es el angulo de direccion del vector de vel. del obj.

	double raices1[2][2], raices2[2][2], raices[2][2];
	int n1, n2;
	Tpf r1, r2;
	Tpfp p1,p2;
	double A1, B1, C1, A2, B2, C2;

	if (radio==0.0) err = 1;
	else{
		err=0;				
		if (rin.B>=-0.001 && rin.B <=0.001){	// recta con pendiente Inf
			// ecuacion 1:
			A1=1.0; B1=0.0; C1=radio;
			SolCirculoRecta(A1, B1, C1, 0.0, radio, radio, raices1, n1);
			// ecuacion_2:
			A2=1.0; B2=0.0; C2=-radio;
			SolCirculoRecta(A2, B2, C2, 0.0, radio, radio, raices2, n2);
			r1.x = raices1[0][0]; r1.y = raices1[0][1];
			r2.x = raices2[0][0]; r2.y = raices2[0][1];
			// hacemos cambio de coordenadas
			car2pol(&r1, &p1);
			car2pol(&r2, &p2);
		
			if (radio>0.0){	// radios positivos
				if (p1.t <= p2.t){
					rsol.A = 1.0; rsol.B = 0.0; rsol.C = C1;
					sol = r1;
				}
				else{
					rsol.A = 1.0; rsol.B = 0.0; rsol.C = C2;
					sol = r2;	
				} 
			}
			else{	//radios negativos, sin el radio +-inf.
				if (p1.t < p2.t){
					rsol.A = 1.0; rsol.B = 0.0; rsol.C = C2;
					sol = r2;
				}
				else{
					rsol.A = 1.0; rsol.B = 0.0; rsol.C = C1;
					sol = r1;	
				} 
			}
		}else if (rin.A >= -0.001 && rin.A <= 0.001){	// recta con pendiente cero
			// solo rotacion, tenemos una posible solucion
			err = 0;
			A1=0.0; B1=1.0; C1=-2.0*radio;
			SolCirculoRecta(A1, B1, C1, 0.0, radio, radio, raices1, n1);
			sol.x = raices1[0][0]; sol.y = raices1[0][1];			
			rsol.A = 0.0; rsol.B = 1.0; rsol.C = C1;
		}else{	// recta con pendiente diferente de cero e Inf
			double m = -(double)((double)rin.A/(double)rin.B);
			A1=-m; B1=1.0; C1=-(double)((1.+ std::sqrt(1.0+m*m))*radio);
			SolCirculoRecta(A1, B1, C1, 0.0, radio, radio, raices1, n1);
			A2=-m; B2=1.0; C2=-(double)((1.- std::sqrt(1.0+m*m))*radio);


			SolCirculoRecta(A2, B2, C2, 0.0, radio, radio, raices2, n2);
			r1.x = raices1[0][0]; r1.y = raices1[0][1];
			r2.x = raices2[0][0]; r2.y = raices2[0][1];
			// hacemos cambio de coordenadas
			car2pol(&r1, &p1);
			car2pol(&r2, &p2);
			// tratamos con radios positivos
			if (n1 != 0 && n2 != 0){
				if (radio>0.0){
					if (p1.t <= p2.t) sol = r1;
					else sol = r2;

					// (0,PI/2)
					if ((0.0 < direccion && direccion < M_PI/2) && esquinas_up[1][0]>robot_up.x){ 	// obj. delante! -> mas135.x > rob.x
						sol.x = raices1[0][0]; sol.y = raices1[0][1];
						rsol.A = A1; rsol.B = B1; rsol.C = C1;
					}else if ((0.0 < direccion && direccion < M_PI/2) && esquinas_up[0][0]<robot_up.x){	// obj. detras! -> mas45.x < rob.x
						sol.x = raices2[0][0]; raices2[0][1] = sol.y;
						rsol.A = A2; rsol.B = B2; rsol.C = C2;
					}// (0,-PI/2)
					else if ((-M_PI/2 < direccion && direccion < 0.0) && esquinas_up[1][0]>robot_up.x){	// obj. delante!
						sol.x = raices1[0][0]; sol.y = raices1[0][1];
						rsol.A = A1; rsol.B = B1; rsol.C = C1;
					}else if ((-M_PI/2 < direccion && direccion < 0.0) && esquinas_up[0][0]<robot_up.x){	// obj. detras!
						sol.x = raices2[0][0]; sol.y = raices2[0][1];
						rsol.A = A2; rsol.B = B2; rsol.C = C2;
					}else if ((M_PI/2 < direccion && direccion < M_PI) && esquinas_down[1][0]>robot_down.x){	// obj. delante
						sol.x = raices2[0][0]; sol.y = raices2[0][1];
						rsol.A = A2; rsol.B = B2; rsol.C = C2;
					}else if ((M_PI/2 < direccion && direccion < M_PI) && esquinas_down[0][0]<robot_down.x){	// obj. detras
						sol.x = raices1[0][0]; sol.y = raices1[0][1];
						rsol.A = A1; rsol.B = B1; rsol.C = C1;
					}else if ((-M_PI < direccion && direccion < -M_PI/2) && esquinas_down[1][0]>robot_down.x){	// obj. delante
						sol.x = raices1[0][0]; sol.y = raices1[0][1];
						rsol.A = A1; rsol.B = B1; rsol.C = C1;
					}else if ((-M_PI < direccion && direccion < -M_PI/2) && esquinas_down[0][0]<robot_down.x){	// obj. detras
						sol.x = raices2[0][0]; sol.y = raices2[0][1];
						rsol.A = A2; rsol.B = B2; rsol.C = C2;
					}//else: alfa es 0, +-pi, +-pi/2, rectas del objeto con pendient inf y 0, casos ya considerados
				}
				else{
					//se consideran todos los radios negativos! (se ha excluido del analisis el radio=-Inf
					//dependiendo de alfa(angulo de direccion del objeto), las sol. deben ser organizadas
					//de manera diferente!
					if (p1.t < p2.t) sol = r2;
					else sol = r1;
					// (0,PI/2)
					if ((0.0 < direccion && direccion < M_PI/2) && esquinas_up[1][0]>robot_up.x){ // obj. delante! -> mas135.x > rob.x
						sol.x = raices1[0][0]; sol.y = raices1[0][1];
						rsol.A = A1; rsol.B = B1; rsol.C = C1;
					}else if ((0.0 < direccion && direccion < M_PI/2) && esquinas_up[0][0]<robot_up.x){ // obj. detras! -> mas45.x < rob.x
						sol.x = raices2[0][0]; sol.y = raices2[0][1];
						rsol.A = A2; rsol.B = B2; rsol.C = C2;
					}// (0,-PI/2)
					else if ((-M_PI/2 < direccion && direccion < 0.0) && esquinas_up[1][0]>robot_up.x){	// obj. delante!
						sol.x = raices1[0][0]; sol.y = raices1[0][1];
						rsol.A = A1; rsol.B = B1; rsol.C = C1;
					}else if ((-M_PI/2 < direccion && direccion < 0.0) && esquinas_up[0][0]<robot_up.x){	// obj. detras!
						sol.x = raices2[0][0]; sol.y = raices2[0][1];
						rsol.A = A2; rsol.B = B2; rsol.C = C2;
					}
					else if ((M_PI/2 < direccion && direccion < M_PI) && esquinas_down[0][0]<robot_down.x){	//  obj. detras
						sol.x = raices1[0][0]; sol.y = raices1[0][1];
						rsol.A = A1; rsol.B = B1; rsol.C = C1;
					}else if ((M_PI/2 < direccion && direccion < M_PI) && esquinas_down[1][0]>robot_down.x){	// obj. delante
						sol.x = raices2[0][0]; sol.y = raices2[0][1];
						rsol.A = A2; rsol.B = B2; rsol.C = C2;
					}
					else if ((-M_PI < direccion && direccion < -M_PI/2) && esquinas_down[0][0]<robot_down.x){	// obj. detras
						sol.x = raices1[0][0]; sol.y = raices1[0][1];
						rsol.A = A1; rsol.B = B1; rsol.C = C1;
					}else if ((-M_PI < direccion && direccion < -M_PI/2) && esquinas_down[1][0]>robot_down.x){	// obj. delante
						sol.x = raices2[0][0]; sol.y = raices2[0][1];
						rsol.A = A2; rsol.B = B2; rsol.C = C2;
					}//else: alfa es 0, +-pi, +-pi/2, rectas del objeto con pendient inf y 0, casos ya considerados

				}
			}else{
				//cout << "no ha detectado las raices" << endl;
				err = 1;
			}
		}
	}
}
//*/

void CalculaVelocidades(double x, double y, double r, double t, Velocidad &vel){
// encuentra el comando de velocidad correspondiente a partir de los parametros de entrada

	double arco = 0;
	double v1, v2, w1, w2;
	double beta_1, beta_2;

	if (r == 10000){	//radio infinito
		vel.w = 0.0; vel.v = x/t;
	}else{
		//se encuentra el arco(r), que es el seno del desplazamiento angular (beta) que debe realizar el robot
		if (r == 0.0){ vel.v = 0.0; vel.w = 0.0;}
		else{
			arco = x/r; // arco = sin(beta) = x/r
			if (arco <-1.0) arco = -1.0;	//valores entre [-1, 1]
			else if (arco >1.0) arco = 1.0;

			// es necesario conocer si los radios de curvatura evaluados son positivos o negativos
			// porque de acuerdo con esto la velocidad w puede ser positiva o negativa
			if (r >= 0.0 ){	// los radios positivos representan giros a la izquierda
				/* beta puede tomar 2 valores diferentes cuyo seno es el mismo, por tanto,
					 se requiere que los dos angulos que den sean positivos para que sean consecuentes
					 con la forma incremental en que se van dando a lo largo del tiempo*/
				////////////////////////
				//beta = atan2(2*x*y,x*x-y*y) => w = beta/t, v = w*r
				////////////////////////
				if (arco >= 0.0){
					beta_1 = std::asin(arco);
					beta_2 = (M_PI-beta_1);
				}else{
					beta_1 = 2.0*M_PI + std::asin(arco);
					beta_2 = M_PI - std::asin(arco);
				}
				// se encuentran los comandos asociados a cada uno de los betas encontrados
				w1 = beta_1/t; v1 = r*w1;
				w2 = beta_2/t; v2 = r*w2;

			}else{	// radios negativos representan giros a la derecha
				if (arco>0.0){
					beta_1 = -2.0*M_PI + std::asin(arco);
					beta_2 = -M_PI - std::asin(arco);	//beta_2 = -2.0*M_PI + (M_PI - std::asin(arco));
				}else{
					beta_1 = std::asin(arco);
					beta_2 = -M_PI - beta_1;	//beta_2 = (M_PI-beta_1)-(2.0*M_PI);
				}
				//se encuentran los comandos asociados a cada uno de los betas encontrados
				w1 = beta_1/t; v1 = r*w1;
				w2 = beta_2/t; v2 = r*w2;
			}
			// evaluamos las soluciones para ver su consistencia,comprobamos las soluciones con la componente en y
			if ((y/r -1.0 + std::cos(beta_1)>=-0.1) && (y/r -1.0 + std::cos(beta_1)<=0.1)){
				vel.w = w1; vel.v = v1;
			}
			else{
				vel.w = w2; vel.v = v2;
			}
		}
	}
}

void DireccionMovimiento(double alfa, double vo, double xo, double yo, int& direction){
/* función que devuelve 1 si el objeto se acerca al robot, 0 si se aleja del
robot y -1 si el objeto se está quieto; cálculos en el sistema de referencia del robot
*/
	double delta_t = 0.15; 	//150 ms, se escoge cualquier valor de tiempo
	double x1 = xo;
	double y1 = yo;

	double x2=xo + (vo*std::cos(alfa)*delta_t);
	double y2=yo + (vo*std::sin(alfa)*delta_t);

	double dist_1 = std::sqrt(x1*x1 + y1*y1);
	double dist_2 = std::sqrt(x2*x2 + y2*y2);
	
	if (dist_2 > dist_1)	// el objeto se aleja
	    direction=0;
	else if (dist_2 < dist_1) // el objeto se acerca
		direction=1;
	else if ((dist_2-dist_1) > -0.1 && (dist_2-dist_1) < 0.1)	//el objeto esta quieto
		direction=-1;
}

void LineTanR2(double radio, Line r, double angulo, int hacia_rob, Tpf& sol){
/* Devuelve el punto tangente entre un radio de curvatura dado y una recta
paralela a 'r'
Se tiene en cuenta si el objeto se desplaza hacia el robot o en direccion contraria a él
*/
	if (std::abs(r.GetA()) <= 1e-5){	//recta con pendiente 0
		double A, B, C;
		double raiz[2][2];
		int n;
		
		A = 0.0; B = 1.0; C = -2.0*radio;
		SolCirculoRecta(A, B, C, 0, radio, radio, raiz, n);
		sol.x = raiz[0][0]; sol.y = raiz[0][1];
	}else{	//recta con pendiente inf o diferente de cero e inf.   
		double A1, B1, C1, A2, B2, C2;
		double raiz1[2][2]; double raiz2[2][2];
		int n1, n2;
		Tsc sistema;
		Tpf sol1, sol2;

		if (std::abs(r.GetB()) <= 1e-5){ // recta con pendiente inf.
			A1 = 1.0; B1 = 0.0; C1 = radio;
			A2 = 1.0; B2 = 0.0; C2 = -radio;
		}else{	//recta con pendiente diferente de cero e inf.
			double m = -(r.GetA()/r.GetB());

			A1 = -m; B1 = 1.0; C1 = -((1.0 + std::sqrt(1.0+m*m))*radio);
			A2 = -m; B2 = 1.0; C2 = -((1.0 - std::sqrt(1.0+m*m))*radio);
		}
		SolCirculoRecta(A1, B1, C1, 0.0, radio, radio, raiz1, n1);
		SolCirculoRecta(A2, B2, C2, 0.0, radio, radio, raiz2, n2);
			
		if (hacia_rob == 0){
			sistema.x = 0.0; sistema.y = 0.0; sistema.tita = angulo;
			transfor_inversa_p(raiz1[0][0], raiz1[0][1], &sistema, &sol1);
			transfor_inversa_p(raiz2[0][0], raiz2[0][1], &sistema, &sol2);
			if (sol1.x >= sol1.y){
			//if (sol1.x >= sol2.x){
				sol.x = raiz1[0][0]; sol.y = raiz1[0][1];
			}else{
				sol.x = raiz2[0][0]; sol.y = raiz2[0][1];
			}
		}else{	//hacia_mi = 1
		// el criterio es la distancia del pto tan a la recta de la parte frontal del obj movil
			double dis1, dis2;
			Tpf p1, p2;

			p1.x = raiz1[0][0]; p1.y = raiz1[0][1];
			p2.x = raiz2[0][0]; p2.y = raiz2[0][1];
			dis1 = r.Distance(p1);
			dis2 = r.Distance(p2);
			if (dis1 >= dis2){
				sol.x = raiz1[0][0]; sol.y = raiz1[0][1];
			}else{
				sol.x = raiz2[0][0]; sol.y = raiz2[0][1];
			}
		}
	}
}

Circumference ComputeCircumference(Tpf p0, Tpf p1, Tpf p2){
//Computes the center and radius of a circumference

	double A, B, C, x, y, r;

	double B1 = (p0.x*p0.x*p2.x - p0.x*p0.x*p1.x + p0.y*p0.y*p2.x - p0.y*p0.y*p1.x - p1.x*p1.x*p2.x + p1.x*p1.x*p0.x - p1.y*p1.y*p2.x + p1.y*p1.y*p0.x + p2.x*p2.x*p1.x - p2.x*p2.x*p0.x + p2.y*p2.y*p1.x - p2.y*p2.y*p0.x);
	double den = (p0.y*p1.x - p0.y*p2.x + p1.y*p2.x - p1.y*p0.x + p2.y*p0.x - p2.y*p1.x);
	if (den == 0)
		std::cout << "Denominador nulo" << std::endl;
	assert( den != 0 );
	B = B1/den;
	assert( (p2.x - p1.x) != 0);
	A = (-p2.x*p2.x + p1.x*p1.x - p2.y*p2.y + p1.y*p1.y - B*(p2.y - p1.y))/(p2.x - p1.x);
	C = -p0.x*p0.x - p0.y*p0.y - A*p0.x - B*p0.y;

	x = -A/2; y = -B/2; r = sqrt(x*x + y*y - C);

	return Circumference(x,y,r);
}

void RadioPtoTanCircunf(double xc, double yc, double r, double& r1, Tpf& p1, double& r2, Tpf& p2){

	r1 = (xc*xc + yc*yc - r*r)*(-yc + r)/(2*(r*r - yc*yc));
	r2 = (xc*xc + yc*yc - r*r)*(-yc - r)/(2*(r*r - yc*yc));

	double b = -xc*(xc*xc + yc*yc - r*r)/((yc-r1)*(yc-r1)) + 2*xc*r1/(yc-r1);
	double a = 1 + (xc*xc/((yc-r1)*(yc-r1)));
	p1.x = -b/(2*a); p1.y = (-2*xc*p1.x + xc*xc + yc*yc - r*r)/(2*(yc-r1));

	b = -xc*(xc*xc + yc*yc - r*r)/((yc-r2)*(yc-r2)) + 2*xc*r2/(yc-r2);
	a = 1 + (xc*xc/((yc-r2)*(yc-r2)));
	p2.x = -b/(2*a); p2.y = (-2*xc*p2.x + xc*xc + yc*yc - r*r)/(2*(yc-r2));
}

void RectaDosCircunf(Circumference c1, Circumference c2, Line& r){
//Ecuacion general de la recta que pasa por los puntos de intersección entre las dos circunferencias

	double A = 2*(c2.x() - c1.x());
	double B = 2*(c2.y() - c1.y());
	double C = (c1.y()*c1.y() - c2.y()*c2.y()) + (c1.x()*c1.x() - c2.x()*c2.x()) + (c2.r()*c2.r() - c1.r()*c1.r());

    r = Line(A, B, C);
    /*
	if (r.GetB() != 0){
		r.m = -r.A/r.B;
		r.b = -r.C/r.B;
	}else if (r.A != 0){
		r.m = 1;
		r.b = -r.C/r.A;
	}
     */
}

void CircunfTanRecta(Circumference cobj, Line r, double robj, std::vector<std::pair<double,double>>& centers){
//Calcula los centros de las cuatro circunferencias tangentes a la recta r

	double hc = cobj.x(); double kc = cobj.y(); double rc = cobj.r();
	//double m = r.m; double b = r.b;
	double hc2 = hc*hc; double kc2 = kc*kc; double rc2 = rc*rc;
	//double m2 = m*m; double m3 = m2*m; double m4 = m3*m; double b2 = b*b;
	double robj2 = robj*robj;

	double xsol, ysol;
	std::vector<double> x, y;
	if (r.GetB() != 0){

        double m = -r.GetA()/r.GetB(); double b = -r.GetC()/r.GetB();
        double m2 = m*m; double m3 = m2*m; double m4 = m3*m; double b2 = b*b;
        double term_1 = std::sqrt(m2+1.0);

		//Soluciones para las coordenadas x
		double data = -m2*robj2-hc2*m2+rc2*m2+2*hc*m*kc+2*robj*term_1*hc*m-2*hc*m*b+2*b*kc-2*robj*term_1*kc-b2+rc2-kc2-robj2+2*robj*term_1*b;
		if (data < 0) return;

		xsol = -1.0/4.0*(-4*kc*m3-4*m2*hc+4*b*m3-4*m*kc-4*hc+4*m*b)/(2*m2+m4+1) + m*robj/term_1 + 1/(m2+1)*std::sqrt(-m2*robj2-hc2*m2+rc2*m2+2*hc*m*kc+2*robj*term_1*hc*m-2*hc*m*b+2*b*kc-2*robj*term_1*kc-b2+rc2-kc2-robj2+2*robj*term_1*b);
		x.push_back(xsol);
		xsol = -1.0/4.0*(-4*kc*m3-4*m2*hc+4*b*m3-4*m*kc-4*hc+4*m*b)/(2*m2+m4+1) + m*robj/term_1 - 1/(m2+1)*std::sqrt(-m2*robj2-hc2*m2+rc2*m2+2*hc*m*kc+2*robj*term_1*hc*m-2*hc*m*b+2*b*kc-2*robj*term_1*kc-b2+rc2-kc2-robj2+2*robj*term_1*b);
		x.push_back(xsol);
		xsol = -1.0/4.0*(-4*kc*m3-4*m2*hc+4*b*m3-4*m*kc-4*hc+4*m*b)/(2*m2+m4+1) - m*robj/term_1 + 1/(m2+1)*std::sqrt(-m2*robj2-hc2*m2+rc2*m2+2*hc*m*kc-2*robj*term_1*hc*m-2*hc*m*b+2*b*kc+2*robj*term_1*kc-b2+rc2-kc2-robj2-2*robj*term_1*b);
		x.push_back(xsol);
		xsol = -1.0/4.0*(-4*kc*m3-4*m2*hc+4*b*m3-4*m*kc-4*hc+4*m*b)/(2*m2+m4+1) - m*robj/term_1 - 1/(m2+1)*std::sqrt(-m2*robj2-hc2*m2+rc2*m2+2*hc*m*kc-2*robj*term_1*hc*m-2*hc*m*b+2*b*kc+2*robj*term_1*kc-b2+rc2-kc2-robj2-2*robj*term_1*b);
		x.push_back(xsol);

		//Soluciones para las coordenadas y
		data = -robj2*m2-m2*hc2+m2*rc2-2*robj*m*term_1*hc+2*hc*kc*m-2*m*b*hc-2*robj*term_1*b-b2-kc2+rc2+2*kc*b-robj2+2*robj*term_1*kc;
		if (data < 0) return;

		ysol = -1.0/4.0*(-4*m3*hc-4*b-4*m*hc-4*m2*b-4*m4*kc-4*m2*kc)/(1+m4+2*m2) + robj/term_1 + m/(m2+1)*std::sqrt(-robj2*m2-m2*hc2+m2*rc2-2*robj*m*term_1*hc+2*hc*kc*m-2*m*b*hc-2*robj*term_1*b-b2-kc2+rc2+2*kc*b-robj2+2*robj*term_1*kc);
		y.push_back(ysol);
		ysol = -1.0/4.0*(-4*m3*hc-4*b-4*m*hc-4*m2*b-4*m4*kc-4*m2*kc)/(1+m4+2*m2) - robj/term_1 + m/(m2+1)*std::sqrt(-robj2*m2-m2*hc2+m2*rc2+2*robj*m*term_1*hc+2*hc*kc*m-2*m*b*hc+2*robj*term_1*b-b2-kc2+rc2+2*kc*b-robj2-2*robj*term_1*kc);
		y.push_back(ysol);
		if (m != 0){
			ysol = -1.0/4.0*(-4*m3*hc-4*b-4*m*hc-4*m2*b-4*m4*kc-4*m2*kc)/(1+m4+2*m2) + robj/term_1 - m/(m2+1)*std::sqrt(-robj2*m2-m2*hc2+m2*rc2-2*robj*m*term_1*hc+2*hc*kc*m-2*m*b*hc-2*robj*term_1*b-b2-kc2+rc2+2*kc*b-robj2+2*robj*term_1*kc);
			y.push_back(ysol);
			ysol = -1.0/4.0*(-4*m3*hc-4*b-4*m*hc-4*m2*b-4*m4*kc-4*m2*kc)/(1+m4+2*m2) - robj/term_1 - m/(m2+1)*std::sqrt(-robj2*m2-m2*hc2+m2*rc2+2*robj*m*term_1*hc+2*hc*kc*m-2*m*b*hc+2*robj*term_1*b-b2-kc2+rc2+2*kc*b-robj2-2*robj*term_1*kc);
			y.push_back(ysol);
		}

		//Comprobamos las soluciones con el discriminante
		double cte = (1+m2)*robj2;
		for (int i=0; i<x.size(); i++){
			double xval = x[i];
			double xval2 = x[i]*x[i];
			for (int j=0; j<y.size(); j++){
				double yval = y[j];
				double yval2 = y[j]*y[j];
				double discriminante = cte - 2*m*b*xval + 2*xval*m*yval - (yval2) - m2*(xval2) + 2*b*yval - b2;

				if (std::abs(discriminante) < 1e-5){
					centers.push_back(std::make_pair(xval, yval));
				}
			}
		}

	}else{

        double m = 1;
        double b = (r.GetA() != 0) ? -r.GetC()/r.GetA() : 0;
		b = -b;

		double data = rc*rc+2*hc*robj+2*b*hc-2*b*robj-b*b-hc*hc-robj*robj;
		if (data < 0) return;

		xsol = b + robj;
		ysol = kc + std::sqrt(rc*rc+2*hc*robj+2*b*hc-2*b*robj-b*b-hc*hc-robj*robj);
		centers.push_back(std::make_pair(xsol, ysol));
		ysol = kc - std::sqrt(rc*rc+2*hc*robj+2*b*hc-2*b*robj-b*b-hc*hc-robj*robj);
		centers.push_back(std::make_pair(xsol, ysol));

		xsol = b - robj;
		ysol = kc + std::sqrt(rc*rc-2*hc*robj+2*b*hc+2*b*robj-b*b-hc*hc-robj*robj);
		centers.push_back(std::make_pair(xsol, ysol));
		ysol = kc - std::sqrt(rc*rc-2*hc*robj+2*b*hc+2*b*robj-b*b-hc*hc-robj*robj);
		centers.push_back(std::make_pair(xsol, ysol));

	}
}