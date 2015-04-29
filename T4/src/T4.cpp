/*
 * T3.cpp
 *
 *  Created on: 08/03/2015
 *      Author: Jaime. David.
 */

#define _USE_MATH_DEFINES // for C++
#include <cmath>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <stdio.h>
#include <string>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "FuncAux.h"

using namespace std;
using namespace cv;

/* Variables globales */

void funcionPrincipal(int tipoCaptura, int detector, int matcher){
	if(tipoCaptura==0){
		panoramaDisco(detector, matcher);
		while(waitKey(1)==-1);
	}
	else if(tipoCaptura==1){
		panoramaPortatilTecla(detector, matcher);
	}
	else if(tipoCaptura==2){
		panoramaPortatilAutomatico(detector, matcher);
	}
}

/*
 * Función main del Trabajo 4 de Visión por Computador
 * Gestiona el funcionamiento principal del programa
 */

int main() {
	//0 = panorama con fotos de disco
	//1 = panorama con fotos hechas con pulsaciones de teclas
	//2 = panorama con fotos en tiempo real
	funcionPrincipal(0,0,0);
	return 0;
}

