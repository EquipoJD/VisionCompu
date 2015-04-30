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

void funcionPrincipal(int tipoCaptura, int detector, int matcher, int tipo){
	if(detector==2 and matcher==1){
		//Orb no puede ir con Flann
		matcher=0;
	}
	if(tipoCaptura==0){
		panoramaDisco(detector, matcher,tipo);
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
	int tipoPanorama = 0;

	//0 = SURF
	//1 = SIFT
	//2 = ORB
	int tipoDetector = 1;

	//0 = Brute force
	//1 = Flann Based
	int tipoMatcher = 1;

	//0 = objeto
	//1 = escena
	int objEsc = 1;

	funcionPrincipal(tipoPanorama,tipoDetector,tipoMatcher,objEsc);
	return 0;
}

