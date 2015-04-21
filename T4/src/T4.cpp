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
#include <opencv2/features2d/features2d.hpp>
#include "FuncAux.h"

using namespace std;
using namespace cv;

/* Variables globales */

/*
 * Función main del Trabajo 4 de Visión por Computador
 * Gestiona el funcionamiento principal del programa
 */

int main() {
	//Lee la imagen
	Mat image;
	image = imread("efectomenendes.jpg", CV_LOAD_IMAGE_GRAYSCALE);
	if (!image.data) {
		cout << "Could not open or find the image" << std::endl;
	} else {
		SURF surf_extractor(5.0e3);
		mostrarImagen("foto", image, true);
	}
	return 0;
}

