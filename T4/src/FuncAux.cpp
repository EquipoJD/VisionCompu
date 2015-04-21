/*
 * FuncAux.cpp
 *
 *  Created on: 08/03/2015
 *      Author: Jaime. David.
 */

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdio.h>
#include "FuncAux.h"

using namespace std;
using namespace cv;

/* Variables globales */


/*
 * Función mostrarImagen del Trabajo 4 de Visión por computador
 * Muestra una imagen "imagen" en un marco con titulo "tituloFoto" y
 * si el booleano espera es true, añade la posibilidad de no avanzar
 * en el código si no se pulsa una tecla
 */

void mostrarImagen(String tituloFoto, Mat imagen, bool espera) {
	namedWindow(tituloFoto, WINDOW_AUTOSIZE);
	imshow(tituloFoto, imagen);
	if (espera) {
		while (waitKey(1) == -1) {
			//No avanza hasta que el individuo pulse una tecla
		}
	}
}
