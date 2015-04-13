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
#include <iostream>
#include <stdio.h>
#include <string>
#include "FuncAux.h"

using namespace std;
using namespace cv;

/* Variables globales */

/*
 * Función main del Trabajo 3 de Visión por Computador
 * Gestiona el funcionamiento principal del programa
 */

int main() {
	Mat image;
	int ksize = 3;
	int sigmax = 0;
	int sigmay = 0;
	int prof = CV_32F;
	int xorder = 0;
	int yorder = 0;
	int escala = -1;
	int delta = 0;

	//Lee la imagen
	image = imread("poster.pgm", CV_LOAD_IMAGE_GRAYSCALE);
	if (!image.data) {
		cout << "Could not open or find the image" << std::endl;
	} else {
		//Booleano para especificar que el programa espere en cada
		//foto o no
		bool espera = false;

		//Imagen original
		mostrarImagen("Original", image, espera);

		//Reducción de ruido con filtro Gausiano
		Mat gauss;
		GaussianBlur(image, gauss, Size(ksize, ksize), sigmax, sigmay,
				BORDER_DEFAULT);
		mostrarImagen("Filtro Gausiano", gauss, espera);

		//Gradiente horizontal
		Mat grad_x, grad_y;
		Mat abs_grad_x, abs_grad_y;
		xorder = 1;
		Sobel(gauss, grad_x, prof, xorder, yorder, ksize, escala, delta,
				BORDER_DEFAULT);

		//Gradiente vertical
		xorder = 0;
		yorder = 1;
		Sobel(gauss, grad_y, prof, xorder, yorder, ksize, escala, delta,
				BORDER_DEFAULT);

		//Aplicar corrección para mostrar por pantalla el gradiente
		Mat mostrar_grad_x, mostrar_grad_y;
		convertScaleAbs(grad_x, mostrar_grad_x, 0.5, 128);
		convertScaleAbs(grad_y, mostrar_grad_y, 0.5, 128);
		mostrarImagen("Gradiente x", mostrar_grad_x, espera);
		mostrarImagen("Gradiente y", mostrar_grad_y, espera);

		//Convertir los resultados parciales a CV_8U
		convertScaleAbs(grad_x, abs_grad_x);
		convertScaleAbs(grad_y, abs_grad_y);

		//Modulo
		Mat grad;
		magnitude(grad_x, grad_y, grad);

		//Corrección para mostrar
		Mat gradm;
		addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, gradm);
		mostrarImagen("Modulo", gradm, espera);

		//Orientación
		Mat orientacion;
		phase(grad_x, grad_y, orientacion, false);

		//Corrección para mostrar la orientación
		Mat dest1;
		convertScaleAbs(orientacion, dest1, 128 / M_PI, 0);
		mostrarImagen("Orientacion", dest1, espera);

		Mat fuga = calcularPuntoFuga(image, grad, orientacion, true, 100, true);
		mostrarImagen("Fuga", fuga, true);

		// Abrir cámara
		VideoCapture cap(0);
		Mat frame;
		// Comprobar que la cámara está abierta
		if (!cap.isOpened())
			return -1;

		// Ventana para la foto
		namedWindow("Vision por Computador - T3", 1);

		while (waitKey(1) == -1) {
			cap >> frame; // saca una foto con la camara
			cvtColor(frame, frame, CV_BGR2GRAY);

			//Reducción de ruido con filtro Gausiano
			Mat step1;
			GaussianBlur(frame, step1, Size(ksize, ksize), sigmax, sigmay,
					BORDER_DEFAULT);

			//Gradiente horizontal
			Mat step2x, step2y;
			xorder = 1;
			Sobel(step1, step2x, prof, xorder, yorder, ksize, escala, delta,
					BORDER_DEFAULT);

			//Gradiente vertical
			xorder = 0;
			yorder = 1;
			Sobel(step1, step2y, prof, xorder, yorder, ksize, escala, delta,
					BORDER_DEFAULT);

			//Modulo
			Mat step3;
			magnitude(step2x, step2y, step3);

			//Orientación
			Mat step4;
			phase(step2x, step2y, step4, false);

			//Fuga
			Mat fugaCamara = calcularPuntoFuga(frame, step3, step4, true, 50,
					true);
			imshow("Vision por Computador - T3", fugaCamara);
		}

		return 0;
	}
}

