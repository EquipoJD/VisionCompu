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
 * Funci�n isHorizontal del Trabajo 3 de Visi�n por computador
 * Devuelve true si una medida en radianes representa un �ngulo que forma
 * una l�nea horizontal o pr�xima a horizontal
 */

bool isHorizontal(float radian, float desviacion) {
	bool resultado = false;
	if (sin(radian) <= desviacion && sin(radian) >= -desviacion) {
		resultado = true;
	}
	return resultado;
}

/*
 * Funci�n isVertical del Trabajo 3 de Visi�n por computador
 * Devuelve true si una medida en radianes representa un �ngulo que forma
 * una l�nea vertical o pr�xima a vertical
 */

bool isVertical(float radian, float desviacion) {
	bool resultado = false;
	if (cos(radian) <= desviacion && cos(radian) >= -desviacion) {
		resultado = true;
	}
	return resultado;
}

/*
 * Funci�n mostrarImagen del Trabajo 3 de Visi�n por computador
 * Muestra una imagen "imagen" en un marco con titulo "tituloFoto" y
 * si el booleano espera es true, a�ade la posibilidad de no avanzar
 * en el c�digo si no se pulsa una tecla
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

/*
 * Funci�n mostrarImagen del Trabajo 3 de Visi�n por computador
 * Muestra una imagen "imagen" en un marco con titulo "tituloFoto" y
 * si el booleano espera es true, a�ade la posibilidad de no avanzar
 * en el c�digo si no se pulsa una tecla
 */

Mat calcularPuntoFuga(Mat image, Mat grad, Mat orientacion, bool correccion,
		int anchuraFranja, bool verVotantes) {
	//L�nea del horizonte
	int horiz = image.rows * 0.5;
	if (anchuraFranja <= 1 || !correccion) {
		//Si la franja es de tama�o 1 o menor, no estamos corrigiendo nada
		correccion = false;
		anchuraFranja = 1;
	}
	//Vector de Votos
	vector<int> votos = vector<int>(image.cols * anchuraFranja);
	//Matriz resultante
	Mat fuga = image.clone();
	cvtColor(fuga, fuga, CV_GRAY2BGR);
	int puntos = 0;

	//Multiplicaci�n hecha para acelerar c�lculos
	int mitadAnchura = anchuraFranja * 0.5;

	//Bucle principal de b�squeda del punto de fuga
	for (int i = 0; i < image.rows; i++) {
		for (int j = 0; j < image.cols; j++) {
			float tita = orientacion.at<float>(i, j);
			//Filtramos qu� puntos pueden votar y cuales no
			if (grad.at<float>(i, j) > 50 && !isHorizontal(tita, 0.05)
					&& !isVertical(tita, 0.05)) {

				double coseno = cos(tita);
				double seno = sin(tita);
				double p = j * coseno + i * seno;

				//Esta parte solo es para cuando queremos corregir el c�lculo
				//del punto de fuga en im�genes torcidas
				if (correccion) {
					int iteracionk = 0;
					for (int k = -mitadAnchura; k < mitadAnchura; k++) {
						int puntoVotado = (p - (seno * (horiz + k))) / coseno;
						if (puntoVotado < image.cols and puntoVotado > 0) {
							int indiceVector = puntoVotado
									+ (image.cols * iteracionk);
							votos[indiceVector] = votos[indiceVector] + 1;
							if (verVotantes) {
								line(fuga, Point(j, i), Point(j, i),
										Scalar(0, 255, 0), 1, 8);
							}
						} else {
							if (verVotantes) {
								line(fuga, Point(j, i), Point(j, i),
										Scalar(0, 0, 255), 1, 8);
								if (false && puntos % 20 == 0 && j<image.rows) {
									line(fuga, Point(j, i),
											Point(puntoVotado, horiz + k),
											Scalar(0, 255, 255), 1, 8);
								}
							}
						}
						iteracionk++;
					}
				} else {
					//Esta parte es la normal, sin correcci�n
					int puntoVotado = (p - (seno * horiz)) / coseno;
					if (puntoVotado < image.cols and puntoVotado > 0) {
						votos[puntoVotado] = votos[puntoVotado] + 1;
						if (verVotantes) {
							line(fuga, Point(j, i), Point(j, i),
									Scalar(0, 255, 0), 1, 8);
						}
					} else {
						if (verVotantes) {
							line(fuga, Point(j, i), Point(j, i),
									Scalar(0, 0, 255), 1, 8);
						}
					}
				}
			}
		}
		puntos++;
	}

	//B�squeda del m�ximo valor
	int indicex = 0;
	int indicey = 0;
	int maxValue = votos[0];
	for (int i = 0; i < image.cols * anchuraFranja; i++) {
		if (votos[i] > maxValue) {
			maxValue = votos[i];
			indicex = i - image.cols * (i / image.cols);
			indicey = (i / image.cols) - mitadAnchura;
		}
	}

	//Dibujo del punto de fuga
	Point p1, p2, p3, p4;
	p1.x = indicex - 5;
	p1.y = horiz + indicey - 5;
	p2.x = indicex + 5;
	p2.y = horiz + indicey + 5;
	line(fuga, p1, p2, Scalar(0, 0, 0), 2, 8);
	p3.x = indicex - 5;
	p3.y = horiz + indicey + 5;
	p4.x = indicex + 5;
	p4.y = horiz + indicey - 5;
	line(fuga, p3, p4, Scalar(0, 0, 0), 2, 8);

	return fuga;
}
