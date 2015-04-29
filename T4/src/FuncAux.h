/*
 * FuncAux.h
 *
 *  Created on: 26/02/2015
 *      Author: Jaime. David.
 */

#ifndef FUNCAUX_H_
#define FUNCAUX_H_

void mostrarImagen(std::string tituloFoto, cv::Mat imagen, bool espera);
double calcularCorreccionX(std::vector<cv::Point2f> im1,
		std::vector<cv::Point2f> im2);
double calcularCorreccionY(std::vector<cv::Point2f> im1,
		std::vector<cv::Point2f> im2);
double calcularAnchura(std::vector<cv::Point2f> im1, std::vector<cv::Point2f> im2);
double calcularAltura(std::vector<cv::Point2f> im1, std::vector<cv::Point2f> im2);
int testingPanorama();
cv::Mat getPanorama(cv::Mat nueva, cv::Mat estatica, bool log);
void panoramaPortatilTecla();
void panoramaPortatilAutomatico();

#endif /* FUNCAUX_H_ */
