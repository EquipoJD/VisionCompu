/*
 * FuncAux.h
 *
 *  Created on: 26/02/2015
 *      Author: Jaime. David.
 */

#ifndef FUNCAUX_H_
#define FUNCAUX_H_

bool isHorizontal(float radian, float desviacion);
bool isVertical(float radian, float desviacion);
void mostrarImagen(std::string tituloFoto, cv::Mat imagen, bool espera);
cv::Mat calcularPuntoFuga(cv::Mat image, cv::Mat grad, cv::Mat orientacion, bool correccion,
		int anchuraFranja, bool verVotantes);

#endif /* FUNCAUX_H_ */
