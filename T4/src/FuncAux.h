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
double calcularAnchura(std::vector<cv::Point2f> im1,
		std::vector<cv::Point2f> im2);
double calcularAltura(std::vector<cv::Point2f> im1,
		std::vector<cv::Point2f> im2);
int testingPanorama();
std::vector<cv::DMatch> getGoodMatches(int matcherType, int detectorType,
		std::vector<cv::KeyPoint> keypoints_object,
		std::vector<cv::KeyPoint> keypoints_scene, cv::Mat descriptors_object,
		cv::Mat descriptors_scene, cv::Mat image1, cv::Mat image2, bool log);
std::vector<cv::Mat> getDescriptors(int detectorType, cv::Mat gray_image1,
		cv::Mat gray_image2, std::vector<cv::KeyPoint> keypoints_object,
		std::vector<cv::KeyPoint> keypoints_scene, bool log);
std::vector<std::vector<cv::KeyPoint> > getKeyPoints(cv::Mat gray_image1,
		cv::Mat gray_image2, int detectorType, bool log);
cv::Mat getPanorama(cv::Mat nueva, cv::Mat estatica, bool log, int detectorType,
		int matcherType);
void panoramaDisco(int detector, int matcher,int tipo);
void panoramaPortatilTecla(int detector, int matcher);
void panoramaPortatilAutomatico(int detector, int matcher);

#endif /* FUNCAUX_H_ */
