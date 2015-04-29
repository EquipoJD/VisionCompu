/*
 * FuncAux.cpp
 *
 *  Created on: 08/03/2015
 *      Author: Jaime. David.
 */

#include <stdio.h>
#include <iostream>

#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;

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

/*
 * Función calcularCorreccionX del trabajo 4 de Visión por computador
 * En base a unos vectores de puntos que representan las esquinas de las dos
 * imágenes que van a formar un panorama, calcula cuál es la corrección en X
 * que va a tener dicho panorama
 */

double calcularCorreccionX(vector<Point2f> im1, vector<Point2f> im2) {
	double correccionXIm1 = min(min(min(im1[0].x, im1[1].x), im1[2].x),
			im1[3].x);
	correccionXIm1 = abs(min(0.0, correccionXIm1 - 1));
	double correccionXIm2 = min(min(min(im2[0].x, im2[1].x), im2[2].x),
			im2[3].x);
	correccionXIm2 = abs(min(0.0, correccionXIm2 - 1));
	return max(correccionXIm2, correccionXIm1);
}

/*
 * Función calcularCorreccionY del trabajo 4 de Visión por computador
 * En base a unos vectores de puntos que representan las esquinas de las dos
 * imágenes que van a formar un panorama, calcula cuál es la corrección en Y
 * que va a tener dicho panorama
 */

double calcularCorreccionY(vector<Point2f> im1, vector<Point2f> im2) {
	double correccionYIm1 = min(min(min(im1[0].y, im1[1].y), im1[2].y),
			im1[3].y);
	correccionYIm1 = abs(min(0.0, correccionYIm1 - 1));
	double correccionYIm2 = min(min(min(im2[0].y, im2[1].y), im2[2].y),
			im2[3].y);
	correccionYIm2 = abs(min(0.0, correccionYIm2 - 1));
	return max(correccionYIm1, correccionYIm2);
}

/*
 * Función calcularAnchura del trabajo 4 de Visión por computador
 * En base a unos vectores de puntos que representan las esquinas de las dos
 * imágenes que van a formar un panorama, calcula cuál es la anchura de la
 * imagen que se va a formar comparando las esquinas 1 y 3 de ambas imágenes
 */

double calcularAnchura(vector<Point2f> im1, vector<Point2f> im2) {
	return max(max(im1[1].x, im1[3].x), max(im2[1].x, im2[3].x));
}

/*
 * Función calcularAltura del trabajo 4 de Visión por computador
 * En base a unos vectores de puntos que representan las esquinas de las dos
 * imágenes que van a formar un panorama, calcula cuál es la altura de la
 * imagen que se va a formar comparando las esquinas 2 y 3 de ambas imágenes
 */

double calcularAltura(vector<Point2f> im1, vector<Point2f> im2) {
	return max(max(im1[2].y, im1[3].y), max(im2[2].y, im2[3].y));
}

/*
 * Función testingPanorama del trabajo 4 de Visión por computador
 * NO BORRAR AUNQUE NO SE USE, USA FLANN MATCHER, SE PUEDE USAR PARA
 * COMPARAR
 */

int testingPanorama() {
	//Lee la imagen
	Mat image1, image2;
	image1 = imread("panorama2.jpg", CV_LOAD_IMAGE_GRAYSCALE);
	image2 = imread("panorama1.jpg", CV_LOAD_IMAGE_GRAYSCALE);
	//image3 = imread("testratong.jpg", CV_LOAD_IMAGE_GRAYSCALE);

// Load the images
	Mat gray_image1;
	Mat gray_image2;
	// Convert to Grayscale
	gray_image1 = imread("panorama2.jpg", CV_LOAD_IMAGE_GRAYSCALE);
	gray_image2 = imread("panorama1.jpg", CV_LOAD_IMAGE_GRAYSCALE);

	imshow("first image", image1);
	imshow("second image", image2);

	if (!gray_image1.data || !gray_image2.data) {
		std::cout << " --(!) Error reading images " << std::endl;
		return -1;
	}

//-- Step 1: Detect the keypoints using SURF Detector
	int minHessian = 400;

	SurfFeatureDetector detector(minHessian);

	std::vector<KeyPoint> keypoints_object, keypoints_scene;

	detector.detect(gray_image1, keypoints_object);
	detector.detect(gray_image2, keypoints_scene);

	//-- Step 2: Calculate descriptors (feature vectors)
	SurfDescriptorExtractor extractor;

	Mat descriptors_object, descriptors_scene;

	extractor.compute(gray_image1, keypoints_object, descriptors_object);
	extractor.compute(gray_image2, keypoints_scene, descriptors_scene);

	//-- Step 3: Matching descriptor vectors using FLANN matcher
	FlannBasedMatcher matcher;
	std::vector<DMatch> matches;
	matcher.match(descriptors_object, descriptors_scene, matches);

	double max_dist = 0;
	double min_dist = 100;

	//-- Quick calculation of max and min distances between keypoints
	for (int i = 0; i < descriptors_object.rows; i++) {
		double dist = matches[i].distance;
		if (dist < min_dist)
			min_dist = dist;
		if (dist > max_dist)
			max_dist = dist;
	}

	printf("-- Max dist : %f \n", max_dist);
	printf("-- Min dist : %f \n", min_dist);

	//-- Use only "good" matches (i.e. whose distance is less than 3*min_dist )
	std::vector<DMatch> good_matches;

	for (int i = 0; i < descriptors_object.rows; i++) {
		if (matches[i].distance < 3 * min_dist) {
			good_matches.push_back(matches[i]);
		}
	}
	std::vector<Point2f> obj;
	std::vector<Point2f> scene;

	for (unsigned int i = 0; i < good_matches.size(); i++) {
		//-- Get the keypoints from the good matches
		obj.push_back(keypoints_object[good_matches[i].queryIdx].pt);
		scene.push_back(keypoints_scene[good_matches[i].trainIdx].pt);
	}

	// Find the Homography Matrix
	Mat H = findHomography(obj, scene, CV_RANSAC);
	imshow("homography", H);

	//Esquinas de las fotos para calcular la nueva Homography Matrix
	vector<Point2f> im1, im2, im1c, im2c, im1f, im2f;
	im1.push_back(Point(0, 0));
	im1.push_back(Point(image1.cols, 0));
	im1.push_back(Point(0, image1.rows));
	im1.push_back(Point(image1.cols, image1.rows));

	im2.push_back(Point(0, 0));
	im2.push_back(Point(image2.cols, 0));
	im2.push_back(Point(0, image2.rows));
	im2.push_back(Point(image2.cols, image2.rows));

	perspectiveTransform(im1, im1c, H);
	perspectiveTransform(im2, im2c, H);

	double correccionX = calcularCorreccionX(im1c, im2c);
	double correccionY = calcularCorreccionY(im1c, im2c);

	Mat newH = Mat::eye(3, 3, H.type());
	newH.at<double>(0, 2) = correccionX;
	newH.at<double>(1, 2) = correccionY;

	perspectiveTransform(im1, im1f, newH * H);
	perspectiveTransform(im2, im2f, newH);

	// Use the Homography Matrix to warp the images
	cv::Mat result1, result2, final;
	warpPerspective(image1, final, newH * H,
			cv::Size(calcularAnchura(im1f, im2f), calcularAltura(im1f, im2f)),
			BORDER_CONSTANT, 0);
	imshow("Result1", final);

	warpPerspective(image2, result2, newH,
			cv::Size(calcularAnchura(im1f, im2f), calcularAltura(im1f, im2f)),
			BORDER_CONSTANT, 0);
	double anchuraImg2 = max(abs(im2f[1].x - im2f[0].x),
			abs(im2f[2].x - im2f[3].x));
	double alturaImg2 = max(abs(im2f[2].y - im2f[0].y),
			abs(im2f[1].y - im2f[3].y));
	imshow("Result2", result2);

	Mat half(final, cv::Rect(im2f[0].x, im2f[0].y, anchuraImg2, alturaImg2));
	image2.copyTo(half);
	imshow("Half", final);

	waitKey(0);
	return 0;
}

/*
 * Función  del trabajo 4 de Visión por computador
 * En base a unos vectores de puntos que representan las esquinas de las dos
 * imágenes que van a formar un panorama, calcula cuál es la anchura de la
 * imagen que se va a formar comparando las esquinas 1 y 3 de ambas imágenes
 */

Mat getPanorama(Mat nueva, Mat estatica, bool log) {
	if (log) {
		printf("Panorama iniciado\n");
		fflush(stdout);
	}

	// Lee la imagen
	Mat image1, image2;
	image1 = nueva.clone();
	image2 = estatica.clone();

	// Las convierte a escala de grises
	Mat gray_image1;
	Mat gray_image2;
	cvtColor(image1, gray_image1, CV_RGB2GRAY);
	cvtColor(image2, gray_image2, CV_RGB2GRAY);

	if(log) {
		printf("Convertido a escala de grises\n");
		fflush(stdout);
	}
	// Detectar puntos usando SURF
	int minHessian = 400;

	SurfFeatureDetector detector(minHessian);

	std::vector<KeyPoint> keypoints_object, keypoints_scene;

	detector.detect(gray_image1, keypoints_object);
	detector.detect(gray_image2, keypoints_scene);

	if(log) {
		printf("Puntos detectados\n");
		fflush(stdout);
	}
	// Calcular descriptores
	SurfDescriptorExtractor extractor;

	Mat descriptors_object, descriptors_scene;

	extractor.compute(gray_image1, keypoints_object, descriptors_object);
	extractor.compute(gray_image2, keypoints_scene, descriptors_scene);

	if(log) {
		printf("Descriptores calculados\n");
		fflush(stdout);
	}
	// Buscar matches con fuerza bruta
	//FlannBasedMatcher matcher;
	BFMatcher matcher(NORM_L2);
	std::vector<vector<DMatch> > matches;
	matcher.knnMatch(descriptors_object, descriptors_scene, matches, 2);

	if(log) {
		printf("Matches encontrados\n");
		fflush(stdout);
	}
	//-- Draw matches
	Mat img_matches;
	drawMatches(image1, keypoints_object, image2, keypoints_scene, matches,
			img_matches);

	//-- Show detected matches
	imshow("Matches1", img_matches);

	// Sacar las matches buenas
	std::vector<DMatch> good_matches;

	for (int i = 0; i < descriptors_object.rows; i++) {
		if (matches[i][0].distance < 0.5 * matches[i][1].distance) {
			good_matches.push_back(matches[i][0]);
		}
	}
	if(log) {
		printf("Matches buenos encontrados\n");
		fflush(stdout);
	}
	// Si no hay por lo menos 15 matches buenas, no haremos nada
	if (good_matches.size() >= 15) {
		if(log) {
			printf("Al menos 15 matches buenos\n");
			fflush(stdout);
		}
		std::vector<Point2f> obj;
		std::vector<Point2f> scene;

		for (unsigned int i = 0; i < good_matches.size(); i++) {
			// Conseguir los puntos de interés de las matches buenas
			obj.push_back(keypoints_object[good_matches[i].queryIdx].pt);
			scene.push_back(keypoints_scene[good_matches[i].trainIdx].pt);
		}
		if(log) {
			printf("Puntos obtenidos para los buenos matches\n");
			fflush(stdout);
		}
		// Encontrar la matriz de Homografía
		Mat good_inliers;

		Mat H = findHomography(obj, scene, CV_RANSAC, 3, good_inliers);
		if(log) {
			printf("Homografía encontrada\n");
			fflush(stdout);
		}
		vector<DMatch> drawResults;
		for (int i = 0; i < good_inliers.rows; i++) {
			if (good_inliers.at<int>(i,0) == 1) {
				drawResults.push_back(good_matches[i]);
			}
		}
		if(log) {
			printf("Buenos inliers obtenidos\n");
			fflush(stdout);
		}
		//-- Draw matches
		Mat img_matches2;
		drawMatches(image1, keypoints_object, image2, keypoints_scene,
				drawResults, img_matches2);

		//-- Show detected matches
		imshow("Matches", img_matches2);

		//Esquinas de las fotos para calcular la nueva matriz de Homografía
		vector<Point2f> im1, im2, im1c, im2c, im1f, im2f;
		im1.push_back(Point(0, 0));
		im1.push_back(Point(image1.cols, 0));
		im1.push_back(Point(0, image1.rows));
		im1.push_back(Point(image1.cols, image1.rows));

		im2.push_back(Point(0, 0));
		im2.push_back(Point(image2.cols, 0));
		im2.push_back(Point(0, image2.rows));
		im2.push_back(Point(image2.cols, image2.rows));

		// Obtener las nuevas esquinas aplicando Homografía
		perspectiveTransform(im1, im1c, H);
		perspectiveTransform(im2, im2c, H);

		// Obtener la corrección para X e Y con las esquinas homografiadas
		double correccionX = calcularCorreccionX(im1c, im2c);
		double correccionY = calcularCorreccionY(im1c, im2c);
		if(log) {
			if(correccionX==0 && correccionY==0)
				printf("Correccion calculada\n");
			else
				printf("No ha sido necesaria correccion\n");
			fflush(stdout);
		}
		// Obtener la matriz de homografía de traslación para aplicar
		// la corrección
		Mat newH = Mat::eye(3, 3, H.type());
		newH.at<double>(0, 2) = correccionX;
		newH.at<double>(1, 2) = correccionY;
		if(log) {
			printf("Matriz de traslacion calculada\n");
			fflush(stdout);
		}
		// Obtener las esquinas definitivas aplicando la nueva homografía
		perspectiveTransform(im1, im1f, newH * H);
		perspectiveTransform(im2, im2f, newH);

		// Usar ambas matrices de homografía para trasladar las imágenes
		Mat final;
		warpPerspective(image2, final, newH,
				cv::Size(calcularAnchura(im1f, im2f),
						calcularAltura(im1f, im2f)), INTER_LINEAR,
				BORDER_CONSTANT, 0);
		warpPerspective(image1, final, newH * H,
				cv::Size(calcularAnchura(im1f, im2f),
						calcularAltura(im1f, im2f)), INTER_LINEAR,
				BORDER_TRANSPARENT, 0);
		if(log) {
			printf("Panorama calculado\n");
			fflush(stdout);
		}
		//Devuelve el panorama
		return final;
	} else {
		if(log) {
			printf("No se han encontrado al menos 15 puntos buenos, panorama cancelado\n");
			fflush(stdout);
		}
		return estatica;
	}
}

/*
* Función panoramaPortatilTecla del trabajo 4 de Visión por computador
* Con cada pulsación del teclado, toma una nueva foto para añadir al panorama
* si se puede formar uno con la nueva foto tomada
*/

void panoramaPortatilTecla() {
	VideoCapture cap(1);
	Mat frame;

	while (waitKey(1) == -1) {
		cap >> frame;
		flip(frame, frame, 1);
		mostrarImagen("Camara", frame, false);
	}

	Mat imagen1 = frame.clone();
	while (true) {
		while (waitKey(1) == -1) {
			cap >> frame;
			flip(frame, frame, 1);
			mostrarImagen("Camara", frame, false);
		}

		Mat imagen2 = frame.clone();

		imagen1 = getPanorama(imagen2, imagen1, true).clone();
		mostrarImagen("Panorama", imagen1, false);
		imwrite("fotito.jpg", imagen1);
	}
}

/*
 * Función panoramaPortatilAutomatico del trabajo 4 de Visión por computador
 * No funciona de momento
 */

void panoramaPortatilAutomatico() {
	VideoCapture cap(1);
	Mat frame1, frame2;

	cap >> frame1;
	while (true) {
		waitKey(30);
		cap >> frame2;

		mostrarImagen("Camara", frame2, false);

		frame1 = getPanorama(frame2, frame1, true).clone();

		mostrarImagen("Panorama", frame1, false);
		imwrite("fotito.jpg", frame1);
	}
}
