/*
 * FuncAux.cpp
 *
 *  Created on: 08/03/2015
 *      Author: Jaime. David.
 */

#include <stdio.h>
#include <iostream>
#include <ctime>
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

std::vector<DMatch> getGoodMatches(int matcherType, int detectorType,
		vector<KeyPoint> keypoints_object, vector<KeyPoint> keypoints_scene,
		Mat descriptors_object, Mat descriptors_scene, Mat image1, Mat image2,
		bool log) {

	std::vector<DMatch> good_matches;

	if (matcherType == 1) {
		FlannBasedMatcher matcher;
		std::vector<DMatch> matches;
		int startMatch = clock();
		matcher.match(descriptors_object, descriptors_scene, matches);
		int stop_s = clock();

		if (log) {
			printf("Matches encontrados\n");
			fflush(stdout);
			std::cout << "Tiempo transcurrido: "
					<< (stop_s - startMatch) / double(CLOCKS_PER_SEC ) * 1000
					<< " milisegundos" << std::endl;
		}
		//-- Draw matches
		Mat img_matches;
		drawMatches(image1, keypoints_object, image2, keypoints_scene, matches,
				img_matches);

		//-- Show detected matches
		imshow("Todas las correspondencias", img_matches);

		// Sacar las matches buenas

		good_matches = matches;

		if (log) {
			printf("Matches buenos encontrados\n");
			fflush(stdout);
		}
	} else {
		std::vector<vector<DMatch> > matches;
		int startMatch, stop_s;
		if(detectorType!=2) {
			BFMatcher matcher(NORM_L2);
			startMatch = clock();
			matcher.knnMatch(descriptors_object, descriptors_scene, matches, 2);
			stop_s = clock();
		}
		else {
			BFMatcher matcher(NORM_HAMMING);
			startMatch = clock();
			matcher.knnMatch(descriptors_object, descriptors_scene, matches, 2);
			stop_s = clock();
		}

		if (log) {
			printf("Matches encontrados\n");
			fflush(stdout);
			std::cout << "Tiempo transcurrido: "
			<< (stop_s - startMatch) / double(CLOCKS_PER_SEC ) * 1000
			<< " milisegundos" << std::endl;
		}

		Mat img_matches;
		drawMatches(image1, keypoints_object, image2, keypoints_scene, matches,
				img_matches);
		imshow("Todas las correspondencias", img_matches);

		for (int i = 0; i < descriptors_object.rows; i++) {
			if (matches[i][0].distance < 0.5 * matches[i][1].distance) {
				good_matches.push_back(matches[i][0]);
			}
		}

		if (log) {
			printf("Matches buenos encontrados\n");
			fflush(stdout);
		}
	}
	return good_matches;
}

std::vector<Mat> getDescriptors(int detectorType, Mat gray_image1,
		Mat gray_image2, vector<KeyPoint> keypoints_object,
		vector<KeyPoint> keypoints_scene, bool log) {
	vector<Mat> returned;
	Mat descriptors_object, descriptors_scene;
	if (detectorType == 1) {
		SiftDescriptorExtractor extractor;

		int startCalc = clock();
		extractor.compute(gray_image1, keypoints_object, descriptors_object);
		extractor.compute(gray_image2, keypoints_scene, descriptors_scene);
		int stop_s = clock();

		if (log) {
			printf("Descriptores calculados\n");
			fflush(stdout);
			std::cout << "Tiempo transcurrido: "
					<< (stop_s - startCalc) / double(CLOCKS_PER_SEC ) * 1000
					<< " milisegundos" << std::endl;
		}
	} else if (detectorType == 2) {
		OrbDescriptorExtractor extractor;

		int startCalc = clock();
		extractor.compute(gray_image1, keypoints_object, descriptors_object);
		extractor.compute(gray_image2, keypoints_scene, descriptors_scene);
		int stop_s = clock();

		if (log) {
			printf("Descriptores calculados\n");
			fflush(stdout);
			std::cout << "Tiempo transcurrido: "
					<< (stop_s - startCalc) / double(CLOCKS_PER_SEC ) * 1000
					<< " milisegundos" << std::endl;
		}
	} else {
		SurfDescriptorExtractor extractor;

		int startCalc = clock();
		extractor.compute(gray_image1, keypoints_object, descriptors_object);
		extractor.compute(gray_image2, keypoints_scene, descriptors_scene);
		int stop_s = clock();

		if (log) {
			printf("Descriptores calculados\n");
			fflush(stdout);
			std::cout << "Tiempo transcurrido: "
					<< (stop_s - startCalc) / double(CLOCKS_PER_SEC ) * 1000
					<< " milisegundos" << std::endl;
		}
	}
	returned.push_back(descriptors_object);
	returned.push_back(descriptors_scene);
	return returned;
}

std::vector<vector<KeyPoint> > getKeyPoints(Mat gray_image1, Mat gray_image2,
		int detectorType, bool log) {
	std::vector<vector<KeyPoint> > returned;

	// Detectar puntos usando un detector
	int minHessian = 400;
	std::vector<KeyPoint> keypoints_object, keypoints_scene;
	if (detectorType == 1) {
		//Sift
		SiftFeatureDetector detector(minHessian);
		int startDet = clock();
		detector.detect(gray_image1, keypoints_object);
		detector.detect(gray_image2, keypoints_scene);
		int stop_s = clock();

		if (log) {
			printf("Puntos detectados\n");
			fflush(stdout);
			std::cout << "Tiempo transcurrido: "
					<< (stop_s - startDet) / double(CLOCKS_PER_SEC ) * 1000
					<< " milisegundos" << std::endl;
		}

		returned.push_back(keypoints_object);
		returned.push_back(keypoints_scene);
	} else if (detectorType == 2) {
		//Orb
		OrbFeatureDetector detector(minHessian);
		int startDet = clock();
		detector.detect(gray_image1, keypoints_object);
		detector.detect(gray_image2, keypoints_scene);
		int stop_s = clock();

		if (log) {
			printf("Puntos detectados\n");
			fflush(stdout);
			std::cout << "Tiempo transcurrido: "
					<< (stop_s - startDet) / double(CLOCKS_PER_SEC ) * 1000
					<< " milisegundos" << std::endl;
		}
		returned.push_back(keypoints_object);
		returned.push_back(keypoints_scene);
	} else {
		//Surf otra vez
		SurfFeatureDetector detector(minHessian);
		int startDet = clock();
		detector.detect(gray_image1, keypoints_object);
		detector.detect(gray_image2, keypoints_scene);
		int stop_s = clock();

		if (log) {
			printf("Puntos detectados\n");
			fflush(stdout);
			std::cout << "Tiempo transcurrido: "
					<< (stop_s - startDet) / double(CLOCKS_PER_SEC ) * 1000
					<< " milisegundos" << std::endl;
		}
		returned.push_back(keypoints_object);
		returned.push_back(keypoints_scene);
	}
	return returned;
}
/*
 * Función  del trabajo 4 de Visión por computador
 * En base a unos vectores de puntos que representan las esquinas de las dos
 * imágenes que van a formar un panorama, calcula cuál es la anchura de la
 * imagen que se va a formar comparando las esquinas 1 y 3 de ambas imágenes
 */

Mat getPanorama(Mat nueva, Mat estatica, bool log, int detectorType,
		int matcherType) {

	int start_s = clock();
	int stop_s;

	Mat image1, image2;
	image1 = nueva.clone();
	image2 = estatica.clone();

	if (log) {
		printf("=================\n");
		printf("Panorama iniciado\n");
		fflush(stdout);
	}

	Mat gray_image1;
	Mat gray_image2;
	cvtColor(image1, gray_image1, CV_RGB2GRAY);
	cvtColor(image2, gray_image2, CV_RGB2GRAY);

	if (log) {
		printf("Convertido a escala de grises\n");
		fflush(stdout);
	}

		// Calcular puntos de interés
	std::vector<vector<KeyPoint> > keypoints;
	keypoints = getKeyPoints(gray_image1, gray_image2, detectorType, log);
	vector<KeyPoint> keypoints_object = keypoints[0];
	vector<KeyPoint> keypoints_scene = keypoints[1];

	// Calcular descriptores
	Mat descriptors_object, descriptors_scene;
	vector<Mat> descriptors;
	descriptors = getDescriptors(detectorType, gray_image1, gray_image2,
			keypoints_object, keypoints_scene, log);

	descriptors_object = descriptors[0];
	descriptors_scene = descriptors[1];

	// Buscar matches y sacar las matches buenas
	std::vector<DMatch> good_matches = getGoodMatches(matcherType, detectorType,
			keypoints_object, keypoints_scene, descriptors_object,
			descriptors_scene, image1, image2, log);

	// Si no hay por lo menos 15 matches buenas,
	// no haremos nada

	unsigned int goodMatchesSize;
	if(detectorType==2){
		goodMatchesSize = 10;
	}
	else{
		goodMatchesSize = 15;
	}
	if (good_matches.size() >= goodMatchesSize) {
		if (log) {
			printf("Al menos %d matches buenos\n",goodMatchesSize);
			fflush(stdout);
		}
		std::vector<Point2f> obj;
		std::vector<Point2f> scene;

		for (unsigned int i = 0; i < good_matches.size(); i++) {
			obj.push_back(keypoints_object[good_matches[i].queryIdx].pt);
			scene.push_back(keypoints_scene[good_matches[i].trainIdx].pt);
		}

		if(log) {
			printf("Puntos obtenidos para los buenos matches\n");
			fflush(stdout);
		}

		Mat good_inliers;
		int startHom=clock();
		Mat H = findHomography(obj, scene, CV_RANSAC, 3, good_inliers);
		stop_s=clock();

		if(log) {
			printf("Homografía encontrada\n");
			fflush(stdout);
			std::cout << "Tiempo transcurrido: "
			<< (stop_s - startHom) / double(CLOCKS_PER_SEC ) * 1000
			<< " milisegundos"
			<< std::endl;
		}
		vector<DMatch> drawResults;
		for (int i = 0; i < good_inliers.rows; i++) {
			if (good_inliers.at<uchar>(i,0) == 1) {
				drawResults.push_back(good_matches[i]);
			}
		}

		if(log) {
			printf("Buenos inliers obtenidos\n");
			fflush(stdout);
		}

		Mat img_matches2;
		drawMatches(image1, keypoints_object, image2, keypoints_scene,
				drawResults, img_matches2);

		imshow("Buenas correspondencias", img_matches2);

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
		if(log) {
			if(correccionX!=0 or correccionY!=0)
			printf("Correccion calculada\n");
			else
			printf("No ha sido necesaria correccion\n");
			fflush(stdout);
		}

		Mat newH = Mat::eye(3, 3, H.type());
		newH.at<double>(0, 2) = correccionX;
		newH.at<double>(1, 2) = correccionY;

		if(log) {
			printf("Matriz de traslacion calculada\n");
			fflush(stdout);
		}

		perspectiveTransform(im1, im1f, newH * H);
		perspectiveTransform(im2, im2f, newH);

		Mat final;
		warpPerspective(image2, final, newH,
				cv::Size(calcularAnchura(im1f, im2f),
						calcularAltura(im1f, im2f)), INTER_LINEAR,
				BORDER_CONSTANT, 0);
		warpPerspective(image1, final, newH * H,
				cv::Size(calcularAnchura(im1f, im2f),
						calcularAltura(im1f, im2f)), INTER_LINEAR,
				BORDER_TRANSPARENT, 0);
		stop_s=clock();

		if(log) {
			printf("Panorama calculado\n");
			fflush(stdout);
			std::cout << "Tiempo transcurrido total: "
			<< (stop_s - start_s) / double(CLOCKS_PER_SEC ) * 1000
			<< " milisegundos"
			<< std::endl;
		}

		return final;
	} else {
		if(log) {
			printf("No se han encontrado al menos %d puntos buenos, panorama cancelado\n"
					,goodMatchesSize);
			fflush(stdout);
		}
		std::vector<Point2f> obj;
		std::vector<Point2f> scene;

		for (unsigned int i = 0; i < good_matches.size(); i++) {
			obj.push_back(keypoints_object[good_matches[i].queryIdx].pt);
			scene.push_back(keypoints_scene[good_matches[i].trainIdx].pt);
		}

		Mat good_inliers;
		Mat H = findHomography(obj, scene, CV_RANSAC, 3, good_inliers);

		vector<DMatch> drawResults;
		for (int i = 0; i < good_inliers.rows; i++) {
			if (good_inliers.at<uchar>(i,0) == 1) {
				drawResults.push_back(good_matches[i]);
			}
		}

		Mat img_matches2;
		drawMatches(image1, keypoints_object, image2, keypoints_scene,
				drawResults, img_matches2);

		imshow("Buenas correspondencias", img_matches2);
		return estatica;
	}

}

/*
 * Función panoramaPortatilAutomatico del trabajo 4 de Visión por computador
 * Crea un panorama automáticamente con las fotos capturadas por la cámara del portátil
 */

void panoramaDisco(int detector, int matcher, int tipo) {
	if(tipo==1){
		Mat image1, image2, image3;
		image1 = imread("ext1.jpg", CV_LOAD_IMAGE_COLOR);
		image2 = imread("ext2.jpg", CV_LOAD_IMAGE_COLOR);
		image3 = imread("ext3.jpg", CV_LOAD_IMAGE_COLOR);
		Mat frame1 = getPanorama(image2, image1, true, detector, matcher);
		mostrarImagen("Panorama", frame1, true);
		Mat frame2 = getPanorama(image3, frame1, true, detector, matcher);
		mostrarImagen("Panorama", frame2, true);
		imwrite("fotito.jpg", frame2);
	}
	else{
		Mat image1, image2, image3, image4, image5, image6;
		image1 = imread("1.jpg", CV_LOAD_IMAGE_COLOR);
		image2 = imread("2.jpg", CV_LOAD_IMAGE_COLOR);
		image3 = imread("3.jpg", CV_LOAD_IMAGE_COLOR);
//		image4 = imread("4.jpg", CV_LOAD_IMAGE_COLOR);
//		image5 = imread("5.jpg", CV_LOAD_IMAGE_COLOR);
//		image6 = imread("6.jpg", CV_LOAD_IMAGE_COLOR);

		Mat frame1 = getPanorama(image1, image2, true, detector, matcher);
		mostrarImagen("Panorama", frame1, true);
		Mat frame2 = getPanorama(image3, frame1, true, detector, matcher);
		mostrarImagen("Panorama", frame2, true);
//		Mat frame3 = getPanorama(image4, frame2, true, detector, matcher);
//		mostrarImagen("Panorama", frame3, true);
//		Mat frame4 = getPanorama(image5, frame3, true, detector, matcher);
//		mostrarImagen("Panorama", frame4, true);
//		Mat frame5 = getPanorama(image6, frame4, true, detector, matcher);
//		mostrarImagen("Panorama", frame5, true);
//		imwrite("fotito.jpg", frame5);
	}
}

/*
 * Función panoramaPortatilTecla del trabajo 4 de Visión por computador
 * Con cada pulsación del teclado, toma una nueva foto para añadir al panorama
 * si se puede formar uno con la nueva foto tomada
 */

void panoramaPortatilTecla(int detector, int matcher) {
	VideoCapture cap(0);
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

		imagen1 =
				getPanorama(imagen2, imagen1, true, detector, matcher).clone();
		mostrarImagen("Panorama", imagen1, false);
		imwrite("fotito.jpg", imagen1);
	}
}

/*
 * Función panoramaPortatilAutomatico del trabajo 4 de Visión por computador
 * Crea un panorama automáticamente con las fotos capturadas por la cámara del portátil
 */

void panoramaPortatilAutomatico(int detector, int matcher) {
	VideoCapture cap(0);
	Mat frame1, frame2;

	cap >> frame1;
	while (true) {
		waitKey(30);
		cap >> frame2;

		mostrarImagen("Camara", frame2, false);

		frame1 = getPanorama(frame2, frame1, true, detector, matcher).clone();

		mostrarImagen("Panorama", frame1, false);
		imwrite("fotito.jpg", frame1);
	}
}
