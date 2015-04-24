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

/*
 * Función main del Trabajo 4 de Visión por Computador
 * Gestiona el funcionamiento principal del programa
 */

int main() {
	//Lee la imagen
	Mat image1, image2;
	image1 = imread("ratong.jpg", CV_LOAD_IMAGE_GRAYSCALE);
	image2 = imread("testratong.jpg", CV_LOAD_IMAGE_GRAYSCALE);
	if (!image1.data || !image2.data) {
		cout << "Could not open or find the image" << std::endl;
	} else {

		//-- Step 1: Detect the keypoints using SURF Detector
		int minHessian = 400;

		SurfFeatureDetector detector(minHessian);

		std::vector<KeyPoint> keypoints_1, keypoints_2;

		detector.detect(image1, keypoints_1);
		detector.detect(image2, keypoints_2);

		//-- Step 2: Calculate descriptors (feature vectors)
		SurfDescriptorExtractor extractor;

		Mat descriptors_1, descriptors_2;

		extractor.compute(image1, keypoints_1, descriptors_1);
		extractor.compute(image2, keypoints_2, descriptors_2);

		//-- Step 3: Matching descriptor vectors with a brute force matcher
		BFMatcher matcher(NORM_L2,false);
		std::vector<DMatch> matches;
		matcher.match(descriptors_1, descriptors_2, matches);

		//-- Draw matches
		Mat img_matches;
		drawMatches(image1, keypoints_1, image2, keypoints_2, matches,img_matches);

		//-- Show detected matches
		imshow("Matches", img_matches);

		waitKey(0);
	}
	return 0;
}

