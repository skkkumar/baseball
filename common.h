/*
 * common.h
 *
 *  Created on: 14-Sep-2017
 *      Author: sriram
 */

#ifndef COMMON_H_
#define COMMON_H_
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include <numeric>
#include <opencv2/video/background_segm.hpp>
#include <opencv2/video.hpp>

#include <opencv2/highgui.hpp>
#include <iostream>
#include <cstdlib>
//#define Visualize
#define Result
using namespace cv;
using namespace std;
const string TemplateString = "../Template.jpg";
const float BallROIScale = 2;
const double FitLine2Ballthreshold = 2;

#endif /* COMMON_H_ */
