/*
 * RefineBallPos.cpp
 *
 *  Created on: 14-Sep-2017
 *      Author: sriram
 */

#include "RefineBallPos.h"

RefineBallPos::RefineBallPos() {
	// TODO Auto-generated constructor stub

}

/**
 * Check if the ROI is inside the image
 */
bool RefineBallPos::checkROIInside(Rect &smallRoi, Mat &image) {
	if (smallRoi.x < 0)
		return false;
	if (smallRoi.y < 0)
		return false;
	if (smallRoi.x + smallRoi.width > image.cols)
		return false;
	if (smallRoi.y + smallRoi.height > image.rows)
		return false;
	return true;
}

/**
 * This Function is to convolve the Edge image with the Mask. Returns Score
 */
double RefineBallPos::calculateEdgeConvolution(Mat edgeRoi, Mat mask) {
	Mat pow1, pow2;
	pow(edgeRoi, 2, pow1);
	pow(mask, 2, pow2);

	Mat conv;
	multiply(mask, edgeRoi, conv);
	double normValue = sqrt(sum(pow1)[0] * sum(pow2)[0]);
	double value = sum(conv)[0] / normValue;

	return value;
}

/**
 * Finds Circle's proper center and radius.
 */
Vec3f RefineBallPos::findProperCircle(Mat image, Mat BGSimage,
		Vec3f initialCircleCenter, Mat Template) {

	Mat edgeImage;
	Mat MaskedEdgeImage;
	int lowThreshold = 100;
	int highThreshold = 255;
	int kernel_size = 3;

	float scaleFactor = 1.0;
	resize(image, image, Size(), scaleFactor, scaleFactor, INTER_LINEAR);
	resize(BGSimage, BGSimage, Size(), scaleFactor, scaleFactor, INTER_LINEAR);
//	Apply Canny on image
	if (scaleFactor == 1.0) {
		Canny(image, edgeImage, lowThreshold, highThreshold, kernel_size);
//	    Applying Mask on the image
		Mat MaskBGSimage;
		threshold(BGSimage, MaskBGSimage, 0, 255, THRESH_BINARY);

		edgeImage.copyTo(MaskedEdgeImage, MaskBGSimage);
	} else {
//		This is for high presions when scaleFactor is greater than 1.0
		Mat grad_x, grad_y;
		int ddepth = CV_16S;
		int scale = 1;
		int delta = 0;
		Mat abs_grad_x, abs_grad_y;
		Sobel(image, grad_x, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT);
		convertScaleAbs(grad_x, abs_grad_x);
		Sobel(image, grad_y, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT);
		convertScaleAbs(grad_y, abs_grad_y);
		addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, edgeImage);
		threshold(edgeImage, edgeImage, 50, 0, THRESH_TOZERO);
		edgeImage.copyTo(MaskedEdgeImage);
	}
//	Center correction
	float factor = (BallROIScale - 1) / BallROIScale;
	int minCenterx = scaleFactor
			* (initialCircleCenter[2] - initialCircleCenter[2] * factor);
	int maxCenterx = scaleFactor
			* (initialCircleCenter[2] + initialCircleCenter[2] * factor);
	int minCentery = scaleFactor
			* (initialCircleCenter[2] - initialCircleCenter[2] * factor);
	int maxCentery = scaleFactor
			* (initialCircleCenter[2] + initialCircleCenter[2] * factor);

	double maxScore = -1;
	float bestCenterX = -1;
	float bestCenterY = -1;
	float bestRadius = 0;
	float radius = scaleFactor * initialCircleCenter[2] / BallROIScale;
	int startRadius = radius;
//	Iteratively Do change the circle radius and check convolution score to pitch the right radius and center
	for (int r = startRadius; r < scaleFactor * initialCircleCenter[2]; r++) {
		Mat Mask2(2 * r + 1, 2 * r + 1, CV_8UC1);
		Mask2 = Mask2 * 0;
		circle(Mask2, Point(r, r), r, 200, 1, 8, 0);

		for (int i = minCenterx; i < maxCenterx; i++) {
			for (int j = minCentery; j < maxCentery; j++) {

				cv::Rect roi(i - r, j - r, 2 * r + 1, 2 * r + 1);

				if (checkROIInside(roi, image)) {
					Mat edgeRoi = MaskedEdgeImage(roi);

					double score = calculateEdgeConvolution(edgeRoi, Mask2);
					if (score > maxScore) {
						maxScore = score;
						bestCenterX = (float) i / scaleFactor;
						bestCenterY = (float) j / scaleFactor;
						bestRadius = (float) r / scaleFactor;
					}
				}
			}
		}

	}
#ifdef Visualize
	cvtColor(image, image, COLOR_GRAY2BGR);
	circle(image, Point(initialCircleCenter[2], initialCircleCenter[2]),
			initialCircleCenter[2] * factor, Scalar(0, 255, 0), 1, 8, 0);
	circle(image, Point(bestCenterX, bestCenterY), bestRadius,
			Scalar(0, 0, 255), 1, 8, 0);
	imshow("best center", image);
	waitKey(0);
#endif
	// Copy the best center and radius to output. Correct to the original image size.
	Vec3f bestCircle;
	bestCircle[0] = (
			(initialCircleCenter[0] - initialCircleCenter[2]) > 0 ?
					(initialCircleCenter[0] - initialCircleCenter[2]) : 0)
			+ bestCenterX;
	bestCircle[1] = (
			(initialCircleCenter[1] - initialCircleCenter[2]) > 0 ?
					(initialCircleCenter[1] - initialCircleCenter[2]) : 0)
			+ bestCenterY;
	bestCircle[2] = bestRadius;

	return bestCircle;
}

/**
 * Fits Line to x and y ball positions. Uses ransac to exclude outliers.
 *
 * Returns M and C to draw the line.
 */
Mat RefineBallPos::FitLine2BallTraj(vector<Vec3f> &MatchBalls) {
	double error = 10000;
	int sampleCount = int(0.75 * float(MatchBalls.size())); //defining sample size adaptive to the MatchBall Size

	Mat w;
//	Random sampling to select the right circles to exclude the outliers
	while (error > FitLine2Ballthreshold) { // continue till error size is less than the threshold
		vector<Vec3f> newMatchBalls;
		vector<int> ranNum;
		int points = 0;
//		Pick samples randomly
		while (points < sampleCount) {
			int randnum;
			randnum = rand() % MatchBalls.size() + 1;
			bool pushin = true;
			for (uint x = 0; x < ranNum.size(); x++) {
				if (randnum == ranNum[x])
					pushin = false;
			}
			if (pushin) {
				newMatchBalls.push_back(MatchBalls[randnum]);
				ranNum.push_back(randnum);
				points++;
			}
		}

		vector<float> YMat;

		cv::Mat x(newMatchBalls.size(), 2, CV_32F);
		for (uint i = 0; i < newMatchBalls.size(); i++) {
			x.at<float>(i, 0) = newMatchBalls[i][0];
			x.at<float>(i, 1) = 1;
			YMat.push_back(newMatchBalls[i][1]);
		}
		Mat y = Mat(YMat, CV_32F);
//		Computes the unknowns
		w = ((x.t() * x).inv()).t() * x.t() * y;
		double dist = 0;
		for (uint i = 0; i < newMatchBalls.size(); i++) {
			float Yval = (newMatchBalls[i][0] * w.at<float>(0, 0))
					+ w.at<float>(1, 0);
			Point2f p1 = Point2f(newMatchBalls[i][0], Yval);
			Point2f p2 = Point2f(newMatchBalls[i][0], newMatchBalls[i][1]);
			dist = dist + sqrt(pow((p2.x - p1.x), 2) + pow((p2.y - p1.y), 2));
		}
//		assigning the error
		error = dist / newMatchBalls.size();
	}
	return w;
}

/**
 * Fits curve to x and frame number.
 *
 * Returns x1 x2 and x3 to fit the curve.
 */
Mat RefineBallPos::FitCurve2XandImage(vector<Vec2f> &newMatchBalls) {
	cv::Mat x(newMatchBalls.size(), 3, CV_32F);
	cv::Mat y(newMatchBalls.size(), 1, CV_32F);
	for (uint i = 0; i < newMatchBalls.size(); i++) {
		x.at<float>(i, 0) = pow(newMatchBalls[i][0], 2);
		x.at<float>(i, 1) = newMatchBalls[i][0];
		x.at<float>(i, 2) = 1;
		y.at<float>(i, 0) = (newMatchBalls[i][1]);
	}
//	Computes the unknowns
	Mat wx = ((x.t() * x).inv()).t() * x.t() * y;
	return wx;
}

/**
 * Finds the outlier circles and correct them and returns the corrected circle center.
 */
vector<Vec3f> RefineBallPos::findOutliersAndCorrect(vector<Vec3f> MatchBalls) {

//	Fit line to the trajectory
	Mat w = FitLine2BallTraj(MatchBalls);

//	Exclude the outliers
	vector<Vec2f> newMatchBalls;
	for (uint i = 0; i < MatchBalls.size(); i++) {
		float Yval = (MatchBalls[i][0] * w.at<float>(0, 0)) + w.at<float>(1, 0);
		Point2f p1 = Point2f(MatchBalls[i][0], Yval);
		Point2f p2 = Point2f(MatchBalls[i][0], MatchBalls[i][1]);
		double dist = sqrt(pow((p2.x - p1.x), 2) + pow((p2.y - p1.y), 2));
		if (dist < FitLine2Ballthreshold)
			newMatchBalls.push_back(Vec2f(i, MatchBalls[i][0]));
	}

//	Fit the curve for X and image number. So the x position can be predicted for outlier frames.
	Mat wx = FitCurve2XandImage(newMatchBalls);

//	Prepare the finalMatchBalls predicting the X and Y for them. Rest of the circles are unchanged.
	vector<Vec3f> FinalMatchBalls;
	for (uint i = 0; i < MatchBalls.size(); i++) {
		bool Push = true;
		for (uint j = 0; j < newMatchBalls.size(); j++) {
			if (newMatchBalls[j][0] == i) {
				FinalMatchBalls.push_back(MatchBalls[i]);
				Push = false;
			}
		}
		if (Push) {
			float xVal = (float(i) * float(i) * wx.at<float>(0, 0))
					+ (float(i) * wx.at<float>(1, 0)) + wx.at<float>(2, 0);
			float Yval = (xVal * w.at<float>(0, 0)) + w.at<float>(1, 0);
			FinalMatchBalls.push_back(Vec3f(xVal, Yval, -1)); // Returns radius as -1 for outliers
		}
	}
	return FinalMatchBalls;
}

RefineBallPos::~RefineBallPos() {
// TODO Auto-generated destructor stub
}

