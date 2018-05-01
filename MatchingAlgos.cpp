/*
 * MatchingAlgos.cpp
 *
 *  Created on: 14-Sep-2017
 *      Author: sriram
 */

#include "MatchingAlgos.h"

MatchingAlgos::MatchingAlgos() {
//	Reads the template
	Template = imread(TemplateString.c_str(), CV_LOAD_IMAGE_GRAYSCALE);
}
/**
 * Returns Smoothness of a trajectory. This was to attempt without template matching
 * to identify the ball with the smoothness of the trajectory, based on velocity and
 * direction. Stopped developing this as it may require more time.
 *
 * Unused function
 */
double MatchingAlgos::ReturnTrajectorySmoothness(Point2f P1, Point2f P2) {
	double weight = 0.5;
	double Velocity = weight
			* ((P1.x * P2.x + P1.y * P2.y)
					/ (sqrt(pow(P1.x, 2) + pow(P1.y, 2))
							* sqrt(pow(P2.x, 2) + pow(P2.y, 2))));
	double speed = (1 - weight)
			* ((2
					* sqrt(
							sqrt(pow(P1.x, 2) + pow(P1.y, 2))
									* sqrt(pow(P2.x, 2) + pow(P2.y, 2))))
					/ (sqrt(pow(P1.x, 2) + pow(P1.y, 2))
							+ sqrt(pow(P2.x, 2) + pow(P2.y, 2))));
	double Trajectory_Smoothness = Velocity + speed;
	return Trajectory_Smoothness;
}

/**
 * Return Sum of absolute difference between 2 images
 *
 * Unused function
 */
double MatchingAlgos::returnSADVal(Mat CroppedImage1, Mat CroppedImage2) {
	CroppedImage1.convertTo(CroppedImage1, CV_64F);
	CroppedImage2.convertTo(CroppedImage2, CV_64F);
	Mat Diff = (CroppedImage1 - CroppedImage2);
	Mat SqDiff = Diff.mul(Diff);
	Mat SqIm1 = CroppedImage1.mul(CroppedImage1);
	Mat SqIm2 = CroppedImage2.mul(CroppedImage2);
	double Den = sqrt(sum(SqIm1)[0] * sum(SqIm2)[0]);
	double NormalisedSAD = sum(SqDiff)[0] / Den;
	return NormalisedSAD;
}

/**
 *
 * Returns the Histogram of Local binary pattern of an image.
 *
 */
Mat MatchingAlgos::returnLBPHistogram(Mat image) {
//	LBP implementation
	image.convertTo(image, CV_16S);
	Mat T1 = (Mat_<double>(3, 3) << 1, 0, 0, 0, -1, 0, 0, 0, 0);
	Mat T2 = (Mat_<double>(3, 3) << 0, 1, 0, 0, -1, 0, 0, 0, 0);
	Mat T3 = (Mat_<double>(3, 3) << 0, 0, 1, 0, -1, 0, 0, 0, 0);
	Mat T4 = (Mat_<double>(3, 3) << 0, 0, 0, 0, -1, 1, 0, 0, 0);
	Mat T5 = (Mat_<double>(3, 3) << 0, 0, 0, 0, -1, 0, 0, 0, 1);
	Mat T6 = (Mat_<double>(3, 3) << 0, 0, 0, 0, -1, 0, 0, 1, 0);
	Mat T7 = (Mat_<double>(3, 3) << 0, 0, 0, 0, -1, 0, 1, 0, 0);
	Mat T8 = (Mat_<double>(3, 3) << 0, 0, 0, 1, -1, 0, 0, 0, 0);
	Mat imT1, imT2, imT3, imT4, imT5, imT6, imT7, imT8;
	filter2D(image, imT1, -1, T1, Point(-1, -1), 0, BORDER_DEFAULT);
	filter2D(image, imT2, -1, T2, Point(-1, -1), 0, BORDER_DEFAULT);
	filter2D(image, imT3, -1, T3, Point(-1, -1), 0, BORDER_DEFAULT);
	filter2D(image, imT4, -1, T4, Point(-1, -1), 0, BORDER_DEFAULT);
	filter2D(image, imT5, -1, T5, Point(-1, -1), 0, BORDER_DEFAULT);
	filter2D(image, imT6, -1, T6, Point(-1, -1), 0, BORDER_DEFAULT);
	filter2D(image, imT7, -1, T7, Point(-1, -1), 0, BORDER_DEFAULT);
	filter2D(image, imT8, -1, T8, Point(-1, -1), 0, BORDER_DEFAULT);

	threshold(imT1, imT1, -1, 1, CV_THRESH_BINARY);
	threshold(imT2, imT2, -1, 1, CV_THRESH_BINARY);
	threshold(imT3, imT3, -1, 1, CV_THRESH_BINARY);
	threshold(imT4, imT4, -1, 1, CV_THRESH_BINARY);
	threshold(imT5, imT5, -1, 1, CV_THRESH_BINARY);
	threshold(imT6, imT6, -1, 1, CV_THRESH_BINARY);
	threshold(imT7, imT7, -1, 1, CV_THRESH_BINARY);
	threshold(imT8, imT8, -1, 1, CV_THRESH_BINARY);
//	LBP image
	Mat imLBP = imT1 + (2 * imT2) + (4 * imT3) + (8 * imT4) + (16 * imT5)
			+ (32 * imT6) + (64 * imT7) + (128 * imT8);

	imLBP.convertTo(imLBP, CV_8U);
	Mat imLBPHistogram;
	float range[] = { 0, 256 };
	const float* histRange = { range };
	int histSize = 256;
//	Compute histogram for the LBP image
	calcHist(&imLBP, 1, 0, Mat(), imLBPHistogram, 1, &histSize, &histRange,
			true, false);

	return imLBPHistogram;
}

/**
 * Returns LBP distance between the cropped image and Template.
 */
double MatchingAlgos::returnLBPDistance(Mat CroppedImage) {

//	Compute LBP histogram for the Cropped image and normalize the histogram
	Mat Lbp1 = returnLBPHistogram(CroppedImage);
	normalize(Lbp1, Lbp1, 0, 1, NORM_MINMAX, -1, Mat());
//	Compute LBP histogram for the Template and normalize the histogram
	Mat Lbp2 = returnLBPHistogram(Template);
	normalize(Lbp2, Lbp2, 0, 1, NORM_MINMAX, -1, Mat());
//  Computing the Chi-Square distance between Histogram.
	Mat diff, factors;
	diff = Lbp1 - Lbp2;
	pow(diff, 2, diff);
	Mat denom = Lbp1 + Lbp2;
	divide(diff, denom, factors);
	double dist = 0.5 * sum(factors)[0];

	return dist;
}

/**
 *
 * Returns cropped image for a ROI given by CirclePosition
 *
 * Apply circular mask for the radius CirclePosition[2] if ApplyMask is true.
 *
 */
Mat MatchingAlgos::returnCroppedImage(Vec3f &CirclePosition, Mat image,
		bool ApplyMask) {
	Point center(CirclePosition[0], CirclePosition[1]);
	int radius = CirclePosition[2];
//	Boundry Condition check
	int x1 = center.x - radius;
	if (x1 < 0)
		x1 = 0;
	int y1 = center.y - radius;
	if (y1 < 0)
		y1 = 0;
	int x2 = (2 * radius);
	if ((x1 + x2) > image.cols)
		x2 = x1 - image.cols;
	int y2 = (2 * radius);
	if ((y1 + y2) > image.rows)
		y2 = y1 - image.rows;
	cv::Rect roi(x1, y1, x2, y2);
//	Copy ROI image
	Mat CI = image(roi);
	if (ApplyMask) {
		resize(CI, CI, Size(100, 100));
//		Create Mask
		Mat Mask(100, 100, CV_8UC1);
		Mask = Mask * 0;
		circle(Mask, Point(50, 50), 50, 255, -1, 8, 0);
		Mat CIMasked;
//		Apply Mask
		CI.copyTo(CIMasked, Mask);
		return CIMasked;
	}
	return CI;
}

MatchingAlgos::~MatchingAlgos() {
	// TODO Auto-generated destructor stub
}

