/*
 * detectAndTrack.cpp
 *
 *  Created on: 14-Sep-2017
 *      Author: sriram
 */

#include "detectAndTrack.h"

detectAndTrack::detectAndTrack() :
		MatchingAlgos() {
	pMOG2 = createBackgroundSubtractorMOG2();
}

/**
 * Gets all the matching balls
 *
 * Matching balls will be saved in MatchingBalls.
 */

bool detectAndTrack::MatchAllBalls() {
	bool AllFrameRead = getAllFrame();
	if (!AllFrameRead)
		return false;
	bool ReadCircle = getAllCirclesFromFrame();
	if (!ReadCircle)
		return false;
	if (!BallMatch()) {
		return false;
	}
	return true;
}

/**
 * Computes the speed between 2 balls;
 */

double detectAndTrack::ComputeBallSpeed() {
	SpeedCalculation cam(MatchingBalls);
	cam.computeSpeed();
	for (uint i = 1; i < images.size() - 1; i++) {
		double ballSpeed = cam.getBallSpeed(i - 1);
#ifdef Result
		Mat Res;
		images[i].copyTo(Res);
		cvtColor(Res, Res, COLOR_GRAY2BGR);
		circle(Res, Point(MatchingBalls[i - 1][0], MatchingBalls[i - 1][1]),
				MatchingBalls[i - 1][2], Scalar(0, 0, 255), 3, 8, 0);
		line(Res, Point(MatchingBalls[i - 1][0], MatchingBalls[i - 1][1]),
				Point(MatchingBalls[i][0], MatchingBalls[i][1]),
				Scalar(255, 0, 0), 3, 8, 0);
		stringstream s;
		s << ballSpeed << " Km/Hr";
		putText(Res, s.str(),
				Point(MatchingBalls[i][0], MatchingBalls[i][1] - 40),
				FONT_HERSHEY_DUPLEX, 1, Scalar(255, 0, 0), 3, 2);
		resize(Res, Res, Size(Res.cols / 2, Res.rows / 2));
		imshow("Speed Result", Res);
		waitKey(2000);
	}
#endif
	return (cam.getAverageSpeed());
}

/**
 * Compute all the circles for each of the frame.
 */

bool detectAndTrack::getAllCirclesFromFrame() {
	for (uint imgIndex = 0; imgIndex < images.size(); imgIndex++) {
//		run background subtration and circle detection
		circlesFrames.push_back(BGSegAndCircleDet(images[imgIndex]));
	}
	return true;
}

/**
 * Doing Template Matching with a Template for all the images.
 *
 * Returns : Corrected Circles of the ball
 */
vector<Vec3f> detectAndTrack::TrackwithTemplate() {
	vector<Vec3f> MatchBalls;

	for (uint imagenum = 1; imagenum < images.size(); imagenum++) {
		int imCircle = 0;
		double imCircleScore = -1;
		for (uint circlenum = 0; circlenum < circlesFrames[imagenum].size();
				circlenum++) {
//			crop the image for the ROI of the circle with Mask applied
			Mat CroppedImage1 = returnCroppedImage(
					circlesFrames[imagenum][circlenum], images[imagenum]);
//			computer LBP distance between Template and circle ROI
			double Score = returnLBPDistance(CroppedImage1);

			if (imCircleScore > Score || imCircleScore == -1) {
				imCircleScore = Score;
				imCircle = circlenum;
			}
		}
//		Declare the center and the ROI
		Point center2(cvRound(circlesFrames[imagenum][imCircle][0]),
				cvRound(circlesFrames[imagenum][imCircle][1]));
		int radius2 = int(BallROIScale * circlesFrames[imagenum][imCircle][2]);
		Vec3f centerNradius(Vec3f(center2.x, center2.y, radius2));
//		Take cropped image of Grayscale and foreground segmented image for the ROI
		Mat CroppedIm = returnCroppedImage(centerNradius, images[imagenum],
				false);
		Mat CroppedBGSIm = returnCroppedImage(centerNradius,
				BGsubImages[imagenum], false);
//		Circle radius and center correction using corelation
		Vec3f bestCircle = findProperCircle(CroppedIm, CroppedBGSIm,
				centerNradius, Template);
		MatchBalls.push_back(bestCircle);
#ifdef Visualize
		Mat Ball;
		images[imagenum].copyTo(Ball);
		cvtColor(Ball, Ball, COLOR_GRAY2BGR);
		circle(Ball,
				Point(MatchBalls[imagenum - 1][0], MatchBalls[imagenum - 1][1]),
				MatchBalls[imagenum - 1][2], Scalar(0, 0, 255), 3, 8, 0);
		Mat Ballsmall;
		imshow("Ball Track Result", Ballsmall);
		waitKey(0);

#endif
	}
	return MatchBalls;
}

/**
 * Do Template Matching + Circle radius correction + trajectory Correction (Outlier detection)
 *
 * Returns : Trajectory, center and radius corrected Circles of the ball
 */
bool detectAndTrack::BallMatch() {
	//Template Matching with LBP feature, Correct the Center and Radius
	vector<Vec3f> MatchBalls = TrackwithTemplate();

	//fit LLS + RANSAC for removing outliers and correcting the center of the circle.
	MatchingBalls = findOutliersAndCorrect(MatchBalls);

	//refine the radius of outlier balls
	float prevRadius = -1;
	for (int i = MatchingBalls.size(); i >= 0; i--) {
		if (MatchingBalls[i][2] != -1)
			prevRadius = MatchingBalls[i][2];
		else {
			Point center2(cvRound(MatchingBalls[i][0]),
					cvRound(MatchingBalls[i][1]));
			int radius2 = int(BallROIScale * prevRadius);
			Vec3f currentCircle(center2.x, center2.y, radius2);
			Vec3f bestCircle = findProperCircle(
					returnCroppedImage(currentCircle, images[i + 1], false),
					returnCroppedImage(currentCircle, BGsubImages[i + 1],
							false), currentCircle, Template);
			if (bestCircle[2] < 0)
				MatchingBalls[i][0] = bestCircle[0];
			MatchingBalls[i][1] = bestCircle[1];
			MatchingBalls[i][2] = bestCircle[2];

			prevRadius = bestCircle[2];
		}
	}

#ifdef Visualize

	for (uint imagenum = 1; imagenum < images.size(); imagenum++) {
		Mat Ball;
		images[imagenum].copyTo(Ball);
		cvtColor(Ball, Ball, COLOR_GRAY2BGR);
		Point center(cvRound(MatchingBalls[imagenum - 1][0]),
				cvRound(MatchingBalls[imagenum - 1][1]));
		int radius = int(MatchingBalls[imagenum - 1][2]);
		circle(Ball, center, radius, Scalar(0, 0, 255), 3, 8, 0);

		Mat Ballsmall;
		imshow("Finding balls", Ballsmall);
		waitKey(0);
	}
#endif
	return true;
}

/**
 * This function will apply Background subtration for do circle detection for each of the frame.
 */

vector<Vec3f> detectAndTrack::BGSegAndCircleDet(Mat image) {

//	Background foreground segmentation
	Mat BGSImage;

	pMOG2->apply(image, BGSImage);

	BGsubImages.push_back(BGSImage);
#ifdef Visualize
	Mat BGSImageSmall;

	imshow("bgsub", BGSImageSmall);
#endif

//	Detect Circles
	vector<Vec3f> circles;

	GaussianBlur(BGSImage, BGSImage, Size(7, 7), 2, 2);
/// Apply the Hough Transform to find the circles
	HoughCircles(BGSImage, circles, CV_HOUGH_GRADIENT, 1, BGSImage.rows / 20,
			200, 10, 12, 22);
#ifdef Visualize
/// Draw the circles detected
	cvtColor(image, image, COLOR_GRAY2BGR);
	for (size_t i = 0; i < circles.size(); i++) {
		Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
		int radius = cvRound(circles[i][2]);
		// circle center
		circle(image, center, 3, Scalar(0, 255, 0), 1, 8, 0);
		// circle outline
		circle(image, center, radius, Scalar(0, 0, 255), 3, 8, 0);
	}
	Mat imageSmall;
	resize(image, imageSmall, Size(image.cols / 2, image.rows / 2));

	imshow("BlobImage", imageSmall);
	waitKey(0);
#endif
	return circles;
}

/**
 * This function extracts all the frame from the folder;
 */

bool detectAndTrack::getAllFrame() {
	Mat image;
	for (int i = 1; i < 16; i++) {
		stringstream filename;
		filename << "../images/IMG" << i << ".bmp";
		image = imread(filename.str(), CV_LOAD_IMAGE_GRAYSCALE); // Read the file

		if (!image.data) {
			cout << "Could not open or find the image" << std::endl;
			return false;
		}
//		Histogram Equalization
		equalizeHist(image, image);
//		Push to an image buffer
		images.push_back(image);
	}
	return true;
}

detectAndTrack::~detectAndTrack() {
// TODO Auto-generated destructor stub
}

