/*
 * CamModule.cpp
 *
 *  Created on: 14-Sep-2017
 *      Author: sriram
 */

#include "SpeedCalculation.h"

SpeedCalculation::SpeedCalculation(vector<Vec3f> MatchBalls_) {
	computeFirstBallPosProj();
	MatchBalls = MatchBalls_;
}

/**
 * Gets the first balls 3D position based on the information given.
 */
void SpeedCalculation::computeFirstBallPosProj() {
//	Get Z of the ball in 3D world
	double firstBallTiltZProj = distanceFromBallMeter
			/ cos(cameraTiltAngleDegree * PI / 180.0);
//	Get Y of the ball in 3D world
	double firstBallTiltYProj = sin(cameraTiltAngleDegree * PI / 180.0)
			* firstBallTiltZProj;
	firstBall3DProj = Point3d(0, firstBallTiltYProj * 1000,
			firstBallTiltZProj * 1000);
}

/**
 * Computes speed for all the balls;
 */
void SpeedCalculation::getAllBallSpeed() {
	for (uint i = 1; i < balls.size(); i++) {
		Point3d velocityVector = balls[i].WorldPosition
				- balls[i - 1].WorldPosition;
//		Compute the distance
		double distance = sqrt(
				pow(velocityVector.x, 2) + pow(velocityVector.y, 2)
						+ pow(velocityVector.z, 2));
//		speed calculated in KM/Hr
		double speed = (distance * (fps/10) * 36) / 1000;
		AllBallspeed.push_back(speed);
#ifdef Result
		cout << "Frame " << i << " Ball Speed = " << speed <<" Km/Hr    ,    "<< endl << flush;
#endif
	}
}
/**
 * Fits the 3D position of the MatchBalls
 * Compute the speed
 */
void SpeedCalculation::computeSpeed() {
//	Computes the 3D position for all the balls
	getAllBall3DPos();
//	Get speed for all the balls
	getAllBallSpeed();
}

/**
 * Returns All the ball speedValues
 */
double SpeedCalculation::getBallSpeed(int Pos){
	return AllBallspeed[Pos];
}

/**
 * Returns the Average Speed of the ball.
 */
double SpeedCalculation::getAverageSpeed() {
	return (std::accumulate(AllBallspeed.begin(), AllBallspeed.end(), 0.0)
			/ AllBallspeed.size());
}

/**
 * Do Trajectory correction in Z axis for all the balls. (Refine the depth)
 */
vector<float> SpeedCalculation::getZtrajector4AllBall(){

//	Compute Linear least squares to compute the Z position for each ball
	cv::Mat x(MatchBalls.size(), 3, CV_32F);
	cv::Mat y(MatchBalls.size(), 1, CV_32F);
	for (uint i = 0; i < MatchBalls.size(); i++) {
		double xDist2OpticalCenterPx = (double(MatchBalls[i][0])
				- double(imageWidthPx / 2));
		double yDist2OpticalCenterPx = (double(MatchBalls[i][1])
				- double(imageHeightPx / 2));
		double BallDisplaceFromOpticalCenterPx = sqrt(
				pow(xDist2OpticalCenterPx, 2) + pow(yDist2OpticalCenterPx, 2));

		double ballDistMagFromFirstBallmm = sqrt(
				pow(focalLengthmm, 2)
						+ pow(BallDisplaceFromOpticalCenterPx * pixelSizemm,
								2));
//		initial ball depth is calculated
		double ballDepth = ((baseBallRadiusMeter * 1000)
				* (ballDistMagFromFirstBallmm))
				/ (MatchBalls[i][2] * pixelSizemm);

		x.at<float>(i, 0) = pow(i, 2);
		x.at<float>(i, 1) = i;
		x.at<float>(i, 2) = 1;
		y.at<float>(i, 0) = (ballDepth);
	}
	Mat wx = ((x.t() * x).inv()).t() * x.t() * y;

//	Actual ball depth is calculated
	vector<float> zTrajectory;
	for (uint i = 0; i < MatchBalls.size(); i++) {
		float xVal = (float(i) * float(i) * wx.at<float>(0, 0))
				+ (float(i) * wx.at<float>(1, 0)) + wx.at<float>(2, 0);
		zTrajectory.push_back(xVal);
	}
	return zTrajectory;
}

/**
 * Returns 3D point for all the balls
 */
void SpeedCalculation::getAllBall3DPos() {
	ballData ballinfo;
//	first ball info is pushed to balls vector
	ballinfo.WorldPosition = firstBall3DProj;
	ballinfo.PosRadiusIM = MatchBalls[0];
	balls.push_back(ballinfo);
//	get the Z world point for each ball
	vector<float> zTrajectory = getZtrajector4AllBall();
	for (uint i = 1; i < MatchBalls.size(); i++) { //For each circle
		ballData ball;
//		Compute the X,Y and Z world positions for the ball
		ball.PosRadiusIM = MatchBalls[i];
		double xDistPx = (double(MatchBalls[i][0]) - double(MatchBalls[0][0]));
		double yDistPx = (double(MatchBalls[i][1]) - double(MatchBalls[0][1]));
		ball.WorldPosition.z = zTrajectory[i];

		ball.WorldPosition.y = firstBall3DProj.y
				+ ((yDistPx * pixelSizemm * ball.WorldPosition.z)
						/ focalLengthmm);
		ball.WorldPosition.x = firstBall3DProj.x
				+ ((xDistPx * pixelSizemm * ball.WorldPosition.z)
						/ focalLengthmm);
		balls.push_back(ball);
#ifdef Visualize
		cout << "Frame " << i << " = " << ball.WorldPosition << " "
				<< ((yDistPx * pixelSizemm * ball.WorldPosition.z)
						/ focalLengthmm) << " "
				<< ((xDistPx * pixelSizemm * ball.WorldPosition.z)
						/ focalLengthmm) << " -- Radius --" << MatchBalls[i][2]
				<< endl << flush;
#endif
	}
}

SpeedCalculation::~SpeedCalculation() {
	// TODO Auto-generated destructor stub
}

