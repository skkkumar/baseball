/*
 * SpeedCalculation.h
 *
 *  Created on: 14-Sep-2017
 *      Author: sriram
 */

#ifndef SPEEDCALCULATION_H_
#define SPEEDCALCULATION_H_
#include "CamIntrinsicExtrinsicConst.h"
#include "common.h"
class SpeedCalculation {
public:
	SpeedCalculation(vector<Vec3f> MatchBalls_);
	struct ballData {
		Vec3f PosRadiusIM;
		Point3d WorldPosition;
	};
	/**
	 * Fits the 3D position of the MatchBalls
	 * Compute the speed
	 */
	void computeSpeed();
	/**
	 * Returns the Average Speed of the ball.
	 */
	double getAverageSpeed();
	/**
	 * Returns All the ball speedValues
	 */
	double getBallSpeed(int Pos);
	virtual ~SpeedCalculation();
private:
	/**
	 * Returns 3D point for all the balls
	 */
	void getAllBall3DPos();
	/**
	 * Do Trajectory correction in Z axis for all the balls. (Refine the depth)
	 */
	vector<float> getZtrajector4AllBall();
	/**
	 * Computes speed for all the balls;
	 */
	void getAllBallSpeed();
	Point3d firstBall3DProj; // First ball 3d position is the reference point.
	/**
	 * Gets the first balls 3D position
	 */
	void computeFirstBallPosProj();
	vector<ballData> balls; // Stores All the ball data.
	vector<Vec3f> MatchBalls; // Stores All the Match circles.
	vector<double> AllBallspeed; // Stores All ball speed.
};

#endif /* SPEEDCALCULATION_H_ */
