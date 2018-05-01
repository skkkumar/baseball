/*
 * RefineBallPos.h
 *
 *  Created on: 14-Sep-2017
 *      Author: sriram
 */

#ifndef REFINEBALLPOS_H_
#define REFINEBALLPOS_H_
#include "common.h"

class RefineBallPos {
private:
	/**
	 * This Function is to convolve the Edge image with the Mask.
	 */
	double calculateEdgeConvolution(Mat edgeRoi, Mat mask);
	/**
	 * Fits Line to x and y ball positions. Uses ransac to exclude outliers.
	 *
	 * Returns M and C to draw the line.
	 */
	Mat FitLine2BallTraj(vector<Vec3f> &MatchBalls);
	/**
	 * Check if the ROI is inside the image
	 */
	bool checkROIInside(Rect &smallRoi, Mat &image);
	/**
	 * Fits curve to x and frame number.
	 *
	 * Returns x1 x2 and x3 to fit the curve.
	 */
	Mat FitCurve2XandImage(vector<Vec2f> &newMatchBalls);
protected:
	/**
	 * Finds Circle's proper center and radius.
	 */
	Vec3f findProperCircle(Mat image, Mat BGSimage, Vec3f lbpCircle,
			Mat Template);
	/**
	 * Finds the outlier circles and correct them and returns the corrected circle center.
	 */
	vector<Vec3f> findOutliersAndCorrect(vector<Vec3f> MatchBalls);
public:
	RefineBallPos();
	virtual ~RefineBallPos();
};

#endif /* REFINEBALLPOS_H_ */
