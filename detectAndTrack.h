/*
 * detectAndTrack.h
 *
 *  Created on: 14-Sep-2017
 *      Author: sriram
 */

#ifndef DETECTANDTRACK_H_
#define DETECTANDTRACK_H_
#include "common.h"
#include "MatchingAlgos.h"
#include "RefineBallPos.h"
#include "SpeedCalculation.h"

/**
 * This class is responsible for Background subtration, Hough circles, Obtain Matching Balls and Refine Circles.
 */
class detectAndTrack: public MatchingAlgos, public RefineBallPos {
private:
	Ptr<BackgroundSubtractor> pMOG2; // Object for Background subtraction.
	vector<Vec3f> MatchingBalls; // Final Matching Balls list
	vector<Mat> BGsubImages; // Foreground segmented images list
	vector<Mat> images; // images list
	vector<vector<Vec3f> > circlesFrames; // Circle for each frame
	/**
	 * This function will apply Background subtration for do circle detection for each of the frame.
	 */

	vector<Vec3f> BGSegAndCircleDet(Mat image);
	/**
	 * Compute all the circles for each of the frame.
	 */
	bool getAllCirclesFromFrame();
	/**
	 * Doing Template Matching with a Template for all the images.
	 *
	 * Returns : Corrected Circles of the ball
	 */
	vector<Vec3f> TrackwithTemplate();
	/**
	 * Do Template Matching + Circle radius correction + trajectory Correction (Outlier detection)
	 *
	 * Returns : Trajectory, center and radius corrected Circles of the ball
	 */
	bool BallMatch();
	/**
	 * This function extracts all the frame from the folder;
	 */
	bool getAllFrame();

public:
	detectAndTrack();
	/**
	 * Gets all the matching balls
	 *
	 * Matching balls will be saved in MatchingBalls.
	 */
	bool MatchAllBalls();
	/**
	 * Computes the speed between 2 balls;
	 */
	double ComputeBallSpeed();
	virtual ~detectAndTrack();
};

#endif /* DETECTANDTRACK_H_ */
