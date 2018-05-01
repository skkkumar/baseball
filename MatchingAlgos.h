/*
 * MatchingAlgos.h
 *
 *  Created on: 14-Sep-2017
 *      Author: sriram
 */

#ifndef MATCHINGALGOS_H_
#define MATCHINGALGOS_H_
#include "common.h"

/**
 * This Class is responsible for all the Matching algorithm implementation
 */
class MatchingAlgos {
private:
	/**
	 * Return Sum of absolute difference between 2 images
	 *
	 * Unused function
	 */
	double returnSADVal(Mat CroppedImage1, Mat CroppedImage2);
	/**
	 * Returns Smoothness of a trajectory. This was to attempt without template matching
	 * to identify the ball with the smoothness of the trajectory, based on velocity and
	 * direction. Stopped developing this as it may require more time.
	 *
	 * Unused function
	 */
	double ReturnTrajectorySmoothness(Point2f P1, Point2f P2);
	/**
	 *
	 * Returns the Histogram of Local binary pattern of an image.
	 *
	 */
	Mat returnLBPHistogram(Mat image);
protected:
	Mat Template;   // Stores the template matching.
	/**
	 *
	 * Returns cropped image for a ROI given by CirclePosition
	 *
	 * Apply circular mask for the radius CirclePosition[2] if ApplyMask is true.
	 *
	 */
	Mat returnCroppedImage(Vec3f &CirclePosition, Mat image, bool ApplyMask = true);
	/**
	 * Returns LBP distance between the cropped image and Template.
	 */
	double returnLBPDistance(Mat CroppedImage1);
public:
	MatchingAlgos();
	virtual ~MatchingAlgos();
};

#endif /* MATCHINGALGOS_H_ */
