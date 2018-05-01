/*
 * main.cpp
 *
 *  Created on: 09-Sep-2017
 *      Author: sriram
 */
#include "detectAndTrack.h"

int main(int argc, char** argv) {
	detectAndTrack* data = new detectAndTrack();
	data->MatchAllBalls();
	cout << "Average Speed = " << data->ComputeBallSpeed() << " Km/Hr"<< endl << flush;
	return 0;
}
