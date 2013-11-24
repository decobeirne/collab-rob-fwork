#include "../../Common/RobotDefs.h"

#if defined(IS_WIN) && defined(SIMULATION)

#ifndef ROBOT_RECOGNITION_TESTING_H
#define ROBOT_RECOGNITION_TESTING_H

const char* RobotRecognitionTesting_mapColourIdToName (const int colourId);

//! Run robot recognition on dir of unmarked imgs
void RobotRecognitionTesting_runRobotColourRecognitionOnDir();

void RobotRecognitionTesting_testRobotColourRecognition();

//! Load obstacle recognition images to check for false +ves
void RobotRecognitionTesting_testRobotColourRecognitionAgainstBackgroundImages();

//! Test equivalent functionality called from RobotRecognition_detectRobots
void RobotRecognitionTesting_testRobotRecognition (
	const int doBlobDetection);

//! Display cells for each colour in turn
void RobotRecognitionTesting_verifyColourGrids();

#endif // ifndef ROBOT_RECOGNITION_TESTING_H

#endif // defined(IS_WIN) && defined(SIMULATION)
