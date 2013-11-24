#include "../../Common/RobotDefs.h"

#if defined(IS_WIN) && defined(SIMULATION)

#ifndef OBSTACLE_RECOGNITION_TESTING_H
#define OBSTACLE_RECOGNITION_TESTING_H

#include "../../Common/BaseTypes.h"
#include "../../Common/RobotCore.h"

/*!
Results grid flags when testing obstacle recognition:
- 0: confident=1 est=correct		colour=red
- 1: confident=1 est=incorrect		colour=white <-- bad
- 2: confident=1 est=wasntKnown		colour=green
- 3: confident=0 est=wasKnown		colour=blue <-- bad
- 4: confident=0 est=wasntKnown		colour=yellow
*/

void ObstacleRecognitionTesting_testCorruptImages();

void ObstacleRecognitionTesting_testObstacleRecognition (
	const int doObstacleRecognition);

void ObstacleRecognitionTesting_displayImagesInDir (
	const int drawGrid);

void ObstacleRecognitionTesting_drawResultsGrid (
	uchar *data,
	const uchar *resultsGrid,
	const int origX,
	const int origY,
	const int gridY);

void ObstacleRecognitionTesting_gridFromOccupancyGrid (
	OccupancyGrid *occupancyGrid,
	uchar *ucharGrid);

#endif // ifndef OBSTACLE_RECOGNITION_TESTING_H

#endif // defined(IS_WIN) && defined(SIMULATION)
