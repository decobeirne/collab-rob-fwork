#include "../../Common/RobotDefs.h"

#if defined(IS_WIN) && defined(SIMULATION)

#ifndef OBSTACLE_RECOGNITION_TRAINING_H
#define OBSTACLE_RECOGNITION_TRAINING_H

#include "../../Common/RobotCore.h"

void ObstacleRecognitionTraining_calcAndWriteFeatures (
	const TrainingData *data,
	const int nTrainingImages,
	const char *trainingDataDir);

int ObstacleRecognitionTraining_setupTrainingData (TrainingData *data);

void ObstacleRecognitionTraining_writeFeaturesForTraining();

#endif // ifndef OBSTACLE_RECOGNITION_TRAINING_H

#endif // defined(IS_WIN) && defined(SIMULATION)
