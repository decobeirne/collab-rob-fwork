#include "../../Common/RobotDefs.h"

#if defined(IS_WIN) && defined(SIMULATION)

#ifndef ROBOT_RECOGNITION_TRAINING_H
#define ROBOT_RECOGNITION_TRAINING_H

#include "../../Common/RobotCore.h"

int RobotRecognitionTraining_setupTrainingData(TrainingData *data);

void RobotRecognitionTraining_writeFeaturesForTrainingNEW2();

#endif // ifndef ROBOT_RECOGNITION_TRAINING_H

#endif // defined(IS_WIN) && defined(SIMULATION)
