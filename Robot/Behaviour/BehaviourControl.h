#include "../../Common/RobotDefs.h"

#ifndef BOARD

/*!
\file BehaviourControl.h
\brief Control robot behaviour.
*/
#ifndef BEHAVIOUR_CONTROL_H
#define BEHAVIOUR_CONTROL_H

#include "../Data/RobotDatabase.h"




void BehaviourControl_processBehaviour (RobotDatabase *db);

void BehaviourControl_implementBehaviour (RobotDatabase *db);

void BehaviourControl_checkNavigationState (RobotDatabase *db);

void BehaviourControl_highLevelBehaviour (RobotDatabase *db);

void BehaviourControl_setCollaborativeBehaviour (RobotDatabase *db);

void BehaviourControl_lowLevelBehaviour (RobotDatabase *db);


#endif // ifndef
#endif // BOARD
