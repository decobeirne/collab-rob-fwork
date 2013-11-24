#include "../../Common/RobotDefs.h"

#if !defined(BOARD)

#ifndef EXP_H
#define EXP_H

#include "../Data/BehaviourData.h"
#include "../Data/RobotDatabase.h"


void Exploration_calcProfit (ExplorationData *e, RobotDatabase *db);


int Exploration_checkIfExpTarIsValid (RobotDatabase *db);

void Exploration_adopt (RobotDatabase *db);

void Exploration_isAtDest (RobotDatabase *db);

void Exploration_ikFailed (RobotDatabase *db);

//! Check how mapped the current target is
int Exploration_checkNavigationState (RobotDatabase *db, Dest *dest);

#endif // !defined(BOARD)
#endif // ifndef EXP_H


