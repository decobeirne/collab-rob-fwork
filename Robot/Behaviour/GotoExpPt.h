#include "../../Common/RobotDefs.h"

#if !defined(BOARD)

#ifndef GOTO_EXP_PT_H
#define GOTO_EXP_PT_H

#include "BehaviourCore.h"
#include "../Data/BehaviourData.h"
#include "../Data/RobotDatabase.h"





void GotoExpPt_setLocalMapExhausted (RobotDatabase *db);

float GotoExpPt_calcBehaviourChangeOverhead (
	const int explorationCoalition,
	const float stdDev);


//! Calculate max achieveabe profit given current state.
void GotoExpPt_calcProfit (GotoExpPtData *g, RobotDatabase *db);

/*!
When adopting GOTO_EXP_PT when in a coalition, the map data collected may
be adjusted later if the robot is able to improve its location estimate. This 
behaviour is adopted such that it does not consider the updated certainty as it
is not known for sure that the map data actually will be updated, and GOTO_EXP_PT
has very little claim to any of the increased profit.
*/
void GotoExpPt_calcProfitForCoal (GotoExpPtCollabData *g, RobotDatabase *db);



void GotoExpPtCollab_adopt (RobotDatabase *db);

void GotoExpPt_adopt (RobotDatabase *db);

void GotoExpPt_isAtDest (RobotDatabase *db, const int isCollabGtep);

void GotoExpPt_ikFailed (RobotDatabase *db);






#endif // GOTO_EXP_PT_H
#endif // !defined(BOARD)


