#include "../../Common/RobotDefs.h"

#if !defined(BOARD)

/*!
\file BehaviourAdoption.h
\brief Functions related to adopting behaviours.
*/
#ifndef BEHAVIOUR_ADOPTION_H
#define BEHAVIOUR_ADOPTION_H

#include "../Data/RobotDatabase.h"

//	#define COLLAB_EXP



//! Set behaviour to available. Tell board if provisional map data is being abandoned.
void adoptAVAILABLE (RobotDatabase *db, const int leaveProvisionalMapData);

//! Set behaviour to stuck.
void adoptStuck (RobotDatabase *db);

//! Return the profit ratio (as calculated when adopting) of the current (base) behaviour
float BehaviourAdoption_getBehaviourRatio (
	RobotDatabase *db,
	const BEHAVIOUR behaviour);

//! Return destination structure associated with behaviour
Dest* getBehaviourDest (RobotDatabase *db, const BEHAVIOUR behaviour);

//! Get behaviour to which profit for the current map scan should be accredited.
BEHAVIOUR getBehaviourToAccredit (RobotDatabase *db);

//! Get index of target to which profit for the current map scan should be accredited.
int getTargetToAccredit (RobotDatabase *db);

//! Check state when performing coarse nav
void BehaviourAdoption_checkCoarseNavMoves (
	RobotDatabase *db,
	const Pose pose,
	Dest *dest,
	IKData *currentIKData);

//! Calculate moves required to get to the dest and check for collisions. Don't calculate cost, etc.
void checkMoves_checkState (RobotDatabase *db, const Pose pose, Dest *dest, IKData *ikData);

//! Testing... override autonomous behaviour and accept input from user.
void BehaviourCore_manualBehaviour (RobotDatabase *db);






#endif // BEHAVIOUR_ADOPTION_H
#endif // !defined(BOARD)
