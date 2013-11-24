#include "../../Common/RobotDefs.h"

#ifndef BOARD

#ifndef BAKCTRACK_H
#define BACKTRACK_H

#include "../Data/RobotDatabase.h"

//! Calculate dest to move to to avoid potential collision.
int Backtrack_calcBacktrackDest (RobotDatabase *db);

//! Determine if the robot has avoided successfully any potential collisions.
void Backtrack_checkBactrackState (RobotDatabase *db);

//! Update behaviour when dest has been achieved.
void isAtDestBACKTRACK (RobotDatabase *db);

//! Push backtrack onto behaviour stack, etc.
Dest* Backtrack_setBehaviour (RobotDatabase *db);

//! Check robot pose against navigation map.
int Backtrack_checkIfNecessary (RobotDatabase *db);

//! Revert to original behaviour so that the backtrack dest can be recalculated.
void Backtrack_reset (RobotDatabase *db);

//! Calculate backtrack destination and adopt
void adoptBacktrack (RobotDatabase *db);
#endif
#endif
