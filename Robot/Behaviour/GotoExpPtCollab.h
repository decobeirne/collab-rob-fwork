#include "../../Common/RobotDefs.h"

#if !defined(BOARD)

#ifndef GOTO_EXP_PT_COLLAB_H
#define GOTO_EXP_PT_COLLAB_H

#include "../Data/RobotDatabase.h"

void GotoExpPtCollab_leaveCoalition (RobotDatabase *db, const int explorerOrSupervisor, const char *reason);

void GotoExpPtCollab_considerProposals (RobotDatabase *db);

void GotoExpPtCollab_checkCollaborationState (RobotDatabase *db);

void GotoExpPtCollab_calcProfit (GotoExpPtCollabData *g, RobotDatabase *db);


#endif // GOTO_EXP_PT_COLLAB_H
#endif // !defined(BOARD)


