#include "../../Common/RobotDefs.h"

#ifndef ROBOT

/*!
\file BoardCoalitionData.h
\brief Store data on all coalitions.
*/
#ifndef COALITION_DATA_H
#define COALITION_DATA_H

#include "../../Common/RobotCore.h"



//! Data on proposals and offers posted on Board by Robots
typedef struct BoardCoalitionData_
{
	int coalitionIndex;							//!< Index to assign to next coalition allocated
	__int8 isBidSuccessful[MAX_N_ROBOTS];			//!< Flags for each robot whether/not their bid was accepted
	__int8 isPropSuccessful[MAX_N_ROBOTS];			//!< Flags for each robot whether/not their proposal was accepted
	List coalitions;								//!< List of active coalitions on the board.
	List proposals;									//!< List of proposal objects
	List bids;										//!< List of bid objects
} BoardCoalitionData;

//! Constructor
BoardCoalitionData initBoardCoalitionData();

//! Destructor
void freeBoardCoalitionData (BoardCoalitionData *c);


#endif // ifndef
#endif