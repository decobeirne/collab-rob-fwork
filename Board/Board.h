#include "../Common/RobotDefs.h"

#ifndef ROBOT
/*!
\file Board.h
\brief Blackboard agent.
*/
#ifndef BOARD_H
#define BOARD_H

#include "BoardManagement.h"

#if defined(IS_WIN) && (defined(SIMULATION) || defined(BOARD))
#include "Visualisation.h"
#endif

#include "Data/BoardDatabase.h"
#include "Map/BoardMapProcessing.h"
#include "Comm/BoardComm.h"



//! Blackboard agent with which robots communicate
typedef struct Board_
{
	BoardDatabase db;
} Board;

//! Constructor
Board initBoard (
#ifdef IS_WIN
	IplImage *globMapIplImage,
	IplImage *localMapIplImage
#endif
	);

//! Destructor
void clearBoard (Board *b);

//! Update all data with that submitted from robots
void Board_processData (Board *b, const int updateVisualisation);

//! Finish mission and print some data
void Board_finish (Board *b);

#ifdef BOARD
//! Standalone blackboard.
void Board_start();
#endif






//! Parameter passed to each thread; contains data for processing robot requests
typedef struct BoardThreadParams_
{
	CommData commData;
	BoardDatabase *db;
	int index;
	FILE *xmlLog;
} BoardThreadParams;

//! Constructor
BoardThreadParams initBoardThreadParams (BoardDatabase *db, const int index);

//! Destructor
void clearBoardThreadParams (BoardThreadParams *params);




#endif  // ifndef
#endif
