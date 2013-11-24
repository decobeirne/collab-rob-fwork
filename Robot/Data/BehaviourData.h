#include "../../Common/RobotDefs.h"

/*!
\file BehaviourData.h
\brief Data on each potential behaviour on each robot.
*/
#ifndef BEHAVIOUR_DATA_H
#define BEHAVIOUR_DATA_H

#include "../../Common/RobotTypes.h"
#include "../../Common/Vector.h"

typedef struct BasicBehaviourData_
{
	BEHAVIOUR id;
} BasicBehaviourData;

typedef struct BasicDataWithProfit_
{
	BEHAVIOUR id;
	ProfitData profit;
} BasicDataWithProfit;

// FollowPathData is in RobotTypes such that is can be used by the board

//! Store data on AVAILABLE
typedef struct AvailableData_
{
	BEHAVIOUR id;
} AvailableData;

//! Constructor
AvailableData initAvailableData();

//! Store data on STUCK
typedef struct StuckData_
{
	BEHAVIOUR id;
	int timeAdopted;
} StuckData;

//! Constructor
StuckData initStuckData();

//! Store data on BACKTRACK
typedef struct BacktrackData_
{
	BEHAVIOUR id;
	Dest dest;
	int cellHalfSize;
} BacktrackData;

//! Constructor
BacktrackData initBacktrackData();

//! Data on performing cooperative localisation and variant of loop closing
typedef struct CloseLoopData_
{
	BEHAVIOUR id;
	ProfitData profit;
	__int8 isLoopClosePossible;							//!< Flags if the robot has a valid sup and is free to explore
	__int8 isMoreMapDataComing;							//!< Flags if more maps are to be submitted
	__int8 isInSupervisionArea;							//!< Indicates what behaviour the robot should adopt
	__int8 nReconSteps;
	__int16 currentCloseLoopSession;					//!< Id of current session. Used when updating data submitted to board
//	__int16 tempSupervisorIndex;
	__int8 initialSync;
	ReconFlag reconFlag;
} CloseLoopData;

//! Constructor
CloseLoopData initCloseLoopData();

//! Store data on EXPLORATION.
typedef struct ExplorationData_
{
	BEHAVIOUR id;
	ProfitData profit;
	float profitToGtep;
	float profitToCoal;
	__int8 isFollowPathImpossible;
} ExplorationData;

//! Constructor
ExplorationData initExplorationData();

void ExplorationData_print(const ExplorationData *e, FILE *f, const int printProfitData);

void ExplorationData_printOutput (ExplorationData *e, FILE *f);

//! Store data on GOTO_EXP_PT.
typedef struct GotoExpPtData_
{
	BEHAVIOUR id;
	ProfitData profit;
	__int8 isFollowPathImpossible;

	// Above this line should be identical to GotoExpPtCollabData

	__int16 sessionIndex;
	__int16 nextSessionIndex;
	PointI requestedLocalMap;
} GotoExpPtData;

//! Constructor
GotoExpPtData initGotoExpPtData();

void GotoExpPtData_print (
	const GotoExpPtData *g,
	FILE *f,
	const int printProfitData);

void GotoExpPtData_printOutput (
	const GotoExpPtData *g,
	FILE *f);

//! Store data on GOTO_EXP_PT_COLLAB.
typedef struct GotoExpPtCollabData_
{
	BEHAVIOUR id;
	ProfitData profit;
	__int8 isFollowPathImpossible;

} GotoExpPtCollabData;

//! Constructor
GotoExpPtCollabData initGotoExpPtCollabData();

//! Store data on SUPERVISION.
typedef struct SupervisionData_
{
	BEHAVIOUR id;
	ProfitData profit;
	__int8 isProposalMade;
	__int8 isBidMade;
	__int8 makeProposal;
	__int8 isAtFinalDest;
	__int8 arrivedAtDest;
	__int8 wasPropSuccessful;
//	float currentCoalRatio;
	float proposalStdDev;
//	int proposalExpCellsMapped;
	PointI area; //!< Index of supervision area
	float estCloseLoopProfit;
//	float proposalTotalProfit;
//	float proposalDuration;
	__int8 isFollowPathImpossible;
} SupervisionData;

//! Constructor
SupervisionData initSupervisionData();



//! Data used by robot's Control class when determining what behaviour to adopt
typedef struct BehaviourData_
{
	ExplorationData exploration;
	GotoExpPtData gotoExpPt;
	GotoExpPtCollabData gotoExpPtCollab;
	FollowPathData followPath;
	AvailableData available;
	BacktrackData backtrack;
	StuckData stuck;
	CloseLoopData closeLoop;
	SupervisionData supervision;

	//!< Indicates behaviour state.
	/*!
		- 1 Was new behaviour just adopted.
		- 2 Is immobilised
	*/
	__int8 state;
	__int8 checkPaths;					//!< Flag if current paths or IK should be re-validated.
	__int8 shouldBacktrack;				//!< Backtrack away from BROAD_OCCUPIED (or BROAD_VEHICLE_TERRAIN) terrain if no new behaviour can be adopted
	BEHAVIOUR behaviour;				//!< Current behaviour.
	BEHAVIOUR baseBehaviour;			//!< Actual behaviour.
	BEHAVIOUR behaviourAtLastMove;		//!< Behaviour when move was made => profit for mapped terrain should be attributed here.
	int targetIndex;					//!< Current target index
	int targetAtLastMove;				//!< Target when last move was made
	int targetCounter;					//!< Index for next target
	BEHAVIOUR maxBehaviour;				//!< New behaviour of maximum profit
	float maxProfit;					//!< Max profit that can be obtained through next behaviour.
	float lastMaxProfit;				
	List stack;							//!< Behaviour stack. High level behaviour is at base.
} BehaviourData;

//! Constructor
BehaviourData initBehaviourData();

//! Destructor. Free data in lists.
void BehaviourData_dtor (BehaviourData *r);















//! Temporary values for calculating profit.
typedef struct ProfitTemp_
{
	// Profit
	float ratio;
	float gross;
	float expenditure;
	float resources;
	float nSteps;
	float gainNMapped;
	float probability;
	float switchOverhead;

	//float nSteps;
	//float resources;
	//float expenditure;
	//float gross;
	//float ratio;
	//float mapGain;
	//float probability;
	//float switchOverhead;

	// Moves
	Dest dest;
	IKData ikData;
	float stdDevAtDest;
	float stdDevInc;
	__int8 isFollowPathImpossible;
	//int blockedDist;
	//float nStepsCloseLoop;
	//float gotoExpPtProfitToAlloc;
	//float proposalGross;

} ProfitTemp;

ProfitTemp initProfitTemp();


//! Set values for optimum target in behaviour data struct.
void setProfitData (ProfitData *d, ProfitTemp *temp, const float initialNMapped);


#if defined(USE_CLOUD)
typedef struct GotoExpPtPayload_
{
	uchar obstructedCellGrid[((NAV_GRID_DIMS*NAV_GRID_DIMS)/8)+1];
	__int16 localMapGrid[GLOB_LOC_MAP_GRID_DIMS][GLOB_LOC_MAP_GRID_DIMS];
	Pose pose;
	float stdDev;

	float behaviourChangeOverhead;
	int considerStdDevInc;
	PointI originCellMidpt;
	PointI gridDims;

	int index;
	int explorationCoalition;
	int niter;
	GeometryConstants geometryConstants;
	CamVectors camVectors;
	IkConstants ikConstants;
	UncertaintyConstants uncertaintyConstants;
	float camPOffset;
	GotoExpPtData gotoExpPtData;
} GotoExpPtPayload;

#if defined(PRINT_CLOUD_DETAIL)
void GotoExpPtPayload_print (
	GotoExpPtPayload *p,
	FILE *f);
#endif

typedef struct ExplorationPayload_
{
	ExplorationData expData;
	__int16 expGrid[EXP_GRID_DIMS][EXP_GRID_DIMS];
	uchar mappedCellGrid[((EXP_GRID_DIMS*EXP_GRID_DIMS)/8)+1];
	uchar obstructedCellGrid[((NAV_GRID_DIMS*NAV_GRID_DIMS)/8)+1];
	uchar unreachableExpCellGrid[((GLOB_EXP_GRID_DIMS*GLOB_EXP_GRID_DIMS)/8)+1];
	Pose pose;
	float stdDev;
	int niter;
	__int8 index;
	int supervisor;
	float bid;
	__int16 sessionIndex;
	float targetDist;
	GeometryConstants geometryConstants; // Remember geometryConstants are robot-specific!!!
	CamVectors camVectors;
	IkConstants ikConstants;
	UncertaintyConstants uncertaintyConstants;
	float camPOffset;
} ExplorationPayload;

#if defined(PRINT_CLOUD_DETAIL)
void ExplorationPayload_print (
	ExplorationPayload *p,
	List *unreachableIkTargets,
	FILE *f);
#endif

#endif // if defined(USE_CLOUD)




#endif // ifndef

