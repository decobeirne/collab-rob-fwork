#include "../../Common/RobotDefs.h"

#ifndef BEH_CORE_H
#define BEH_CORE_H

//#include "../../Common/RobotCore.h"
//#include "../../Common/Uncertainty.h"
//#include "../Data/RobotDatabase.h"
//#include "../../Common/Actuator.h"
//#include "../Ik/Ik.h"

#include "../Data/BehaviourData.h"
#include "../../Common/RobotTypes.h"
#include "../../Common/Image.h"
#include "../../Common/Vector.h"


	#define GTEP_ALLOC_PROFIT		0.3f
//	#define GTEP_ALLOC_PROFIT		0.8f
//	#define GTEP_ALLOC_PROFIT		0.5f
//	#define GTEP_ALLOC_PROFIT		0.1f




//! Wipe any current profit data in behaviour struct.
void BehaviourCore_clearProfitData (ProfitData *d);

//! Calculate deduction due based on the probability of another robot mapping the target first.
float calcTargetConflictDeduction (const float gross,
								   const float targetConflicts);

//! Calc profit from mapping target
float calcMappingProfit (const float gainNMapped, const float stdDev);

//! Calculate profit for improving certainty in robot pose from which a target had been mapped
float calcStdDevReductionProfit (const float gainNMapped, const float stdDevReduction);

//! Return target pt from Dest
PointF getTargetToPrint (const Dest *dest);

//! Print profit data for behavior.
void BehaviourCore_printBehaviourProfit (
	FILE *f,
	const BEHAVIOUR behaviour,
	const int calledFromCloudFunc,
	const int isMaxProfit,
	const Dest *dest,
	const float ratio,
	const float gross,
	const float expenditure,
	const float resources,
	const float nSteps,
	const float gainNMapped,
	const float stdDevAtDest,
	const float stdDevInc,
	const float initialNMapped);

//! Print closing tag of profit data
void printBehaviourProfit_tail (FILE *f, const int isMaxProfit);

//! Get location associated with a destination: make be a target or a location.
PointF getDestPt (const Dest *dest);



PointI calcCellMidpt (const PointI pt,
					  const int distBetweenMidpts,
					  const PointI originCellMidpt);

//! Calculate the cost of moving to a dest, and check if a collision will occur.
void calcMoves_evalBehaviour (
	FILE *xmlLog,
	const Image *navMap,
	Image *localMap,
	const GeometryConstants *geometryConstants,
	CamVectors *camVectors,
	const IkConstants *ikConstants,
	const UncertaintyConstants *uncertaintyConstants,
	const float camPOffset,
	const Pose pose,
	Dest *dest,
	float *nMoves,
	float *stdDev,
	const BEHAVIOUR behaviour,
	const int calcGain,
	float *gain);

void BehaviourCore_updateUnreachableGrid (
	FILE *xmlLog,
	const int index,
	const Pose pose,
	uchar *unreachableGrid,
	int *hasUnreachableLocalMapGridBeenUpdated,
	Dest *dest,
	const BEHAVIOUR behaviour);

//! Calc nMoves, stdDev and detect collisions given parameters set in ProfitTemp.
void BehaviourCore_calcMovesAndPathIfReqd (
	FILE *xmlLog,
	FollowPathData *followPathData,
	const Image *navMap,
	Image *localMap, // Not const, as scans are integrated in ordre to calc gain
	const uchar *obstructedCellGrid,
	uchar *unreachableGrid, // May be expGrid, gtepGrid, etc.
	int *hasUnreachableLocalMapGridBeenUpdated,
	const GeometryConstants *geometryConstants,
	CamVectors *camVectors,
	const IkConstants *ikConstants,
	const UncertaintyConstants *uncertaintyConstants,
	const float camPOffset,
	const Pose pose,
	const float stdDev,
	const int index,
	ProfitTemp *temp,
	const BEHAVIOUR behaviour,
	const int calcGain);

void BehaviourCore_printStackToLog (FILE *stream, List *stack);

void BehaviourCore_printStack (List *stack, const char *msg);

//! Get previous behaviour when currently a secondary behaviour such as FOLLOW_PATH
BEHAVIOUR BehaviourCore_getPreviousBehaviour (List *behaviourStack);

#endif // BEH_CORE_H

