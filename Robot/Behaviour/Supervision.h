#include "../../Common/RobotDefs.h"

#ifndef BOARD

#ifndef SUP_H
#define SUP_H

#include "BehaviourCore.h"
#include "../Data/RobotDatabase.h"

PointI Supervision_getMostApplicableSupArea (
	const PointF currentLoc);

//! Calculate maximum profit achieveable through SUPERVISION behaviour.
void calcProfitSUPERVISION (SupervisionData *e, RobotDatabase *db);

int Supervision_isProposalOrBidPending (RobotDatabase *db);

void Supervision_makeProposal (RobotDatabase *db);

int Supervision_getNPartners (RobotDatabase *db);

PROPOSAL_STATUS Supervision_checkProposalStatus (RobotDatabase *db);

void adoptSUPERVISION (RobotDatabase *db);

void isAtDestSUPERVISION (RobotDatabase *db);

void Supervision_ikFailed (RobotDatabase *db);


























#if 0

__inline PointI Supervision_supAreaIdFromCentre (const PointF centre)
{
	PointI pt;
	pt.x = (centre.x - SUP_DIMS / 2) / SUP_DIFF;
	pt.y = (centre.y - SUP_DIMS / 2) / SUP_DIFF;
	return pt;
}

__inline PointI Supervision_supAreaIdFromCentreI (const PointI centre)
{
	PointI pt;
	pt.x = (centre.x - SUP_DIMS / 2) / SUP_DIFF;
	pt.y = (centre.y - SUP_DIMS / 2) / SUP_DIFF;
	return pt;
}

__inline float Supervision_getNExpCellsMappedPerIter (const int currentNExpCellsMapped)
{
	/*!
	Value obtained from graphing nExpCellsMapped over iteration in grossAttribWrtAreaMapped.xlsx.
	Recent graph in dir: experiment_sim_1376174318-20130810-Sat-233838__2robs__collab.
	*/
//	float f = 0.58f ? (currentNExpCellsMapped < 70) : 0.1f;
//	float f = (currentNExpCellsMapped < 70) ? 0.58f : (currentNExpCellsMapped < 90 ? 0.2f : 0.05f);
	float f = (currentNExpCellsMapped < 120) ? 0.58f : 0.1f;
	return f;
}

__inline float Supervision_estAvgStdDevOverLoopClose (const float supStdDev)
{
	/*!
	Value obtained from graphing offsetSession0, etc. in closeLoopScans2.xlsx
	*/
	return (supStdDev + 1.659276455f);
}

__inline float Supervision_estResourcesPerIter()
{
	/*!
	From exploration and gtep tarets in logs. This is estimated resources per iteration.
	*/
	return 20.0f;
}

__inline float Supervision_getTypicalCloseLoopDuration()
{
	/*!
	Value obtained from closeLoopScans2.xlsx
	*/
	return 119.0f;
}

__inline float Supervision_getSupResourcesPerIter()
{
	/*!
	Had previously been using BATTERY_LOSS_IDLE (0.3f)
	*/
	return 0.2f;
}

__inline float Supervision_getAllocRatio()
{
//	return (BATTERY_LOSS_IDLE / BATTERY_LOSS_MOVE);
	return (0.2f / BATTERY_LOSS_MOVE);
}

#else

PointI Supervision_supAreaIdFromCentre (const PointF centre);

PointI Supervision_supAreaIdFromCentreI (const PointI centre);

float Supervision_getNExpCellsMappedPerIter (const int currentNExpCellsMapped);

float Supervision_estAvgStdDevOverLoopClose (const float supStdDev);

float Supervision_estResourcesPerIter();

float Supervision_getTypicalCloseLoopDuration();

float Supervision_getSupResourcesPerIter();

float Supervision_getAllocRatio();

#endif




































void Supervision_checkExplorers (RobotDatabase *db, const int readCoalitions);

void Supervision_leaveCoalition (RobotDatabase *db, const int explorerOrSupervisor, const char *reason);

//! \todo DELETE ME DEPRECATED
float Supervision_calcBidSplitProfit (RobotDatabase *db, const ProfitTemp *temp, const Proposal *proposal, const float currentMaxProfit);

void Supervision_calcMovesForProposal (
	FILE *xmlLog,
	FollowPathData *followPathData,
	Image *navMap,
	Image *localMap,
	uchar *obstructedCellGrid,
	uchar *unreachableGrid,
	int *hasUnreachableLocalMapGridBeenUpdated,
	const GeometryConstants *geometryConstants,
	CamVectors *camVectors,
	const IkConstants *ikConstants,
	const UncertaintyConstants *uncertaintyConstants,
	const float camPOffset,
	const Pose pose,
	const float stdDev,
	const int index,
#if defined(SIMULATION) || defined(BOARD)
	uchar *boardUnreachableLocalMapGrid,
	BoardSensorData *boardSensorData,
#else
	RobotDatabase *db,
#endif
	ProfitTemp *temp,
	const PointF pt);

void Supervision_considerProposals (RobotDatabase *db);

void Supervision_checkCollaborationState (RobotDatabase *db);

#endif // SUP_H
#endif // ifndef BOARD

