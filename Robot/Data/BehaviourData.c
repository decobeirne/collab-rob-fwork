
#include "BehaviourData.h"
#include "../../Common/Map/MapCore.h"




ProfitData initProfitData()
{
	ProfitData p;
	p.ratio = MIN_FLT;
	return p;
}





AvailableData initAvailableData()
{
	AvailableData d;
	d.id = AVAILABLE;
	return d;
}

StuckData initStuckData()
{
	StuckData d;
	d.id = STUCK;
	return d;
}

BacktrackData initBacktrackData()
{
	BacktrackData d;
	d.id = BACKTRACK;
	d.dest = initDest();
	d.cellHalfSize = 2;
	return d;
}

CloseLoopData initCloseLoopData()
{
	CloseLoopData d;
	d.id = CLOSE_LOOP;
	d.profit = initProfitData();
	d.reconFlag = NOT_DOING_RECON;
	d.nReconSteps = 0;
	d.isLoopClosePossible = 0;
	d.isMoreMapDataComing = 0;
	d.currentCloseLoopSession = 0;
	d.isInSupervisionArea = 0;

	return d;
}

ExplorationData initExplorationData()
{
	ExplorationData d;
	d.id = EXPLORATION;
	d.profit = initProfitData();
	d.profitToGtep = 0.0f;
	d.profitToCoal = 0.0f;
	d.isFollowPathImpossible = 0;

	return d;
}

void ExplorationData_print(const ExplorationData *e, FILE *f, const int printProfitData)
{
	fprintf (f, "<ExplorationData>\
id=%d isFollowPathImpossible=%d profitToCoal=%f profitToGtep=%f\
</ExplorationData>\n",
		e->id,
		e->isFollowPathImpossible,
		e->profitToCoal,
		e->profitToGtep);

	if (printProfitData)
	{
		ProfitData_print (&e->profit, f);
	}
}

extern void BehaviourCore_printBehaviourProfit (
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

void printBehaviourProfit_tail (FILE *f, const int isMaxProfit);

void ExplorationData_printOutput (ExplorationData *e, FILE *f)
{
	ProfitData *p;

	fprintf (f, "<OutputEXPLORATION>\n");

	fprintf (f, "id=%d isFollowPathImpossible=%d profitToCoal=%f profitToGtep=%f\n",
		e->id, e->isFollowPathImpossible,
		e->profitToCoal,
		e->profitToGtep);

	p = &e->profit;
	BehaviourCore_printBehaviourProfit (
		f, EXPLORATION,
		1,
		1,
		&p->dest,
		p->ratio,
		p->gross,
		p->expenditure,
		p->resources,
		p->nSteps,
		p->gainNMapped,
		p->stdDevAtDest,
		p->stdDevInc,
		p->initialNMapped);
	printBehaviourProfit_tail (f, 1);

	fprintf (f, "</OutputEXPLORATION>\n");
}

GotoExpPtData initGotoExpPtData()
{
	GotoExpPtData d;
	d.id = GOTO_EXP_PT;
	d.profit = initProfitData();
	d.sessionIndex = -1;
	d.nextSessionIndex = 0;
	d.isFollowPathImpossible = 0;
	d.requestedLocalMap = initPointI (MIN_FLT, MIN_FLT);

	return d;
}

void GotoExpPtData_print (
	const GotoExpPtData *g,
	FILE *f,
	const int printProfitData)
{
	fprintf (f, "<GotoExpPtData>\
id=%d isFollowPathImpossible=%d sessionIndex=%d nextSessionIndex=%d\
</GotoExpPtData>\n",
		g->id,
		g->isFollowPathImpossible,
		g->sessionIndex,
		g->nextSessionIndex);

	if (printProfitData)
	{
		ProfitData_print (&g->profit, f);
	}
}

void GotoExpPtData_printOutput (
	const GotoExpPtData *g,
	FILE *f)
{
	fprintf (f, "<OutputGOTO_EXP_PT>\n");

	fprintf (f, "id=%d isFollowPathImpossible=%d nextSessionIndex=%d sessionIndex=%d\n",
		g->id, g->isFollowPathImpossible, g->nextSessionIndex, g->sessionIndex);

	BehaviourCore_printBehaviourProfit (
		f, GOTO_EXP_PT,
		1,
		1,
		&g->profit.dest,
		g->profit.ratio,
		g->profit.gross,
		g->profit.expenditure,
		g->profit.resources,
		g->profit.nSteps,
		g->profit.gainNMapped,
		g->profit.stdDevAtDest,
		g->profit.stdDevInc,
		g->profit.initialNMapped);
	printBehaviourProfit_tail (f, 1);

	fprintf (f, "</OutputGOTO_EXP_PT>\n");
}


GotoExpPtCollabData initGotoExpPtCollabData()
{
	GotoExpPtCollabData d;
	d.id = GOTO_EXP_PT_COLLAB;
	d.profit = initProfitData();
	d.isFollowPathImpossible = 0;

	return d;
}

SupervisionData initSupervisionData()
{
	SupervisionData d;
	d.id = SUPERVISION;
	d.profit = initProfitData();
	d.isProposalMade = 0;
	d.isBidMade = 0;
	d.makeProposal = 0;
	d.isAtFinalDest = 0;
	d.arrivedAtDest = 0;
	d.wasPropSuccessful = 0;
//	d.currentCoalRatio = MIN_FLT;
	d.proposalStdDev = 0.0f;
//	d.proposalExpCellsMapped = 0.0f;
//	d.proposalDuration = 0.0f;
//	d.proposalTotalProfit = 0.0f;
	d.isFollowPathImpossible = 0;

	return d;
}

BehaviourData initBehaviourData()
{
	BehaviourData b;
	b.exploration = initExplorationData();
	b.gotoExpPt = initGotoExpPtData();
	b.gotoExpPtCollab = initGotoExpPtCollabData();
	b.available = initAvailableData();
	b.backtrack = initBacktrackData();
	b.followPath = initFollowPathData();
	b.stuck = initStuckData();
	b.closeLoop = initCloseLoopData();
	b.supervision = initSupervisionData();

	b.maxBehaviour = AVAILABLE;
	b.maxProfit = MIN_FLT;
	b.checkPaths = 0;
	b.state = 0;
	b.shouldBacktrack = 0;
	b.behaviour = AVAILABLE;
	b.baseBehaviour = AVAILABLE;
	b.behaviourAtLastMove = AVAILABLE;
	b.targetIndex = -1;
	b.targetAtLastMove = -1; // AVAILABLE;
	b.targetCounter = 0;
	b.stack = initList();

	return b;
}

void BehaviourData_dtor (BehaviourData *b)
{
	FollowPathData_clear (&b->followPath);

	List_clear (&b->stack, 0);
}






ProfitTemp initProfitTemp()
{
	ProfitTemp t;
	t.isFollowPathImpossible = 0;
	return t;
}


//! Set values for optimum target in behaviour data struct.
void setProfitData (ProfitData *d, ProfitTemp *temp, const float initialNMapped)
{
	d->dest = temp->dest;
	d->ratio = temp->ratio;
	d->gross = temp->gross;
	d->expenditure = temp->expenditure;
	d->resources = temp->resources;
	d->nSteps = temp->nSteps;
	d->initialNMapped = initialNMapped;
	d->gainNMapped = temp->gainNMapped;
	d->stdDevAtDest = temp->stdDevAtDest;
	d->stdDevInc = temp->stdDevInc;
}



#if defined(USE_CLOUD)
extern void MapCore_printExplorationGrid (
	FILE *f,
	__int16 expGrid[EXP_GRID_DIMS][EXP_GRID_DIMS]);

extern void BitArray_display (
	uchar *grid,
	FILE *f,
	const int dimsX,
	const int dimsY);

extern void MapCore_printObstructedCellGrid (
	FILE *f,
	uchar obstructedCellGrid[((NAV_GRID_DIMS*NAV_GRID_DIMS)/8)+1]);

extern void MapCore_printUnreachableExpCellGrid (
	FILE *f,
	uchar unreachableExpCellGrid[((GLOB_EXP_GRID_DIMS*GLOB_EXP_GRID_DIMS)/8)+1]);

extern void MapCore_printUnreachableLocalMapGrid (
	FILE *f,
	uchar unreachableLocalMapGrid[((GLOB_LOC_MAP_GRID_DIMS*GLOB_LOC_MAP_GRID_DIMS)/8)+1]);


#if defined(PRINT_CLOUD_DETAIL)
void GotoExpPtPayload_print (
	GotoExpPtPayload *p,
	FILE *f)
{
	fprintf (f, "<PayloadGOTO_EXP_PT>\n");

	fprintf (f, "niter=%d index=%d stdDev=%f expCoal=%d behaviourChangeOverhead=%f \
considerStdDevInc=%d originCellMidpt=(%d,%d) gridDims=(%d,%d)\n",
		p->niter,
		p->index,
		p->stdDev,
		p->explorationCoalition,
		p->behaviourChangeOverhead,
		p->considerStdDevInc,
		p->originCellMidpt.x,
		p->originCellMidpt.y,
		p->gridDims.x,
		p->gridDims.y);

	Pose_print (&p->pose, f);
	GotoExpPtData_print (&p->gotoExpPtData, f, 0); // Don't print profit data as won't be set yet
	MapCore_printObstructedCellGrid (f, p->obstructedCellGrid);
	MapCore_printLocalMapGrid (f, p->localMapGrid);
	GeometryConstants_print (&p->geometryConstants, f);
	CamVectors_print (&p->camVectors, f);

	fprintf (f, "</PayloadGOTO_EXP_PT>\n");
}

void ExplorationPayload_print (
	ExplorationPayload *p,
	List *unreachableIkTargets,
	FILE *f)
{
	
	fprintf (f, "<PayloadEXPLORATION>\n");

	fprintf (f, "bid=%f index=%d niter=%d sessionIndex=%d stdDev=%f supervisor=%d\n",
		p->bid,
		p->index,
		p->niter,
		p->sessionIndex,
		p->stdDev,
		p->supervisor);

	Pose_print (&p->pose, f);
	ExplorationData_print (&p->expData, f, 0); // Don't print profit data as it won't be set yet
	MapCore_printExplorationGrid (f, p->expGrid);
	GeometryConstants_print (&p->geometryConstants, f);
	CamVectors_print (&p->camVectors, f);
	fprintf (f, "<MappedCellGrid>\n");
	BitArray_display (p->mappedCellGrid, f, EXP_GRID_DIMS, EXP_GRID_DIMS);
	fprintf (f, "</MappedCellGrid>\n");
	MapCore_printObstructedCellGrid (f, p->obstructedCellGrid);
	MapCore_printUnreachableExpCellGrid (f, p->unreachableExpCellGrid);

	fprintf (f, "</PayloadEXPLORATION>\n");
}
#endif





#endif // defined(USE_CLOUD)
