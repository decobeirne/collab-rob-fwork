#include "../../Common/RobotDefs.h"

#if !defined(BOARD) && !defined(IS_GUMSTIX) && !defined(RERUNNING_GUMSTIX)

#include "SimulatedProcessing.h"

//! Simulate visible robots in visual data
/*!
Iterate through robot poses in group data on board, deciding which robot if any should be
simulated in the robots visual data. This can be carried out using the actual locs of the 
group of robots. In simulation this can be carried out on the robot. 
*/
int detectVisibleRobot_sim (
	RobotDatabase *db,
	const int index,
	const PointF *loc,
	CamVectors *v)
{
	int i;
	int visInd = -1;
	float d, closestDist = MAX_FLT;
	PointF otherRobLoc, relRobLoc;
	RobotData *data;

	for (i = 0; i < N_ROBOTS; ++i)
	{
		if (i == index)
		{
			continue;
		}
		data = &db->groupData.robots[i];

		otherRobLoc.x = data->pose.loc.x;
		otherRobLoc.y = data->pose.loc.y;

#ifdef SIM_ERROR
		otherRobLoc.x += data->actualLocOffset.x;
		otherRobLoc.y += data->actualLocOffset.y;
#endif

		relRobLoc.x = otherRobLoc.x - loc->x;
		relRobLoc.y = otherRobLoc.y - loc->y;

		if (1 == Geometry_isVisualContactNEW (
			relRobLoc,
			db->geometryConstants.cam_xLimit,
			db->geometryConstants.cam_yNegLimit,
			db->geometryConstants.cam_yPosLimit,
			v))
		{
			d = Geometry_dist (
				loc->x,
				loc->y,
				otherRobLoc.x,
				otherRobLoc.y);

			if (d < closestDist)
			{
				visInd = i;
				closestDist = d;
			}
		}
	}

	return visInd;
}

void SimulatedProcessing_calcVisEstCov (
	RobotDatabase *db,
	float *cov)
{
	cov[0] = db->uncertaintyConstants.visEst_final_varFwd;
	cov[1] = db->uncertaintyConstants.visEst_final_covar;
	cov[2] = db->uncertaintyConstants.visEst_final_covar;
	cov[3] = db->uncertaintyConstants.visEst_final_varLat;

	Uncertainty_calcIndividualCovariance (cov, db->status.pose.orient);
	Uncertainty_scaleCovToSim (cov);
}

//! Determine closest visible robot
void detectRobots_sim (RobotDatabase *db,
						const int index)
{
	PointF loc;
	int i;
	RobotStatus *rs = &db->status;
	RobotGroupData *rgd = &db->groupData;
	RobotData *visRobotData;
	float *mat;

	db->sensorData.visRobotIndex = -1;

	loc = rs->pose.loc;
#ifdef SIM_ERROR
	loc.x += rs->actualLocOffset.x;
	loc.y += rs->actualLocOffset.y;
#endif

	EXPLICIT_DEBUG_ASSERT (db->camVectors.currentAdjustedOrient == rs->pose.orient)
	i = detectVisibleRobot_sim (
		db,
		rs->index,
		&loc,
		&db->camVectors);

	if (-1 != i)
	{
		db->sensorData.visRobotIndex = i;
		visRobotData = &rgd->robots[i];
#ifdef SIM_ERROR
		db->sensorData.visRobotRelLoc.x = (visRobotData->pose.loc.x + visRobotData->actualLocOffset.x) - loc.x;
		db->sensorData.visRobotRelLoc.y = (visRobotData->pose.loc.y + visRobotData->actualLocOffset.y) - loc.y;
#else
		db->sensorData.visRobotRelLoc.x = visRobotData->pose.loc.x - loc.x;
		db->sensorData.visRobotRelLoc.y = visRobotData->pose.loc.y - loc.y;
#endif

		mat = db->sensorData.visRobotEstCov.mat;
		SimulatedProcessing_calcVisEstCov (
			db,
			mat);

#ifdef PRINT_EVENTS
		fprintf (db->xmlLog, "<DetectRobots>index=%d relativeLoc=(%f,%f) type=\"sim\" cov=(%f,%f,%f,%f)</DetectRobots>\n",
			db->sensorData.visRobotIndex, db->sensorData.visRobotRelLoc.x, db->sensorData.visRobotRelLoc.y,
			mat[0], mat[1], mat[2], mat[3]);
#endif
	}
}

extern BEHAVIOUR getBehaviourToAccredit (RobotDatabase *db);
extern int getTargetToAccredit (RobotDatabase *db);

void recordMapScan_sim (RobotDatabase *db)
{
	PointF actualLoc;
	Pose *p = &db->status.pose;
	MapScan *ms;
	BEHAVIOUR behaviourToAccredit;
	int targetToAccredit;
#if defined(MAP_SCANS_HAVE_GRIDS)
	OccupancyGrid *grid;
#endif

	actualLoc = p->loc;
#ifdef SIM_ERROR
	actualLoc.x += db->status.actualLocOffset.x;
	actualLoc.y += db->status.actualLocOffset.y;
#endif

	behaviourToAccredit = getBehaviourToAccredit (db);
	targetToAccredit = getTargetToAccredit (db);

#if defined(MAP_SCANS_HAVE_GRIDS)
#if !defined(RERUNNING_IMAGES_ONLY)
#error "MAP_SCANS_HAVE_GRIDS should only be set in SIMULATION mode when RERUNNING_IMAGES_ONLY is set"
#endif

	grid = 0;
#endif

	ms = (MapScan*)malloc (sizeof (MapScan));
	MapScan_init (
		ms,
#if defined(SIM_ERROR)
		actualLoc,
#endif
#if defined(MAP_SCANS_HAVE_GRIDS)
		grid,
#endif
		p->loc,
		p->orient,
		behaviourToAccredit,
		targetToAccredit,
		db->sensorData.localMap->orig,
		db->sensorData.localMapDataIndex,
		db->status.stdDev);

	ms->estdGain = estMapScanGain (
		db->status.scanCentrePt,
		db->sensorData.localMap->orig,
		db->environment.expGrid);

	List_pushValue (&db->sensorData.mapScanList, ms);

	db->sensorData.isUnincorporatedMapScan = 1;
}

#ifdef SIM_ERROR
void insertOrientError (RobotDatabase *db)
{
	float normal0, normal1;
	float current;

	boxMuller (&normal0, &normal1, db->xmlLog);

	// 3.14 radians == 180 => 1 radians = 60 deg
	// ... from experiments where compasses were crap, var was circa 0.07f, but that
	// discounted obvious outliers when calculating... make rough stab of 0.1f
	normal0 *= 0.1f;

	current = db->status.pose.orient;
	db->status.pose.orient += normal0;
#ifdef PRINT_EVENTS
	fprintf (db->xmlLog, "<SimulateOrientError>currentOrient=%14.12f reading=%14.12f</SimulateOrientError>\n",
		current, db->status.pose.orient);
#endif
}
#endif

void SimulatedProcessing_processSensorData (RobotDatabase *db, const int isFakeMove)
{
#ifdef SIM_ERROR
	// Error in orient when rotation/move has huge variance, so for now
	// just insert error for any move.
	if (db->status.isNewPose && db->status.nIterations && !isFakeMove)
	{
		insertOrientError (db);
	}

	// Need to rotate cam vectors again as we now have a different orient
	CamVectors_rotateToRobotOrient (&db->camVectors, db->status.pose.orient);
#endif

	if (1 != N_ROBOTS)
	{
		detectRobots_sim (db, db->status.index);
	}

	if (db->status.isNewPose)
	{
		recordMapScan_sim (db);
	}
}

#endif // !defined(BOARD) && !defined(IS_GUMSTIX) && !defined(RERUNNING_GUMSTIX)
