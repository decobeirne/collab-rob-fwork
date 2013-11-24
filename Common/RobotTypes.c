
#include "RobotTypes.h"


//
// Instances of global variables.
//
int g_nRobots = 1;
#ifdef BOARD
int g_nRobotThreads = 1;
#endif
int g_duration = 2000;
int g_forceQuit = 0;
int g_nIters = 70;
float ATAN_ORIENT_VAR = 0.0f; //!< Should be set in UncertaintyConstants_init

//
// Camera params as global variables. In simulation we will want to either set
// these up normally to simulate robot exploration, or overrided when training
// cooperative localisation. These should not be accessible on the board, as any
// processing done here should be passed robot-specific parameters
//
#if defined(SIMULATION)

//! On windows we want to know what platform to setup cam params for based on our index
int robotPlatformIds[3] = {2, 3, 4};

float CAM_P_OFFSET_WORLD = MIN_FLT;
float CAM_HEIGHT_WORLD = MIN_FLT;
float CAM_THETA = MIN_FLT;
float FOCAL_LEN = MIN_FLT;
float IMAGE_CENTRE_PIXEL_Y = MIN_FLT;
int CAM_OCCUPANCY_GRID_ORIGIN_X = MIN_FLT;
int CAM_OCCUPANCY_GRID_ORIGIN_Y = MIN_FLT;
int CAM_OCCUPANCY_GRID_Y_EXP = MIN_FLT;
float SCAN_CENTRE_DIST = MIN_FLT;
float CAM_P_OFFSET = MIN_FLT;
float CAM_HEIGHT = MIN_FLT;

void setupCamParams (const int robotPlatformId)
{
	switch (robotPlatformId)
	{
	case 2:
		CAM_P_OFFSET_WORLD = CAM_P_OFFSET_WORLD_R2;
		CAM_HEIGHT_WORLD = CAM_HEIGHT_WORLD_R2;
		CAM_THETA = CAM_THETA_R2;
		FOCAL_LEN = FOCAL_LEN_R2;
		IMAGE_CENTRE_PIXEL_Y = IMAGE_CENTRE_PIXEL_Y_R2;
		CAM_OCCUPANCY_GRID_ORIGIN_X = CAM_OCCUPANCY_GRID_ORIGIN_X_R2;
		CAM_OCCUPANCY_GRID_ORIGIN_Y = CAM_OCCUPANCY_GRID_ORIGIN_Y_R2;
		CAM_OCCUPANCY_GRID_Y_EXP = CAM_OCCUPANCY_GRID_Y_EXP_R2;
		SCAN_CENTRE_DIST = SCAN_CENTRE_DIST_R2;
		break;
	case 3:
		CAM_P_OFFSET_WORLD = CAM_P_OFFSET_WORLD_R3;
		CAM_HEIGHT_WORLD = CAM_HEIGHT_WORLD_R3;
		CAM_THETA = CAM_THETA_R3;
		FOCAL_LEN = FOCAL_LEN_R3;
		IMAGE_CENTRE_PIXEL_Y = IMAGE_CENTRE_PIXEL_Y_R3;
		CAM_OCCUPANCY_GRID_ORIGIN_X = CAM_OCCUPANCY_GRID_ORIGIN_X_R3;
		CAM_OCCUPANCY_GRID_ORIGIN_Y = CAM_OCCUPANCY_GRID_ORIGIN_Y_R3;
		CAM_OCCUPANCY_GRID_Y_EXP = CAM_OCCUPANCY_GRID_Y_EXP_R3;
		SCAN_CENTRE_DIST = SCAN_CENTRE_DIST_R3;
		break;
	default:
		CAM_P_OFFSET_WORLD = CAM_P_OFFSET_WORLD_R4;
		CAM_HEIGHT_WORLD = CAM_HEIGHT_WORLD_R4;
		CAM_THETA = CAM_THETA_R4;
		FOCAL_LEN = FOCAL_LEN_R4;
		IMAGE_CENTRE_PIXEL_Y = IMAGE_CENTRE_PIXEL_Y_R4;
		CAM_OCCUPANCY_GRID_ORIGIN_X = CAM_OCCUPANCY_GRID_ORIGIN_X_R4;
		CAM_OCCUPANCY_GRID_ORIGIN_Y = CAM_OCCUPANCY_GRID_ORIGIN_Y_R4;
		CAM_OCCUPANCY_GRID_Y_EXP = CAM_OCCUPANCY_GRID_Y_EXP_R4;
		SCAN_CENTRE_DIST = SCAN_CENTRE_DIST_R4;
		break;
	}

	CAM_P_OFFSET = CAM_P_OFFSET_WORLD / MAP_SCALE;
	CAM_HEIGHT = CAM_HEIGHT_WORLD / MAP_SCALE;
}
#endif // defined(SIMULATION)


//
// Robot colour ids
//

/*
This is used for [gumstix|robotRecognitionTesting]
For gumstix: we want to map ROBOT_PLATFORM to array index here
For testing: we should just be able to setup data before testing
such that the visible robot's index and location match up to
whatever is visible
*/
const int robotColourIds[9] =
{
	// Robot id=0 platform=2
	0,
	6,
	7,

	// Robot id=1 platform=3
	2,
	10,
	8,

	// Robot id=2 platform=4
	4,
	9,
	11
};

// For experiments, these are the indices assigned to each of the robots we're using
const int robotIndexPlatforms[3] = {
	ROBOT_INDEX_0_PLATFORM,
	ROBOT_INDEX_1_PLATFORM,
	ROBOT_INDEX_2_PLATFORM};

const char* behaviourHandles[] = {
	"EXPLORATION",
	"GOTO_EXP_PT",
	"GOTO_EXP_PT_COLLAB",
	"CLOSE_LOOP",
	"GOTO_SUPERVISOR",
	"SUPERVISION",
	"BACKTRACK",
	"AVAILABLE",
	"STUCK",
	"COLLAB_NAV",
	"FOLLOW_PATH"};

#if !defined(IS_GUMSTIX)
//! Initial robots locations for testing simulation. Obviously on real robot, loc should be actually determined.
float grobotsLocs[MAX_N_ROBOTS * 3] = {
	200.0f, 100.0f, 0.0f,
	450.0f, 200.0f, 0.0f,
	350.0f, 200.0f, 0.0f,
	200.0f, 300.0f, 0.0f,
	300.0f, 300.0f, 0.0f,
	170.0f, 50.0f, 0.0f,
	210.0f, 50.0f, 0.0f,
	250.0f, 50.0f, 0.0f};
#endif





#ifdef PRINT_TIME_DETAIL

#define N_TIMERS 100
TimerData g_timers[N_TIMERS];

void wipeTimerData()
{
	int i;
	for (i = 0; i < N_TIMERS; ++i)
	{
		g_timers[i].label[0] = '\0';
		g_timers[i].startTime = 0;
	}
}

void addStartTime (const char *label)
{
#ifdef IS_WIN
	int startTime = (int)clock();
#else
	int startTime;
	struct timeval tv;
	gettimeofday (&tv, NULL);
	startTime = (tv.tv_sec * 1000000) + tv.tv_usec;
#endif
	int i;
	for (i = 0; i < N_TIMERS; ++i)
	{
		if ('\0' == g_timers[i].label[0])
		{
			strcpy (g_timers[i].label, label);
			g_timers[i].startTime = startTime;
			return;
		}
	}
	assert (0);
}

void addStopTime (FILE *f, const char *label)
{
#ifdef IS_WIN
	int stopTime = (int)clock();
#else
	int stopTime;
	struct timeval tv;
	gettimeofday (&tv, NULL);
	stopTime = (tv.tv_sec * 1000000) + tv.tv_usec;
#endif
	int i;
	for (i = 0; i < N_TIMERS; ++i)
	{
		if (0 == strcmp (g_timers[i].label, label))
		{
			fprintf (f, "<Timer>function=\"%s\" ticks=%d</Timer>\n", label, (stopTime - g_timers[i].startTime));
			g_timers[i].label[0] = '\0';
			return;
		}
	}
	assert (0);
}
#undef N_TIMERS
#endif // PRINT_TIME_DETAIL


















//*****************************************************************************
Move initMove()
{
	Move m;
	m.dir = -1;
	m.moveDist = 0.0f;
	m.usBurst = 0;
	return m;
}

Move initMoveParams (const __int16 n, const __int16 dir, const float moveDist, const unsigned int usBurst)
{
	Move m;
	m.n = n;
	m.dir = dir;
	m.moveDist = moveDist;
	m.usBurst = usBurst;
	return m;
}

IKData initIKData()
{
	IKData d;
	d.index = -1;
	d.len = 0;

	return d;
}

Dest initDest()
{
	Dest d;
	d.targetDist = 0.0f;
	d.leeway = 0.0f;
	d.maxDist = MAX_FLT;
	d.dest.loc.x = 0.0f;
	d.dest.loc.y = 0.0f;
	d.dest.orient = 0.0f;
	d.target.x = 0.0f;
	d.target.y = 0.0f;
	d.type = NORMAL_DEST;
	d.flags = NORMAL_IK;
	return d;
}

void Dest_print (const Dest *d, FILE *f)
{
	fprintf (f, "<Dest>\n\
destLocX=%f destLocY=%f destOrient=%f flags=%d leeway=%f targetX=%f targetY=%f targetDist=%f type=%d\n\
<Dest>\n",
	d->dest.loc.x,
	d->dest.loc.y,
	d->dest.orient,
	(int)d->flags,
	d->leeway,
	d->target.x,
	d->target.y,
	d->targetDist,
	(int)d->type);
}

void ProfitData_print (const ProfitData *p, FILE *f)
{
	fprintf (f, "<ProfitData>\n\
ratio=%f gross=%f expenditure=%f resources=%f nSteps=%f stdDevAtDest=%f \
stdDevInc=%f probability=%f initialNMapped=%f gainNMapped=%f\n\
<ProfitData>\n",
		p->ratio,
		p->gross,
		p->expenditure,
		p->resources,
		p->nSteps,
		p->stdDevAtDest,
		p->stdDevInc,
		p->probability,
		p->initialNMapped,
		p->gainNMapped);

	Dest_print (&p->dest, f);
}

FollowPathData initFollowPathData()
{
	FollowPathData d;
	d.id = FOLLOW_PATH;
	d.nodes = initList();
	d.dest = initDest();
	d.nPathTries = 0;
	return d;
}

void FollowPathData_clear (FollowPathData *d)
{
	List_clear (&d->nodes, 1);
}








BlobGrid* allocBlobGrid()
{
	BlobGrid *grid = (BlobGrid*)malloc (sizeof (BlobGrid));
	memset (grid, 0, SIZE_COOP_LOC_BITARRAY);
	return grid;
}

void initBlobGrid (BlobGrid *grid)
{
	memset (grid, 0, SIZE_COOP_LOC_BITARRAY);
}
















//*****************************************************************************
void MapScan_init (
	MapScan *m,
#if defined(SIM_ERROR)
	const PointF actualPoseLoc,
#endif
#if defined(MAP_SCANS_HAVE_GRIDS)
	OccupancyGrid *grid,
#endif
	const PointF loc,
	const float orient,
	const BEHAVIOUR behaviour,
	const int targetIndex,
	const PointI localMapOrigin,
	const int localMapIndex,
	const float stdDev)
{
#if defined(SIM_ERROR)
	m->actualPoseLoc = actualPoseLoc;
#endif
#if defined(MAP_SCANS_HAVE_GRIDS)
	m->grid = grid;
#endif

	m->pose.loc = loc;
	m->pose.orient = orient;
	m->behaviour = behaviour;
	m->targetIndex = targetIndex;
	m->localMapOrigin = localMapOrigin;
	m->localMapIndex = localMapIndex;
	m->stdDev = stdDev;
}

void freeMapScan (void *v)
{
#if defined(MAP_SCANS_HAVE_GRIDS)
	MapScan *m = (MapScan*)v;
	if (m->grid)
	{
		free (m->grid);
	}
#endif
}


//*****************************************************************************
Proposal initProposal()
{
	Proposal p;
	int i;

	for (i = 0; i < N_ROBOTS; ++i)
	{
		p.considered[i] = 0;
	}

	return p;
}

void initProposal_(Proposal *p)
{
	int i;
	for (i = 0; i < N_ROBOTS; ++i)
	{
		p->considered[i] = 0;
	}
}

int Proposal_isGreater (const void *p1, const void *p2)
{
	Proposal *prop1, *prop2;
	prop1 = (Proposal*)p1;
	prop2 = (Proposal*)p2;

	return (prop1->estCloseLoopProfit > prop2->estCloseLoopProfit);
}

Bid initBid()
{
	Bid bidData;
	bidData.supervisor = -1;
	bidData.explorer = -1;
//	bidData.bid = 0.0f;
	bidData.nSteps = 0.0f;

	return bidData;
}

CoalitionCollabData initCoalitionCollabData()
{
	CoalitionCollabData c;
	c.isSupAtFinalDest = 0;
	c.isCollabReqd = 0;

	return c;
}

Coalition initCoalition (const int id)
{
	Coalition coalition;
	coalition.id = id;
	coalition.area = initPointI (-1, -1);
	coalition.stdDev = MAX_FLT;
	coalition.bid = 0.0f;
	coalition.supervisor = -1;
	coalition.explorer = -1;
	coalition.collabData = initCoalitionCollabData();

	return coalition;
}


//*****************************************************************************
void initCompressedImage (CompressedImage *c)
{
	c->orig.x = MIN_FLT;
	c->orig.y = MIN_FLT;
	c->bufferSize = 0;
	c->usedSize = 0;
	//if (c->buffer)
	//{
	//	free (c->buffer);
	//	c->buffer = NULL;
	//}
	c->buffer = NULL;
}

void freeCompressedImage (void *v)
{
	CompressedImage *c = (CompressedImage*)v;
	if (c->buffer)
	{
		free (c->buffer);
	}
}

void freeCompressedImageWithMapInfo (void *v)
{
	CompressedImageWithMapInfo *c = (CompressedImageWithMapInfo*)v;
	if (c->image.buffer)
	{
		free (c->image.buffer);
	}
}

void GeometryConstants_print (const GeometryConstants *g, FILE *f)
{
	fprintf (f, "<GeometryConstants>\n\
cam_xLimit=%f cam_yNegLimit=%f cam_yPosLimit=%f nav_maxDist=%f exp_optDist=%f exp_minDist=%f exp_maxDist=%f cam_minDist=%f cam_maxDist=%f\n\
</GeometryConstants>\n",
		g->cam_xLimit,
		g->cam_yNegLimit,
		g->cam_yPosLimit,
		g->nav_maxDist,
		g->exp_optDist,
		g->exp_minDist,
		g->exp_maxDist,
		g->cam_minDist,
		g->cam_maxDist);
}

// extern
void Actuator_setupIkConstants (
	const GeometryConstants *geometryConstants,
	IkConstants *ik);

void IkConstants_init (
	const GeometryConstants *g,
	IkConstants *ik)
{
	Actuator_setupIkConstants (g, ik);
}

extern void Uncertainty_calcEigenvalues (const float *cov, float *evals, PointF *evec, float *stdDev);
extern void Uncertainty_scaleCovToSim (float *cov);

float UncertaintyConstants_calcTypicalStdDev (UncertaintyConstants *u)
{
	UnionVector4F cov;
	float evals[2];
	PointF evec;
	float stdDev;
	const float mapScaleSqrdInv = (1.0f / (MAP_SCALE * MAP_SCALE));

	cov.mat[0] = u->visEst_final_varFwd * mapScaleSqrdInv;
	cov.mat[1] = u->visEst_final_covar * mapScaleSqrdInv;
	cov.mat[2] = u->visEst_final_covar * mapScaleSqrdInv;
	cov.mat[3] = u->visEst_final_varLat * mapScaleSqrdInv;

	// Account for (small) uncertainty in robot's orient
	cov.mat[3] += ATAN_ORIENT_VAR * u->visEst_typicalDist;

//	Uncertainty_scaleCovToSim (cov.mat);
	Uncertainty_calcEigenvalues (cov.mat, evals, &evec, &stdDev);

	return stdDev;
}

void UncertaintyConstants_init (UncertaintyConstants *u)
{
	// Values updated for robots on 20130427 from output of processBlobEsts.py
	u->visEst_coarseFace_haveCorner_varFwd = 624.927089f;
	u->visEst_coarseFace_haveCorner_varLat = 272.594839f;
	u->visEst_coarseFace_haveCorner_covar = 0.0f; //282.179671f;

	u->visEst_coarseFace_noCorner_varFwd = 672.484077f;
	u->visEst_coarseFace_noCorner_varLat = 1949.308644f;
	u->visEst_coarseFace_noCorner_covar = 0.0f; //782.646234f;

	u->visEst_accurateFace_horEdge_varFwd = 5844.514388f;
	u->visEst_accurateFace_horEdge_varLat = 1566.869316f;
	u->visEst_accurateFace_horEdge_covar = 0.0f; //1730.772585f;

	u->visEst_accurateFace_vertEdge_varFwd = 1408.748302f;
	u->visEst_accurateFace_vertEdge_varLat = 655.860325f;
	u->visEst_accurateFace_vertEdge_covar = 0.0f; //536.442218f;

	u->visEst_final_varFwd = 512.010827f;
	u->visEst_final_varLat = 664.849327f;
	u->visEst_final_covar = 0.0f; //782.646234f;

	u->visEst_typicalDist = 250.0f;

//	u->closeLoop_varFwd = 0.008f; //! < Established from experimentation data (using post-processing scripts)
//	u->closeLoop_varLat = 0.007f;
//	u->closeLoop_covar = 0.000f;
//	u->closeLoop_stdDev = 0.022f;
	u->closeLoop_varFwd = 0.004f; //! < Established from experimentation data (using post-processing scripts)
	u->closeLoop_varLat = 0.003f;
	u->closeLoop_covar = 0.00001f;
	u->closeLoop_stdDev = (sqrt (u->closeLoop_varFwd) * sqrt (u->closeLoop_varLat)); //!< Calculate the area of the error ellipse

	// This is used when propogating uncertainty, but trigonometry operations
	// are expensive so only calculate once.
//	ATAN_ORIENT_VAR = atan (ORIENT_VAR);
	ATAN_ORIENT_VAR = atan (ORIENT_VAR * 2);
#if 1
	ATAN_ORIENT_VAR = atan (ORIENT_VAR * 2); // Testing with increased error
#endif

	// Do this after ATAN_ORIENT_VAR is setup
	u->visEst_typicalStdDev = UncertaintyConstants_calcTypicalStdDev (u);

#if defined(EXPLICIT_DEBUG)
	u->isSetup = 1;
#endif
}


//*****************************************************************************
CommData initCommData()
{
	CommData c;
	c.bufferCurrentSize = 0;
	c.socket = 0;
	c.port = 7171;
	c.quitFlag = 0;
	c.deleteme = 0;

#ifdef IS_WIN
	strcpy_s ((char*)c.address, 20, "127.0.0.1");
#elif defined(IS_GUMSTIX)
	strcpy (c.address, "192.168.99.1");
#else
	strcpy ((char*)c.address, "127.0.0.1");
#endif
	return c;
}

LocalMapInfo initLocalMapInfo()
{
	LocalMapInfo localMapInfo;
	localMapInfo.robotIndex = -1;
	localMapInfo.localMapIndex = -1;
	localMapInfo.isProvisional = 0;
	localMapInfo.closeLoopSession = -1;
	localMapInfo.gross = 0.0f;
	localMapInfo.mapGain = 0.0f;
	localMapInfo.avgStdDev = 0.0f;
	localMapInfo.nScans = 0.0f;
	return localMapInfo;
}


RobotData initRobotData()
{
	RobotData r;
	r.pose = initPose();
	r.behaviour = AVAILABLE;
	r.localMapOrigin = initPointI (MIN_INT32, MIN_INT32);
	r.stdDev = MIN_FLT;
	r.target = initPointF(MIN_FLT);
	r.actualLocOffset = initPointF (MIN_FLT);
	return r;
}

MapStatusFromBoard initMapStatusFromBoard()
{
	MapStatusFromBoard m;
	m.isTimeElapsed = 0;
	m.hasDataBeenIncd = 0;
	m.isNewGlobalMap = 0;
	m.isNewUnreachableLocalMap = 0;
	m.mapAreaEffectedForRobot = initEffectedArea();
	return m;
}

void MapStatusFromBoard_resetNewUnreachableLocalMap (MapStatusFromBoard *statusList)
{
	int i;
	for (i = 0; i < N_ROBOTS; ++i)
	{
		statusList[i].isNewUnreachableLocalMap = 1;
	}
}


//*****************************************************************************
ImgCellFeatures initImgCellFeatures()
{
	int i;
	ImgCellFeatures feats;
	for (i = 0; i < 6; ++i)
	{
		feats.vals[i] = 0.0f;
	}
	feats.isOccupied = 0;
	return feats;
}



#if defined(IS_WIN) && defined(SIMULATION)
void TrainingData_init (TrainingData *data, const int n)
{
	int i;
	i = CAM_P_OFFSET_WORLD;

	for (i = 0; i < n; ++i)
	{
		data[i].vis = -1; // Valid platforms will be 2,3,4
		data[i].isLoc = 0;
		data[i].loc.x = 0.0f; // Relative loc of visible robot, so (0,0) is not valid
		data[i].loc.y = 0.0f;
		data[i].orient = 0.0f;
		data[i].obs = -1;
	}
}
#endif // defined(IS_WIN) && defined(SIMULATION)
