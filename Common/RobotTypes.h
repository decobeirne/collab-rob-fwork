/*!
\file RobotTypes.h
\brief Data structures used in system.
*/
#ifndef ROBOT_TYPES_H
#define ROBOT_TYPES_H

#include "RobotDefs.h"
#include "BaseTypes.h"
#include "List.h"
#include "Vector.h"


#ifdef PRINT_TIME_DETAIL
//! Struct to store data for timing functions.
typedef struct TimerData_
{
	char label[32];
	int startTime;
} TimerData;

//! Reset timers for current step.
void wipeTimerData();

//! Add new start time to array.
void addStartTime (const char *label);

//! Add stop time corresponding to an existing start time, print elapsed time.
void addStopTime (FILE *f, const char *label);
#endif

#if defined(SIMULATION)

//! Setup camera parameters at run time. Otherwise setup at compile time for linux
void setupCamParams (const int robotPlatformId);

#endif // defined(SIMULATION)


//! Possible behaviours a robot can adopt
typedef enum BEHAVIOUR_
{
	EXPLORATION =			0, //!< Explore free space, looking for target objects
	GOTO_EXP_PT =			1, //!< Navigate to a new point to explore around
	GOTO_EXP_PT_COLLAB =	2, //!< Navigate to a new point local map. May correct map data using info from supervisor.
	CLOSE_LOOP =			3, //!< Perform loop closing with supervisor robot to remove errors from data
	GOTO_SUPERVISOR =		4, //!< Goto supervisor after joining coalition
	SUPERVISION =			5, //!< Supervise robots carrying out exploration
	BACKTRACK =				6, //!< Robot has to backtrack its last moves to avoid a collision
	AVAILABLE =				7, //!< Robot has not adopted any behaviour
	STUCK =					8,
	COLLAB_NAV =			9,
	FOLLOW_PATH =			10
} BEHAVIOUR;

//! Communication flags
typedef enum COMM_FLAG_
{
	R_STATUS =								1,
	R_GRIDS =								2,
	R_GLOBAL_MAP_SECTION =					3,
	R_GROUP =								4,
	R_PROPOSALS =							5,
	R_COALITION_FLAGS =						6,
	R_BIDS =								7,
	R_COALITIONS =							8,
	R_LOC_MAP_GRID =						9,


	W_FINISHED =							20,
	W_ROBOT_DATA =							21,
	W_LOCAL_MAP =							22,
	W_LEAVE_PROV_DATA =						23,
	W_PROPOSAL =							24,
	W_BID =									25,
	W_PROPOSALS_CONSIDERED =				26,
	W_LEAVE_COALITION =						27,
	W_SUP_AT_FINAL_DEST =					28,
	W_LOC_MAP_GRID =						29,
	W_ROBOT_LOGFILE =						30,
	W_GTEP_JOB =							31,
	W_MAP =									32,
	W_EXP_JOB =								33,
	W_LIST =								34,
	W_LOC_MAP_PT =							35
} COMM_FLAG;









//! RGB colour values, uchar
typedef struct COLOUR_
{
	uchar r;
	uchar g;
	uchar b;
} COLOUR;

//! Flag to indicate how local grid of exploration cells should be updated
typedef enum EXP_GRID_UPDATE_FLAG_
{
	DONT_UPDATE_SEARCH_GRID =		0,
	UPDATE_SEARCH_GRID_PATCH =		1,
	UPDATE_WHOLE_SEARCH_GRID =		2
} EXP_GRID_UPDATE_FLAG;

//! Flag to indicate if a local map being submitted is either provisional data or is overwriting previous data
typedef enum LOC_MAP_PROVISIONAL_FLAG_
{
	NOT_CLOSE_LOOP_DATA =					0,
	PROVISIONAL_CLOSE_LOOP_DATA =			1,
	ADJUSTED_CLOSE_LOOP_DATA =				2
} LOC_MAP_PROVISIONAL_FLAG;

//! Flag to indicate that a robot's local map should be reset
typedef enum LOC_MAP_RESET_FLAG_
{
	DONT_RESET_LOC_MAP =			0,
	SET_REQUESTED_LOC_MAP =			1,
	CENTRE_LOC_MAP =				2,
	SET_LOC_MAP_ADJACENT_AREA =		3

} LOC_MAP_RESET_FLAG;

//! Flag to indicate type of movement required
typedef enum MOVE_TYPE_
{
	NO_MOVE_REQD =					0,
	MOVE_REQD =						1,
	NO_VALID_MOVE =					2
} MOVE_TYPE;

typedef enum UPDATE_NAV_MAP_FLAG_
{
	DONT_UPDATE_NAV_MAP =			0,
	UPDATE_OCCUPIED_TERRAIN =		1,
	UPDATE_WHOLE_NAV_MAP =			2
} UPDATE_NAV_MAP_FLAG;

//! Flag to indicate version of Bresenham algorithm that should be used.
typedef enum VISUAL_CONTACT_TYPE_
{
	CHECK_NAV_MAP_ONLY =			0,
	CHECK_CELLS_ADJACENT =			1,
	CHECK_CELLS_BRESENHAM =			2
} VISUAL_CONTACT_TYPE;

//! Flags to indicate if proposal, bids are pending, successful.
typedef enum PROPOSAL_STATUS_
{
	PROPOSAL_SUCCESSFUL =			0,
	BID_SUCCESSFUL =				1,
	PROPOSAL_PENDING =				2,
	BID_PENDING =					3,
	NO_PENDING_COALITION =			4,
} PROPOSAL_STATUS;

//! Flags to indicate result of path calculation.
typedef enum PATH_RESULT_
{
	PATH_OK =						0,
	ROBOT_INVALID =					1,
	TARGET_INVALID =				2,
	CANT_CALC_LAST_NODE =			3,
	CANT_COMBINE =					4,
	CANT_REFINE_PATH =				5
} PATH_RESULT;






//! Move carried out by robot
/*!
Can be a move forward, or a turn clockwise or anti-clockwise
*/
typedef struct Move_
{
	__int16 n;
	__int16 dir;
	float moveDist;
	unsigned int usBurst;
} Move;

//! Constructor
Move initMove();

//! Constructor with full params.
Move initMoveParams (const __int16 n, const __int16 dir, const float moveDist, const unsigned int usBurst);

//! Queue of moves to reach current destination.
typedef struct IKData_
{
	__int16 len;
	__int16 index;
	Move moves[6];
} IKData;

//! Constructor
IKData initIKData();

//! Destination characteristics.
typedef enum DestType_
{
	NORMAL_DEST = 0,
	IS_TARGET_CENTRIC = 1,
	IS_COARSE_NAV_OK = 2,
	IS_NEW_DEST = 8,
	IS_PATH_REQUIRED = 16,
	IS_AT_DEST = 32,
	IS_IK_CURRENTLY_IMPOSSIBLE = 64,
	DOES_DEST_REQ_BACKTRACK = 128
} DestType;

typedef enum DestFlags_
{
	NORMAL_IK = 0,
	SIMPLE_IK_ONLY = 1
} DestFlags;

//! Destination pose.
typedef struct Dest_
{
	PoseSimple dest; //!< Specifies dest pose (if calculated).
	float targetDist; //!< If 0, means that not-target-centric, otherwise specifies optimum dist.
	float leeway; //!< If large leeway, can just use simple hill climbing to get to dest/target.
	float maxDist; //!< We may want to limit movement when e.g. the robot is close to obstacles or other robots
	PointF target; //!< Specifies target (if target-centric).
	DestType type;
	DestFlags flags;
} Dest;

//! Constructor
Dest initDest();

void Dest_print (const Dest *d, FILE *f);

typedef enum ReconFlag_
{
	NOT_DOING_RECON = 0,
	INITIAL_RECON = 1,
	RECON_SPIRAL = 2,
	RECON_PANNING = 3
} ReconFlag;

//! Internal profit data
typedef struct ProfitData_
{
	Dest dest;
	float ratio;
	float gross;
	float expenditure;
	float resources;
	float nSteps;
	float stdDevAtDest;
	float stdDevInc;
	float probability;
	float initialNMapped;
	float gainNMapped;
/*	float stdDevAtDest;
	float stdDevInc;	*/

	//float profitPerStep;
	//float net;
	//float gross;
	//float gainNMapped;
	//float nSteps;
	//float cost;
	//float probability;
	//int initialNMapped;
} ProfitData;

void ProfitData_print (const ProfitData *p, FILE *f);

//! Store data on FOLLOW_PATH
typedef struct FollowPathData_
{
	BEHAVIOUR id;
	Dest dest; //!< This is updated as each node along the path is traversed
	List nodes;
	int nPathTries;
} FollowPathData;

//! Constructor
FollowPathData initFollowPathData();

//! Destructor
void FollowPathData_clear (FollowPathData *d);

















//! Array of bool values stored per bit
typedef struct BlobGrid_
{
	uchar grid[SIZE_COOP_LOC_BITARRAY];
} BlobGrid;

BlobGrid* allocBlobGrid();

void initBoolGrid(BlobGrid *grid);




//! Object representing attributes of a map patch 
typedef struct MapScan_
{
	int estdGain; //!< Initial est'd gain in n mapped cells
	BEHAVIOUR behaviour; //!< Behaviour in use when map patch was obtained
	int targetIndex; //!< Target counter; used in post-processing
	int localMapIndex; //!< Index of local map into which scan is integrated
	PoseSimple pose; //!< Robot pose when scan was obtained
	PointI localMapOrigin; //!< Origin of local map containing map scan.
	float stdDev; //!< Std dev of pose covariance

#if defined(MAP_SCANS_HAVE_GRIDS)
	OccupancyGrid *grid; //!< Occupancy grid from camera image, NULL if no artefacts detected
#endif

#if defined(SIM_ERROR)
	PointF actualPoseLoc; //!< Actual loc of robot in simulation
#endif
} MapScan;

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
	const float stdDev);

//! Map scan destructor
void freeMapScan (void *m);








//! Proposal to form a coalition broadcast on the blackboard.
typedef struct Proposal_
{
	__int16 supervisor;
	__int8 considered[MAX_N_ROBOTS];
//	float duration;
//	float totalProfit;
	float stdDev;
	float estCloseLoopProfit;
//	int expCellsMapped;
//	PointF pt;
//	float supervisorResources;
//	float supervisorReserve;
	time_t postedTime;
	PointI area;
} Proposal;

Proposal initProposal();

void initProposal_(Proposal *p);

int Proposal_isGreater (const void *p1, const void *p2);

//! Bid to join coalition with a supervisor robot.
typedef struct Bid_
{
	int supervisor;
	int explorer;
//	float bid;
	float nSteps;
} Bid;

Bid initBid();

//! Collaboration data shared between robots in a coalition
typedef struct CoalitionCollabData_
{
	__int8 isSupAtFinalDest;		//!< Supervisor is in a stationary position.
	__int8 isCollabReqd;			//!< Supervisor requires collaborative navigation to get to supervision area.


} CoalitionCollabData;

CoalitionCollabData initCoalitionCollabData();

//! Coalition between an explorer and supervisor robot
typedef struct Coalition_
{
	int id;								//!< Coalition index.
	PointI area;
	float estCloseLoopProfit;
//	PointF pt;							//!< Centre point of supervision area.
	float stdDev;						//!< Std dev in supervisor's location estimate at supervision location.
	float bid;							//!< Ratio of profit that explorer has stated it will attribute to the supervisor.
	int supervisor;						//!< Stationary robot acting as landmark.
	int explorer;						//!< Active robot mapping new terrain.
	CoalitionCollabData collabData;		//!< Data to sync collaborative exploration
} Coalition;

Coalition initCoalition (const int id);

//! Information on local map to submit to the blackboard
typedef struct LocalMapInfo_
{
	int robotIndex;
	int localMapIndex;
	int isProvisional;
	int closeLoopSession;
	float gross; //!< \todo These 4 members can be deleted. We can calc these when redrawing the locmap and print out straight away. The board can incorp map and we can match subd against incd map in postProc
	float mapGain;
	float avgStdDev;
	float nScans;
} LocalMapInfo;

//! Default constructor.
LocalMapInfo initLocalMapInfo();

//! Compressed local map using run-length-encoding
typedef struct CompressedImage_
{
	PointI orig;
	__int16 bufferSize;
	__int16 usedSize;
	uchar *buffer;
} CompressedImage;

void initCompressedImage (CompressedImage *c);

void freeCompressedImage (void *v);

typedef struct CompressedImageWithMapInfo_
{
	LocalMapInfo mapInfo;
	CompressedImage image;
} CompressedImageWithMapInfo;

void freeCompressedImageWithMapInfo (void *v);

//! Robot specific parameters.
typedef struct GeometryConstants_
{
	float cam_xLimit;
	float cam_yNegLimit;
	float cam_yPosLimit;
	float cam_minDist;				//!< Min dist visible to robots (sim scale) (dist from focal pt)
	float cam_maxDist;				//!< Max (useful) dist visible to robots (sim scale)
	float cam_angleAtMinDist;		//!< Visible angle at min visible distance

	float exp_optDist;				//!< Opt dist for mapping a whole map square
	float exp_minDist;				//!< Min allowed dist to exp target
	float exp_maxDist;				//!< Max allowed dist to exp target

	float nav_maxDist;
#if defined(EXPLICIT_DEBUG)
	int areExpDistsSetup;
	int areCamDistsSetup;
#endif
} GeometryConstants;

void GeometryConstants_print (const GeometryConstants *g, FILE *F);


typedef struct IkConstants_
{
	// Stores offsets incurred when rotations are made
	PointF rotationOffsets[N_ROTATIONS_IN_TOTAL];
	float rotations[N_ROTATIONS_IN_TOTAL];
	int moveDirs[N_ROTATIONS_IN_TOTAL];
	int moveNRotations[N_ROTATIONS_IN_TOTAL];
	float moveRange;
	float moveMedian;
	float offsetMedian;
	float usMedian;
	float orientChangeRange;
	float orientChangeMedian;
} IkConstants;

void IkConstants_init (
	const GeometryConstants *geometryConstants,
	IkConstants *ik);



//! Calculate uncertainty in robot move and sensory estimation
typedef struct UncertaintyConstants_
{
	float visEst_coarseFace_haveCorner_varFwd;
	float visEst_coarseFace_haveCorner_varLat;
	float visEst_coarseFace_haveCorner_covar;

	float visEst_coarseFace_noCorner_varFwd;
	float visEst_coarseFace_noCorner_varLat;
	float visEst_coarseFace_noCorner_covar;

	float visEst_accurateFace_horEdge_varFwd;
	float visEst_accurateFace_horEdge_varLat;
	float visEst_accurateFace_horEdge_covar;

	float visEst_accurateFace_vertEdge_varFwd;
	float visEst_accurateFace_vertEdge_varLat;
	float visEst_accurateFace_vertEdge_covar;

	float visEst_final_varFwd;
	float visEst_final_varLat;
	float visEst_final_covar;

	float visEst_typicalDist; //!< From cam's focal pt p
	float visEst_typicalStdDev;

	float closeLoop_varFwd;
	float closeLoop_varLat;
	float closeLoop_covar;
	float closeLoop_stdDev; //!< Std dev per step in loop closing, i.e. variance of location of map scan when updated based on neighbouring map scan

#if defined(EXPLICIT_DEBUG)
	int isSetup;
#endif
} UncertaintyConstants;

//! Constructor
void UncertaintyConstants_init (UncertaintyConstants *u);













typedef struct CommData_
{
	int bufferCurrentSize;			//!< Size of data to send in current buffer
	schar buffer[BUFFER_SIZE];		//!< Data buffer for comm with board
	int socket;						//!< Socket for communication with board
	int port;						//!< Port number
	schar address[20];				//!< Address of client/server
	int quitFlag;					//!< If handling requests from robot, set flag to quit board thread
	int deleteme;
} CommData;

//! Constructor
CommData initCommData();







//! Flag to indicate version of Bresenham algorithm that should be used.
typedef enum BRESENHAM_TYPE_
{
	SINGLE_PIXEL =		0,
	TWO_PIXELS =		1,
	THREE_PIXELS =		2
} BRESENHAM_TYPE;

//! Data for incorporating simulated map scans
typedef struct BresData_
{
	int count; //!< N pixels along main axis
	PointI bres; //!< Point being incremented
	PointI step; //!< Sign of increment along each axis
	PointI d; //!< Change over each axis
	int axis; //!< Main axis
	PointF lens; //!< Focal pt of lens from which vectors are traced, rel to loc map
	float poseOrient; //!< Lens orient
	float blockedDist; //!< Dist along vector at which obstacle is found
	int obstacleWidth; //!< Width of obstacle to fill
	PointI offset; //!< Offset between estimated and actual robot loc
} BresData;






//! Robot status posted to board.
typedef struct RobotData_
{
	Pose pose;
	BEHAVIOUR behaviour;
	BEHAVIOUR baseBehaviour;
	float stdDev;
	PointF target;
	PointI localMapOrigin;
	PointF actualLocOffset;
} RobotData;

RobotData initRobotData();


//! Data for location estimate posted by another robot on the blackboard.
typedef struct LocEstimateForRobot_
{
	int robotMakingEstimate;
	float estimateStdDev;
	float estimateCov[4];
	PointF estimateLoc;
} LocEstimateForRobot;

//! Status of map data being submitted by robot.
typedef struct MapStatusFromRobot_
{
	__int16 isMoreDataComing;
//	__int16 isNewMapData;
} MapStatusFromRobot;

//! Status of map data on blackboard.
typedef struct MapStatusFromBoard_
{
	__int8 isTimeElapsed;
	__int8 hasDataBeenIncd;
	__int8 isNewGlobalMap;
	__int8 isNewUnreachableLocalMap;
	Vector4I mapAreaEffectedForRobot;
} MapStatusFromBoard;

MapStatusFromBoard initMapStatusFromBoard();

void MapStatusFromBoard_resetNewUnreachableLocalMap (MapStatusFromBoard *statusList);

//! Status of experiment/simulation, stored on blackboard
typedef struct ExperimentStatus_
{
	__int8 updateFlag; //!< Has board been updated with new data from robots
	__int8 isTimeElapsed; //!< Time or number of iterations elapsed in experiment.
	__int16 nFinished; //!< Counter for robots that have returned finished signals.
	int iteration; //!< Current iteration if running a simulated experiment.
} ExperimentStatus;

//! Status of map on board
typedef struct MapStatus_
{
	__int8 isNewMapData;							//!< Flag indicating if there is any new data at all on board
	__int8 isNewDataToDisplay;						//!< Tells if visualisation should update
} MapStatus;

//! Image cell features calculated during training.
/*!
The features used to describe each image cell can be changed, but the ones used here a H,S,V and texture for each of
these values over the image cell
*/
typedef struct ImgCellFeatures_
{
	float vals[6];
	uchar isOccupied;
	uchar hasWeirdPixels;
} ImgCellFeatures;

//! Constructor
ImgCellFeatures initImgCellFeatures();

//! Image features, calculated from the features of its constituent image cells.
/*!
Vals 0-5 are averages across image cells. 6-11 are variances in these values
*/
typedef struct ImgFeatures_
{
	float vals[12];
	int nCellsWithWeirdPixels;
} ImgFeatures;



#if defined(IS_WIN) && defined(SIMULATION)

//! Struct to hold data recorded from training images.
typedef struct TrainingData_
{
//	int orig;
//	int flip;
//	int gridY;

	int vis; //!< Index of visible robot. Each platform has colours specified. -1 if not a robot
	int isLoc; //!< Is relative location and orient of visible robot setup
	PointF loc; //!< For training coop loc, with visible robot assumed ot be at 0,0
	float orient; //!< Observing robot assumed to have orient 0.
	int obs; //!< Index of observing robot

	char imgName[128];
	uchar occupancyGrid[SIZE_COOP_LOC_GRID];
} TrainingData;

void TrainingData_init (TrainingData *data, const int n);
#endif


#endif // ifndef
