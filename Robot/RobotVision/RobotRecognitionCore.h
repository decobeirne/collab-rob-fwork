#include "../../Common/RobotDefs.h"

#ifndef BOARD

#ifndef ROBOT_RECOGNITION_CORE_H
#define ROBOT_RECOGNITION_CORE_H

#include "../../Common/RobotCore.h"
#include "ObstacleRecognitionCore.h"

#define N_ROBOT_COLOURS_TOTAL 16 //!< Number of trained robot colours.

#if !defined(IS_LINUX)
#define N_ROBOT_COLOURS 11 //!< Number of colours actually in use
#else
#define N_ROBOT_COLOURS 5 //!< Each robot will be able to see 3 colours on the other 2 robots, but the "top" colour is common to all.
#endif

#define COLOUR_CONFIDENCE_THRESHOLD 3.0f
#define COLOUR_CONFIDENCE_COARSE_THRESHOLD (COLOUR_CONFIDENCE_THRESHOLD * 1.2f)

#define COLOUR_SCORE_SEED_THRESHOLD 2.0f
#define COLOUR_SCORE_NORMAL_THRESHOLD 3.0f
#define COLOUR_SCORE_RELAXED_THRESHOLD_2 4.0f
#define NBOUR_SCORE_THRESHOLD 4.0f
#define NBOUR_INFLUENCE_COEFF 0.1f

#define BLOB_STRENGTH_COLOUR_COEFF 1.0f
#define BLOB_STRENGTH_DENSITY_COEFF 2.0f

#define BLOB_MIN_NCELLS 30
#define BLOB_MIN_NCELLS_SMALLER 20 //!< For blobs that we can use for a rough est, but not an accurate est
#define BLOB_MAX_COLOUR_SCORE 2.0f // was 1.6, but missed some valid blobs (for colour 5 in particular)
#define BLOB_MAX_DENSITY_SCORE 2.0f

#define CALC_FACE_EDGE_MAX_N_PTS 48

// When testing, we wish to pass additional flags to robot rec functions
//#define DEBUGGING_ROBOT_REC


typedef enum RobotFaceTypes_
{
	FACE_TYPE_FRONTBACK,
	FACE_TYPE_LEFTRIGHT,
	FACE_TYPE_TOP
} RobotFaceTypes;

typedef enum BlobType_
{
	BLOB_INVALID,
	BLOB_VALID_BUT_TOO_SMALL,
	BLOB_VALID,
	BLOB_TOP_FACE,
	BLOB_NOT_SET
} BlobType;

typedef enum FaceEstType_
{
	FACE_EST_NOTHING,
	FACE_EST_TOP,
	FACE_EST_PROV_OK,
	FACE_EST_CANT_USE,
	FACE_EST_EDGE, //!< Could only detect a point on the top/bottom edge of face
	FACE_EST_CORNER, //!< Could detect a corner of face
	FACE_EST_ACCURATE //!< Could accurately determine bounds of face
} FaceEstType;

//! Blob extracted from robot camera image
typedef struct ColourBlob_
{
	int colourIndex;
	int id;
	int nCells;
	float colourScore;
	float densityScore;
	PointI bl;
	PointI tr;
	int isUnique;
	int isConflicting;
	BlobType blobType;
#if VERBOSE_BLOB_DETECTION
	int isColourInImg;
#endif
	uchar blobGrid[SIZE_COOP_LOC_GRID];
} ColourBlob;

ColourBlob* ColourBlob3_alloc (
	const int colourIndex,
	const int id);

ColourBlob ColourBlob3_init (
	const int colourIndex,
	const int id);

//! Return distance from robot COG to given edge of face.
/*!
Useful when we are making a rough estimate of a robot location based
on the corner of one of its faces.
*/
float RobotRecognitionCore_getDistToSide (
	const int robotIndex,
	const int faceIndex,
	const int wantDistToLeftOfFace);

float RobotRecognitionCore_getFaceHalfLen (
	const int robotIndex,
	const int faceIndex);

//! Face of visible robot
typedef struct RobotFace_
{
	ColourBlob *blob;
	RobotFaceTypes faceTypeIndex;
	int faceIndex; //!< fblr
	float faceOrient;
	float perceivedFaceRelativeOrient;
	PointF locEst;
	UnionVector4F cov; //!< Uncertainty covariance corresponding to 2-D face centre pt estimate
	FaceEstType estType;
	float slopeCameraSpace;
	int useCameraXAxis;
} RobotFace;

RobotFace RobotFace_init();

//! Location estimate for robot visible in camera image
typedef struct RobotEstimate_
{
	RobotFace robotFaces[3];
	float robotOrient;
	float robotRelativeOrient;
	int robotIndex;
	Vector3F robotDir;
	int isEst;
	PointF estLoc;
	UnionVector4F estCov;
	FaceEstType estType;
} RobotEstimate;

RobotEstimate RobotEstimate_init();

//! Store robot dimensions
typedef struct RobotParams_
{
	float faceEdgeBottomHeight;
	float faceHeight;
	float faceTopEdgeHeight;
	float cogToFace[4]; //f/b/l/r
} RobotParams;

//! Store further robot dimensions - orient/dist from corners to cog
/*!
Corners are order anti-clockwise around forward orient, therefore corners
are: fl, bl, br, fr
*/
typedef struct CornerParams_
{
	float orientFromCogToCorner[4];
	float distFromCogToCorner[4];
} CornerParams;

int RobotRecognitionCore_isColourOkForTraining (const int colour);

float RobotRecognitionCore_calcColourDiff (
	const float *cellDesc,
	const float *colourDesc);

//! Return anyRobotCells
int RobotRecognitionCore_markRobotOccupiedCells (
	FILE *outputFile,
	const float *colourScoreGrid,
	OccupancyGrid *occupancyGrid,
	const float colourScoreThreshold);

void RobotRecognitionCore_getCellsNbouringValue (
	const uchar *blobGrid,
	const uchar requiredBlobId,
	uchar *nbourGrid);

void RobotRecognitionCore_calcNbourInfluenceGrid (
#if VERBOSE_BLOB_DETECTION
	FILE *outputFile,
#endif
	const float *colourScoreGrid,
	const int colourIndex,
	float *nbourInfluenceGrid);

PointI RobotRecognitionCore_gridPtToCameraPixel (
	const PointI gridPt);

#endif // ifndef ROBOT_RECOGNITION_CORE_H
#endif
