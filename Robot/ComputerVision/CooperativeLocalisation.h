#include "../../Common/RobotDefs.h"

#ifndef COOP_LOC_H
#define COOP_LOC_H

#include "../../Common/RobotCore.h"

#define COLOUR_BACKGROUND 20
#define COLOUR_CANNOT_EST 19


#define N_ROBOT_COLOURS 16 //!< Number of trained robot colours.


#define STRICT_BLOB_SCORE 12.0f
#define RELAXED_BLOB_SCORE 20.0f


typedef struct ColourResponse_
{
	float vals[N_ROBOT_COLOURS];
} ColourResponse;

//! Initialise array of responses to trained robot colours.
ColourResponse initColourResponse();

//! Calculate difference between image cells and trained robot colours.
void CooperativeLocalisation_calcColourResponses (
	ImgCellFeatures cellFeatures[SIZE_COOP_LOC_GRID],
	ImgFeatures *imgInvFeatures,
	const uchar occupancyGrid[SIZE_COOP_LOC_GRID],
	ColourResponse colourGrid[SIZE_COOP_LOC_GRID],
	FILE *f);

//! Set occupancy grid given scores for occupied/unoccupied and robot colours
/*!
\todo This is a copy of the func in ObstacleDetection. One of these should be deprecated
*/
int CooperativeLocalisation_calcOccupiedEst(
	const int camOccupancyGridY,
	OccupancyGrid *camOccupancyGrid,
	PointF *camScoreGrid,
	uchar *colourGrid,
	int *isObstacleInCamOccupancyGrid);





//! Return list of robot blobs
/*!
The given colourGrid is altered such that any blobs that are not found to be valid, i.e. not
enough connected cells, or more than one blob found for a single colour, are set to an invalid
value.
*/
void CooperativeLocalisation_groupRobotBlobs (
	ColourResponse colourResponseGrid[SIZE_COOP_LOC_GRID],
	List *colourBlobs);

//! Determine index of closest visible robot given visible colours in image.
void CooperativeLocalisation_determineVisibleRobotFromBlobs (
	List *colourBlobs,
	VisibleRobot *closestVisibleRobot,
	const int ownIndex);

void CooperativeLocalisation_detectRobotBlobEdges (
	Image *img, Image *temp,
	const PointI orig,
	const uchar colourGrid[SIZE_COOP_LOC_GRID],
	List *colourBlobs,
	VisibleRobot *closestVisibleRobot,
	const Pose ownPose,
	const PoseSimple *groupPoses,
	const CamVectors *camVectors);

#ifdef SIMULATION

//! Test detection of robot colours in images
/*!
Possibly deprecated by temp__blobDet__useStuffFromPrevFuncs()
*/
void CooperativeLocalisation_testBlobDetection();

//! Extract and write features from cooperative localisation images.
void CooperativeLocalisation_writeFeatures();

//! Setup image training data for cooperative localisation images.
/*
Images from the following directories are used:
 - robot3_locRobot2_20111113
 - robot3_locRobot2_20111211
 - robot2_locRobot3_20120319
*/
int CooperativeLocalisation_setupTrainingData(TrainingData *data);

//! Print features (i.e. hue etc) to text file to copy back into code.
void temp__printOutColourFeats();

//! Record usefulness of each image in coop loc training data
/*!
Calc n colour cells in each image and each correct, etc. Write to a
text file for evaluating
*/
void temp__testColourRecPerImg();

//! Test new(?) colour rep on training images
/*
Calc closest colour for each cell and compare to the actual colour from
training data.
*/
void temp__testNewColourRep();

//! Compare (new) colour rep to cells in *obst det* training images.
/*!
For each colour, go through all cells in training images and calc diff.
Write out to bins in order to see distribution.
*/
void temp__testColoursAgainstBackground();

//! Calc colour responses and grow blobs in images.
/*!
Uses new(again... ?) colour rep. In progress.
*/
void temp__blobDet__useStuffFromPrevFuncs();

#endif // ifdef SIMULATION
#endif // ifndef COOP_LOC_H
