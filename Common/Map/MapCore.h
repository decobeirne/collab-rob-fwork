#ifndef MAP_CORE_H
#define MAP_CORE_H

#include "../Image.h"
#include "../RobotTypes.h"

#if !defined(BOARD)
#ifdef PRINT_EVENTS
//! Print integrated map scan to log file.
void MapCore_logMapScan (FILE *f, const float optDist, const int nIterations, const MapScan *mapScan);
#endif
#endif

//! Create threshold image from input image
/*!
The input and output images should be greyscale and of the same dimensions.
The function thresholds the input image either below or above the given
uchar colour. The pixels in the output image are set to the constant values
OCCUPIED_TERRAIN and FREE_TERRAIN accordingly
*/
void MapCore_thresholdImage (
	Image *inputImg,
	Image *outputImg,
	const uchar thresholdColour,
	const uchar occpuiedColour,
	const uchar freeColour,
	const int thresholdBelow,
	const int dims);

//! Draw occupied border around edge of local map to stop robot from straying onto another map.
void MapCore_dilateBorder (
	Image *inputImg,
	Image *outputImg,
	const uchar broadOccupiedColour,
	const int broadDist);

//! Dilate function specific to robot navigation maps
void MapCore_dilateNavMap (
	Image *inputImg,
	Image *outputImg,
	const uchar occupiedColour,
	const uchar narrowDilate,
	const uchar broadDilate,
	const uchar freeColour,
	const int narrowDist,
	const int broadDist);

void MapCore_printSupGridCells (FILE *f,
								__int16 supGrid[SUP_GRID_DIMS][SUP_GRID_DIMS]);

void MapCore_updateSupGridCells (FILE *f,
								   __int16 localMapGrid[GLOB_LOC_MAP_GRID_DIMS][GLOB_LOC_MAP_GRID_DIMS],
								   __int16 supGrid[SUP_GRID_DIMS][SUP_GRID_DIMS]);



//! Determine if a pt is within a local map
/*! 
The function is passed the centre of the loc map and the robot pt in question.
The dimensions of the loc map are also passed. This may be the distance from 
the centre pt to the edge, but other dists may also be used. The envir dims are 
also passed in, as rob pts outside the envir are not considered valid for this 
function
*/
int MapCore_isPtWithinLocalMap (
	const int robx,
	const int roby,
	const PointI localMapCentrePt,
	const int localMapRadius);

void MapCore_copyMap_greaterCertainty (Image *src, Image *dest);

//! Compress local map. Run length encoding.
void MapCore_compressLocalMap (Image *image, CompressedImage *compressed);

//! Fill local map from buffer.
void MapCore_decompressLocalMap (Image *image, CompressedImage *compressed);

#ifndef IS_GUMSTIX
//! Setup simulated obstacles.
void MapCore_setupObstacles (List *obstacleList);

//! Draw simulated obstacles on simulation map.
void MapCore_displayTerrain (Image *image, List *obstacleList);
#endif

//! Check if cell in search grid should be considered as a target.
int MapCore_checkIfCellValid (const float cellNMapped);

//! Debugging
void MapCore_printObstructedCellGrid (FILE *f, uchar obstructedCellGrid[((NAV_GRID_DIMS*NAV_GRID_DIMS)/8)+1]);

//! Debugging
void MapCore_printUnreachableExpCellGrid (FILE *f, uchar unreachableExpCellGrid[((GLOB_EXP_GRID_DIMS*GLOB_EXP_GRID_DIMS)/8)+1]);

//! Debugging
void MapCore_printLocalMapGrid (FILE *f, __int16 localMapGrid[GLOB_LOC_MAP_GRID_DIMS][GLOB_LOC_MAP_GRID_DIMS]);

//! Debugging
void MapCore_printExplorationGridSection (
	FILE *f,
	const PointI bl,
	const PointI tr,
	__int16 expGrid[EXP_GRID_DIMS][EXP_GRID_DIMS]);

//! Debugging
void MapCore_printExplorationGrid (
	FILE *f,
	__int16 expGrid[EXP_GRID_DIMS][EXP_GRID_DIMS]);

//! Debugging
void MapCore_printUnreachableLocalMapGrid (FILE *f, uchar unreachableLocalMapGrid[((GLOB_LOC_MAP_GRID_DIMS*GLOB_LOC_MAP_GRID_DIMS)/8)+1]);

#endif

