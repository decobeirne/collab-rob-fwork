#include "../../Common/RobotDefs.h"

#ifdef SIMULATION

#include "CooperativeLocalisation.h"
#include "ObstacleDetection.h"

/*!
I need new colours:
yellow and navy are crap: the camera can't pick them out.
already using: orange, green, light blue, pink
could try and get: purple, red

The schema for encoding robot data in training images is as follows:
- 0 yellowGreen
- 1 darkOrange
- 2 yellow
- 3 lightPink (indistinguishable from lightOrange in imgs!)
- 4 whiteGreen
- 5 aquaGreen
- 6 lightAqua
- 7 lightPurple
- 8 darkPurple
- 9 brown
- 10 darkAqua
- 11 mustard
- 12 darkPink
- 13 lightOrange
- 14 lightGreen
- 15 darkGreen
- 16 navy


- 18 white card on top of robot - deprectaed

- 20 background
*/

/*
Note on coopLoc images taken:
pics of robot 2 (from robot 3) start at 700mm and move in jumps of 50mm
for verification... ended at 250mm for front-on pics
*/

// Don't use any of these (for blob detection anyway) as white balance was on :(
//	#define USE_ROB3_IMAGES_FROM_1113 // Coop loc dist est imgs from lab, yellowGreen & darkOrange
//	#define USE_ROB3_IMAGES_FROM_1211 // As with 2011_11_13

//	#define USE_ROB2_IMAGES_FROM_0319 // Rob2 imgs of rob3 downstairs, navy & yellow

//	#define USE_ROB3_IMAGES_FROM_0331 // Coloured cards, may be mis-marked

//	#define USE_ROB3_IMAGES_FROM_0422 // Rob2 imgs of rob2 downstairs, yellowGreen & darkOrange

//	#define USE_ROB2_IMAGES_FROM_0506_0 // Coloured cards, check marking, also check whiteBalance
//	#define USE_ROB2_IMAGES_FROM_0506_1 // Coloured cards, whiteBalance looks bad
//	#define USE_ROB3_IMAGES_FROM_0506_2 // Rob3 imgs of rob2 at dists, darkOrange, lightPurple, brown, darkAqua

//	#define USE_ROB2_IMAGES_FROM_0520 // Coloured cards

// Can use imgs below this line, as they should have white balance off and
// auto gain on
	#define USE_ROB2_IMAGES_FROM_0527
	#define USE_ROB2_IMAGES_FROM_0715

#include "CooperativeLocalisationGrids2011_11_13.c"
#include "CooperativeLocalisationGrids2011_12_11.c"
#include "CooperativeLocalisationGrids2012_03_19.c"
#include "CooperativeLocalisationGrids2012_03_31.c"
#include "CooperativeLocalisationGrids2012_04_22.c"
#include "CooperativeLocalisationGrids2012_05_06.c"
#include "CooperativeLocalisationGrids2012_05_20.c"
#include "CooperativeLocalisationGrids2012_05_27.c"
#include "CooperativeLocalisationGrids2012_07_15.c"

int CooperativeLocalisation_setupTrainingData(TrainingData *data)
{
	int nTrainingImages = 0;
	int i = -1;
	unsigned int orig;
	unsigned int flip;

	orig = (1 << 8) | 4;
	flip = 2; // x =  1, y = 0

	#include "CooperativeLocalisationData2011_11_13.c"
	#include "CooperativeLocalisationData2011_12_11.c"
	#include "CooperativeLocalisationData2012_03_19.c"
	#include "CooperativeLocalisationData2012_03_31.c"
	#include "CooperativeLocalisationData2012_04_22.c"
	#include "CooperativeLocalisationData2012_05_06.c"
	#include "CooperativeLocalisationData2012_05_20.c"
	#include "CooperativeLocalisationData2012_05_27.c"
	#include "CooperativeLocalisationData2012_07_15.c"

	nTrainingImages = i + 1;
	return nTrainingImages;
}

void CooperativeLocalisation_writeFeatures()
{
	TrainingData data[300];
	int nTrainingImages = CooperativeLocalisation_setupTrainingData (data);
	const char *trainingDataDir = "cooperativeLocalisation";

	TrainingData_init (data, 300);
	ObstacleDetection_writeImageFeatures(data, nTrainingImages, trainingDataDir);
}


#endif // SIMULATION
