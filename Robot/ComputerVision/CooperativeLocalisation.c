#include "../../Common/RobotDefs.h"

#include "CooperativeLocalisation.h"
#include "ObstacleDetection.h"
#include "../../Common/Geometry.h"
#include "../../Common/BitArray.h"

//	#define OWN_COLOUR_0 // These ones are crap
//	#define OWN_COLOUR_3

//	#define OWN_COLOUR_2 // Conflicts with 1, and 1 is in the image I'm currrently testing
//	#define OWN_COLOUR_9
//	#define OWN_COLOUR_10

//	#define USE_COLOUR_REP_0
//	#define USE_COLOUR_REP_1
//	#define USE_COLOUR_REP_2
	#define USE_COLOUR_REP_3 // From temp__printOutColourFeats



#ifdef USE_COLOUR_REP_0
//const float robotColourDescs[N_ROBOT_COLOURS_TOTAL][12] = {
const float robotColourDescs[6][12] = { // should only have colours on other 2 robots
#ifndef OWN_COLOUR_0
	{ // From 556 cells
    0.752728f,     1.525723f,     0.732522f,     0.105502f,     0.608368f,     0.116954f,
    0.068959f,     0.109306f,     0.139359f,     0.032507f,     0.185794f,     0.020169f
},
#endif
#ifndef OWN_COLOUR_1
{ // From 1697 cells
    1.099415f,     1.737076f,     0.968260f,     0.039431f,     0.003926f,     0.108471f,
    0.051571f,     0.075628f,     0.123507f,     0.010048f,     0.007325f,     0.020209f
},
#endif
#ifndef OWN_COLOUR_2
{ // From 518 cells
    0.914152f,     1.754081f,     1.129168f,     0.057938f,     0.007099f,     0.130313f,
    0.107639f,     0.203203f,     0.215267f,     0.015091f,     0.012563f,     0.032308f
},
#endif
#ifndef OWN_COLOUR_3
{ // From 1538 cells
    0.387705f,     0.967913f,     0.484235f,     1.204983f,     0.721672f,     0.199529f,
    0.118815f,     0.137840f,     0.081500f,     0.440427f,     0.144533f,     0.041056f
},
#endif
#ifndef OWN_COLOUR_4
{ // From 131 cells
    0.643227f,     1.155346f,     0.822138f,     0.063248f,     0.230362f,     0.084028f,
    0.005154f,     0.018844f,     0.049226f,     0.009768f,     0.032097f,     0.011441f
},
#endif
#ifndef OWN_COLOUR_5
{ // From 262 cells
    0.563400f,     1.197261f,     0.736452f,     0.078835f,     0.257862f,     0.083015f,
    0.015012f,     0.021522f,     0.048998f,     0.012604f,     0.037425f,     0.010445f
},
#endif
#ifndef OWN_COLOUR_6
{ // From 145 cells
    0.454747f,     1.243364f,     0.557644f,     0.128092f,     0.450510f,     0.088365f,
    0.019554f,     0.054478f,     0.050482f,     0.023198f,     0.098989f,     0.011766f
},
#endif
#ifndef OWN_COLOUR_7
{ // From 128 cells
    1.160652f,     0.917446f,     0.594074f,     0.156726f,     0.428904f,     0.095112f,
    0.040895f,     0.044650f,     0.054763f,     0.029432f,     0.066981f,     0.012642f
},
#endif
#ifndef OWN_COLOUR_8
{ // From 122 cells
    1.342053f,     0.770275f,     0.357493f,     0.366835f,     0.633204f,     0.090235f,
    0.054347f,     0.057527f,     0.033102f,     0.080160f,     0.126221f,     0.011621f
},
#endif
#ifndef OWN_COLOUR_9
{ // From 144 cells
    1.209998f,     1.835458f,     0.350924f,     0.195122f,     0.721140f,     0.091982f,
    0.059089f,     0.104296f,     0.039340f,     0.037853f,     0.214409f,     0.013590f
},
#endif
#ifndef OWN_COLOUR_10
{ // From 122 cells
    0.316456f,     1.535703f,     0.385348f,     0.168838f,     1.132250f,     0.098694f,
    0.018869f,     0.077500f,     0.036929f,     0.034240f,     0.173069f,     0.015364f
},
#endif
};
#endif




#ifdef USE_COLOUR_REP_1

const float robotColourDescs[N_ROBOT_COLOURS][6] = {
//const float robotColourDescs[11][6] = {
#ifndef OWN_COLOUR_0
{ // From 45 images, 3364 cells
  116.012764f,  224.355881f,  123.771217f,
   16.446165f,   12.271070f,   36.013004f,
},
#endif
#ifndef OWN_COLOUR_1
{ // From 82 images, 7530 cells
  159.738678f,  254.612289f,  125.134750f,
    2.927326f,    0.707350f,   33.127377f,
},
#endif
#ifndef OWN_COLOUR_2
{ // From 17 images, 1013 cells
  133.218994f,  253.895966f,  159.761368f,
    1.596377f,    2.033947f,   17.799515f,
},
#endif
#ifndef OWN_COLOUR_3
{ // From 19 images, 1538 cells
   47.561687f,  119.920303f,   59.392750f,
   13.367056f,   16.337004f,    7.256581f,
},
#endif
#ifndef OWN_COLOUR_4
{ // From 7 images, 762 cells
  102.697166f,  151.337265f,  152.573761f,
    4.342885f,    8.221472f,   19.656998f,
},
#endif
#ifndef OWN_COLOUR_5
{ // From 7 images, 809 cells
   76.405708f,  141.622253f,  125.342232f,
    1.742907f,   15.558426f,   22.434458f,
},
#endif
#ifndef OWN_COLOUR_6
{ // From 7 images, 833 cells
   49.989727f,  131.817917f,  113.954552f,
    3.610213f,   15.403238f,   24.002598f,
},
#endif
#ifndef OWN_COLOUR_7
{ // From 37 images, 3753 cells
  140.472183f,   84.644768f,   90.531639f,
   13.261211f,   18.130688f,   11.598166f,
},
#endif
#ifndef OWN_COLOUR_8
{ // From 8 images, 997 cells
  188.417419f,   98.591988f,   68.659203f,
    9.254007f,    7.537184f,    9.921002f,
},
#endif
#ifndef OWN_COLOUR_9
{ // From 31 images, 4018 cells
  145.051483f,  163.287140f,   43.564049f,
   11.463391f,   36.922482f,    8.490932f,
},
#endif
#ifndef OWN_COLOUR_10
{ // From 25 images, 2796 cells
   41.501404f,  149.184143f,   69.241188f,
    9.098320f,   29.598652f,   14.327435f,
},
#endif
};


#endif



#ifdef USE_COLOUR_REP_2

// Colour ids are listed at top of CooperativeLocalisationTraining.c



const float robotColourDescs[N_ROBOT_COLOURS][12] = {
{ // From 764 cells
   98.069251f,   224.279670f,   155.056357f,     4.542965f,    11.150589f,    13.123920f,
    0.568122f,     3.899053f,    18.731133f,     0.770523f,     2.967854f,     2.057104f
},
{ // From 830 cells
  168.554530f,   229.276039f,   164.545360f,     3.148554f,     2.180934f,    13.832048f,
    0.751031f,     2.554837f,    16.412797f,     0.547000f,     0.539597f,     1.775786f
},
{ // From 578 cells
  133.735744f,   227.646420f,   155.906544f,     5.248919f,     2.452941f,    13.770026f,
    1.165583f,     3.552135f,    21.054595f,     0.939354f,     0.652208f,     1.857726f
},
{ // From 886 cells
  158.314878f,   227.921329f,   165.576228f,     3.577088f,     6.901693f,    13.871219f,
    0.755270f,     2.712227f,    14.765812f,     0.555418f,     2.125734f,     1.780090f
},
{ // From 821 cells
   95.311522f,   191.662992f,   158.301147f,     5.112820f,    18.281882f,    12.476736f,
    0.579221f,     2.609624f,    15.606421f,     0.859212f,     3.078454f,     1.690839f
},
{ // From 877 cells
   80.646544f,   184.919445f,   154.372990f,     5.452081f,    23.252110f,    13.289282f,
    0.675655f,     3.005554f,    20.567532f,     0.947655f,     3.780178f,     2.126393f
},
{ // From 994 cells
   62.483923f,   186.231651f,   145.855486f,     8.151082f,    32.476308f,    17.860815f,
    5.906361f,     5.199992f,    14.515878f,     2.473637f,     7.053277f,     3.849767f
},
{ // From 844 cells
  143.923382f,   139.041580f,   133.303011f,    12.276955f,    29.335664f,    15.800030f,
    2.076863f,     5.648804f,    26.707798f,     2.243866f,     5.095907f,     2.447068f
},
{ // From 671 cells
  167.727854f,   115.249711f,    89.380030f,    19.848734f,    44.300001f,    16.001863f,
    1.967116f,     5.831054f,     7.283577f,     3.495913f,     7.541058f,     2.210891f
},
{ // From 647 cells
  161.215726f,   195.696006f,    74.307431f,    10.606724f,    15.747373f,    14.847682f,
    1.466395f,    10.110309f,    11.852719f,     1.735065f,     4.310840f,     2.303625f
},
{ // From 479 cells
   49.142659f,   203.290113f,   111.756438f,    10.998539f,    34.954750f,    17.405794f,
    1.221042f,     3.921276f,     9.661662f,     2.074356f,     5.539648f,     2.396492f
},
{ // From 937 cells
  141.612516f,   229.987155f,   169.000535f,     4.370864f,     2.096985f,    14.136046f,
    0.856308f,     2.422093f,    16.050898f,     0.715633f,     0.490135f,     1.915484f
},
{ // From 1045 cells
  170.332342f,   225.095111f,   146.567500f,     2.196770f,     3.506196f,    14.426507f,
    0.320942f,     5.603602f,    24.924132f,     0.783466f,     1.184484f,     2.268741f
},
{ // From 983 cells
  163.792077f,   228.701916f,   160.536247f,     3.278255f,     2.220956f,    13.527467f,
    0.782142f,     2.500886f,    15.352987f,     0.541867f,     0.504465f,     1.727703f
},
{ // From 778 cells
   85.146935f,   202.680498f,   159.631368f,     5.365681f,    21.893863f,    15.881138f,
    1.035672f,     2.346171f,    13.793633f,     0.929358f,     3.397775f,     2.233110f
},
{ // From 984 cells
   82.214112f,   186.009825f,   125.875942f,     8.863567f,    33.555488f,    17.555996f,
    1.217082f,     5.748667f,    22.594652f,     2.154550f,     6.440390f,     4.200639f
},
};

const uchar coloursToLeaveOut[N_ROBOT_COLOURS] = {0, 0, 1, 0, 0, 0, 1, 1, 0, 1, 0, 1, 1, 0, 0, 1};
#endif

#ifdef USE_COLOUR_REP_3

const float robotColourDescs[N_ROBOT_COLOURS][6] = {
#ifndef OWN_COLOUR_0
{ // From 6 images, 1289 cells
   98.049088f,  218.795410f,  157.133362f,
  196.808960f,  444.729553f,  330.885712f,
},
#endif
#ifndef OWN_COLOUR_1
{ // From 6 images, 1329 cells
  168.675797f,  229.078842f,  163.026199f,
  338.076904f,  460.575897f,  341.267914f,
},
#endif
#ifndef OWN_COLOUR_2
{ // From 3 images, 578 cells
  133.735855f,  227.646484f,  155.906555f,
  268.635284f,  458.840668f,  332.867065f,
},
#endif
#ifndef OWN_COLOUR_3
{ // From 6 images, 1418 cells
  159.087814f,  227.066132f,  167.184235f,
  319.281830f,  457.160004f,  349.844574f,
},
#endif
#ifndef OWN_COLOUR_4
{ // From 6 images, 1334 cells
   94.374573f,  182.655701f,  159.556320f,
  189.970932f,  376.412750f,  335.090881f,
},
#endif
#ifndef OWN_COLOUR_5
{ // From 6 images, 1371 cells
   78.437813f,  180.790146f,  150.567474f,
  159.715027f,  367.379944f,  319.869202f,
},
#endif
#ifndef OWN_COLOUR_6
{ // From 4 images, 994 cells
   62.483883f,  186.231674f,  145.855438f,
  130.874893f,  377.658508f,  306.225189f,
},
#endif
#ifndef OWN_COLOUR_7
{ // From 5 images, 844 cells
  143.923508f,  139.041550f,  133.303024f,
  289.920959f,  283.735809f,  293.309479f,
},
#endif
#ifndef OWN_COLOUR_8
{ // From 5 images, 1222 cells
  174.476822f,  106.822464f,   78.276756f,
  357.635376f,  223.669296f,  169.526154f,
},
#endif
#ifndef OWN_COLOUR_9
{ // From 3 images, 647 cells
  161.215775f,  195.696030f,   74.307434f,
  323.901733f,  401.508606f,  160.466064f,
},
#endif
#ifndef OWN_COLOUR_10
{ // From 5 images, 1296 cells
   44.136047f,  189.971863f,   90.729599f,
   95.559250f,  390.452271f,  199.866257f,
},
#endif
#ifndef OWN_COLOUR_11
{ // From 4 images, 937 cells
  141.612503f,  229.987167f,  169.000519f,
  284.077301f,  462.404297f,  354.051025f,
},
#endif
#ifndef OWN_COLOUR_12
{ // From 4 images, 1045 cells
  170.332336f,  225.094910f,  146.567642f,
  340.989594f,  455.791504f,  318.053711f,
},
#endif
#ifndef OWN_COLOUR_13
{ // From 6 images, 1553 cells
  164.161743f,  229.133606f,  163.317780f,
  329.207031f,  460.751373f,  342.347076f,
},
#endif
#ifndef OWN_COLOUR_14
{ // From 5 images, 1386 cells
   83.725601f,  195.411789f,  141.953690f,
  169.136765f,  399.006256f,  305.744263f,
},
#endif
#ifndef OWN_COLOUR_15
{ // From 4 images, 984 cells
   82.214104f,  186.009872f,  125.875969f,
  165.644958f,  377.766602f,  274.344666f,
},
#endif
};



const uchar coloursToLeaveOut[N_ROBOT_COLOURS] = {0, 0, 1, 0, 0, 0, 1, 1, 0, 1, 0, 1, 1, 0, 0, 1};
#endif














ColourResponse initColourResponse()
{
	ColourResponse c;
	int i;
	for (i = 0; i < N_ROBOT_COLOURS; ++i)
	{
		c.vals[i] = 0.0f;
	}
	return c;
}

//! Calculate similarity between a cell in the camera image and a given colour.
float __calcCellColourDiff (const float cellVals[6],
							ImgFeatures *imgInvFeatures,
							const float colourDesc[12])
{
	float diff;
	/*diff = fabs (cellVals[0] - colourDesc[0]) / colourDesc[0 + 6] + 
		fabs (cellVals[1] - colourDesc[1]) / colourDesc[1 + 6] + 
		fabs (cellVals[2] - colourDesc[2]) / colourDesc[2 + 6] + 
		fabs (cellVals[3] - colourDesc[3]) / colourDesc[3 + 6] + 
		fabs (cellVals[4] - colourDesc[4]) / colourDesc[4 + 6] + 
		fabs (cellVals[5] - colourDesc[5]) / colourDesc[5 + 6];*/
	float tempVals[6];
	int i;
	for (i = 0; i < 6; ++i)
	{
		tempVals[i] = cellVals[i] * imgInvFeatures->vals[2];
	}
	diff = fabs (tempVals[0] - colourDesc[0]) / colourDesc[0 + 6] + 
		fabs (tempVals[1] - colourDesc[1]) / colourDesc[1 + 6] + 
		fabs (tempVals[2] - colourDesc[2]) / colourDesc[2 + 6] + 
		fabs (tempVals[3] - colourDesc[3]) / colourDesc[3 + 6] + 
		fabs (tempVals[4] - colourDesc[4]) / colourDesc[4 + 6] + 
		fabs (tempVals[5] - colourDesc[5]) / colourDesc[5 + 6];
	return diff;
}

void CooperativeLocalisation_calcColourResponses (ImgCellFeatures cellFeatures[SIZE_COOP_LOC_GRID],
												  ImgFeatures *imgInvFeatures,
												  const uchar occupancyGrid[SIZE_COOP_LOC_GRID],
												  ColourResponse colourResponseGrid[SIZE_COOP_LOC_GRID],
												  FILE *f)
{
	int i, j, c;
	float diff;
	ImgCellFeatures *features;
	ColourResponse *response;

	for (j = 0; j < CAM_OCCUPANCY_GRID_Y; ++j)
	{
		for (i = 0; i < CAM_OCCUPANCY_GRID_X; ++i)
		{
			features = &cellFeatures[i + j * CAM_OCCUPANCY_GRID_X];
			response = &colourResponseGrid[i + j * CAM_OCCUPANCY_GRID_X];

			for (c = 0; c < N_ROBOT_COLOURS; ++c)
			{
				diff = __calcCellColourDiff (
					features->vals,
					imgInvFeatures,
					robotColourDescs[c]);

				response->vals[c] = diff;
			}

			//fprintf (f, "colour:%2d, res0:%12f, res1:%12f\n", occupancyGrid[i + j * CAM_OCCUPANCY_GRID_X], res[0], res[1]);
			//colourGrid[i + j * CAM_OCCUPANCY_GRID_X] = currentColour;
		}
	}
}

//void __smoothColourResponses (const ColourResponse *colourResponseGrid, ColourResponse *tempGrid)
//{
//	int i, j, k, l, c;
//	const float coeff = 1.0f;
//	const float thresh = 10.0f;
//	const ColourResponse *ptr, *ptr2;
//	ColourResponse *destPtr;
//
//	for (j = 1; j < CAM_OCCUPANCY_GRID_X - 1; ++j)
//	{
//		for (i = 1; i < CAM_OCCUPANCY_GRID_X - 1; ++i)
//		{
//			ptr = &colourResponseGrid[i + j * CAM_OCCUPANCY_GRID_X];
//			destPtr = &tempGrid[i + j * CAM_OCCUPANCY_GRID_X];
//			for (l = j - 1; l < j + 2; ++l)
//			{
//				for (k = i - 1; k < i + 2; ++k)
//				{
//					ptr2 = &colourResponseGrid[k + l * CAM_OCCUPANCY_GRID_X];
//					for (c = 0; c < N_ROBOT_COLOURS; ++c)
//					{
//						if (ptr->vals[c] < thresh && ptr2->vals[c] < thresh)
//						{
//							destPtr->vals[c] -= coeff * ((ptr2->vals[c] - thresh) / thresh);
//						}
//					}
//				}
//			}
//		}
//	}
//}

extern uchar __calcOccupiedEst (PointF *camScoreGrid, const int cellI, const int cellJ, const int occupGridX, const int occupGridY);

int CooperativeLocalisation_calcOccupiedEst(const int camOccupancyGridY,
											OccupancyGrid *camOccupancyGrid,
											PointF *camScoreGrid,
											uchar *colourGrid,
											int *isObstacleInCamOccupancyGrid)
{
	int i, j;
	uchar gridVal;
	uchar isOccupiedEst;
	int isAnyOccupiedEst = 0;
	int anyRobotCell;

	initOccupancyGrid (camOccupancyGrid);

	for (j = 0; j < camOccupancyGridY; ++j)
	{
		for (i = 0; i < CAM_OCCUPANCY_GRID_X; ++i)
		{
			//gridVal = TwoBitArray_checkCoords (camOccupancyGrid->grid, i, j, CAM_OCCUPANCY_GRID_X);
			//if (gridVal == (uchar)3)
			//{
			//	continue;
			//}

			gridVal = colourGrid[i + j * CAM_OCCUPANCY_GRID_X];

			if (gridVal == COLOUR_CANNOT_EST)
			{
				TwoBitArray_setCoords (camOccupancyGrid->grid, i, j, CAM_OCCUPANCY_GRID_X, (uchar)2);
			}
			else if (gridVal != COLOUR_BACKGROUND)
			{
				TwoBitArray_setCoords (camOccupancyGrid->grid, i, j, CAM_OCCUPANCY_GRID_X, (uchar)3);
			}
			else
			{
				isOccupiedEst = __calcOccupiedEst (camScoreGrid, i, j, CAM_OCCUPANCY_GRID_X, camOccupancyGridY);
				isAnyOccupiedEst |= (int)(isOccupiedEst == (uchar)1);
				TwoBitArray_setCoords (camOccupancyGrid->grid, i, j, CAM_OCCUPANCY_GRID_X, isOccupiedEst);
			}
		}
	}

	// Fill any cells above robot cells as unknown
	for (i = 0; i < CAM_OCCUPANCY_GRID_X; ++i)
	{
		anyRobotCell = 0;

		for (j = 0; j < camOccupancyGridY; ++j)
		{
			isOccupiedEst = TwoBitArray_checkCoords (camOccupancyGrid->grid, i, j, CAM_OCCUPANCY_GRID_X);
			anyRobotCell |= (isOccupiedEst == (uchar)3 || isOccupiedEst == (uchar)2);

			if (anyRobotCell && isOccupiedEst != (uchar)3)
			{
				TwoBitArray_setCoords (camOccupancyGrid->grid, i, j, CAM_OCCUPANCY_GRID_X, (uchar)2);
			}
		}
	}

	return isAnyOccupiedEst;
}

void __propogateBlobJoins (List *coloursToJoin)
{
	int wasJoinPropogated;
	PointI *ptPtr, *ptPtr2, *ptPtr3, *ptPtr4;
	ListNode *iter, *iter2;

//	#define VERBOSE_BLOBS

	do
	{
		wasJoinPropogated = 0;

		// Propogate sequences of joins, e.g. 22<23 and 23<24 go to 22<23 and 22<24
		iter = coloursToJoin->front;
		while (iter)
		{
			ptPtr = (PointI*)iter->value;

			iter2 = coloursToJoin->front;
			while (iter2)
			{
				ptPtr2 = (PointI*)iter2->value;
				if (ptPtr2->x == ptPtr->y)
				{
#ifdef VERBOSE_BLOBS
					printf ("propogating %d<%d to %d<%d\n",
						ptPtr2->x, ptPtr2->y, ptPtr->x, ptPtr2->y);
#endif
					ptPtr3 = ptPtr;
					ptPtr4 = ptPtr2;
					if (ptPtr->x > ptPtr2->x)
					{
						ptPtr3 = ptPtr2;
						ptPtr4 = ptPtr;
					}

					ptPtr4->x = ptPtr3->x;

					wasJoinPropogated = 1;
				}
				iter2 = iter2->next;
			}
			iter = iter->next;
		}

		List_uniqueify (coloursToJoin, PointI_comparePtrs);

		// Propogate joins from common source blobColours, e.g. A<C and B<C go to A<B and A<C
		// So if 2 joins have a common source, get the dest from each and join the higher dest to
		// the lower. The original join to the higher dest can then be removed.
		iter = coloursToJoin->front;
		while (iter)
		{
			ptPtr = (PointI*)iter->value;

			iter2 = coloursToJoin->front;
			while (iter2)
			{
				ptPtr2 = (PointI*)iter2->value;
				if (ptPtr2 != ptPtr && ptPtr2->y == ptPtr->y)
				{
					// Both joins have the same source, but obviously a different dest. Find the
					// join with the higher dest, as this won't be needed anymore.
					ptPtr3 = ptPtr;
					ptPtr4 = ptPtr2;
					if (ptPtr->x > ptPtr2->x)
					{
						ptPtr3 = ptPtr2;
						ptPtr4 = ptPtr;
					}

#ifdef VERBOSE_BLOBS
					printf ("%d<%d and %d<%d have a common source, replace %d<%d with %d<%d\n",
						ptPtr3->x, ptPtr3->y, ptPtr4->x, ptPtr4->y, ptPtr4->x, ptPtr4->y, ptPtr3->x, ptPtr4->x);
#endif

					// Instead of removing one join and adding another, just alter the contents
					// of the join we're removing. ptPtr4 pts to the join with the greater dest,
					// this will become A<B.
					ptPtr4->y = ptPtr4->x;
					ptPtr4->x = ptPtr3->x;

					wasJoinPropogated = 1;
				}
				iter2 = iter2->next;
			}
			iter = iter->next;
		}
	}
	while (wasJoinPropogated);


#ifdef VERBOSE_BLOBS
	if (coloursToJoin->size)
	{
		printf ("colours to join:\n");
		iter = coloursToJoin->front;
		while (iter)
		{
			ptPtr = (PointI*)iter->value;
			printf ("%d<%d\n", ptPtr->x, ptPtr->y);
			iter = iter->next;
		}
	}
#endif

#ifdef VERBOSE_BLOBS
#undef VERBOSE_BLOBS
#endif
}

void __joinBlobs (uchar blobColourGrid[SIZE_COOP_LOC_GRID], List *coloursToJoin)
{
	int i, j;
	uchar cellColour;
	ListNode *iter;
	PointI *ptr;
	for (j = 0; j < CAM_OCCUPANCY_GRID_Y; ++j)
	{
		for (i = 0; i < CAM_OCCUPANCY_GRID_X; ++i)
		{
			cellColour = blobColourGrid[OCCUP_COORD(i, j)];
			if (cellColour > COLOUR_BACKGROUND) // Has been marked as any blob
			{
				iter = coloursToJoin->front;
				while (iter)
				{
					ptr = (PointI*)iter->value;
					if (cellColour == ptr->y)
					{
						blobColourGrid[OCCUP_COORD(i, j)] = ptr->x;
						break;
					}
					iter = iter->next;
				}
			}
		}
	}
}

/*!
Good score is < 15, neutral is 10-20, poor is > 20
*/
__inline float __getResponseScore (const float response)
{
	float score;
	score = (response - 15.0f);
	//if (fabs (score) < 5.0f)
	//{
	//	score = 0.0f;
	//}
	score = min (score, 30.0f);
	return score;
}

float __getColourBlobScore (const int c,
							ColourResponse colourResponseGrid[SIZE_COOP_LOC_GRID],
							const int i,
							const int j)
{
	int k, l;
	float score, ownResponse, nboursResponse;

	ownResponse = __getResponseScore (colourResponseGrid[OCCUP_COORD(i, j)].vals[c]);
	nboursResponse = 0.0f;

	for (k = max (0, i - 1); k < min (CAM_OCCUPANCY_GRID_X - 1, i + 1); ++k)
	{
		for (l = max (0, j - 1); l < min (CAM_OCCUPANCY_GRID_Y - 1, j + 1); ++l)
		{
			if (k == i && l == j)
			{
				continue;
			}

			nboursResponse += __getResponseScore (colourResponseGrid[OCCUP_COORD(k, l)].vals[c]);
		}
	}

	score = 1.0f * ownResponse + 0.125f * nboursResponse; //!< \todo Need to experiment w weights
	return score;
}

//! Create new blob around seed coordinates.
/*!
\return currentBlobColour; may not be incremented if the new blob is appended to an
adjacent one
*/
void __growBlobAroundSeed (const int i,
						  const int j,
						  float blobScoreGrid[SIZE_COOP_LOC_GRID],
						  uchar blobColourGrid[SIZE_COOP_LOC_GRID],
						  const int currentBlobColour)
{
	int k, l;
	blobColourGrid[OCCUP_COORD(i, j)] = currentBlobColour;
	for (k = max (0, i - 1); k < min (CAM_OCCUPANCY_GRID_X - 1, i + 2); ++k)
	{
		for (l = max (0, j - 1); l < min (CAM_OCCUPANCY_GRID_Y - 1, j + 2); ++l)
		{
			if (k == i && l == j)
			{
				continue;
			}

			if (blobScoreGrid[OCCUP_COORD(k, l)] < RELAXED_BLOB_SCORE && 
				blobColourGrid[OCCUP_COORD(k, l)] == COLOUR_BACKGROUND)
			{
				blobColourGrid[OCCUP_COORD(k, l)] = currentBlobColour;
			}
		}
	}
}

void CooperativeLocalisation_groupRobotBlobs (
	ColourResponse colourResponseGrid[SIZE_COOP_LOC_GRID],
	List *colourBlobs)
{
	int i, j, k, l;
	int c;
	uchar cellColour, neighbourColour;
	float colourScoreGrid[SIZE_COOP_LOC_GRID]; //!< \todo Need smaller footprint
	uchar colourGrid[SIZE_COOP_LOC_GRID];
	int nextBlobColour;
	uchar blobIndex;
	int nBlobs, nCellsThisBlob;
	BlobGrid *blobGrid;
	ColourBlob2 *blob2;
	PointI pt, *connectedColours;
	List coloursToJoin;
	PointI bl, tr;
	float avgColourScore;

//	#define VERBOSE_BLOBS

	for (c = 0; c < N_ROBOT_COLOURS; ++c)
	{
		for (i = 0; i < SIZE_COOP_LOC_GRID; ++i)
		{
			colourScoreGrid[i] = colourResponseGrid[i].vals[c]; // Temp
		}

		for (i = 0; i < SIZE_COOP_LOC_GRID; ++i)
		{
			colourGrid[i] = COLOUR_BACKGROUND;
		}

		coloursToJoin = initList();

		// Iterate through blob score grid, growing blobs around seed points
		for (j = 0; j < CAM_OCCUPANCY_GRID_Y; ++j)
		{
			for (i = 0; i < CAM_OCCUPANCY_GRID_X; ++i)
			{
				// Could just grow separate blobs first and then merge these
				// in a subsequent step.
				if (colourGrid[OCCUP_COORD(i, j)] == COLOUR_BACKGROUND &&
					colourScoreGrid[OCCUP_COORD(i, j)] < STRICT_BLOB_SCORE)
				{
					__growBlobAroundSeed (i, j, colourScoreGrid, colourGrid, c);
				}
			}
		}

		// Iterate through blob grid, merge adjacent blobs, merge across gaps later; use
		// manhattan off each blob in turn

		nextBlobColour = COLOUR_BACKGROUND + 1;
		for (j = 0; j < CAM_OCCUPANCY_GRID_Y; ++j)
		{
			for (i = 0; i < CAM_OCCUPANCY_GRID_X; ++i)
			{
				if (colourGrid[OCCUP_COORD(i, j)] == c)
				{
					cellColour = COLOUR_BACKGROUND;

					for (l = max (j - 1, 0); l < min (j + 2, CAM_OCCUPANCY_GRID_Y); ++l)
					{
						for (k = max (i - 1, 0); k < min (i + 2, CAM_OCCUPANCY_GRID_X); ++k)
						{
							if (k == i && l == j)
							{
								continue;
							}

							neighbourColour = colourGrid[OCCUP_COORD(k, l)];
							if (neighbourColour > COLOUR_BACKGROUND)
							{
								if (cellColour == COLOUR_BACKGROUND)
								{
									cellColour = neighbourColour;
								}
								else if (neighbourColour != cellColour)
								{
									pt.x = min (cellColour, neighbourColour);
									pt.y = max (cellColour, neighbourColour);
									if (!List_isElement(&coloursToJoin, &pt, PointI_comparePtrs))
									{
										connectedColours = (PointI*)malloc (sizeof (PointI));
										*connectedColours = pt;
										List_pushValue (&coloursToJoin, connectedColours);
#ifdef VERBOSE_BLOBS
										printf ("join blobs %d<%d (at pt (%d,%d))\n",
											pt.x, pt.y,
											i, j);
#endif
									}
								}
							}
						}
					}

					if (cellColour == COLOUR_BACKGROUND)
					{
						cellColour = nextBlobColour++;
#ifdef VERBOSE_BLOBS
						printf ("new blob %d\n", cellColour);
#endif
					}

					colourGrid[OCCUP_COORD(i, j)] = cellColour;
				}
			}
		}

		__propogateBlobJoins (&coloursToJoin);
		__joinBlobs (colourGrid, &coloursToJoin);

		nBlobs = 0;
		for (blobIndex = COLOUR_BACKGROUND + 1; blobIndex < nextBlobColour; ++blobIndex)
		{
			nCellsThisBlob = 0;
			bl.x = bl.y = 100;
			tr.x = tr.y = -100;
			avgColourScore = 0.0f;
			blobGrid = allocBlobGrid();
			for (j = 0; j < CAM_OCCUPANCY_GRID_Y; ++j)
			{
				for (i = 0; i < CAM_OCCUPANCY_GRID_X; ++i)
				{
					if (colourGrid[OCCUP_COORD(i, j)] == blobIndex)
					{
						pt.x = i;
						pt.y = j;
						BitArray_setElement_pt (
							blobGrid->grid,
							//i,
							pt,
							CAM_OCCUPANCY_GRID_X,
							CAM_OCCUPANCY_GRID_Y,
							1);
						++nCellsThisBlob;
						avgColourScore += colourScoreGrid[OCCUP_COORD(i, j)];
						bl.x = min (bl.x, pt.x);
						bl.y = min (bl.y, pt.y);
						tr.x = max (tr.x, pt.x);
						tr.y = max (tr.y, pt.y);
					}
				}
			}

			if (nCellsThisBlob)
			{
				++nBlobs;
				blob2 = allocColourBlob2();
				blob2->blobGrid = blobGrid;
				blob2->nCells = nCellsThisBlob;
				blob2->blobIndex = blobIndex;
				blob2->colour = c;
				blob2->avgScore = avgColourScore / nCellsThisBlob;
				blob2->bl = bl;
				blob2->tr = tr;
				List_pushValue (colourBlobs, blob2);
			}
			else
			{
				free (blobGrid);
			}
		}

		List_clear (&coloursToJoin, 1);
	}

#ifdef VERBOSE_BLOBS
#undef VERBOSE_BLOBS
#endif
}






















//! \todo Deprecated
//void CooperativeLocalisation_estBlobRoughLoc (ColourBlob *blob, VisibleRobot *visibleRobot, const PointI orig)
//{
//	Vector4I edgeUsedForEst;
//
//	// Estimate whether or not edges of robot side are visible.
//	blob->edgesVisible = initVector4I (1,1,1,1); // TBLR
//	blob->edgesVisible.x = (blob->tr.y != CAM_OCCUPANCY_GRID_Y - 1);
//	blob->edgesVisible.y = (blob->bl.y != 0);
//	blob->edgesVisible.z = (blob->bl.x != 0);
//	blob->edgesVisible.w = (blob->tr.x != CAM_OCCUPANCY_GRID_X - 1);
//
//	edgeUsedForEst = initVector4I (0,0,0,0);
//
//	if (blob->edgesVisible.x && blob->edgesVisible.y &&
//		blob->edgesVisible.z && blob->edgesVisible.w)
//	{
//		
//	}
//}




