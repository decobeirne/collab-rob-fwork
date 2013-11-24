#include "RobotRecognitionModel.h"
#include "RobotRecognitionCore.h"

#ifndef BOARD

/*!
\file RobotRecognitionModel.c
Contains representation of appearance of different robots in group. Each robot
had been constructed with different colours to aid recognition.
*/


#if defined(SIMULATION) // Only want this data for training/testing
/*
DEPRECATED 20130128
0	in
1	leave out
2	in
3	leave out
4	in
5	leave out
6	in
7	in
8	in
9	in
10	in
11	in
12	leave out
13	leave out
14	leave out
15	leave out
*/

/*
0	leave out
1	leave out
2	in
3	in
4	leave out
5	in
6	in
7	leave out
8	leave out
9	in
10	leave out
11	in
12	in
13	in
14	in
15	leave out
*/


/*
DEPRECATED 20120121
const uchar coloursToLeaveOut[N_ROBOT_COLOURS_TOTAL] = {0, 0, 1, 0, 0, 0, 1, 1, 0, 1, 0, 1, 1, 0, 0, 1};
*/
/*
DEPRECATED 20130128
const uchar coloursToLeaveOut[N_ROBOT_COLOURS_TOTAL] = {0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1};
*/
/*
DEPRECATED 20130421 - including 07|lightPurple and 08|darkPurple
const uchar coloursToLeaveOut[N_ROBOT_COLOURS_TOTAL] = {1, 1, 0, 0, 1, 0, 0, 1, 1, 0, 1, 0, 0, 0, 0, 1};
*/
const uchar coloursToLeaveOut[N_ROBOT_COLOURS_TOTAL] = {1, 1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};

int RobotRecognitionModel_leaveOutColourInTraining (const int colour)
{
	return (int)coloursToLeaveOut[colour];
}
#endif // if defined(SIMULATION)


























/*
The data below is defined per robot - as we only need 5 colours for each robot, whil
we want them all for testing/training
*/
#if defined(SIMULATION) // This gets all the colours (that are in training images)
const int RobotRecognitionModel_mapColourDescriptionArrayToColourId[N_ROBOT_COLOURS] = {
2,
3,
5,
6,
7,
8,
9,
11,
12,
13,
14,
};

const float RobotRecognitionModel_colourAvgs[N_ROBOT_COLOURS] = {
    5.079165f,
   11.806927f,
   15.028613f,
   15.773170f,
   25.269556f,
   18.228935f,
    8.101561f,
    8.064010f,
    7.868307f,
    4.966689f,
   11.126181f,
};
#define USING_COLOUR_2
#define USING_COLOUR_3
#define USING_COLOUR_5
#define USING_COLOUR_6
#define USING_COLOUR_7
#define USING_COLOUR_8
#define USING_COLOUR_9
#define USING_COLOUR_11
#define USING_COLOUR_12
#define USING_COLOUR_13
#define USING_COLOUR_14
#else // if defined(SIMULATION)

#if ROBOT_PLATFORM == 2
// N_ROBOT_COLOURS should be 5
// r2 can see (6,11,8) on r3 and (12,14,8) on r4 => 6,8,11,12,14
const int RobotRecognitionModel_mapColourDescriptionArrayToColourId[N_ROBOT_COLOURS] = {
//
//
//
6,
//
8,
//
11,
12,
//
14,
};

const float RobotRecognitionModel_colourAvgs[N_ROBOT_COLOURS] = {
//
//
//
   15.773170f,
//
   18.228935f,
//
    8.064010f,
    7.868307f,
//
   11.126181f,
};
#define USING_COLOUR_6
#define USING_COLOUR_8
#define USING_COLOUR_11
#define USING_COLOUR_12
#define USING_COLOUR_14
#endif // if ROBOT_PLATFORM == 2
#if ROBOT_PLATFORM == 3
// N_ROBOT_COLOURS should be 5
// r3 can see (2,5,8) on r2 and (12,14,8) on r4 => 2,5,8,12,14
const int RobotRecognitionModel_mapColourDescriptionArrayToColourId[N_ROBOT_COLOURS] = {
2,
//
5,
//
//
8,
//
//
12,
//
14,
};

const float RobotRecognitionModel_colourAvgs[N_ROBOT_COLOURS] = {
    5.079165f,
//
   15.028613f,
//
//
   18.228935f,
//
//
    7.868307f,
//
   11.126181f,
};
#define USING_COLOUR_2
#define USING_COLOUR_5
#define USING_COLOUR_8
#define USING_COLOUR_12
#define USING_COLOUR_14
#endif // if ROBOT_PLATFORM == 3
#if ROBOT_PLATFORM == 4
// N_ROBOT_COLOURS should be 5
// r4 can see (2,5,8) on r2 and (6,11,8) on r3 => 2,5,6,8,11
const int RobotRecognitionModel_mapColourDescriptionArrayToColourId[N_ROBOT_COLOURS] = {
2,
//
5,
6,
//
8,
//
11,
//
//
//
};

const float RobotRecognitionModel_colourAvgs[N_ROBOT_COLOURS] = {
    5.079165f,
//
   15.028613f,
   15.773170f,
//
   18.228935f,
//
    8.064010f,
//
//
//
};
#define USING_COLOUR_2
#define USING_COLOUR_5
#define USING_COLOUR_6
#define USING_COLOUR_8
#define USING_COLOUR_11
#endif // if ROBOT_PLATFORM == 4
#endif // if defined(SIMULATION)

int RobotRecognitionModel_mapColourIdToIndex(const int colourId)
{
	int i;
	for (i = 0; i < N_ROBOT_COLOURS; ++i)
	{
		if (RobotRecognitionModel_mapColourDescriptionArrayToColourId[i] == colourId)
		{
			return i;
		}
	}
	DEBUG_ASSERT(0)
	return -1;
}

























#ifdef USING_COLOUR_2
const int nGroupsForColour2 = 32;
const float groupsForColour2[32][6] = {
{  132.524320f,   225.222223f,   139.473004f,     5.062349f,     2.415663f,    11.661446f}, // From 83 cells, with group error   680.561707
{  129.571099f,   218.575067f,   114.119379f,     7.779464f,     2.762946f,     9.004911f}, // From 56 cells, with group error   673.397870
{  130.937471f,   222.016116f,   125.947572f,     6.039610f,     2.239286f,     8.838312f}, // From 77 cells, with group error   887.295367
{  131.743477f,   224.331020f,   135.363216f,     6.020455f,     2.606534f,    11.508807f}, // From 88 cells, with group error   788.881660
{  132.841457f,   229.465401f,   163.131093f,     4.902632f,     1.859430f,    12.151535f}, // From 114 cells, with group error   664.862576
{  133.742713f,   228.755124f,   158.653862f,     4.719947f,     2.009840f,    12.415160f}, // From 94 cells, with group error   627.558671
{  132.834910f,   219.897756f,   118.233452f,     5.827394f,     3.337234f,    10.951064f}, // From 94 cells, with group error   841.249677
{  135.528194f,   231.129355f,   175.340521f,     4.961567f,     1.911194f,    14.121269f}, // From 67 cells, with group error   977.362299
{  131.805633f,   223.520177f,   131.861940f,     5.979675f,     2.786585f,    11.600000f}, // From 123 cells, with group error   864.154094
{  132.357859f,   227.263099f,   149.983985f,     5.250901f,     2.100676f,    11.549550f}, // From 111 cells, with group error   924.458654
{  132.144835f,   219.775246f,   117.769396f,     6.089474f,     3.410526f,    11.302105f}, // From 95 cells, with group error   687.121245
{  131.994662f,   227.031867f,   148.716051f,     5.369369f,     2.002477f,    11.138739f}, // From 111 cells, with group error   643.617728
{  132.138756f,   223.993426f,   133.959341f,     4.759783f,     2.173913f,     9.510326f}, // From 138 cells, with group error  1253.837046
{  135.106482f,   227.270427f,   150.295208f,     5.429412f,     2.552941f,    13.299633f}, // From 68 cells, with group error   771.641372
{  133.189110f,   225.467045f,   140.778721f,     5.697034f,     2.626059f,    12.735805f}, // From 118 cells, with group error  1222.509042
{  133.549780f,   229.049101f,   160.598540f,     4.781422f,     2.110092f,    13.094954f}, // From 109 cells, with group error   988.901305
{  131.252438f,   217.826512f,   111.711697f,     6.827632f,     3.396842f,     9.852369f}, // From 95 cells, with group error   815.307863
{  133.169381f,   226.128993f,   143.982442f,     5.366810f,     2.402802f,    12.152802f}, // From 116 cells, with group error   926.893098
{  131.690645f,   221.725927f,   124.761989f,     6.726053f,     3.041579f,    11.390526f}, // From 95 cells, with group error   820.405593
{  133.383868f,   228.234779f,   155.482582f,     5.333898f,     2.040890f,    11.959746f}, // From 118 cells, with group error   989.116178
{  132.547669f,   220.877575f,   122.951133f,     6.194907f,     3.413657f,    11.941898f}, // From 108 cells, with group error  2108.236328
{  133.644629f,   230.300736f,   168.801041f,     4.854545f,     1.851240f,    12.866116f}, // From 121 cells, with group error   847.287243
{  131.785067f,   222.928556f,   129.500359f,     5.886129f,     2.747742f,    11.299839f}, // From 155 cells, with group error  1360.989997
{  131.638596f,   219.459584f,   117.537478f,     6.298611f,     3.031151f,    10.423016f}, // From 126 cells, with group error  2074.196566
{  133.049520f,   226.549316f,   146.089013f,     5.057459f,     2.175967f,    11.776658f}, // From 181 cells, with group error  1147.053545
{  133.261906f,   229.940650f,   166.374397f,     4.526553f,     1.750311f,    11.959006f}, // From 161 cells, with group error  1372.140190
{  132.787095f,   227.802471f,   153.004154f,     5.197121f,     2.129394f,    11.979243f}, // From 165 cells, with group error  1118.119315
{  132.141374f,   223.861000f,   133.329383f,     5.743293f,     2.466311f,    10.673933f}, // From 164 cells, with group error  1471.898932
{  133.956263f,   229.597346f,   164.195907f,     4.751316f,     1.877961f,    12.242105f}, // From 152 cells, with group error  1419.683358
{  132.963717f,   220.761971f,   122.640319f,     5.718699f,     3.086992f,    11.117277f}, // From 123 cells, with group error  2340.440742
{  132.099900f,   220.153190f,   119.101664f,     6.028742f,     3.185204f,    10.793538f}, // From 147 cells, with group error  1129.111353
{  132.183289f,   225.497021f,   140.714726f,     5.294146f,     2.113415f,    10.601586f}, // From 205 cells, with group error  1457.158314
};
#endif
#ifdef USING_COLOUR_3
const int nGroupsForColour3 = 32;
const float groupsForColour3[32][6] = {
{  158.189816f,   219.565907f,   201.751636f,     3.654412f,    14.680147f,    14.557353f}, // From 68 cells, with group error  1871.263375
{  157.811003f,   217.774785f,   145.280774f,     4.096324f,    16.816912f,    11.261765f}, // From 68 cells, with group error  1422.951770
{  158.208403f,   219.121063f,   194.036209f,     4.073507f,    15.894776f,    16.385075f}, // From 67 cells, with group error  1350.222725
{  157.726056f,   214.074077f,   140.734036f,     4.492241f,    18.268104f,    16.439655f}, // From 58 cells, with group error  1284.439608
{  158.186455f,   216.344751f,   149.139271f,     3.917123f,    16.933219f,    13.349315f}, // From 73 cells, with group error  1384.606132
{  157.563954f,   217.325581f,   137.451982f,     4.052616f,    15.018896f,    11.807558f}, // From 86 cells, with group error  1290.495385
{  159.127760f,   224.846803f,   150.141790f,     3.510101f,     7.246970f,    12.292930f}, // From 99 cells, with group error  2625.397538
{  158.506750f,   224.325489f,   157.791746f,     3.584110f,    11.366102f,    11.937288f}, // From 118 cells, with group error  1917.456916
{  158.381124f,   226.524296f,   209.587218f,     3.033065f,    12.214248f,    12.171774f}, // From 93 cells, with group error  2802.085350
{  157.935103f,   220.260596f,   179.462630f,     3.627928f,    16.253379f,    12.089189f}, // From 111 cells, with group error  1281.600271
{  157.254347f,   219.532126f,   142.788551f,     4.025255f,    13.618878f,    13.433674f}, // From 98 cells, with group error  1927.490999
{  157.911802f,   219.553046f,   184.524797f,     3.694915f,    15.700212f,    13.077966f}, // From 118 cells, with group error  2709.469176
{  159.126137f,   226.246857f,   168.022538f,     3.289858f,    10.405425f,    13.961085f}, // From 106 cells, with group error  2183.473144
{  158.591234f,   221.460417f,   149.968910f,     3.571789f,    13.936239f,    11.537385f}, // From 109 cells, with group error  1845.476559
{  158.042118f,   223.990988f,   201.463456f,     3.465929f,    15.042036f,    13.703761f}, // From 113 cells, with group error  1464.646653
{  157.367324f,   209.477732f,   119.553213f,     4.907595f,    18.987659f,    12.481646f}, // From 79 cells, with group error  1524.045418
{  157.643640f,   221.274075f,   160.093399f,     3.673696f,    13.956522f,    12.669348f}, // From 115 cells, with group error  1853.228730
{  157.952382f,   222.618167f,   165.747797f,     3.794048f,    14.033929f,    13.501389f}, // From 126 cells, with group error  1522.804724
{  157.807705f,   219.652001f,   146.274223f,     3.955600f,    13.812000f,    12.126800f}, // From 125 cells, with group error  1943.208046
{  158.056628f,   223.209368f,   158.657791f,     3.790083f,    11.977686f,    13.054339f}, // From 121 cells, with group error  1509.880682
{  157.001463f,   209.223361f,   131.340319f,     4.724342f,    19.781799f,    12.161404f}, // From 114 cells, with group error  2116.175209
{  158.485857f,   226.643776f,   194.611404f,     3.130118f,    12.542717f,    12.509646f}, // From 127 cells, with group error  2981.456107
{  159.021661f,   219.784889f,   127.760087f,     4.074107f,     8.782813f,    12.290848f}, // From 112 cells, with group error  2171.336451
{  158.186668f,   223.157224f,   191.699631f,     3.699250f,    14.531000f,    14.960750f}, // From 100 cells, with group error  2153.312712
{  158.540950f,   221.878718f,   168.360069f,     3.642782f,    12.863733f,    12.197183f}, // From 142 cells, with group error  2595.747093
{  158.408832f,   223.053116f,   170.918804f,     3.504121f,    13.010302f,    12.559753f}, // From 182 cells, with group error  2907.029543
{  158.089189f,   217.390524f,   129.081972f,     4.191544f,    12.157169f,    11.879780f}, // From 136 cells, with group error  2766.056174
{  157.694564f,   220.648862f,   177.213439f,     3.628045f,    15.032052f,    13.023077f}, // From 156 cells, with group error  3033.470453
{  157.915429f,   218.491273f,   135.741449f,     4.087420f,    12.618631f,    12.296816f}, // From 157 cells, with group error  4665.002584
{  158.214050f,   218.247157f,   147.179807f,     3.986592f,    15.842877f,    12.580168f}, // From 179 cells, with group error  2844.975115
{  158.194033f,   226.815996f,   197.806559f,     3.360669f,    11.584077f,    13.422293f}, // From 157 cells, with group error  4036.933120
{  158.111213f,   219.578303f,   162.432368f,     3.848234f,    16.089267f,    12.789946f}, // From 184 cells, with group error  5068.891834
};
#endif
#ifdef USING_COLOUR_5
const int nGroupsForColour5 = 32;
const float groupsForColour5[32][6] = {
{   77.377257f,   173.741858f,   150.558069f,     6.179648f,    23.361433f,    11.791709f}, // From 199 cells, with group error  4795.568117
{   77.602177f,   171.997646f,   115.870940f,     6.406031f,    23.802632f,     9.337171f}, // From 228 cells, with group error  5137.039375
{   79.547753f,   166.955578f,   115.386705f,     6.358146f,    23.647894f,     9.684972f}, // From 178 cells, with group error  4177.980931
{   86.201022f,   154.690465f,   127.996280f,     6.768575f,    20.132360f,     9.933762f}, // From 214 cells, with group error  4226.950765
{   79.931347f,   162.500152f,    99.105014f,     6.808537f,    24.396444f,     8.930793f}, // From 246 cells, with group error  7400.382341
{   80.297367f,   167.852264f,   132.639096f,     6.350111f,    20.914667f,    10.882889f}, // From 225 cells, with group error  4810.464648
{   77.354683f,   167.851853f,   166.595020f,     8.392686f,    27.028931f,    15.226856f}, // From 229 cells, with group error  5766.939852
{   80.382419f,   166.572142f,   102.906441f,     6.875602f,    24.435342f,     9.055221f}, // From 249 cells, with group error  5405.508620
{   83.483738f,   151.864080f,   121.036563f,    12.030609f,    34.140866f,    13.937180f}, // From 156 cells, with group error  7169.389037
{   78.851780f,   168.811127f,   120.907408f,     7.034570f,    25.787403f,    10.435156f}, // From 256 cells, with group error  4298.352749
{   79.856039f,   163.595895f,    91.280918f,     7.935218f,    27.490000f,     8.952935f}, // From 230 cells, with group error  4179.361292
{   78.603282f,   170.178910f,   117.320284f,     6.360067f,    24.437584f,     9.682970f}, // From 298 cells, with group error  6276.611528
{   84.749003f,   156.513150f,   118.292029f,     6.920539f,    22.318123f,     9.931320f}, // From 269 cells, with group error  5478.996910
{   81.122439f,   162.014777f,   153.268421f,     7.727793f,    24.265426f,    13.763830f}, // From 188 cells, with group error  4992.141346
{   80.448337f,   165.548109f,   113.154792f,     6.382699f,    22.206250f,     9.228714f}, // From 276 cells, with group error  7395.379884
{   76.043230f,   174.863138f,   104.979052f,     6.297734f,    22.569766f,     8.925547f}, // From 320 cells, with group error  4804.317053
{   77.099604f,   175.589257f,   149.331159f,     5.737402f,    20.994648f,    10.881528f}, // From 383 cells, with group error  4995.781080
{   80.790249f,   162.377979f,    94.987094f,     7.588805f,    27.539563f,     8.705556f}, // From 297 cells, with group error  5210.448126
{   78.178361f,   171.145280f,   119.709269f,     6.215200f,    23.672007f,     9.767547f}, // From 426 cells, with group error  5468.473178
{   78.569584f,   169.573465f,   119.148205f,     6.471632f,    24.862800f,     9.794611f}, // From 334 cells, with group error  9666.551766
{   78.227127f,   164.561223f,   142.905209f,     9.124772f,    33.077854f,    15.555936f}, // From 219 cells, with group error  8764.126134
{   78.486234f,   170.350106f,   145.940113f,     6.561698f,    22.429434f,    11.419906f}, // From 265 cells, with group error  6617.475649
{   77.946285f,   168.233560f,    97.091762f,     7.130289f,    26.619151f,     9.017949f}, // From 312 cells, with group error  6132.800354
{   79.103741f,   160.988300f,   112.118206f,    10.229229f,    34.708209f,    13.415548f}, // From 201 cells, with group error 11776.915662
{   77.703547f,   164.163607f,   171.895560f,     9.661653f,    33.242585f,    18.555085f}, // From 236 cells, with group error  8150.141228
{   81.965409f,   167.226285f,   117.856919f,     6.456722f,    22.847937f,     9.435967f}, // From 424 cells, with group error 10607.387562
{   77.543676f,   169.484898f,   125.445696f,     7.576948f,    26.621299f,    11.512013f}, // From 385 cells, with group error 16992.964290
{   84.044170f,   158.781095f,   116.953892f,     7.211402f,    23.615794f,     9.849916f}, // From 296 cells, with group error  7295.943562
{   78.253087f,   155.643948f,   147.308643f,    14.183334f,    43.514584f,    20.428010f}, // From 216 cells, with group error  9026.075447
{   78.226133f,   172.456177f,   143.397115f,     5.956129f,    21.908799f,    11.018932f}, // From 412 cells, with group error  8340.725570
{   77.871861f,   165.604427f,   123.893949f,     9.537256f,    32.379940f,    13.463903f}, // From 410 cells, with group error 18933.627673
{   77.866292f,   171.344266f,   134.258607f,     6.720760f,    25.341851f,    11.146531f}, // From 454 cells, with group error 12598.133646
};
#endif
#ifdef USING_COLOUR_6
const int nGroupsForColour6 = 32;
const float groupsForColour6[32][6] = {
{   58.904116f,   145.600824f,   176.528809f,    10.121111f,    28.403334f,    18.961111f}, // From 45 cells, with group error  1067.028593
{   59.345486f,   185.666669f,   118.315973f,    14.789063f,    36.611720f,    23.107032f}, // From 32 cells, with group error  1067.540790
{   57.949630f,   193.428149f,   152.687778f,    10.594500f,    26.923001f,    23.393000f}, // From 50 cells, with group error  1096.067310
{   61.130907f,   140.439655f,   100.155173f,    10.240948f,    33.022414f,    11.316810f}, // From 58 cells, with group error  1178.434098
{   57.613604f,   145.835827f,   171.198005f,    10.887981f,    32.313462f,    19.045193f}, // From 52 cells, with group error  1305.793122
{   59.146991f,   143.845777f,   133.661458f,    10.416797f,    31.694141f,    14.288281f}, // From 64 cells, with group error  1221.965927
{   57.290467f,   193.911182f,   152.945132f,    10.640741f,    24.869445f,    19.646760f}, // From 54 cells, with group error  1290.282210
{   60.483298f,   141.063908f,   170.301381f,    11.685784f,    36.367648f,    19.539706f}, // From 51 cells, with group error  1078.403729
{   64.104701f,   182.796297f,   133.573363f,    13.062500f,    32.149039f,    21.659616f}, // From 52 cells, with group error  1085.176682
{   57.386396f,   192.832978f,   147.315172f,    11.207212f,    27.047597f,    22.246154f}, // From 52 cells, with group error  1216.064336
{   58.097692f,   146.802738f,   166.887011f,    11.190580f,    34.572465f,    17.986595f}, // From 69 cells, with group error  1271.972107
{   59.481482f,   144.168694f,    82.359954f,     9.923047f,    31.335547f,    10.155078f}, // From 64 cells, with group error  1101.969311
{   58.254182f,   138.472522f,    74.773597f,    11.299194f,    38.457259f,     9.626613f}, // From 62 cells, with group error  1060.211371
{   59.333334f,   149.064950f,   170.415728f,     9.545290f,    26.710508f,    16.253986f}, // From 69 cells, with group error  1081.010804
{   64.816722f,   182.134533f,   126.627724f,    13.645956f,    32.859927f,    20.908089f}, // From 68 cells, with group error  1204.640622
{   58.779957f,   146.389707f,   178.912311f,    10.429412f,    29.737133f,    18.750000f}, // From 68 cells, with group error  1248.473352
{   59.723237f,   142.935060f,    90.221716f,    10.217808f,    30.072261f,     9.794178f}, // From 73 cells, with group error  1692.036189
{   62.570086f,   186.983762f,   151.734189f,    10.676539f,    27.342693f,    21.818846f}, // From 65 cells, with group error  1201.332858
{   58.617285f,   141.508334f,   147.128705f,    12.329584f,    31.310001f,    16.076667f}, // From 60 cells, with group error  1374.748039
{   60.681819f,   141.798702f,    98.225591f,    10.204546f,    36.833442f,    11.048052f}, // From 77 cells, with group error  1671.007475
{   54.885894f,   194.777779f,   143.176744f,    11.889706f,    29.313971f,    21.621324f}, // From 68 cells, with group error  1616.347156
{   61.733514f,   157.075656f,    96.387760f,    10.938720f,    34.873171f,    13.546037f}, // From 82 cells, with group error  3505.490104
{   59.156495f,   143.792385f,   155.243350f,    10.778873f,    38.642254f,    16.799296f}, // From 71 cells, with group error  2582.876877
{   59.132112f,   142.128294f,    84.178313f,     9.899485f,    32.612630f,     9.667784f}, // From 97 cells, with group error  2296.966917
{   59.846166f,   151.283320f,   100.311171f,     8.643110f,    29.323229f,    10.880118f}, // From 127 cells, with group error  1632.606438
{   58.859765f,   148.196803f,   173.422560f,    10.087046f,    29.880910f,    17.580909f}, // From 110 cells, with group error  1744.188736
{   59.804939f,   144.626809f,   105.535274f,    10.204524f,    28.693334f,    12.258572f}, // From 105 cells, with group error  3531.903726
{   65.369383f,   180.428396f,   119.028643f,    13.885334f,    35.276334f,    21.272000f}, // From 75 cells, with group error  1975.463972
{   60.061656f,   138.103051f,   115.695644f,    11.130882f,    34.696471f,    12.948530f}, // From 85 cells, with group error  3542.230717
{   59.510537f,   189.063220f,   133.665072f,    12.950431f,    26.880388f,    21.323707f}, // From 116 cells, with group error  3058.275324
{   59.423183f,   147.775378f,   124.738170f,     9.448380f,    31.295834f,    12.535648f}, // From 108 cells, with group error  4734.485899
{   59.654027f,   141.249413f,    90.721194f,    10.407937f,    32.779961f,    10.344841f}, // From 126 cells, with group error  2454.083426
};
#endif
#ifdef USING_COLOUR_7
const int nGroupsForColour7 = 32;
const float groupsForColour7[32][6] = {
{  127.038722f,    96.886364f,    89.315657f,    38.643183f,    78.622729f,    14.453409f}, // From 22 cells, with group error   519.945094
{  130.939260f,    96.570371f,   103.857038f,    32.413000f,    61.843001f,    13.104000f}, // From 25 cells, with group error   600.679484
{  118.811112f,    88.405557f,    97.747223f,    35.792501f,    53.387501f,    16.620000f}, // From 20 cells, with group error   534.007397
{  128.574076f,    84.399177f,    88.904322f,    49.581944f,    97.505557f,    13.709722f}, // From 18 cells, with group error   495.950026
{   97.384533f,    45.040305f,   160.551200f,    71.033825f,    50.708825f,    23.245589f}, // From 17 cells, with group error   675.217535
{  129.793881f,    95.667473f,    88.183575f,    36.900001f,    77.914132f,    13.035870f}, // From 23 cells, with group error   692.147808
{  120.182100f,    81.475310f,    80.526749f,    63.265279f,    95.756946f,    15.776389f}, // From 18 cells, with group error   637.856581
{  124.023570f,    94.734008f,   109.470540f,    33.202273f,    71.575002f,    15.756818f}, // From 22 cells, with group error   576.227953
{  121.244214f,    81.142361f,    83.828705f,    65.432814f,    79.470315f,    16.867188f}, // From 16 cells, with group error   646.191632
{  133.690239f,   100.920876f,    96.597644f,    33.795456f,    88.531821f,    13.415909f}, // From 22 cells, with group error   648.661927
{  117.169630f,    76.931853f,   105.109631f,    39.532001f,    61.132001f,    17.411001f}, // From 25 cells, with group error   544.724460
{  112.847223f,    77.556482f,   106.770371f,    42.576251f,    73.046251f,    20.323750f}, // From 20 cells, with group error   708.126829
{  129.917824f,    89.015047f,    81.050927f,    57.931251f,   108.704688f,    14.159375f}, // From 16 cells, with group error   614.515270
{  122.785802f,    87.096915f,    91.859877f,    38.962501f,    78.814168f,    14.997500f}, // From 30 cells, with group error   615.486243
{  131.733199f,   101.618656f,   102.327161f,    33.528704f,    55.326852f,    14.168519f}, // From 27 cells, with group error   680.684686
{  111.243828f,    46.436214f,   154.565845f,    57.977779f,    54.138890f,    19.913889f}, // From 18 cells, with group error   718.724740
{  126.183449f,    92.390047f,   103.478589f,    34.304688f,    80.120314f,    14.731250f}, // From 32 cells, with group error   788.572823
{  122.987408f,    77.428889f,    87.288890f,    52.312001f,    85.892002f,    14.873000f}, // From 25 cells, with group error   736.958123
{  130.638096f,    97.165080f,   100.827514f,    33.935001f,    71.720716f,    13.295000f}, // From 35 cells, with group error   750.404028
{  119.159144f,    88.251737f,    98.681714f,    36.225782f,    66.210158f,    17.653125f}, // From 32 cells, with group error   771.208133
{  121.244048f,    82.927249f,    81.199075f,    55.402679f,    74.943751f,    15.388393f}, // From 28 cells, with group error   772.613829
{  125.891536f,    86.539683f,    91.693784f,    47.156251f,    64.444644f,    15.837500f}, // From 28 cells, with group error   721.927917
{  130.671446f,    98.540025f,    97.201315f,    35.215323f,    81.847581f,    12.476613f}, // From 31 cells, with group error   844.919640
{  117.095680f,    81.943829f,   100.753087f,    40.599167f,    77.401668f,    17.359167f}, // From 30 cells, with group error   749.904186
{   97.373458f,    47.317130f,   166.510804f,    43.828126f,    49.300001f,    23.453125f}, // From 24 cells, with group error   794.117450
{  126.834969f,    95.826798f,   102.358933f,    31.689707f,    63.764707f,    15.001471f}, // From 34 cells, with group error   853.055910
{  117.225695f,    87.540510f,    95.445602f,    48.170314f,    87.092189f,    17.769532f}, // From 32 cells, with group error   973.214844
{  126.538411f,    91.341564f,    84.704390f,    55.290742f,   101.681483f,    15.143519f}, // From 27 cells, with group error  1186.028322
{   95.409466f,    43.277778f,   160.038410f,    73.960188f,    49.260186f,    22.445371f}, // From 27 cells, with group error  1117.497659
{  121.730508f,    84.177388f,   103.249026f,    36.816448f,    64.209870f,    15.798685f}, // From 38 cells, with group error  1061.620659
{  124.120835f,    79.163889f,   100.966667f,    44.466876f,    78.205002f,    14.726875f}, // From 40 cells, with group error  1362.414992
{  104.532716f,    47.351852f,   160.146298f,    53.623334f,    50.782501f,    22.285834f}, // From 30 cells, with group error  1444.386353
};
#endif
#ifdef USING_COLOUR_8
const int nGroupsForColour8 = 32;
const float groupsForColour8[32][6] = {
{  193.154322f,   118.135803f,    76.604938f,    60.558335f,    74.804167f,    20.875000f}, // From 6 cells, with group error   304.860790
{  186.278868f,   129.233118f,   102.479304f,    22.533824f,    70.888236f,    24.376471f}, // From 17 cells, with group error   360.532981
{  185.058480f,   128.883043f,   119.473685f,    19.586842f,    49.086843f,    20.846053f}, // From 19 cells, with group error   383.414516
{  181.483247f,   142.256613f,   152.070549f,    15.136905f,    38.736905f,    23.035715f}, // From 21 cells, with group error   393.846170
{  187.375514f,   145.036011f,   129.855968f,    16.909722f,    61.254168f,    26.981945f}, // From 18 cells, with group error   416.786016
{  183.475001f,   130.016668f,   124.493519f,    21.920000f,    55.611251f,    25.998750f}, // From 20 cells, with group error   356.371278
{  183.502745f,   145.016461f,   154.446503f,    14.551852f,    50.612038f,    28.030556f}, // From 27 cells, with group error   352.243764
{  192.589743f,   130.304844f,    84.717949f,    27.998078f,    69.030771f,    22.859616f}, // From 13 cells, with group error   410.115874
{  182.466931f,   126.563493f,   101.507937f,    23.646429f,    62.491073f,    22.928572f}, // From 14 cells, with group error   352.206086
{  181.824395f,   139.621967f,   152.100256f,    15.023276f,    46.556035f,    23.940517f}, // From 29 cells, with group error   708.760931
{  188.190478f,   144.783953f,   127.109349f,    15.302381f,    45.779763f,    21.698810f}, // From 21 cells, with group error   483.424104
{  182.665185f,   141.037779f,   151.214815f,    15.673000f,    53.475001f,    30.555000f}, // From 25 cells, with group error   434.068975
{  185.161867f,   145.712621f,   150.778465f,    14.055556f,    39.975927f,    24.870371f}, // From 27 cells, with group error   441.152099
{  187.586060f,   134.824621f,   107.814815f,    21.214706f,    80.766178f,    27.502941f}, // From 17 cells, with group error   546.138741
{  184.939337f,   134.947638f,   123.972542f,    17.613794f,    48.428449f,    22.317242f}, // From 29 cells, with group error   453.476664
{  182.924012f,   139.881864f,   154.804599f,    15.418966f,    40.087932f,    24.150000f}, // From 29 cells, with group error   604.135925
{  182.367903f,   136.540740f,   142.624693f,    16.995000f,    41.601668f,    22.618334f}, // From 30 cells, with group error   556.797765
{  182.407409f,   127.976608f,   119.935673f,    21.811843f,    73.250000f,    30.439474f}, // From 19 cells, with group error   580.715544
{  192.229347f,   128.568377f,    83.366098f,    34.532693f,    89.711539f,    23.867308f}, // From 13 cells, with group error   547.878630
{  182.547193f,   138.494029f,   134.559141f,    15.891936f,    47.988710f,    22.661291f}, // From 31 cells, with group error   542.110090
{  180.804399f,   129.424769f,   143.444445f,    18.054688f,    54.158595f,    26.913282f}, // From 32 cells, with group error   713.529163
{  182.940740f,   127.877038f,   110.605926f,    21.154000f,    51.994001f,    22.341001f}, // From 25 cells, with group error   585.860618
{  184.425927f,   134.957012f,   123.724208f,    18.611608f,    60.481251f,    26.466965f}, // From 28 cells, with group error   609.741913
{  192.112436f,   143.695769f,   107.328705f,    19.100000f,    71.136609f,    27.690179f}, // From 28 cells, with group error   565.603189
{  184.891784f,   138.281830f,   138.664353f,    16.131250f,    53.450782f,    26.939063f}, // From 32 cells, with group error   586.569269
{  181.396993f,   135.322339f,   149.103589f,    15.917188f,    51.957032f,    26.796094f}, // From 32 cells, with group error   819.965076
{  183.846298f,   131.668520f,   123.008026f,    21.896667f,    50.690834f,    23.837500f}, // From 30 cells, with group error   910.884490
{  182.641205f,   140.782408f,   145.830441f,    16.439063f,    50.250782f,    27.094532f}, // From 32 cells, with group error   806.761066
{  194.562582f,   137.032568f,    95.931673f,    22.912931f,    60.851725f,    22.050000f}, // From 29 cells, with group error   661.281154
{  190.131748f,   141.986244f,   114.931747f,    19.586429f,    61.451429f,    24.039286f}, // From 35 cells, with group error   687.790222
{  180.225030f,   132.964647f,   116.204827f,    17.331818f,    41.809849f,    18.387879f}, // From 33 cells, with group error   836.654839
{  183.261190f,   142.572146f,   152.176698f,    13.957813f,    46.621876f,    24.157292f}, // From 48 cells, with group error   828.629202
};
#endif
#ifdef USING_COLOUR_9
const int nGroupsForColour9 = 32;
const float groupsForColour9[32][6] = {
{  163.106483f,   214.078701f,   124.370371f,     8.581250f,    23.643750f,    21.987501f}, // From 4 cells, with group error    40.843500
{  164.972225f,   189.314816f,    64.476851f,     7.950000f,    13.950000f,    10.406250f}, // From 4 cells, with group error    36.724999
{  162.750002f,   214.367287f,   116.756176f,     7.895834f,    18.870834f,    19.116667f}, // From 6 cells, with group error    47.336726
{  164.247687f,   214.520834f,   130.375004f,     7.475000f,    24.534375f,    16.921875f}, // From 8 cells, with group error    48.454667
{  163.841267f,   210.772487f,   124.224869f,     8.207143f,    31.785715f,    17.817857f}, // From 7 cells, with group error    61.194729
{  162.537039f,   212.988428f,   119.983798f,     8.675000f,    27.568751f,    19.615626f}, // From 8 cells, with group error    48.405325
{  163.858025f,   199.916669f,   104.240743f,     8.679167f,    37.275001f,    14.912500f}, // From 6 cells, with group error    46.663887
{  164.770373f,   173.555554f,    53.351852f,     9.410000f,    23.670001f,    11.375001f}, // From 5 cells, with group error    56.415911
{  163.768519f,   210.033953f,   112.771606f,     9.537500f,    28.662500f,    22.075001f}, // From 6 cells, with group error    57.525034
{  164.020372f,   205.837038f,    85.457407f,     6.722500f,     8.197500f,    12.420000f}, // From 10 cells, with group error    53.704449
{  163.432873f,   216.761576f,   128.953704f,     6.984375f,    19.315625f,    22.218750f}, // From 8 cells, with group error    60.387023
{  163.563494f,   217.304234f,   135.883597f,     7.050000f,    24.860715f,    17.871429f}, // From 7 cells, with group error    56.582285
{  163.546295f,   213.564817f,   122.053242f,     7.640625f,    22.328125f,    19.043750f}, // From 8 cells, with group error    51.741221
{  164.547328f,   202.549383f,    82.028806f,     5.811111f,    12.794445f,    11.988889f}, // From 9 cells, with group error    55.271418
{  163.777780f,   209.423283f,   118.436509f,     6.807143f,    27.117857f,    19.917857f}, // From 7 cells, with group error    53.740743
{  163.988890f,   220.377779f,   140.655556f,     6.547500f,    16.957500f,    18.755000f}, // From 10 cells, with group error    79.254969
{  164.974075f,   189.092596f,    98.177778f,     9.880000f,    44.300002f,    16.520000f}, // From 5 cells, with group error    57.995575
{  163.820369f,   213.529628f,   127.674075f,     8.330000f,    28.597500f,    19.122500f}, // From 10 cells, with group error    63.335207
{  164.893520f,   184.583336f,    60.884260f,     9.925000f,    18.403125f,    12.250000f}, // From 8 cells, with group error    81.518332
{  162.616403f,   214.304234f,   116.835981f,     7.728572f,    18.535715f,    18.339286f}, // From 7 cells, with group error    77.871427
{  164.263889f,   205.038582f,    85.572531f,     6.650000f,    12.468750f,    13.387500f}, // From 12 cells, with group error    74.486143
{  163.277777f,   220.108264f,   135.820512f,     6.138462f,    15.901923f,    17.838462f}, // From 13 cells, with group error    85.930924
{  164.421814f,   201.020577f,   107.061730f,     9.413889f,    32.191667f,    17.925000f}, // From 9 cells, with group error    86.799813
{  164.364199f,   197.934157f,    75.950617f,     7.783333f,    16.719445f,    13.552778f}, // From 9 cells, with group error    97.380267
{  163.846562f,   207.317461f,   116.394180f,     8.260714f,    36.417858f,    18.100000f}, // From 7 cells, with group error    73.670098
{  163.453705f,   208.592593f,   108.794445f,     8.892500f,    23.267501f,    18.005000f}, // From 10 cells, with group error    84.661123
{  164.523812f,   195.042330f,   102.949737f,    11.125000f,    33.050001f,    15.003572f}, // From 7 cells, with group error    83.662480
{  163.783069f,   216.399472f,   131.288363f,     7.394643f,    23.700000f,    15.187500f}, // From 14 cells, with group error   111.295529
{  163.604939f,   210.779323f,   118.422840f,     7.722917f,    24.808334f,    16.445834f}, // From 12 cells, with group error    98.975335
{  164.821760f,   177.833334f,    55.902779f,    11.443750f,    20.450000f,    12.775000f}, // From 8 cells, with group error   135.080103
{  164.015152f,   219.606065f,   136.148149f,     6.425000f,    20.486364f,    24.109091f}, // From 11 cells, with group error   105.967710
{  163.743827f,   210.274692f,   121.151235f,     8.454167f,    33.620835f,    16.620833f}, // From 12 cells, with group error   120.543831
};
#endif
#ifdef USING_COLOUR_11
const int nGroupsForColour11 = 32;
const float groupsForColour11[32][6] = {
{  143.460651f,   234.534724f,   205.715279f,     4.064583f,     1.476042f,    15.583334f}, // From 24 cells, with group error   520.526845
{  127.123264f,   228.761576f,   158.750578f,     9.463281f,     2.796875f,    16.935157f}, // From 32 cells, with group error   310.737253
{  132.041042f,   231.578080f,   178.221222f,     8.912838f,     1.868243f,    14.890541f}, // From 37 cells, with group error   275.666325
{  142.000501f,   226.203204f,   144.474475f,     4.472297f,     2.532432f,    13.021622f}, // From 37 cells, with group error   378.224722
{  142.275405f,   228.903611f,   159.660495f,     4.391026f,     2.237179f,    13.530128f}, // From 39 cells, with group error   365.515243
{  125.466565f,   230.972739f,   173.725824f,     8.411806f,     2.537500f,    18.775695f}, // From 36 cells, with group error   294.076898
{  130.030672f,   221.962386f,   125.609955f,    11.558594f,     3.325781f,    12.947657f}, // From 32 cells, with group error   279.777376
{  142.761189f,   226.511191f,   146.006174f,     4.252604f,     2.400521f,    12.381250f}, // From 48 cells, with group error   467.197102
{  141.221772f,   220.186992f,   119.599368f,     4.978659f,     3.306098f,    11.710366f}, // From 41 cells, with group error   565.925127
{  142.960187f,   232.888890f,   189.962965f,     4.096875f,     1.716250f,    14.398125f}, // From 40 cells, with group error   727.301544
{  137.653116f,   230.330173f,   168.966577f,     6.534146f,     2.195122f,    15.323171f}, // From 41 cells, with group error   507.414543
{  128.950451f,   226.146650f,   144.139139f,     9.514865f,     2.778378f,    14.439190f}, // From 37 cells, with group error   386.864245
{  129.770834f,   228.140048f,   155.282022f,     9.805208f,     1.852604f,    11.170313f}, // From 48 cells, with group error   589.085317
{  135.372874f,   224.153155f,   134.644145f,     8.638514f,     3.357432f,    14.677027f}, // From 37 cells, with group error   623.857456
{  129.843365f,   227.922071f,   153.652778f,     9.459375f,     2.248958f,    12.597396f}, // From 48 cells, with group error   321.296480
{  142.372223f,   220.488520f,   120.135927f,     4.699500f,     3.193500f,    10.993000f}, // From 50 cells, with group error   397.319012
{  131.060015f,   229.645749f,   164.310701f,     9.911574f,     2.492130f,    16.256019f}, // From 54 cells, with group error   319.207819
{  129.924331f,   228.751279f,   158.558430f,    10.194828f,     2.162500f,    13.737500f}, // From 58 cells, with group error   332.607289
{  129.512546f,   233.646357f,   195.906214f,     6.551613f,     1.677419f,    15.157258f}, // From 31 cells, with group error   438.500850
{  124.454883f,   231.019194f,   174.305388f,     7.489091f,     2.340909f,    17.565455f}, // From 55 cells, with group error   669.434708
{  129.016549f,   226.795904f,   147.563044f,    10.628723f,     2.686702f,    14.888298f}, // From 47 cells, with group error   473.408346
{  135.621448f,   221.016797f,   122.133076f,     7.819186f,     3.389535f,    12.357558f}, // From 43 cells, with group error   667.257386
{  138.314430f,   231.122300f,   174.567904f,     6.505729f,     1.779167f,    14.008855f}, // From 48 cells, with group error   687.867920
{  129.247408f,   224.282593f,   135.257039f,    11.914500f,     3.324000f,    14.859500f}, // From 50 cells, with group error   585.650984
{  142.176662f,   225.649325f,   141.535862f,     4.341667f,     2.420635f,    12.382937f}, // From 63 cells, with group error   426.801637
{  128.283429f,   227.906781f,   153.687383f,     9.273305f,     2.593220f,    14.782204f}, // From 59 cells, with group error   530.816924
{  143.049709f,   233.619559f,   196.259585f,     3.969737f,     1.571491f,    14.953948f}, // From 57 cells, with group error   900.257508
{  142.651793f,   224.927142f,   138.611719f,     4.287705f,     2.702459f,    12.678279f}, // From 61 cells, with group error   857.105061
{  126.119530f,   231.556904f,   178.164648f,     7.843182f,     2.103182f,    17.014546f}, // From 55 cells, with group error   652.449618
{  128.490255f,   226.142952f,   144.094869f,    10.655263f,     2.646053f,    13.850439f}, // From 57 cells, with group error   555.721857
{  130.375662f,   229.964408f,   166.496153f,     8.706494f,     1.822727f,    12.411364f}, // From 77 cells, with group error   575.351660
{  135.620916f,   228.376363f,   156.438728f,     7.459559f,     2.487500f,    15.138971f}, // From 68 cells, with group error  1265.223741
};
#endif
#ifdef USING_COLOUR_12
const int nGroupsForColour12 = 32;
const float groupsForColour12[32][6] = {
{  168.975308f,   218.775308f,   119.411111f,     3.478333f,     7.725000f,    10.008334f}, // From 15 cells, with group error   110.221057
{  168.012823f,   215.185188f,   106.831910f,     3.565385f,     6.444231f,     9.955769f}, // From 13 cells, with group error   111.565982
{  169.262347f,   217.180045f,   114.920783f,     3.802778f,     8.865278f,    11.741667f}, // From 18 cells, with group error   112.584873
{  168.579633f,   225.062038f,   144.250925f,     3.312500f,     6.531250f,    14.716250f}, // From 20 cells, with group error   115.569218
{  167.511787f,   206.323234f,    88.181819f,     4.195455f,    10.425000f,    14.297727f}, // From 11 cells, with group error   125.105254
{  164.956229f,   218.395624f,   134.927612f,     4.297727f,    18.700000f,    12.970455f}, // From 11 cells, with group error   119.281177
{  167.989107f,   231.708066f,   188.008716f,     2.894118f,     5.685294f,    12.775000f}, // From 17 cells, with group error   111.991494
{  168.425002f,   226.223151f,   148.237963f,     3.782500f,     4.527500f,    13.993750f}, // From 20 cells, with group error   104.257277
{  167.106756f,   215.494555f,   108.606754f,     3.985294f,     6.930882f,    11.373530f}, // From 17 cells, with group error   137.810994
{  167.486930f,   227.389979f,   160.488018f,     3.404412f,     7.236765f,    15.688236f}, // From 17 cells, with group error   127.601911
{  165.362140f,   210.909464f,   110.253086f,     5.311111f,    20.444445f,    12.400000f}, // From 9 cells, with group error   151.052664
{  169.026620f,   217.825234f,   116.799769f,     3.660938f,     8.470313f,    15.143750f}, // From 16 cells, with group error   159.434466
{  168.136834f,   213.468110f,   102.355968f,     4.344444f,     6.815278f,    10.218056f}, // From 18 cells, with group error   142.227160
{  168.370371f,   229.079923f,   167.525343f,     3.390789f,     4.964474f,    13.046053f}, // From 19 cells, with group error   209.647400
{  163.814816f,   206.460907f,    90.870370f,     5.391667f,    13.666667f,    15.313889f}, // From 9 cells, with group error   117.239088
{  167.151368f,   217.474236f,   114.164252f,     4.038043f,     6.885870f,    12.529348f}, // From 23 cells, with group error   175.492098
{  167.947916f,   223.466437f,   140.557871f,     3.979687f,     8.893750f,    12.898438f}, // From 16 cells, with group error   137.445802
{  167.707233f,   227.375663f,   157.069665f,     3.346429f,     5.569048f,    14.465476f}, // From 21 cells, with group error   137.487008
{  169.235044f,   219.644586f,   119.730058f,     3.474038f,     5.172115f,    12.815385f}, // From 26 cells, with group error   140.980947
{  169.187038f,   212.165742f,   100.375002f,     3.955000f,     8.383750f,    13.000000f}, // From 20 cells, with group error   188.145822
{  163.914816f,   211.118518f,   132.814816f,    12.100000f,    30.515000f,    18.750001f}, // From 5 cells, with group error   159.041847
{  168.526572f,   226.088567f,   151.933980f,     3.546739f,     7.433696f,    13.609783f}, // From 23 cells, with group error   153.991496
{  167.963770f,   231.179549f,   183.076490f,     3.030435f,     5.183696f,    14.582609f}, // From 23 cells, with group error   154.175437
{  168.071112f,   227.848891f,   158.571855f,     3.276000f,     5.050000f,    17.180000f}, // From 25 cells, with group error   293.439159
{  168.834657f,   223.072092f,   134.546298f,     3.675893f,     6.162500f,    14.511607f}, // From 28 cells, with group error   197.189827
{  164.574076f,   213.538274f,   108.693828f,     5.461667f,    13.203334f,    12.843334f}, // From 15 cells, with group error   184.325820
{  167.141975f,   216.213583f,   110.863581f,     4.133333f,     7.861667f,    11.986667f}, // From 30 cells, with group error   269.453945
{  167.781895f,   218.226340f,   125.729425f,     3.786111f,    13.177778f,    14.048611f}, // From 18 cells, with group error   197.109418
{  168.432871f,   223.415124f,   142.351852f,     3.581250f,    10.015625f,    12.100000f}, // From 24 cells, with group error   270.885983
{  167.180976f,   226.952020f,   165.031145f,     3.319318f,    10.245455f,    14.043182f}, // From 22 cells, with group error   372.632876
{  169.162178f,   221.657130f,   128.381596f,     3.640909f,     6.243182f,    12.450758f}, // From 33 cells, with group error   263.993825
{  164.769362f,   195.915827f,    70.872055f,     7.011364f,     9.653409f,    11.153409f}, // From 22 cells, with group error   223.117215
};
#endif
#ifdef USING_COLOUR_13
const int nGroupsForColour13 = 32;
const float groupsForColour13[32][6] = {
{  163.253089f,   229.641206f,   164.187502f,     2.779167f,     1.972917f,    12.409375f}, // From 24 cells, with group error   138.201798
{  163.250547f,   222.971681f,   129.551198f,     3.477941f,     3.275000f,    13.452941f}, // From 17 cells, with group error   124.781565
{  161.882224f,   235.300002f,   212.088149f,     2.402000f,     1.019000f,    12.425000f}, // From 25 cells, with group error   162.474418
{  162.658333f,   227.362038f,   150.714814f,     3.302500f,     2.416250f,    13.797500f}, // From 20 cells, with group error   136.800039
{  162.875663f,   233.056087f,   190.465081f,     2.644286f,     1.415714f,    13.093572f}, // From 35 cells, with group error   194.859642
{  162.336421f,   233.319445f,   192.908953f,     2.642708f,     1.618750f,    15.080209f}, // From 24 cells, with group error   146.302531
{  162.611114f,   232.387206f,   185.233166f,     2.569318f,     1.893182f,    16.138636f}, // From 22 cells, with group error   151.485515
{  159.340101f,   237.894232f,   239.555201f,     2.789904f,     0.330769f,     1.478365f}, // From 52 cells, with group error   269.045103
{  162.682872f,   230.185187f,   167.885995f,     2.631250f,     2.042187f,    13.321094f}, // From 32 cells, with group error   137.332429
{  162.124579f,   235.128509f,   210.365883f,     2.529545f,     1.126515f,    13.462879f}, // From 33 cells, with group error   155.051147
{  162.697339f,   229.377896f,   162.612847f,     2.839062f,     1.972656f,    13.157813f}, // From 32 cells, with group error   148.080627
{  163.266204f,   227.122686f,   149.238427f,     3.085417f,     2.171875f,    11.907292f}, // From 24 cells, with group error   178.876360
{  161.392593f,   236.400742f,   225.083706f,     2.643000f,     1.254000f,    15.351000f}, // From 25 cells, with group error   191.856211
{  163.028457f,   233.889793f,   198.156280f,     2.556707f,     1.409756f,    13.592683f}, // From 41 cells, with group error   131.007419
{  162.566874f,   232.001031f,   181.490741f,     2.390278f,     1.744444f,    13.850000f}, // From 36 cells, with group error   164.758827
{  163.089670f,   231.622321f,   178.573588f,     2.993421f,     1.661842f,    13.672369f}, // From 38 cells, with group error   163.734323
{  162.349850f,   232.693194f,   187.397899f,     2.797297f,     1.600676f,    13.633108f}, // From 37 cells, with group error   290.887560
{  162.380001f,   234.977780f,   209.174817f,     2.651000f,     1.734000f,    18.574000f}, // From 25 cells, with group error   271.729774
{  162.539887f,   231.048910f,   174.133903f,     2.725641f,     1.955128f,    15.060257f}, // From 39 cells, with group error   228.325747
{  159.204060f,   237.866453f,   239.228633f,     2.778365f,     0.258173f,     1.992788f}, // From 52 cells, with group error   366.885072
{  163.014722f,   233.787277f,   197.142927f,     2.340385f,     1.700000f,    14.376923f}, // From 39 cells, with group error   272.966245
{  162.798355f,   228.962963f,   160.100310f,     2.854167f,     2.339583f,    14.472223f}, // From 36 cells, with group error   367.942769
{  163.590068f,   223.634682f,   133.763468f,     3.498864f,     3.092045f,    11.878409f}, // From 22 cells, with group error   310.509162
{  162.799459f,   234.396568f,   202.759714f,     2.697561f,     1.207317f,    11.646342f}, // From 41 cells, with group error   243.302850
{  162.469269f,   231.546495f,   178.037826f,     2.736702f,     1.419681f,    11.925532f}, // From 47 cells, with group error   295.719563
{  162.180466f,   234.368920f,   202.644881f,     2.672059f,     1.624510f,    14.480392f}, // From 51 cells, with group error   245.206321
{  163.030985f,   232.989319f,   189.890671f,     2.530288f,     1.449038f,    13.025000f}, // From 52 cells, with group error   224.347395
{  162.422635f,   229.126338f,   160.996298f,     2.985000f,     2.093333f,    13.377778f}, // From 45 cells, with group error   263.069838
{  162.607619f,   230.915795f,   173.074075f,     2.703302f,     1.796698f,    13.333962f}, // From 53 cells, with group error   286.446692
{  162.218856f,   234.425927f,   203.625844f,     2.564773f,     1.515341f,    15.375000f}, // From 44 cells, with group error   430.135666
{  162.522104f,   233.092594f,   190.986262f,     2.665726f,     1.350806f,    11.858065f}, // From 62 cells, with group error   455.050885
{  161.780216f,   236.581874f,   227.335772f,     2.670395f,     1.219737f,    13.913816f}, // From 38 cells, with group error   451.011351
};
#endif
#ifdef USING_COLOUR_14
const int nGroupsForColour14 = 32;
const float groupsForColour14[32][6] = {
{   90.531032f,   190.037039f,    87.037538f,    10.979730f,    28.817568f,    12.252027f}, // From 37 cells, with group error  1217.027669
{   85.803031f,   196.254547f,   148.489226f,     5.165000f,    24.417273f,    12.747727f}, // From 55 cells, with group error   899.045687
{   86.226943f,   196.927742f,   152.189180f,     6.091177f,    23.898040f,    13.137255f}, // From 51 cells, with group error   859.313348
{   85.406800f,   188.754403f,   103.360050f,     5.721721f,    25.436886f,     9.539754f}, // From 61 cells, with group error   842.690629
{   85.486181f,   193.797680f,   130.697070f,     5.415672f,    23.569030f,    13.083955f}, // From 67 cells, with group error   980.410699
{   85.259741f,   188.281148f,    99.766475f,     5.732468f,    25.372728f,    10.598701f}, // From 77 cells, with group error   930.331589
{   85.785864f,   198.170581f,   153.214398f,     5.172535f,    21.350000f,    13.307747f}, // From 71 cells, with group error   851.260432
{   85.638498f,   191.978614f,   106.667972f,     6.069718f,    25.063029f,    11.823240f}, // From 71 cells, with group error  1028.976107
{   84.997195f,   186.191920f,    89.178451f,     6.269697f,    32.722349f,     9.272349f}, // From 66 cells, with group error   866.466046
{   85.530971f,   189.500959f,   117.204024f,     6.451724f,    29.480173f,    11.017673f}, // From 58 cells, with group error  1048.568603
{   84.774966f,   190.444447f,    98.494141f,     5.250316f,    26.554747f,     9.675633f}, // From 79 cells, with group error   875.767103
{   87.139815f,   191.070681f,   105.675310f,     6.595417f,    28.920834f,    10.900417f}, // From 60 cells, with group error   801.686350
{   84.927855f,   186.859569f,    90.998843f,     6.217969f,    30.518490f,    10.519792f}, // From 96 cells, with group error  1240.261193
{   86.638219f,   192.622385f,   116.356416f,     6.066304f,    27.097826f,    11.285870f}, // From 69 cells, with group error  1005.889466
{   85.413123f,   196.130088f,   140.737998f,     5.405247f,    24.473766f,    12.237037f}, // From 81 cells, with group error  1024.303381
{   85.464540f,   195.477937f,   141.371356f,     5.564362f,    19.536703f,    13.627128f}, // From 94 cells, with group error   959.441595
{   88.237655f,   187.813581f,    70.873457f,     7.597500f,    25.044584f,     9.458334f}, // From 60 cells, with group error   859.172900
{   85.030179f,   188.827391f,   104.026521f,     5.462963f,    21.576852f,    11.123457f}, // From 81 cells, with group error  1243.654344
{   86.765873f,   193.100531f,   144.428043f,     6.216786f,    26.612500f,    12.967143f}, // From 70 cells, with group error  2554.170388
{   87.026354f,   188.363487f,    80.352090f,     6.632372f,    24.932372f,     9.704808f}, // From 78 cells, with group error  1273.235622
{   86.583334f,   196.991831f,   160.580883f,     5.644118f,    23.012500f,    14.601103f}, // From 68 cells, with group error  1084.991382
{   85.511575f,   190.040616f,   117.809133f,     5.818182f,    24.970455f,    11.292330f}, // From 88 cells, with group error  2017.910371
{   86.456058f,   190.083806f,    87.498431f,     6.052542f,    20.594916f,    10.301695f}, // From 59 cells, with group error  1361.900014
{   86.335132f,   195.052321f,   153.295758f,     6.034223f,    24.765777f,    13.951214f}, // From 103 cells, with group error  2765.373836
{   88.205161f,   192.999793f,    85.831461f,     6.929775f,    31.537360f,     9.895787f}, // From 89 cells, with group error  1233.254805
{   85.716221f,   185.785838f,    81.432614f,     6.572917f,    29.647917f,     9.811343f}, // From 108 cells, with group error  1681.866545
{   86.517747f,   195.237078f,   148.820797f,     5.837760f,    24.039584f,    14.145834f}, // From 96 cells, with group error  1455.528168
{   84.891050f,   189.558644f,    94.755248f,     5.513125f,    27.420209f,     9.921667f}, // From 120 cells, with group error  1171.181020
{   85.866234f,   192.636921f,   107.916865f,     5.726330f,    24.797873f,    11.041224f}, // From 94 cells, with group error  1438.761664
{   85.539775f,   191.595010f,   122.196620f,     5.534130f,    22.965000f,    11.827174f}, // From 115 cells, with group error  2256.097033
{   86.160170f,   191.835610f,   118.047597f,     5.941228f,    26.720834f,    11.473246f}, // From 114 cells, with group error  1553.857313
{   85.710728f,   191.057686f,   107.478716f,     6.248851f,    28.246840f,    11.451437f}, // From 87 cells, with group error  3063.554708
};
#endif

















float RobotRecognitionModel_calcSingleColourScore (
	const int colourIndex,
	ImgCellFeatures *cell)
{
	int colourId, groupIndex;
	int nGroups;
	const float (*groups)[6];
	float diff;
	float bestMatch;

	colourId = RobotRecognitionModel_mapColourDescriptionArrayToColourId[colourIndex];

	// Get set of groups for this colour
	switch (colourId)
	{
#ifdef USING_COLOUR_2
	case 2:
		nGroups = nGroupsForColour2;
		groups = groupsForColour2;
		break;
#endif
#ifdef USING_COLOUR_3
	case 3:
		nGroups = nGroupsForColour3;
		groups = groupsForColour3;
		break;
#endif
#ifdef USING_COLOUR_5
	case 5:
		nGroups = nGroupsForColour5;
		groups = groupsForColour5;
		break;
#endif
#ifdef USING_COLOUR_6
	case 6:
		nGroups = nGroupsForColour6;
		groups = groupsForColour6;
		break;
#endif
#ifdef USING_COLOUR_9
	case 9:
		nGroups = nGroupsForColour9;
		groups = groupsForColour9;
		break;
#endif
#ifdef USING_COLOUR_11
	case 11:
		nGroups = nGroupsForColour11;
		groups = groupsForColour11;
		break;
#endif
#ifdef USING_COLOUR_12
	case 12:
		nGroups = nGroupsForColour12;
		groups = groupsForColour12;
		break;
#endif
#ifdef USING_COLOUR_13
	case 13:
		nGroups = nGroupsForColour13;
		groups = groupsForColour13;
		break;
#endif
#ifdef USING_COLOUR_14
	case 14:
		nGroups = nGroupsForColour14;
		groups = groupsForColour14;
		break;
#endif
	default:
		nGroups = 0;
		groups = 0;
	}

	bestMatch = MAX_FLT;
	for (groupIndex = 0; groupIndex < nGroups; ++groupIndex)
	{
		diff = RobotRecognitionCore_calcColourDiff (
			cell->vals,
			groups[groupIndex]);
		diff /= RobotRecognitionModel_colourAvgs[colourIndex];

		bestMatch = min(diff, bestMatch);
	}

	return bestMatch;
}

float RobotRecognitionModel_calcColourScores (
	ImgCellFeatures *cellFeatures,
	float *colourScoreGrid)
{
	int cellIndex, colourIndex;
	float bestMatch;
	float bestOverallMatch;

	bestOverallMatch = MAX_FLT;
	for (cellIndex = 0; cellIndex < SIZE_COOP_LOC_GRID; ++cellIndex)
	{
		for (colourIndex = 0; colourIndex < N_ROBOT_COLOURS; ++colourIndex)
		{
			bestMatch = RobotRecognitionModel_calcSingleColourScore (colourIndex, &cellFeatures[cellIndex]);

			colourScoreGrid[colourIndex + cellIndex * N_ROBOT_COLOURS] = bestMatch;
			bestOverallMatch = min (bestMatch, bestOverallMatch);
		}
	}

	return bestOverallMatch;
}

#endif


