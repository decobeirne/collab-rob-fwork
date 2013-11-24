/*!
\file RobotDefs.h
\brief Experiment setup and robot parameters.
*/
#pragma once

#ifndef GLOB_DEFS_H
#define GLOB_DEFS_H

//
// Platform
//
#ifdef _WIN32
#define IS_WIN

	// Defines how screens are arranged
	#define ON_LAPTOP

	#define SIMULATION
//	#define ROBOT
//	#define BOARD

#ifdef ROBOT
// If running a robot as a separate process on windows, the index must be
// explicitly assigned
	#define ROBOT_INDEX 0
//	#define ROBOT_INDEX 1
//	#define ROBOT_INDEX 2

// Explicitly define robot platform - when running an experiment where the robot
// ids do not match the assigned platforms
	#define ROBOT_PLATFORM 2



#else // ifdef ROBOT

// When running a simulation or blackboard must still specify a robot platform. Various
// camera and motor parameters require this to be specified when being setup.
	#define ROBOT_PLATFORM 2
//	#define ROBOT_PLATFORM 3
//	#define ROBOT_PLATFORM 4
#endif // else ifdef ROBOT

#else // _WIN32

#define IS_LINUX

// If on Linux, the experiment type must be specified in the makefile.
// #define ROBOT
// #define BOARD
// #define IS_GUMSTIX
// #define IS_MOTOR
// #define ROBOT_INDEX
// #define ROBOT_PLATFORM
#endif // else _WIN32










//
// Robot
//

// Robot index and physical robot must be uniquely specified in the makefile when running experiments
#if defined(ROBOT) && !defined(ROBOT_INDEX)
#error "You must specify the robot id! This identifies the robot when communicating with the blackboard agent."
#endif

#if defined(ROBOT)
//	#define RECORDING_ROBOT // All data read from board is serialized out
//	#define RERUNNING_ROBOT // Read serialized data. Experiment directory must be specified.

#if defined(RECORDING_ROBOT) && defined(RERUNNING_ROBOT)
#error "You can't record and rerun at the same time"
#endif

// When rerunning, we will hit an assert if we don't set this flag correctly
//	#define RERUNNING_GUMSTIX

#elif defined(SIMULATION)// if defined(ROBOT)

//	#define RERUNNING_IMAGES_ONLY // Read serialized images only - when the code has been updated and other data will not be relevant
#endif // if defined(ROBOT)

#define ROBOT_INDEX_0_PLATFORM 2
#define ROBOT_INDEX_1_PLATFORM 3
#define ROBOT_INDEX_2_PLATFORM 4

#if defined(SIMULATION)
// Defined in RobotTypes.c
extern int robotPlatformIds[3];
#endif

#if defined(ROBOT) && !defined(ROBOT_PLATFORM)
// We haven't overridden the robot platform, so we assume it corresponds
// to that specified for the ROBOT_INDEX
#define GET_ROBOT_PLATFORM_POST_SCAN(x) ROBOT_INDEX_##x##_PLATFORM
#define GET_ROBOT_PLATFORM(x) GET_ROBOT_PLATFORM_POST_SCAN(x)
#define ROBOT_PLATFORM GET_ROBOT_PLATFORM(ROBOT_INDEX)
#endif // defined(ROBOT) && !defined(ROBOT_PLATFORM)

#if (defined(ROBOT) || defined(SIMULATION)) && !defined(ROBOT_PLATFORM)
#error "You must speficy which physical robot the code is running on! (The value should be identified on the robot itself.)"
#endif












//
// Experiment type
//
	#define AUTONOMOUS				//!< Comment out to test image capturing and map building without automomous behaviour.
//	#define IGNORE_CAM_DATA			//!< For testing motors, just throw out image and assume no obstacles
//	#define IGNORE_COMPASS_DATA		//!< May be necessary to throw out compass readings if robot is being kept stationary.
	#define SIM_ERROR				//!< Generate random error when running simulated experiments.
//	#define RAND_ENVIR				//!< Flags that a random environment should be created
//	#define RAND_ROBOT_LOCS			//!< Flags that initial robot locations should be randomly generated
///	#define COLLAB_EXP				//!< Independent / collaborative exploration
	#define SUP_PROFIT_SPLIT		//!< In a coalition, split all profit evenly between explorer and supervisor (after cost for each is taken out)
//	#define STRICT_EXP_BEH			//!< Only allow GTEP after all EXP targets have been exhausted
	#define JUST_LOOK_AT_GAIN_FROM_FINAL_TARGET //!< Use coarser approach for calcing exploration map gain (quicker and encourages further targets)
	#define USE_CLOUD				//!< Farm profit calculation out to cloud
	#define EXPLICIT_DEBUG			//!< Exclude asserts when debug and release code are talking to each other
//	#define NO_SIMULATED_ENVIRONMENT
//	#define SETUP_TEST_COALITION


#ifdef EXPLICIT_DEBUG
#define EXPLICIT_DEBUG_ASSERT(cond) assert(cond);
#else
#define EXPLICIT_DEBUG_ASSERT(cond)
#endif

#ifndef SUP_PROFIT_SPLIT
//! In a coalition, allocate profit to the supervisor based on the actual improvement in certainty that it facilitates.
#define SUP_PROFIT_BY_STD_DEV
#endif

#if defined(IS_GUMSTIX) || defined(RERUNNING_GUMSTIX) || (defined(SIMULATION) && defined(RERUNNING_IMAGES_ONLY))
#define MAP_SCANS_HAVE_GRIDS
#endif

#if (defined(IS_GUMSTIX) || defined(RERUNNING_GUMSTIX)) && defined(SIM_ERROR)
#undef SIM_ERROR
#endif

#if defined(RERUNNING_IMAGES_ONLY) && defined(IGNORE_CAM_DATA)
#error "It doesn't make any sense to define these two values"
#endif













//
// Warnings
//
#ifdef IS_WIN
#pragma warning( disable : 4305 996 244 )
#endif
























//
// Preprocessor utilities
//
#define PI						3.1415926f
#define RADS_110				1.9198621f
#define RADS_360				(PI * 2.0f)
#define RADS_90					(PI * 0.5f)
#define RADS_135				(PI * 0.75f)
#define RADS_45					(PI * 0.25f)
#define RADS_30					0.523598f
#define RADS_TEN				0.1745329f
#define RADS_TWO				0.0349065f
#define RADS_FIVE_DEGREES		0.087266f
#define RADS_TEN_DEGREES		0.174532f
#define MAX_INT32				16777216 // Could use 2147483647, but it doesn't matter.
#define MIN_INT32				-16777216
#define MAX_FLT					16777216.0f
#define MIN_FLT					-16777216.0f

#define DEGS_TO_RADS(A) ((A * PI) * (1.0f / 180.0f))
#define SQR(A) (A * A)














//
// Per-robot camera parameters in WORLD space
//

#define HMC6352_STD_DEV 1.25f //!< Error from Honeywell reference manual.
#define CMPS03 1.75f //!< Error from other compass reference manual.

#if ROBOT_PLATFORM == 2
	#define FLIP_COMPASS //!< Should compass reading be inverted.
	#define IS_H_BRIDGE //!< Is robot controlled via h-bridge.
	#define IS_HMC6352 //!< Flags if this robot is equipped with a HMC6352 compass module.
	#define SWITCH_MOTORS 2 //!< Should commands to differential motors be flipped.

// Use std dev from calibration: error when taking compass measurement and mapping
// to orient. The compass docs report a measurement granularity or 1 degree as well,
// so add this.
#ifdef IS_HMC6352
//	#define COMPASS_STDEV                           HMC6352_STD_DEV
	#define COMPASS_STDEV							4.131984 //!< Calculated from log files from Gumstix experiments.
#else
#define COMPASS_STDEV                           CMPS03
#endif
#define COMPASS_ORIENT_OFFSET                   0.0f
#endif
#if ROBOT_PLATFORM == 2 || defined(SIMULATION)
#define CAM_P_OFFSET_WORLD_R2                      109.000000000000000f //!< Offset to cameras focal pt (from centre of robot).
#define CAM_HEIGHT_WORLD_R2                        111.000000000000000f //!< Height of camera focal pt.
#define CAM_THETA_R2                               0.499264f //!< Angle between camera normal and horizon.
#define FOCAL_LEN_R2                               253.759170f //!< Focal length of camera.
#define IMAGE_CENTRE_PIXEL_Y_R2                    71 //!< Y value in captured images corresponding to centre.
#define CAM_OCCUPANCY_GRID_ORIGIN_X_R2             2 //!< Coordinate in grabbed images which should be considered the origin.
#define CAM_OCCUPANCY_GRID_ORIGIN_Y_R2             4
#define CAM_OCCUPANCY_GRID_Y_EXP_R2                23 //!< Dimensions of grid used for detecting obstacles.
#define SCAN_CENTRE_DIST_R2						   18.0f //!< Rought est - not important
#endif

#if ROBOT_PLATFORM == 3
	#define FLIP_COMPASS //!< Should compass reading be inverted.
	#define IS_H_BRIDGE //!< Is robot controlled via h-bridge.
	#define IS_HMC6352 //!< Flags if this robot is equipped with a HMC6352 compass module.

#ifdef IS_HMC6352
#define COMPASS_STDEV								HMC6352_STD_DEV
#else
#define COMPASS_STDEV								CMPS03
#endif
#define COMPASS_ORIENT_OFFSET 0.0f
#endif
#if ROBOT_PLATFORM == 3 || defined(SIMULATION)
#define CAM_P_OFFSET_WORLD_R3                      134.0f //!< Offset to cameras focal pt (from centre of robot).
#define CAM_HEIGHT_WORLD_R3                        109.0f //!< Height of camera focal pt.
#define CAM_THETA_R3                               0.49f //!< Angle between camera normal and horizon.
#define FOCAL_LEN_R3                               207.292641f //!< Focal length of camera.
#define IMAGE_CENTRE_PIXEL_Y_R3                    71 //!< Y value in captured images corresponding to centre.
#define CAM_OCCUPANCY_GRID_ORIGIN_X_R3             2 //!< Coordinate in grabbed images which should be considered the origin.
#define CAM_OCCUPANCY_GRID_ORIGIN_Y_R3             4
#define CAM_OCCUPANCY_GRID_Y_EXP_R3                23 //!< Dimensions of grid used for detecting obstacles.
#define SCAN_CENTRE_DIST_R3						   18.0f //!< Established from output from GeometryConstants_detExpDists_verbose
#endif






#if ROBOT_PLATFORM == 4
	#define FLIP_COMPASS //!< Should compass reading be inverted.
//	#define IS_H_BRIDGE //!< Is robot controlled via h-bridge.
//	#define IS_HMC6352 //!< Flags if this robot is equipped with a HMC6352 compass module.

#ifdef IS_HMC6352
#define COMPASS_STDEV					HMC6352_STD_DEV
#else
#define COMPASS_STDEV					CMPS03
#endif
#define COMPASS_ORIENT_OFFSET                   0.0f
#endif
#if ROBOT_PLATFORM == 4 || defined(SIMULATION)
#define CAM_P_OFFSET_WORLD_R4                      116 //!< Offset to cameras focal pt (from centre of robot).
#define CAM_HEIGHT_WORLD_R4                        101.000000000000000f //!< Height of camera focal pt.
#define CAM_THETA_R4                               0.484952f //!< Angle between camera normal and horizon.
#define FOCAL_LEN_R4                               251.211250f //!< Focal length of camera.
#define IMAGE_CENTRE_PIXEL_Y_R4                    71 //!< Y value in captured images corresponding to centre.
#define CAM_OCCUPANCY_GRID_ORIGIN_X_R4             2 //!< Coordinate in grabbed images which should be considered the origin.
#define CAM_OCCUPANCY_GRID_ORIGIN_Y_R4             4
#define CAM_OCCUPANCY_GRID_Y_EXP_R4                23 //!< Dimensions of grid used for detecting obstacles.
#define SCAN_CENTRE_DIST_R4						   18.0f
#endif


















//
// Per-robot camera parameters in WORLD space - *ONLY* do this when on robots,
// as otherwise we want to be able to change what camera params are used in order
// to train coopeative localisation certainty model.
//
// Global values are setup in RobotTypes.c for simulation (performance doesn't matter here)
//
#if defined(ROBOT)

#if ROBOT_PLATFORM == 2
#define CAM_P_OFFSET_WORLD				CAM_P_OFFSET_WORLD_R2
#define CAM_HEIGHT_WORLD				CAM_HEIGHT_WORLD_R2
#define CAM_THETA						CAM_THETA_R2
#define FOCAL_LEN						FOCAL_LEN_R2
#define IMAGE_CENTRE_PIXEL_Y			IMAGE_CENTRE_PIXEL_Y_R2
#define CAM_OCCUPANCY_GRID_ORIGIN_X		CAM_OCCUPANCY_GRID_ORIGIN_X_R2
#define CAM_OCCUPANCY_GRID_ORIGIN_Y		CAM_OCCUPANCY_GRID_ORIGIN_Y_R2
#define CAM_OCCUPANCY_GRID_Y_EXP		CAM_OCCUPANCY_GRID_Y_EXP_R2
#define SCAN_CENTRE_DIST				SCAN_CENTRE_DIST_R2

#elif ROBOT_PLATFORM == 3

#define CAM_P_OFFSET_WORLD				CAM_P_OFFSET_WORLD_R3
#define CAM_HEIGHT_WORLD				CAM_HEIGHT_WORLD_R3
#define CAM_THETA						CAM_THETA_R3
#define FOCAL_LEN						FOCAL_LEN_R3
#define IMAGE_CENTRE_PIXEL_Y			IMAGE_CENTRE_PIXEL_Y_R3
#define CAM_OCCUPANCY_GRID_ORIGIN_X		CAM_OCCUPANCY_GRID_ORIGIN_X_R3
#define CAM_OCCUPANCY_GRID_ORIGIN_Y		CAM_OCCUPANCY_GRID_ORIGIN_Y_R3
#define CAM_OCCUPANCY_GRID_Y_EXP		CAM_OCCUPANCY_GRID_Y_EXP_R3
#define SCAN_CENTRE_DIST				SCAN_CENTRE_DIST_R3

#else

#define CAM_P_OFFSET_WORLD				CAM_P_OFFSET_WORLD_R4
#define CAM_HEIGHT_WORLD				CAM_HEIGHT_WORLD_R4
#define CAM_THETA						CAM_THETA_R4
#define FOCAL_LEN						FOCAL_LEN_R4
#define IMAGE_CENTRE_PIXEL_Y			IMAGE_CENTRE_PIXEL_Y_R4
#define CAM_OCCUPANCY_GRID_ORIGIN_X		CAM_OCCUPANCY_GRID_ORIGIN_X_R4
#define CAM_OCCUPANCY_GRID_ORIGIN_Y		CAM_OCCUPANCY_GRID_ORIGIN_Y_R4
#define CAM_OCCUPANCY_GRID_Y_EXP		CAM_OCCUPANCY_GRID_Y_EXP_R4
#define SCAN_CENTRE_DIST				SCAN_CENTRE_DIST_R4
#endif

//
// Scaled camera parameters
//
#define CAM_P_OFFSET					(CAM_P_OFFSET_WORLD / MAP_SCALE)
#define CAM_HEIGHT						(CAM_HEIGHT_WORLD / MAP_SCALE)

#endif // defined(ROBOT)
#if defined(SIMULATION)

// Defined in RobotTypes.c
// Available on simulation, where we want to do robot exploration or cooperative
// localisation, but not on board
extern float CAM_P_OFFSET_WORLD;
extern float CAM_HEIGHT_WORLD;
extern float CAM_THETA;
extern float FOCAL_LEN;
extern float IMAGE_CENTRE_PIXEL_Y;
extern int CAM_OCCUPANCY_GRID_ORIGIN_X;
extern int CAM_OCCUPANCY_GRID_ORIGIN_Y;
extern int CAM_OCCUPANCY_GRID_Y_EXP;
extern float SCAN_CENTRE_DIST;
extern float CAM_P_OFFSET;
extern float CAM_HEIGHT;

#endif // defined(ROBOT)

















//
// Robot move parameters in WORLD space
//
#if ROBOT_PLATFORM == 2
// Orient variance updated on 2012/06/24. Re-did proper calibration of compass.
#define ORIENT_VAR SQR(DEGS_TO_RADS(COMPASS_STDEV))

// Orients updated on 2012/06/10. Have to check what's up with compasses
//#define ROT_LEFT_DELTA_ORIENT                               0.828660059139694f
//#define ROT_LEFT_VAR_ORIENT                                 0.004111054075077f
//#define ROT_RIGHT_DELTA_ORIENT                              -0.844227778449122f
//#define ROT_RIGHT_VAR_ORIENT                                0.001488083948824f
//#define ROT_AVG_DELTA_ORIENT                                0.836443918794408f
//#define BASE_MOVE_DELTA_ORIENT                              0.001926266265142f
//#define BASE_MOVE_VAR_ORIENT                                0.000095927126446f
//#define MOVE_US_DELTA_ORIENT                                -0.000000500348823f
//#define MOVE_US_VAR_ORIENT                                  0.000000017373140f

// Updated again on 2012/05/20. Per-us moves are now done.
#define ROT_LEFT_DELTA_ORIENT                               0.828660059139694f
#define ROT_LEFT_DELTA_FWD_WORLD                            1.038863566825077f
#define ROT_LEFT_DELTA_LAT_WORLD                            1.831134279207705f
#define ROT_LEFT_VAR_ORIENT                                 0.004111054075077f
#define ROT_LEFT_VAR_FWD_WORLD                              16.446537677340324f
#define ROT_LEFT_VAR_LAT_WORLD                              2.631394285915838f
#define ROT_LEFT_COVAR_WORLD                                3.769846262466532f

#define ROT_RIGHT_DELTA_ORIENT                              -0.844227778449122f
#define ROT_RIGHT_DELTA_FWD_WORLD                           -3.466562388591270f
#define ROT_RIGHT_DELTA_LAT_WORLD                           1.353442303175682f
#define ROT_RIGHT_VAR_ORIENT                                0.001488083948824f
#define ROT_RIGHT_VAR_FWD_WORLD                             1.322516921503631f
#define ROT_RIGHT_VAR_LAT_WORLD                             0.455641616827991f
#define ROT_RIGHT_COVAR_WORLD                               0.397285147314804f //-0.397285147314804f

#define ROT_AVG_DELTA_ORIENT                                0.836443918794408f
#define N_ROTATIONS_FOR_180									5
#define N_ROTATIONS_IN_TOTAL								(1 + 2 * N_ROTATIONS_FOR_180) // 1 + e.g. 5 either side.

#define BASE_MOVE_DELTA_ORIENT                              0.001926266265142f //!< Base orient change in robot for 0ms move.
#define BASE_MOVE_DELTA_FWD_WORLD                           58.007307886548077f //!< Base move (made with 0ms burst from motors).
#define BASE_MOVE_DELTA_LAT_WORLD                           1.037339825851078f //!< Base offset (perpindicular to direction) for 0ms move.
#define BASE_MOVE_VAR_ORIENT                                0.000095927126446f //!< Variance in base orient change measurement.
#define BASE_MOVE_VAR_FWD_WORLD                             6.258169911429055f //!< Variance in base fwd move measurement.
#define BASE_MOVE_VAR_LAT_WORLD                             0.658728670231838f //!< Variance in base lateral move measurement.
#define BASE_MOVE_COVAR_WORLD                               -0.268971069608106f //!< Covariance between fwd and lateral move for 0ms burst.

#define MOVE_US_DELTA_ORIENT                                -0.000000083391470f //!< Increase in orient change per extra us moved.
#define MOVE_US_DELTA_FWD_WORLD                             0.000337470627025f //!< Increase in dist moved per each extra us motors are run.
#define MOVE_US_DELTA_LAT_WORLD                             0.000011529341924f //!< Increase in offset (perp to direction) per each extra us.
#define MOVE_US_VAR_ORIENT                                  0.000000002895523f //!< Increase in orient change variance per us.
#define MOVE_US_VAR_FWD_WORLD                               0.000084228961570f //!< Increase in fwd move variance per us.
#define MOVE_US_VAR_LAT_WORLD                               0.000009243601086f //!< Increase in lateral move variance per us.
#define MOVE_US_COVAR_WORLD                                 0.000010679430444f //!< Increase in move covariance between us.

#endif

#if ROBOT_PLATFORM == 3
// Updated on 2013/02/20

//! \todo Have to calibrate compass error, and probably fit values to curve
#define ORIENT_VAR SQR(DEGS_TO_RADS(COMPASS_STDEV))

#define ROT_LEFT_DELTA_ORIENT                               0.708459927606646f
#define ROT_LEFT_DELTA_FWD_WORLD                            5.215392059023332f
#define ROT_LEFT_DELTA_LAT_WORLD                            4.135985510096907f
#define ROT_LEFT_VAR_ORIENT                                 0.003866836934647f
#define ROT_LEFT_VAR_FWD_WORLD                              22.792992408117669f
#define ROT_LEFT_VAR_LAT_WORLD                              13.452815728469417f
#define ROT_LEFT_COVAR_WORLD                                8.928645871325624f //-8.928645871325624f

#define ROT_RIGHT_DELTA_ORIENT                              -0.726179495442484f
#define ROT_RIGHT_DELTA_FWD_WORLD                           0.169720013415643f
#define ROT_RIGHT_DELTA_LAT_WORLD                           1.483051041254330f
#define ROT_RIGHT_VAR_ORIENT                                0.004403755591380f
#define ROT_RIGHT_VAR_FWD_WORLD                             6.761064091474340f
#define ROT_RIGHT_VAR_LAT_WORLD                             0.628609170822145f
#define ROT_RIGHT_COVAR_WORLD                               0.474113773718120f

#define ROT_AVG_DELTA_ORIENT                                0.717319711524565f
#define N_ROTATIONS_FOR_180									5
#define N_ROTATIONS_IN_TOTAL								(1 + 2 * N_ROTATIONS_FOR_180) // 1 + e.g. 5 either side.

#define BASE_MOVE_DELTA_ORIENT                              0.019239386369274f //!< Base orient change in robot for 0ms move.
#define BASE_MOVE_DELTA_FWD_WORLD                           51.976782087798817f //!< Base move (made with 0ms burst from motors).
#define BASE_MOVE_DELTA_LAT_WORLD                           0.738406373661037f //!< Base offset (perpindicular to direction) for 0ms move.
#define BASE_MOVE_VAR_ORIENT                                0.000349454915652f //!< Variance in base orient change measurement.
#define BASE_MOVE_VAR_FWD_WORLD                             22.768199727977859f //!< Variance in base fwd move measurement.
#define BASE_MOVE_VAR_LAT_WORLD                             0.457819515587284f //!< Variance in base lateral move measurement.
#define BASE_MOVE_COVAR_WORLD                               0.295036145553406f //0.995036145553406f //!< Covariance between fwd and lateral move for 0ms burst.

#define MOVE_US_DELTA_ORIENT                                -0.000000065837699f //!< Increase in orient change per extra us moved.
#define MOVE_US_DELTA_FWD_WORLD                             0.000313687609493f //!< Increase in dist moved per each extra us motors are run.
#define MOVE_US_DELTA_LAT_WORLD                             0.000010253627318f //!< Increase in offset (perp to direction) per each extra us.
#define MOVE_US_VAR_ORIENT                                  0.000000004679152f //!< Increase in orient change variance per us.
#define MOVE_US_VAR_FWD_WORLD                               0.000025650687129f //-0.000025650687129f //!< Increase in fwd move variance per us.
#define MOVE_US_VAR_LAT_WORLD                               0.000025748439468f //!< Increase in lateral move variance per us.
#define MOVE_US_COVAR_WORLD                                 0.000005852264791f //!< Increase in move covariance between us.
#endif

#if ROBOT_PLATFORM == 4
// Updated on 2013/04/23
#define ORIENT_VAR SQR(DEGS_TO_RADS(COMPASS_STDEV))

#define ROT_LEFT_DELTA_ORIENT                               0.303654381936933f
#define ROT_LEFT_DELTA_FWD_WORLD                            4.371627567379570f
#define ROT_LEFT_DELTA_LAT_WORLD                            10.291437508589265f
#define ROT_LEFT_VAR_ORIENT                                 0.000918686073937f
#define ROT_LEFT_VAR_FWD_WORLD                              5.967083646281304f
#define ROT_LEFT_VAR_LAT_WORLD                              4.758102772647616f
#define ROT_LEFT_COVAR_WORLD                                3.073040826393439f

#define ROT_RIGHT_DELTA_ORIENT                              -0.298065873451236f
#define ROT_RIGHT_DELTA_FWD_WORLD                           2.555380281985574f
#define ROT_RIGHT_DELTA_LAT_WORLD                           5.983562016467570f
#define ROT_RIGHT_VAR_ORIENT                                0.021093229323720f
#define ROT_RIGHT_VAR_FWD_WORLD                             136.188239293676958f
#define ROT_RIGHT_VAR_LAT_WORLD                             12.572527915848960f
#define ROT_RIGHT_COVAR_WORLD                               9.362753729739266f

#define ROT_AVG_DELTA_ORIENT                                0.300860127694084f
#define N_ROTATIONS_FOR_180									10
#define N_ROTATIONS_IN_TOTAL								(1 + 2 * N_ROTATIONS_FOR_180) // 1 + e.g. 5 either side.

#define BASE_MOVE_DELTA_ORIENT                              -0.011282170448995f //!< Base orient change in robot for 0ms move.
#define BASE_MOVE_DELTA_FWD_WORLD                           39.847689222292189f //!< Base move (made with 0ms burst from motors).
#define BASE_MOVE_DELTA_LAT_WORLD                           0.763110739407041f //!< Base offset (perpindicular to direction) for 0ms move.
#define BASE_MOVE_VAR_ORIENT                                0.000158683515223f //!< Variance in base orient change measurement.
#define BASE_MOVE_VAR_FWD_WORLD                             6.016997854060487f //!< Variance in base fwd move measurement.
#define BASE_MOVE_VAR_LAT_WORLD                             0.395661122293547f //!< Variance in base lateral move measurement.
#define BASE_MOVE_COVAR_WORLD                               0.773399146548265f //!< Covariance between fwd and lateral move for 0ms burst.

#define MOVE_US_DELTA_ORIENT                                0.000000030984264f //!< Increase in orient change per extra us moved.
#define MOVE_US_DELTA_FWD_WORLD                             0.000179506604466f //!< Increase in dist moved per each extra us motors are run.
#define MOVE_US_DELTA_LAT_WORLD                             0.000002241354936f //!< Increase in offset (perp to direction) per each extra us.
#define MOVE_US_VAR_ORIENT                                  0.000000002022777f //!< Increase in orient change variance per us.
#define MOVE_US_VAR_FWD_WORLD                               -0.000016330343298f //!< Increase in fwd move variance per us.
#define MOVE_US_VAR_LAT_WORLD                               0.000003398563246f //!< Increase in lateral move variance per us.
#define MOVE_US_COVAR_WORLD                                 -0.000004036884329f //!< Increase in move covariance between us.
#endif

//
// Scaled move parameters
//
#define ROT_LEFT_DELTA_FWD			(ROT_LEFT_DELTA_FWD_WORLD / MAP_SCALE)
#define ROT_LEFT_DELTA_LAT			(ROT_LEFT_DELTA_LAT_WORLD / MAP_SCALE)
#define ROT_LEFT_VAR_FWD			(ROT_LEFT_VAR_FWD_WORLD / (MAP_SCALE * MAP_SCALE))
#define ROT_LEFT_VAR_LAT			(ROT_LEFT_VAR_LAT_WORLD / (MAP_SCALE * MAP_SCALE))
#define ROT_LEFT_COVAR				(ROT_LEFT_COVAR_WORLD / (MAP_SCALE * MAP_SCALE))

#define ROT_RIGHT_DELTA_FWD			(ROT_RIGHT_DELTA_FWD_WORLD / MAP_SCALE)
#define ROT_RIGHT_DELTA_LAT			(ROT_RIGHT_DELTA_LAT_WORLD / MAP_SCALE)
#define ROT_RIGHT_VAR_FWD			(ROT_RIGHT_VAR_FWD_WORLD / (MAP_SCALE * MAP_SCALE))
#define ROT_RIGHT_VAR_LAT			(ROT_RIGHT_VAR_LAT_WORLD / (MAP_SCALE * MAP_SCALE))
#define ROT_RIGHT_COVAR				(ROT_RIGHT_COVAR_WORLD / (MAP_SCALE * MAP_SCALE))

#define BASE_MOVE_DELTA_FWD			(BASE_MOVE_DELTA_FWD_WORLD / MAP_SCALE)
#define BASE_MOVE_DELTA_LAT			(BASE_MOVE_DELTA_LAT_WORLD / MAP_SCALE)
#define BASE_MOVE_VAR_FWD			(BASE_MOVE_VAR_FWD_WORLD / (MAP_SCALE * MAP_SCALE))
#define BASE_MOVE_VAR_LAT			(BASE_MOVE_VAR_LAT_WORLD / (MAP_SCALE * MAP_SCALE))
#define BASE_MOVE_COVAR				(BASE_MOVE_COVAR_WORLD / (MAP_SCALE * MAP_SCALE))

#define MOVE_US_DELTA_FWD			(MOVE_US_DELTA_FWD_WORLD / MAP_SCALE)
#define MOVE_US_DELTA_LAT			(MOVE_US_DELTA_LAT_WORLD / MAP_SCALE)
#define MOVE_US_VAR_FWD				(MOVE_US_VAR_FWD_WORLD / (MAP_SCALE * MAP_SCALE))
#define MOVE_US_VAR_LAT				(MOVE_US_VAR_LAT_WORLD / (MAP_SCALE * MAP_SCALE))
#define MOVE_US_COVAR				(MOVE_US_COVAR_WORLD / (MAP_SCALE * MAP_SCALE))



























//
// LOD in simulation log
//
//	#define PRINT_DEBUG
	#define PRINT_PROFIT
	#define PRINT_EVENTS
	#define PRINT_PROFIT_DETAIL
	#define PRINT_MAP_DETAIL
	#define PRINT_IK_DETAIL
	#define PRINT_COMM_DETAIL
	#define PRINT_CAM_DETAIL
	#define PRINT_TIME_DETAIL
	#define PRINT_CLOUD_DETAIL
	#define RECORDING_IMAGES
	#define RECORDING_LOCAL_MAPS
	#define CALC_LIVE_MAP_STATS

#ifdef IS_LINUX
#undef CALC_LIVE_MAP_STATS
#endif

#ifdef IS_LINUX
#undef PRINT_MAP_DETAIL
#undef PRINT_CLOUD_DETAIL
#endif



#ifdef PRINT_DEBUG
#define DEBUG_MARKER(X) printf(" - ");printf(X);printf("\n");fflush(stdout);
#else
#define DEBUG_MARKER(X)
#endif

#ifdef SIMULATION
//	#define SHOW_PATHS
	#define SHOW_IMGS
//	#define SHOW_ROBOT_IMGS
#endif

#if defined(BOARD) && defined(IS_WIN)
//	#define SHOW_PATHS
	#define SHOW_IMGS
#endif

#if defined(RECORDING_ROBOT) && !defined(RECORDING_IMAGES)
#error "Should probably record images when robot status is being recorded"
#endif

#ifdef RERUNNING_ROBOT
#define SHOW_ROBOT_IMGS
#endif



















//
// Experiment details
//
extern int						g_nRobots; // Defined in RobotTypes.cpp
#define SET_N_ROBOTS(n)			g_nRobots = n;
#define N_ROBOTS				g_nRobots //!< By default 1
#ifdef BOARD
extern int						g_nRobotThreads;
#define SET_N_ROBOTS_THREADS_FOR_TEST_COAL(n) g_nRobotThreads = n;
#define N_ROBOT_THREADS			g_nRobotThreads
#endif
#define MAX_N_ROBOTS			8 //!< Max num robots when batch running experiments
extern int						g_duration; // Defined in RobotTypes.cpp
#define SET_DURATION(d)			g_duration = d;
#define DURATION				g_duration //!< Either ms or iterations
#define DENSITY					0 //!< Density of obstacles in generated environment
	#define STD_DEV_MAX				20 //!< max allowable std dev of robot's pose est (was 10 before)
//	#define STD_DEV_MAX				20 //!< max allowable std dev of robot's pose est (was 10 before)
//	#define STD_DEV_MAX				100 //!< max allowable std dev of robot's pose est (was 10 before)
#define STD_DEV_PIXEL_GAP		20 //!< Value used for encoding std dev in map
//	#define GTEP_ALLOC_PROFIT		0.4f //!< Coefficient for allocating profit to GTEP behaviour
//	#define GTEP_ALLOC_PROFIT		0.15f //!< Coefficient for allocating profit to GTEP behaviour
//	#define GTEP_ALLOC_PROFIT		0.3f //!< Coefficient for allocating profit to GTEP behaviour
//	#define GTEP_ALLOC_PROFIT		0.35f //!< Coefficient for allocating profit to GTEP behaviour
#define SUP_ALLOC_PROFIT		0.3f //!< Rough estimate of coefficient for allocating profit to partner robot. Actual amount will depend on coalition.
	#define STD_DEV_COST_COEFF		200.0f //!< Reduction in profit imposed related to increase in stdDev
//	#define STD_DEV_COST_COEFF		50.0f //!< Reduction in profit imposed related to increase in stdDev
	#define BATTERY_LOSS_MOVE		20.0f //!< Reduction in profit imposed relative to use of battery for movement.
//	#define BATTERY_LOSS_MOVE		200.0f //!< Reduction in profit imposed relative to use of battery for movement.

#define BATTERY_LOSS_IDLE		0.1f //!< Reduction in profit imposed relative to use of battery for staying idle.
#define COALITION_QUIT_COEFF	5 //!< If quitting out of a coalition, estimate that profit from this many iterations will be lost.
#define BUFFER_SIZE				2048
#define BID_PROFIT				1.1f
#define MIN_BID_PROFIT			10.0f
#define MAP_SCAN_LIST_LIMIT		50
#define AVG_ACTUAL_CLOSE_LOOP_MOVES			1.8f
//
// Move parameters determined from experiment results
//
//#define AVG_STD_DEV_PER_MOVE				0.383471756676557f //!< Calculated from experiment data 2012/02/05.
#define AVG_STD_DEV_PER_MOVE				0.05f //!< Calculated from experiment data 2012/02/05.
#define AVG_N_STEPS_PER_EXP_TAR				3.659987975f //!< Calculated from experiment data 2012/02/05.
#define AVG_N_STEPS_PER_LOC_MAP				26.375f //!< Calculated from experiment data 2012/02/05.
//#define PROFIT_PER_MOVE_OVER_STD_DEV_M		-3.86009f //!< m of linear regression line fit to measurements from experiments
//#define PROFIT_PER_MOVE_OVER_STD_DEV_B		105.1904f //!< b of linear regression line fit to measurements from experiments
#define AVG_PROFIT_RED_PER_STD_DEV			(-5.0f / STD_DEV_MAX) //!< Typical profit ratio when stdDev is 0
#define AVG_CLOSE_LOOP_STD_DEV				2.0f //!< \todo Not property estimated yet
#define AVG_INC_MOVES_CLOSE_LOOP			1.3f //!< \todo May require updating when more experimentation data available
#define AVG_N_LOC_MAPS_CLOSE_LOOP			2.5f //!< \todo May require updating when more experimentation data available
#define TYPICAL_SUP_N_CELLS_MAPPED			144.0f //!< Roughly 4 local maps worth of exploration cells typically mapped per coalition

//
// Map processing
//
#define EXP_CELL_MAPPED_THRESHOLD		0.3f //0.4f
#define INVALID_CELL					(uchar)255
#define NAV_CELL_OCCUPIED_THRESHOLD		10
#define NARROW_NAV_MAP_DILATE_DIST		9 //!< Robot half-length is circa 8 (150mm), dilate beyond this to avoid collisions.
#define BROAD_NAV_MAP_DILATE_DIST		13
#define NARROW_VEHICLE_DIST				14 //!< circa 2x robot half dist (so 2 robots don't collide)
#define BROAD_VEHICLE_DIST				16
#define THRESHOLD_TERRAIN				(uchar)125
#define FREE_TERRAIN					(uchar)254
#define FAKE_NBOUR_TERRAIN				(uchar)156 //!< Used to represent assumed terrain mapped by other robots nearby.
#define FAKE_TEMP_TERRAIN				(uchar)155 //!< Used to represent terrain from moves when calculating behaviour profit.
#define OCCUPIED_TERRAIN				(uchar)0
#define NARROW_VEHICLE_TERRAIN			(uchar)10 //!< Terrain on which another robot is sitting.
#define BROAD_VEHICLE_TERRAIN			(uchar)60
#define NARROW_OCCUPIED					(uchar)20
#define BROAD_OCCUPIED					(uchar)70
#define DONT_ALLOW_OUTSIDE_LOCAL_MAP	0
#define DO_ALLOW_OUTSIDE_LOCAL_MAP		1
#define NOT_OCCUPIED_FLAG				(uchar)0
#define OCCUPIED_FLAG					(uchar)1
#define UNKNOWN_OCCUPANCY_FLAG			(uchar)2
#define ROBOT_OCCUPIED_FLAG				(uchar)3

//
// Navigation
//
//#define NAV_MAXDIST					10.0f //!< Currently deprecated, but may decide to use over already-mapped terrain
//#define NAV_MAXDIST_COARSE			20.0f //!< Currently deprecated, but may decide to use over already-mapped terrain
#define VISEST_LEEWAY				4.0f
#define EXPLORATION_LEEWAY			5.0f
#define BACKTRACK_LEEWAY			6.0f
#define FOLLOW_PATH_LEEWAY			6.0f
#define SUPERVISION_LEEWAY			16.0f
#define GTEP_LEEWAY					16.0f
//#define ROBOT_ON_LOC_MAP_LEEWAY		2 //!< Dist within which another robot is considered on a local map.
#define ROBOT_ON_LOC_MAP_LEEWAY		-1 //!< Dist within which another robot is considered on a local map.
#define LOC_MAP_EDGE_LEEWAY			5 //!< Used in IK. More strict than value used when verifying, to allow for numerical accuracy or rounding issues.
#define LOC_MAP_EDGE_LEEWAY_2		(LOC_MAP_EDGE_LEEWAY + 1) //!< Used in isLocalMapCentreOK.

//
// Map dims
//
#define ENVIR_DIMS							504
#define LOC_MAP_DIMS						84 //!< Dims of local map used by robots to communicate map data
#define LOC_MAP_DIFF						42 //!< Dist between overlapping loc maps' centre pts
//#define SUP_DIMS							168 //!< Dims of supervision area
//#define SUP_DIFF							84 //!< Dist between supervision areas' centre pts
#define SUP_DIMS							252 //!< Dims of supervision area
#define SUP_DIFF							126 //!< Dist between supervision areas' centre pts
#define EXP_AREA							14 //!< Dims of area to explore
#define NAV_CELL_AREA						10 //!< Dimensions of navigation cell
#define EXP_GRID_DIMS						(LOC_MAP_DIMS / EXP_AREA)
#define GLOB_EXP_GRID_DIMS					(ENVIR_DIMS / EXP_AREA)
#define GLOB_LOC_MAP_GRID_DIMS				(((ENVIR_DIMS - LOC_MAP_DIMS) / LOC_MAP_DIFF) + 1)
#define SUP_GRID_DIMS						(((ENVIR_DIMS - SUP_DIMS) / SUP_DIFF) + 1)
#define NAV_GRID_DIMS						(ENVIR_DIMS / NAV_CELL_AREA)
#define N_CELLS_LOC_MAP						36
//#define N_LOC_MAPS_SUP_AREA					4
#define N_LOC_MAPS_SUP_AREA					9
#define N_CELLS_SUP_AREA					(N_CELLS_LOC_MAP * N_LOC_MAPS_SUP_AREA)
#define MAP_SCALE							20.0f //!< 1 cell == 2cm
#define MAP_SCALE_INV						0.05f
#define MAP_SCALE_I							20

//
// Camera image processing: CMUCam2
//
#define CAM_IMG_W							176
#define CAM_IMG_H							143
#define CAM_IMG_W_HALF						88
#define CAM_IMG_H_HALF						71
#define CAM_IMG_ORIG_X						2
#define CAM_IMG_ORIG_Y						4
#define CAM_IMG_FLIP_V						1
#define CAM_IMG_FLIP_H						0
#define SIZE_CAM_IMG (CAM_IMG_W * CAM_IMG_H)
#define CAM_OCCUPANCY_GRID_X				19 // 19 * 9 = 171
#define CAM_OCCUPANCY_GRID_Y				23 // 23 * 6 = 138
#define CAM_OCCUPANCY_CELL_X				9
#define CAM_OCCUPANCY_CELL_Y				6
#define CAM_OCCUPANCY_CELL_X_HALF			5
#define CAM_OCCUPANCY_CELL_Y_HALF			3
#define CAM_OCCUPANCY_PIXELS_X (CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_CELL_X)
#define CAM_OCCUPANCY_PIXELS_Y (CAM_OCCUPANCY_GRID_Y * CAM_OCCUPANCY_CELL_Y)
#define SIZE_COOP_LOC_GRID (CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y)
#define MAX_COOP_LOC_NBOUR_DIST (CAM_OCCUPANCY_GRID_X + 1) //!< The max dist (difference in indices) between cell (i,j) and a neighbour is 1+CAM_OCCUPANCY_GRID_X
#define SIZE_OCCUP_GRID (CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y)
#define SIZE_FOR_TWOBITARRAY(x) ((x % 4) ? (x / 4 + 1) : (x / 4))
#define SIZE_COOP_LOC_TWOBITARRAY SIZE_FOR_TWOBITARRAY(SIZE_COOP_LOC_GRID)
#define SIZE_OCCUP_TWOBITARRAY SIZE_FOR_TWOBITARRAY(SIZE_OCCUP_GRID)
#define SIZE_FOR_BITARRAY(x) ((x % 8) ? (x / 8 + 1) : (x / 8))
#define SIZE_COOP_LOC_BITARRAY SIZE_FOR_TWOBITARRAY(SIZE_COOP_LOC_GRID)
#define OCCUP_COORD(a, b) (a + b * CAM_OCCUPANCY_GRID_X)
























//
// Types
//
typedef unsigned char uchar;
#ifdef IS_GUMSTIX
typedef char schar;
#else
typedef signed char schar;
#endif
typedef unsigned char tBOOL;
typedef unsigned char BYTE;
typedef unsigned short UINT16;
typedef short INT16;
typedef unsigned int UINT32;
typedef int INT32;

#ifdef IS_LINUX
typedef schar __int8;
typedef short __int16;
typedef int __int32;
#endif

#ifdef IS_WIN
#define __func__ __FUNCTION__
#endif


//
// Includes
//

// For functions such as sleep, special headers have to be included
// in linux, or in windows if OpenCV is not already included.
#ifdef IS_LINUX

#include <unistd.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <netdb.h>
#include <pthread.h>

#else // IS_LINUX

#define CRT_SECURE_NO_DEPRECATE 1
#define _CRTDBG_MAP_ALLOC
#include <winsock2.h>
#include <windows.h>
#include <assert.h>
#include <crtdbg.h>
#include <direct.h>
#include <time.h>

#endif // IS_LINUX

// Include for all platforms
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <float.h>

//
// Defines
//
#ifdef _DEBUG
#define DEBUG_ASSERT(cond) assert(cond);
#define DEBUG_ASSERT_IS_BOOL(var) assert(var == 0 || var == 1);
#else
#define DEBUG_ASSERT(cond)
#define DEBUG_ASSERT_IS_BOOL(cond)
#endif

#define RELEASE_ASSERT(cond) assert(cond);

#define min(a,b) (((a) < (b)) ? (a) : (b))
#define max(a,b) (((a) > (b)) ? (a) : (b))

#ifdef IS_LINUX
#define SLEEP_FOR_MS(A) usleep(A*1000); // us
#else
#define SLEEP_FOR_MS(A) Sleep(A); // ms
#endif

#ifdef PRINT_TIME_DETAIL
#define WIPE_TIMERS wipeTimerData();
#define TIMER_START(A) addStartTime (A);
#define TIMER_STOP(F, A) addStopTime (F, A);

#else // PRINT_TIME_DETAIL

#define WIPE_TIMERS
#define TIMER_START(A)
#define TIMER_STOP(A)
#endif

#ifndef ROBOT
#define ROBOT_REQUEST_PARAMS (BoardDatabase *db, CommData *commData, const int payload, FILE *f)
#endif

#endif // #ifndef GLOB_DEFS_H

