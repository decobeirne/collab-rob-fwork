



#ifdef USE_ROB3_IMAGES_FROM_0209

#define IS_LOC_STUFF data[i].obs = 1; data[i].vis = 0; data[i].isLoc = 1;
#define NO_LOC_STUFF data[i].obs = 1; data[i].vis = 0; data[i].isLoc = 0;
#define COG_OFFSET 149.0f
#define OBS_LOC(X, Y) data[i].loc.x = -(X.0f + COG_OFFSET); data[i].loc.y = Y.0f;
#define VIS_ORIENT(X) data[i].orient = X;

	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00000.dat"); IS_LOC_STUFF OBS_LOC(100, 150) VIS_ORIENT(RADS_90) //0-100 (line at 146-23)
	memcpy (data[i].occupancyGrid, grid_r3_0209_00, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00001.dat"); IS_LOC_STUFF OBS_LOC(100, 100) VIS_ORIENT(RADS_90)
	memcpy (data[i].occupancyGrid, grid_r3_0209_01, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00002.dat"); IS_LOC_STUFF OBS_LOC(100, 50) VIS_ORIENT(RADS_90)
	memcpy (data[i].occupancyGrid, grid_r3_0209_02, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	// 3 is same as 2
	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00004.dat"); IS_LOC_STUFF OBS_LOC(100, -50) VIS_ORIENT(RADS_90)
	memcpy (data[i].occupancyGrid, grid_r3_0209_04, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00005.dat"); IS_LOC_STUFF OBS_LOC(100, -100) VIS_ORIENT(RADS_90)
	memcpy (data[i].occupancyGrid, grid_r3_0209_05, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00006.dat"); IS_LOC_STUFF OBS_LOC(100, -150) VIS_ORIENT(RADS_90)
	memcpy (data[i].occupancyGrid, grid_r3_0209_06, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00007.dat"); IS_LOC_STUFF OBS_LOC(150, -150) VIS_ORIENT(RADS_90) //6-150 (line at 106-23) (bottom at 146-23)
	memcpy (data[i].occupancyGrid, grid_r3_0209_07, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00008.dat"); IS_LOC_STUFF OBS_LOC(150, -100) VIS_ORIENT(RADS_90)
	memcpy (data[i].occupancyGrid, grid_r3_0209_08, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00009.dat"); IS_LOC_STUFF OBS_LOC(150, 0) VIS_ORIENT(RADS_90)
	memcpy (data[i].occupancyGrid, grid_r3_0209_09, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);













	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00010.dat"); IS_LOC_STUFF OBS_LOC(150, 50) VIS_ORIENT(RADS_90)
	memcpy (data[i].occupancyGrid, grid_r3_0209_10, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00011.dat"); IS_LOC_STUFF OBS_LOC(150, 100) VIS_ORIENT(RADS_90)
	memcpy (data[i].occupancyGrid, grid_r3_0209_11, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00012.dat"); IS_LOC_STUFF OBS_LOC(150, 150) VIS_ORIENT(RADS_90) //11
	memcpy (data[i].occupancyGrid, grid_r3_0209_12, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00013.dat"); IS_LOC_STUFF OBS_LOC(200, 150) VIS_ORIENT(RADS_90) //12-200 (line at 87-23) (bottom at 112-23)
	memcpy (data[i].occupancyGrid, grid_r3_0209_13, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00014.dat"); IS_LOC_STUFF OBS_LOC(200, 100) VIS_ORIENT(RADS_90)
	memcpy (data[i].occupancyGrid, grid_r3_0209_14, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00015.dat"); IS_LOC_STUFF OBS_LOC(200, 50) VIS_ORIENT(RADS_90)
	memcpy (data[i].occupancyGrid, grid_r3_0209_15, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00016.dat"); IS_LOC_STUFF OBS_LOC(200, 0) VIS_ORIENT(RADS_90)
	memcpy (data[i].occupancyGrid, grid_r3_0209_16, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00017.dat"); IS_LOC_STUFF OBS_LOC(200, -50) VIS_ORIENT(RADS_90)
	memcpy (data[i].occupancyGrid, grid_r3_0209_17, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00018.dat"); IS_LOC_STUFF OBS_LOC(200, -100) VIS_ORIENT(RADS_90)
	memcpy (data[i].occupancyGrid, grid_r3_0209_18, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	// 19 same as 18














	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00020.dat"); IS_LOC_STUFF OBS_LOC(200, -150) VIS_ORIENT(RADS_90) //18
	memcpy (data[i].occupancyGrid, grid_r3_0209_20, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	// 21 same as 20
	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00022.dat"); IS_LOC_STUFF OBS_LOC(200, -100) VIS_ORIENT(RADS_90) //19-200 again (line at 87-23) (bottom at 111-23)
	memcpy (data[i].occupancyGrid, grid_r3_0209_22, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00023.dat"); IS_LOC_STUFF OBS_LOC(200, -50) VIS_ORIENT(RADS_90)
	memcpy (data[i].occupancyGrid, grid_r3_0209_23, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00024.dat"); IS_LOC_STUFF OBS_LOC(200, 0) VIS_ORIENT(RADS_90)
	memcpy (data[i].occupancyGrid, grid_r3_0209_24, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00025.dat"); IS_LOC_STUFF OBS_LOC(200, 50) VIS_ORIENT(RADS_90)
	memcpy (data[i].occupancyGrid, grid_r3_0209_25, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00026.dat"); IS_LOC_STUFF OBS_LOC(200, 100) VIS_ORIENT(RADS_90)
	memcpy (data[i].occupancyGrid, grid_r3_0209_26, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00027.dat"); IS_LOC_STUFF OBS_LOC(200, 150) VIS_ORIENT(RADS_90) //24
	memcpy (data[i].occupancyGrid, grid_r3_0209_27, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00028.dat"); IS_LOC_STUFF OBS_LOC(250, 150) VIS_ORIENT(RADS_90) //25-250 (line at 71-23) (bottom at 88-23)
	memcpy (data[i].occupancyGrid, grid_r3_0209_28, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00029.dat"); IS_LOC_STUFF OBS_LOC(250, 100) VIS_ORIENT(RADS_90)
	memcpy (data[i].occupancyGrid, grid_r3_0209_29, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);



























	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00030.dat"); IS_LOC_STUFF OBS_LOC(250, 50) VIS_ORIENT(RADS_90) //27
	memcpy (data[i].occupancyGrid, grid_r3_0209_30, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00031.dat"); IS_LOC_STUFF OBS_LOC(250, 0) VIS_ORIENT(RADS_90)
	memcpy (data[i].occupancyGrid, grid_r3_0209_31, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00032.dat"); IS_LOC_STUFF OBS_LOC(250, -50) VIS_ORIENT(RADS_90)
	memcpy (data[i].occupancyGrid, grid_r3_0209_32, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00033.dat"); IS_LOC_STUFF OBS_LOC(250, -100) VIS_ORIENT(RADS_90)
	memcpy (data[i].occupancyGrid, grid_r3_0209_33, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00034.dat"); IS_LOC_STUFF OBS_LOC(250, -150) VIS_ORIENT(RADS_90) //31
	memcpy (data[i].occupancyGrid, grid_r3_0209_34, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00035.dat"); IS_LOC_STUFF OBS_LOC(300, -150) VIS_ORIENT(RADS_90) //32-300 (line at 58-23) (bottom at 70-23)
	memcpy (data[i].occupancyGrid, grid_r3_0209_35, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00036.dat"); IS_LOC_STUFF OBS_LOC(300, -100) VIS_ORIENT(RADS_90)
	memcpy (data[i].occupancyGrid, grid_r3_0209_36, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00037.dat"); IS_LOC_STUFF OBS_LOC(300, -50) VIS_ORIENT(RADS_90)
	memcpy (data[i].occupancyGrid, grid_r3_0209_37, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00038.dat"); IS_LOC_STUFF OBS_LOC(300, 0) VIS_ORIENT(RADS_90)
	memcpy (data[i].occupancyGrid, grid_r3_0209_38, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00039.dat"); IS_LOC_STUFF OBS_LOC(300, 50) VIS_ORIENT(RADS_90)
	memcpy (data[i].occupancyGrid, grid_r3_0209_39, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);




















	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00040.dat"); IS_LOC_STUFF OBS_LOC(300, 100) VIS_ORIENT(RADS_90)
	memcpy (data[i].occupancyGrid, grid_r3_0209_40, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00041.dat"); IS_LOC_STUFF OBS_LOC(300, 150) VIS_ORIENT(RADS_90) //38
	memcpy (data[i].occupancyGrid, grid_r3_0209_41, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00042.dat"); IS_LOC_STUFF OBS_LOC(300, 0) VIS_ORIENT(RADS_90) //39-300 (line at 60-23) (bottom at 70-23)
	memcpy (data[i].occupancyGrid, grid_r3_0209_42, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	// 43 same as 42
	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00044.dat"); IS_LOC_STUFF OBS_LOC(250, 0) VIS_ORIENT(RADS_90) //40-250 (bottom at 86-23)
	memcpy (data[i].occupancyGrid, grid_r3_0209_44, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00045.dat"); IS_LOC_STUFF OBS_LOC(200, 0) VIS_ORIENT(RADS_90) //41-200 (think) (bottom at 111-23)
	memcpy (data[i].occupancyGrid, grid_r3_0209_45, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00046.dat"); IS_LOC_STUFF OBS_LOC(150, 0) VIS_ORIENT(RADS_90) //42-150 (bottom at 151)
	memcpy (data[i].occupancyGrid, grid_r3_0209_46, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00047.dat"); IS_LOC_STUFF OBS_LOC(100, 0) VIS_ORIENT(RADS_90) //43-100 (top at 91-23)
	memcpy (data[i].occupancyGrid, grid_r3_0209_47, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	// 48 same as 47
	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00049.dat"); IS_LOC_STUFF OBS_LOC(100, 0) VIS_ORIENT(PI) //44-100
	memcpy (data[i].occupancyGrid, grid_r3_0209_49, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);






































	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00050.dat"); IS_LOC_STUFF OBS_LOC(100, 150) VIS_ORIENT(PI) //45-100 (line at 147-23)
	memcpy (data[i].occupancyGrid, grid_r3_0209_50, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00051.dat"); IS_LOC_STUFF OBS_LOC(100, 100) VIS_ORIENT(PI)
	memcpy (data[i].occupancyGrid, grid_r3_0209_51, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00052.dat"); IS_LOC_STUFF OBS_LOC(100, 50) VIS_ORIENT(PI)
	memcpy (data[i].occupancyGrid, grid_r3_0209_52, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	// 53 same as 49
	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00054.dat"); IS_LOC_STUFF OBS_LOC(100, -50) VIS_ORIENT(PI)
	memcpy (data[i].occupancyGrid, grid_r3_0209_54, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00055.dat"); IS_LOC_STUFF OBS_LOC(100, -100) VIS_ORIENT(PI)
	memcpy (data[i].occupancyGrid, grid_r3_0209_55, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	// 56 same as 55
	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00057.dat"); IS_LOC_STUFF OBS_LOC(100, -150) VIS_ORIENT(PI)
	memcpy (data[i].occupancyGrid, grid_r3_0209_57, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00058.dat"); IS_LOC_STUFF OBS_LOC(150, -150) VIS_ORIENT(PI) //51-150 (line at 111)
	memcpy (data[i].occupancyGrid, grid_r3_0209_58, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00059.dat"); IS_LOC_STUFF OBS_LOC(150, -100) VIS_ORIENT(PI)
	memcpy (data[i].occupancyGrid, grid_r3_0209_59, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);



















	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00060.dat"); IS_LOC_STUFF OBS_LOC(150, -50) VIS_ORIENT(PI) //53-150 (line at 111) (top at 84)
	memcpy (data[i].occupancyGrid, grid_r3_0209_60, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00061.dat"); IS_LOC_STUFF OBS_LOC(150, 0) VIS_ORIENT(PI)
	memcpy (data[i].occupancyGrid, grid_r3_0209_61, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00062.dat"); IS_LOC_STUFF OBS_LOC(150, 50) VIS_ORIENT(PI)
	memcpy (data[i].occupancyGrid, grid_r3_0209_62, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00063.dat"); IS_LOC_STUFF OBS_LOC(150, 100) VIS_ORIENT(PI)
	memcpy (data[i].occupancyGrid, grid_r3_0209_63, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00064.dat"); IS_LOC_STUFF OBS_LOC(150, 150) VIS_ORIENT(PI) //57
	memcpy (data[i].occupancyGrid, grid_r3_0209_64, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00065.dat"); IS_LOC_STUFF OBS_LOC(200, 150) VIS_ORIENT(PI) //58-200 (line at 87)
	memcpy (data[i].occupancyGrid, grid_r3_0209_65, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00066.dat"); IS_LOC_STUFF OBS_LOC(200, 100) VIS_ORIENT(PI)
	memcpy (data[i].occupancyGrid, grid_r3_0209_66, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00067.dat"); IS_LOC_STUFF OBS_LOC(200, 50) VIS_ORIENT(PI)
	memcpy (data[i].occupancyGrid, grid_r3_0209_67, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00068.dat"); IS_LOC_STUFF OBS_LOC(200, 0) VIS_ORIENT(PI)
	memcpy (data[i].occupancyGrid, grid_r3_0209_68, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00069.dat"); IS_LOC_STUFF OBS_LOC(200, -50) VIS_ORIENT(PI) //62-200 (line at 87-23) (bottom at 140) (top at 49)
	memcpy (data[i].occupancyGrid, grid_r3_0209_69, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);

















	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00070.dat"); IS_LOC_STUFF OBS_LOC(200, -100) VIS_ORIENT(PI)
	memcpy (data[i].occupancyGrid, grid_r3_0209_70, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00071.dat"); IS_LOC_STUFF OBS_LOC(200, -150) VIS_ORIENT(PI)
	memcpy (data[i].occupancyGrid, grid_r3_0209_71, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00072.dat"); IS_LOC_STUFF OBS_LOC(250, -150) VIS_ORIENT(PI) //65-250
	memcpy (data[i].occupancyGrid, grid_r3_0209_72, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00073.dat"); IS_LOC_STUFF OBS_LOC(250, -100) VIS_ORIENT(PI) //66 (line at 71) (top at 34) (bottom at 106)
	memcpy (data[i].occupancyGrid, grid_r3_0209_73, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00074.dat"); IS_LOC_STUFF OBS_LOC(250, -50) VIS_ORIENT(PI)
	memcpy (data[i].occupancyGrid, grid_r3_0209_74, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00075.dat"); IS_LOC_STUFF OBS_LOC(250, 0) VIS_ORIENT(PI)
	memcpy (data[i].occupancyGrid, grid_r3_0209_75, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00076.dat"); IS_LOC_STUFF OBS_LOC(250, 50) VIS_ORIENT(PI)
	memcpy (data[i].occupancyGrid, grid_r3_0209_76, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00077.dat"); IS_LOC_STUFF OBS_LOC(250, 100) VIS_ORIENT(PI)
	memcpy (data[i].occupancyGrid, grid_r3_0209_77, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00078.dat"); IS_LOC_STUFF OBS_LOC(250, 150) VIS_ORIENT(PI) //71
	memcpy (data[i].occupancyGrid, grid_r3_0209_78, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00079.dat"); IS_LOC_STUFF OBS_LOC(300, 150) VIS_ORIENT(PI) //72
	memcpy (data[i].occupancyGrid, grid_r3_0209_79, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);






















	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00080.dat"); IS_LOC_STUFF OBS_LOC(300, 100) VIS_ORIENT(PI)
	memcpy (data[i].occupancyGrid, grid_r3_0209_80, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00081.dat"); IS_LOC_STUFF OBS_LOC(300, 50) VIS_ORIENT(PI)
	memcpy (data[i].occupancyGrid, grid_r3_0209_81, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00082.dat"); IS_LOC_STUFF OBS_LOC(300, 0) VIS_ORIENT(PI) //75-300 (line at 60) (bottom at 82)
	memcpy (data[i].occupancyGrid, grid_r3_0209_82, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00083.dat"); IS_LOC_STUFF OBS_LOC(300, -50) VIS_ORIENT(PI)
	memcpy (data[i].occupancyGrid, grid_r3_0209_83, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	// 84 same as 83
	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00085.dat"); IS_LOC_STUFF OBS_LOC(300, -100) VIS_ORIENT(PI)
	memcpy (data[i].occupancyGrid, grid_r3_0209_85, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00086.dat"); IS_LOC_STUFF OBS_LOC(300, -150) VIS_ORIENT(PI) //78
	memcpy (data[i].occupancyGrid, grid_r3_0209_86, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00087.dat"); IS_LOC_STUFF OBS_LOC(300, -150) VIS_ORIENT(RADS_135) //79
	memcpy (data[i].occupancyGrid, grid_r3_0209_87, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00088.dat"); IS_LOC_STUFF OBS_LOC(300, -100) VIS_ORIENT(RADS_135)
	memcpy (data[i].occupancyGrid, grid_r3_0209_88, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00089.dat"); IS_LOC_STUFF OBS_LOC(300, -50) VIS_ORIENT(RADS_135) //81
	memcpy (data[i].occupancyGrid, grid_r3_0209_89, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);





















	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00090.dat"); IS_LOC_STUFF OBS_LOC(300, 0) VIS_ORIENT(RADS_135) //82-300 (line at 59) (bottomCorner at 89) (topCorner at 24)
	memcpy (data[i].occupancyGrid, grid_r3_0209_90, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00091.dat"); IS_LOC_STUFF OBS_LOC(300, 50) VIS_ORIENT(RADS_135)
	memcpy (data[i].occupancyGrid, grid_r3_0209_91, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00092.dat"); IS_LOC_STUFF OBS_LOC(300, 100) VIS_ORIENT(RADS_135)
	memcpy (data[i].occupancyGrid, grid_r3_0209_92, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00093.dat"); IS_LOC_STUFF OBS_LOC(300, 150) VIS_ORIENT(RADS_135) //85 (line at 58)
	memcpy (data[i].occupancyGrid, grid_r3_0209_93, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00094.dat"); IS_LOC_STUFF OBS_LOC(250, 150) VIS_ORIENT(RADS_135) //86-250
	memcpy (data[i].occupancyGrid, grid_r3_0209_94, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00095.dat"); IS_LOC_STUFF OBS_LOC(250, 100) VIS_ORIENT(RADS_135)
	memcpy (data[i].occupancyGrid, grid_r3_0209_95, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00096.dat"); IS_LOC_STUFF OBS_LOC(250, 50) VIS_ORIENT(RADS_135)
	memcpy (data[i].occupancyGrid, grid_r3_0209_96, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00097.dat"); IS_LOC_STUFF OBS_LOC(250, 0) VIS_ORIENT(RADS_135) //89-250 (line at 71) (bottomCorner at 114) (topCorner at 36)
	memcpy (data[i].occupancyGrid, grid_r3_0209_97, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00098.dat"); IS_LOC_STUFF OBS_LOC(250, -50) VIS_ORIENT(RADS_135)
	memcpy (data[i].occupancyGrid, grid_r3_0209_98, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00099.dat"); IS_LOC_STUFF OBS_LOC(250, -100) VIS_ORIENT(RADS_135)
	memcpy (data[i].occupancyGrid, grid_r3_0209_99, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);























	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00100.dat"); IS_LOC_STUFF OBS_LOC(250, -150) VIS_ORIENT(RADS_135) //92
	memcpy (data[i].occupancyGrid, grid_r3_0209_100, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00101.dat"); IS_LOC_STUFF OBS_LOC(200, -150) VIS_ORIENT(RADS_135) //93-200
	memcpy (data[i].occupancyGrid, grid_r3_0209_101, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00102.dat"); IS_LOC_STUFF OBS_LOC(200, -100) VIS_ORIENT(RADS_135)
	memcpy (data[i].occupancyGrid, grid_r3_0209_102, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00103.dat"); IS_LOC_STUFF OBS_LOC(200, -50) VIS_ORIENT(RADS_135)
	memcpy (data[i].occupancyGrid, grid_r3_0209_103, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00104.dat"); IS_LOC_STUFF OBS_LOC(200, 0) VIS_ORIENT(RADS_135) //96-200 (line at 87) (bottomCorner at 155) (topCorner at 58)
	memcpy (data[i].occupancyGrid, grid_r3_0209_104, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00105.dat"); IS_LOC_STUFF OBS_LOC(200, 50) VIS_ORIENT(RADS_135)
	memcpy (data[i].occupancyGrid, grid_r3_0209_105, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00106.dat"); IS_LOC_STUFF OBS_LOC(200, 100) VIS_ORIENT(RADS_135)
	memcpy (data[i].occupancyGrid, grid_r3_0209_106, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00107.dat"); IS_LOC_STUFF OBS_LOC(200, 150) VIS_ORIENT(RADS_135) //99
	memcpy (data[i].occupancyGrid, grid_r3_0209_107, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00108.dat"); IS_LOC_STUFF OBS_LOC(150, 150) VIS_ORIENT(RADS_135) //100
	memcpy (data[i].occupancyGrid, grid_r3_0209_108, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00109.dat"); IS_LOC_STUFF OBS_LOC(150, 100) VIS_ORIENT(RADS_135)
	memcpy (data[i].occupancyGrid, grid_r3_0209_109, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);





















	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00110.dat"); IS_LOC_STUFF OBS_LOC(150, 50) VIS_ORIENT(RADS_135) //102-150 (line at 111)
	memcpy (data[i].occupancyGrid, grid_r3_0209_110, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00111.dat"); IS_LOC_STUFF OBS_LOC(150, 0) VIS_ORIENT(RADS_135) //103-150 (topCorner at 108)
	memcpy (data[i].occupancyGrid, grid_r3_0209_111, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00112.dat"); IS_LOC_STUFF OBS_LOC(150, -50) VIS_ORIENT(RADS_135)
	memcpy (data[i].occupancyGrid, grid_r3_0209_112, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00113.dat"); IS_LOC_STUFF OBS_LOC(150, -100) VIS_ORIENT(RADS_135)
	memcpy (data[i].occupancyGrid, grid_r3_0209_113, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00114.dat"); IS_LOC_STUFF OBS_LOC(150, -150) VIS_ORIENT(RADS_135)
	memcpy (data[i].occupancyGrid, grid_r3_0209_114, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00115.dat"); IS_LOC_STUFF OBS_LOC(100, -150) VIS_ORIENT(RADS_135) //107-100 (line at 146)
	memcpy (data[i].occupancyGrid, grid_r3_0209_115, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00116.dat"); IS_LOC_STUFF OBS_LOC(100, -100) VIS_ORIENT(RADS_135)
	memcpy (data[i].occupancyGrid, grid_r3_0209_116, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00117.dat"); IS_LOC_STUFF OBS_LOC(100, -50) VIS_ORIENT(RADS_135)
	memcpy (data[i].occupancyGrid, grid_r3_0209_117, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00118.dat"); IS_LOC_STUFF OBS_LOC(100, 0) VIS_ORIENT(RADS_135)
	memcpy (data[i].occupancyGrid, grid_r3_0209_118, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00119.dat"); IS_LOC_STUFF OBS_LOC(100, 50) VIS_ORIENT(RADS_135) //111
	memcpy (data[i].occupancyGrid, grid_r3_0209_119, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);























	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00120.dat"); IS_LOC_STUFF OBS_LOC(100, 100) VIS_ORIENT(RADS_135) //112 (line at 147) (frTopCorner at 75)
	memcpy (data[i].occupancyGrid, grid_r3_0209_120, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);  
	// Cannot see robot in 121
	// Images without locations start at 122
	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00122.dat"); NO_LOC_STUFF
	memcpy (data[i].occupancyGrid, grid_r3_0209_122, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00123.dat"); NO_LOC_STUFF
	memcpy (data[i].occupancyGrid, grid_r3_0209_123, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00124.dat"); NO_LOC_STUFF
	memcpy (data[i].occupancyGrid, grid_r3_0209_124, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00125.dat"); NO_LOC_STUFF
	memcpy (data[i].occupancyGrid, grid_r3_0209_125, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00126.dat"); NO_LOC_STUFF
	memcpy (data[i].occupancyGrid, grid_r3_0209_126, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00127.dat"); NO_LOC_STUFF
	memcpy (data[i].occupancyGrid, grid_r3_0209_127, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00128.dat"); NO_LOC_STUFF
	memcpy (data[i].occupancyGrid, grid_r3_0209_128, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00129.dat"); NO_LOC_STUFF
	memcpy (data[i].occupancyGrid, grid_r3_0209_129, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);








	++i;
	strcpy (data[i].imgName, "robot3_locRobot2_20130209\\img00130.dat"); NO_LOC_STUFF
	memcpy (data[i].occupancyGrid, grid_r3_0209_130, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);




#undef VIS_ORIENT
#undef OBS_LOC
#undef COG_OFFSET
#undef NO_LOC_STUFF
#undef IS_LOC_STUFF

#endif // ifdef USE_ROB3_IMAGES_FROM_0209




