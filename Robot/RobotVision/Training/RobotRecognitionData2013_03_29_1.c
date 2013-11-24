



#ifdef USE_ROB2_IMAGES_FROM_0329

#define IS_LOC_STUFF data[i].obs = 0; data[i].vis = 2; data[i].isLoc = 1;
#define NO_LOC_STUFF data[i].obs = 0; data[i].vis = 2; data[i].isLoc = 0;
#define COG_OFFSET 149.0f
#define OBS_LOC(X, Y) data[i].loc.x = -(X.0f + COG_OFFSET); data[i].loc.y = Y.0f;
#define VIS_ORIENT(X) data[i].orient = X;

	++i;
	strcpy (data[i].imgName, "robot2_locRobot4_20130329\\img00000.dat"); IS_LOC_STUFF OBS_LOC(100, 150) VIS_ORIENT(RADS_90) // orient=pi*.5 xdist=150 ydist=150
	memcpy (data[i].occupancyGrid, grid_r2_0329_1_00, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot2_locRobot4_20130329\\img00001.dat"); IS_LOC_STUFF OBS_LOC(100, 100) VIS_ORIENT(RADS_90)
	memcpy (data[i].occupancyGrid, grid_r2_0329_1_01, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot2_locRobot4_20130329\\img00002.dat"); IS_LOC_STUFF OBS_LOC(100, 50) VIS_ORIENT(RADS_90)
	memcpy (data[i].occupancyGrid, grid_r2_0329_1_02, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot2_locRobot4_20130329\\img00003.dat"); IS_LOC_STUFF OBS_LOC(100, 0) VIS_ORIENT(RADS_90)
	memcpy (data[i].occupancyGrid, grid_r2_0329_1_03, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot2_locRobot4_20130329\\img00004.dat"); IS_LOC_STUFF OBS_LOC(100, -50) VIS_ORIENT(RADS_90)
	memcpy (data[i].occupancyGrid, grid_r2_0329_1_04, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot2_locRobot4_20130329\\img00005.dat"); IS_LOC_STUFF OBS_LOC(100, -100) VIS_ORIENT(RADS_90)
	memcpy (data[i].occupancyGrid, grid_r2_0329_1_05, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot2_locRobot4_20130329\\img00006.dat"); IS_LOC_STUFF OBS_LOC(100, -150) VIS_ORIENT(RADS_90)
	memcpy (data[i].occupancyGrid, grid_r2_0329_1_06, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot2_locRobot4_20130329\\img00007.dat"); IS_LOC_STUFF OBS_LOC(200, -150) VIS_ORIENT(RADS_90)
	memcpy (data[i].occupancyGrid, grid_r2_0329_1_07, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot2_locRobot4_20130329\\img00008.dat"); IS_LOC_STUFF OBS_LOC(200, -100) VIS_ORIENT(RADS_90)
	memcpy (data[i].occupancyGrid, grid_r2_0329_1_08, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot2_locRobot4_20130329\\img00009.dat"); IS_LOC_STUFF OBS_LOC(200, -50) VIS_ORIENT(RADS_90)
	memcpy (data[i].occupancyGrid, grid_r2_0329_1_09, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);













	++i;
	strcpy (data[i].imgName, "robot2_locRobot4_20130329\\img00010.dat"); IS_LOC_STUFF OBS_LOC(200, 0) VIS_ORIENT(RADS_90)
	memcpy (data[i].occupancyGrid, grid_r2_0329_1_10, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot2_locRobot4_20130329\\img00011.dat"); IS_LOC_STUFF OBS_LOC(200, 50) VIS_ORIENT(RADS_90)
	memcpy (data[i].occupancyGrid, grid_r2_0329_1_11, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot2_locRobot4_20130329\\img00012.dat"); IS_LOC_STUFF OBS_LOC(200, 100) VIS_ORIENT(RADS_90)
	memcpy (data[i].occupancyGrid, grid_r2_0329_1_12, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot2_locRobot4_20130329\\img00013.dat"); IS_LOC_STUFF OBS_LOC(200, 150) VIS_ORIENT(RADS_90)
	memcpy (data[i].occupancyGrid, grid_r2_0329_1_13, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	// 14 same as 13
	++i;
	strcpy (data[i].imgName, "robot2_locRobot4_20130329\\img00015.dat"); IS_LOC_STUFF OBS_LOC(300, 150) VIS_ORIENT(RADS_90)
	memcpy (data[i].occupancyGrid, grid_r2_0329_1_15, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot2_locRobot4_20130329\\img00016.dat"); IS_LOC_STUFF OBS_LOC(300, 100) VIS_ORIENT(RADS_90)
	memcpy (data[i].occupancyGrid, grid_r2_0329_1_16, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot2_locRobot4_20130329\\img00017.dat"); IS_LOC_STUFF OBS_LOC(300, 50) VIS_ORIENT(RADS_90)
	memcpy (data[i].occupancyGrid, grid_r2_0329_1_17, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot2_locRobot4_20130329\\img00018.dat"); IS_LOC_STUFF OBS_LOC(300, 0) VIS_ORIENT(RADS_90)
	memcpy (data[i].occupancyGrid, grid_r2_0329_1_18, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot2_locRobot4_20130329\\img00019.dat"); IS_LOC_STUFF OBS_LOC(300, -50) VIS_ORIENT(RADS_90)
	memcpy (data[i].occupancyGrid, grid_r2_0329_1_19, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);














	++i;
	strcpy (data[i].imgName, "robot2_locRobot4_20130329\\img00020.dat"); IS_LOC_STUFF OBS_LOC(300, -100) VIS_ORIENT(RADS_90)
	memcpy (data[i].occupancyGrid, grid_r2_0329_1_20, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot2_locRobot4_20130329\\img00021.dat"); IS_LOC_STUFF OBS_LOC(300, -150) VIS_ORIENT(RADS_90)
	memcpy (data[i].occupancyGrid, grid_r2_0329_1_21, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	// 22 has garbage pixels and is the same as 21 anyway
	++i;
	strcpy (data[i].imgName, "robot2_locRobot4_20130329\\img00023.dat"); IS_LOC_STUFF OBS_LOC(150, 150) VIS_ORIENT(RADS_135) // orient=pi*.75 xdist=150 ydist=150
	memcpy (data[i].occupancyGrid, grid_r2_0329_1_23, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot2_locRobot4_20130329\\img00024.dat"); IS_LOC_STUFF OBS_LOC(150, 100) VIS_ORIENT(RADS_135)
	memcpy (data[i].occupancyGrid, grid_r2_0329_1_24, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot2_locRobot4_20130329\\img00025.dat"); IS_LOC_STUFF OBS_LOC(150, 50) VIS_ORIENT(RADS_135)
	memcpy (data[i].occupancyGrid, grid_r2_0329_1_25, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot2_locRobot4_20130329\\img00026.dat"); IS_LOC_STUFF OBS_LOC(150, 0) VIS_ORIENT(RADS_135)
	memcpy (data[i].occupancyGrid, grid_r2_0329_1_26, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot2_locRobot4_20130329\\img00027.dat"); IS_LOC_STUFF OBS_LOC(150, -50) VIS_ORIENT(RADS_135)
	memcpy (data[i].occupancyGrid, grid_r2_0329_1_27, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot2_locRobot4_20130329\\img00028.dat"); IS_LOC_STUFF OBS_LOC(150, -150) VIS_ORIENT(RADS_135) // looks like 150, -100 was skipped
	memcpy (data[i].occupancyGrid, grid_r2_0329_1_28, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	// 29 same as 28


























	++i;
	strcpy (data[i].imgName, "robot2_locRobot4_20130329\\img00030.dat"); IS_LOC_STUFF OBS_LOC(250, -150) VIS_ORIENT(RADS_135) // dist changed
	memcpy (data[i].occupancyGrid, grid_r2_0329_1_30, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot2_locRobot4_20130329\\img00031.dat"); IS_LOC_STUFF OBS_LOC(250, -100) VIS_ORIENT(RADS_135)
	memcpy (data[i].occupancyGrid, grid_r2_0329_1_31, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot2_locRobot4_20130329\\img00032.dat"); IS_LOC_STUFF OBS_LOC(250, -50) VIS_ORIENT(RADS_135)
	memcpy (data[i].occupancyGrid, grid_r2_0329_1_32, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot2_locRobot4_20130329\\img00033.dat"); IS_LOC_STUFF OBS_LOC(250, 0) VIS_ORIENT(RADS_135)
	memcpy (data[i].occupancyGrid, grid_r2_0329_1_33, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot2_locRobot4_20130329\\img00034.dat"); IS_LOC_STUFF OBS_LOC(250, 50) VIS_ORIENT(RADS_135)
	memcpy (data[i].occupancyGrid, grid_r2_0329_1_34, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot2_locRobot4_20130329\\img00035.dat"); IS_LOC_STUFF OBS_LOC(250, 100) VIS_ORIENT(RADS_135)
	memcpy (data[i].occupancyGrid, grid_r2_0329_1_35, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot2_locRobot4_20130329\\img00036.dat"); IS_LOC_STUFF OBS_LOC(250, 150) VIS_ORIENT(RADS_135)
	memcpy (data[i].occupancyGrid, grid_r2_0329_1_36, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot2_locRobot4_20130329\\img00037.dat"); IS_LOC_STUFF OBS_LOC(350, 150) VIS_ORIENT(RADS_135) // dist changed!
	memcpy (data[i].occupancyGrid, grid_r2_0329_1_37, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot2_locRobot4_20130329\\img00038.dat"); IS_LOC_STUFF OBS_LOC(350, 100) VIS_ORIENT(RADS_135)
	memcpy (data[i].occupancyGrid, grid_r2_0329_1_38, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot2_locRobot4_20130329\\img00039.dat"); IS_LOC_STUFF OBS_LOC(350, 50) VIS_ORIENT(RADS_135)
	memcpy (data[i].occupancyGrid, grid_r2_0329_1_39, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);




















	++i;
	strcpy (data[i].imgName, "robot2_locRobot4_20130329\\img00040.dat"); IS_LOC_STUFF OBS_LOC(350, 0) VIS_ORIENT(RADS_135)
	memcpy (data[i].occupancyGrid, grid_r2_0329_1_40, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot2_locRobot4_20130329\\img00041.dat"); IS_LOC_STUFF OBS_LOC(350, -50) VIS_ORIENT(RADS_135)
	memcpy (data[i].occupancyGrid, grid_r2_0329_1_41, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	// 42 and 43 are junk images - of size 0




#undef VIS_ORIENT
#undef OBS_LOC
#undef COG_OFFSET
#undef NO_LOC_STUFF
#undef IS_LOC_STUFF

#endif // ifdef USE_ROB2_IMAGES_FROM_0329




