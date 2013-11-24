



#ifdef USE_ROB4_IMAGES_FROM_0329

#define IS_LOC_STUFF data[i].obs = 2; data[i].vis = 1; data[i].isLoc = 1;
#define NO_LOC_STUFF data[i].obs = 2; data[i].vis = 1; data[i].isLoc = 0;
#define COG_OFFSET 149.0f
#define OBS_LOC(X, Y) data[i].loc.x = -(X.0f + COG_OFFSET); data[i].loc.y = Y.0f;
#define VIS_ORIENT(X) data[i].orient = X;

	++i;
	strcpy (data[i].imgName, "robot4_locRobot3_20130329\\img00000.dat"); IS_LOC_STUFF OBS_LOC(150, 150) VIS_ORIENT(PI)
	memcpy (data[i].occupancyGrid, grid_r4_0329_00, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot4_locRobot3_20130329\\img00001.dat"); IS_LOC_STUFF OBS_LOC(150, 100) VIS_ORIENT(PI)
	memcpy (data[i].occupancyGrid, grid_r4_0329_01, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot4_locRobot3_20130329\\img00002.dat"); IS_LOC_STUFF OBS_LOC(150, 50) VIS_ORIENT(PI)
	memcpy (data[i].occupancyGrid, grid_r4_0329_02, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot4_locRobot3_20130329\\img00003.dat"); IS_LOC_STUFF OBS_LOC(150, 0) VIS_ORIENT(PI)
	memcpy (data[i].occupancyGrid, grid_r4_0329_03, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot4_locRobot3_20130329\\img00004.dat"); IS_LOC_STUFF OBS_LOC(150, -50) VIS_ORIENT(PI)
	memcpy (data[i].occupancyGrid, grid_r4_0329_04, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot4_locRobot3_20130329\\img00005.dat"); IS_LOC_STUFF OBS_LOC(150, -100) VIS_ORIENT(RADS_90)
	memcpy (data[i].occupancyGrid, grid_r4_0329_05, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	// at 150,-150, but can't really see any colours in image - cells only half covered
	++i;
	strcpy (data[i].imgName, "robot4_locRobot3_20130329\\img00007.dat"); IS_LOC_STUFF OBS_LOC(250, -150) VIS_ORIENT(RADS_90) // dist changed!
	memcpy (data[i].occupancyGrid, grid_r4_0329_07, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot4_locRobot3_20130329\\img00008.dat"); IS_LOC_STUFF OBS_LOC(250, -100) VIS_ORIENT(RADS_90)
	memcpy (data[i].occupancyGrid, grid_r4_0329_08, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot4_locRobot3_20130329\\img00009.dat"); IS_LOC_STUFF OBS_LOC(250, -50) VIS_ORIENT(RADS_90)
	memcpy (data[i].occupancyGrid, grid_r4_0329_09, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);













	++i;
	strcpy (data[i].imgName, "robot4_locRobot3_20130329\\img00010.dat"); IS_LOC_STUFF OBS_LOC(250, 0) VIS_ORIENT(RADS_90)
	memcpy (data[i].occupancyGrid, grid_r4_0329_10, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot4_locRobot3_20130329\\img00011.dat"); IS_LOC_STUFF OBS_LOC(250, 50) VIS_ORIENT(RADS_90)
	memcpy (data[i].occupancyGrid, grid_r4_0329_11, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot4_locRobot3_20130329\\img00012.dat"); IS_LOC_STUFF OBS_LOC(250, 100) VIS_ORIENT(RADS_90)
	memcpy (data[i].occupancyGrid, grid_r4_0329_12, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot4_locRobot3_20130329\\img00013.dat"); IS_LOC_STUFF OBS_LOC(250, 150) VIS_ORIENT(RADS_90)
	memcpy (data[i].occupancyGrid, grid_r4_0329_13, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot4_locRobot3_20130329\\img00014.dat"); IS_LOC_STUFF OBS_LOC(250, 150) VIS_ORIENT(RADS_135) // orient changed
	memcpy (data[i].occupancyGrid, grid_r4_0329_14, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot4_locRobot3_20130329\\img00015.dat"); IS_LOC_STUFF OBS_LOC(250, 100) VIS_ORIENT(RADS_135)
	memcpy (data[i].occupancyGrid, grid_r4_0329_15, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot4_locRobot3_20130329\\img00016.dat"); IS_LOC_STUFF OBS_LOC(250, 50) VIS_ORIENT(RADS_135)
	memcpy (data[i].occupancyGrid, grid_r4_0329_16, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot4_locRobot3_20130329\\img00017.dat"); IS_LOC_STUFF OBS_LOC(250, 0) VIS_ORIENT(RADS_135)
	memcpy (data[i].occupancyGrid, grid_r4_0329_17, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot4_locRobot3_20130329\\img00018.dat"); IS_LOC_STUFF OBS_LOC(250, -50) VIS_ORIENT(RADS_135)
	memcpy (data[i].occupancyGrid, grid_r4_0329_18, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot4_locRobot3_20130329\\img00019.dat"); IS_LOC_STUFF OBS_LOC(250, -100) VIS_ORIENT(RADS_135)
	memcpy (data[i].occupancyGrid, grid_r4_0329_19, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);














	++i;
	strcpy (data[i].imgName, "robot4_locRobot3_20130329\\img00020.dat"); IS_LOC_STUFF OBS_LOC(250, -150) VIS_ORIENT(RADS_135)
	memcpy (data[i].occupancyGrid, grid_r4_0329_20, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot4_locRobot3_20130329\\img00021.dat"); IS_LOC_STUFF OBS_LOC(150, -150) VIS_ORIENT(RADS_135)
	memcpy (data[i].occupancyGrid, grid_r4_0329_21, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot4_locRobot3_20130329\\img00022.dat"); IS_LOC_STUFF OBS_LOC(150, -100) VIS_ORIENT(RADS_135)
	memcpy (data[i].occupancyGrid, grid_r4_0329_22, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot4_locRobot3_20130329\\img00023.dat"); IS_LOC_STUFF OBS_LOC(150, -50) VIS_ORIENT(RADS_135)
	memcpy (data[i].occupancyGrid, grid_r4_0329_23, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot4_locRobot3_20130329\\img00024.dat"); IS_LOC_STUFF OBS_LOC(150, 0) VIS_ORIENT(RADS_135)
	memcpy (data[i].occupancyGrid, grid_r4_0329_24, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot4_locRobot3_20130329\\img00025.dat"); IS_LOC_STUFF OBS_LOC(150, 50) VIS_ORIENT(RADS_135)
	memcpy (data[i].occupancyGrid, grid_r4_0329_25, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot4_locRobot3_20130329\\img00026.dat"); IS_LOC_STUFF OBS_LOC(150, 100) VIS_ORIENT(RADS_135)
	memcpy (data[i].occupancyGrid, grid_r4_0329_26, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot4_locRobot3_20130329\\img00027.dat"); IS_LOC_STUFF OBS_LOC(150, 150) VIS_ORIENT(RADS_135)
	memcpy (data[i].occupancyGrid, grid_r4_0329_27, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);




#undef VIS_ORIENT
#undef OBS_LOC
#undef COG_OFFSET
#undef NO_LOC_STUFF
#undef IS_LOC_STUFF

#endif // ifdef USE_ROB4_IMAGES_FROM_0329




