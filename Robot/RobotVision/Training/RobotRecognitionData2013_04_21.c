



#ifdef USE_ROB2_IMAGES_FROM_0421
#define NO_LOC_STUFF data[i].obs = 0; data[i].vis = 1; data[i].isLoc = 0;


	++i;
	strcpy (data[i].imgName, "robot2_newColours_20130421\\img00000.dat"); NO_LOC_STUFF
	memcpy (data[i].occupancyGrid, grid_r2_0421_00, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot2_newColours_20130421\\img00001.dat"); NO_LOC_STUFF
	memcpy (data[i].occupancyGrid, grid_r2_0421_01, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot2_newColours_20130421\\img00002.dat"); NO_LOC_STUFF
	memcpy (data[i].occupancyGrid, grid_r2_0421_02, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot2_newColours_20130421\\img00003.dat"); NO_LOC_STUFF
	memcpy (data[i].occupancyGrid, grid_r2_0421_03, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	// 4 same as 3
	++i;
	strcpy (data[i].imgName, "robot2_newColours_20130421\\img00005.dat"); NO_LOC_STUFF
	memcpy (data[i].occupancyGrid, grid_r2_0421_05, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot2_newColours_20130421\\img00006.dat"); NO_LOC_STUFF
	memcpy (data[i].occupancyGrid, grid_r2_0421_06, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot2_newColours_20130421\\img00007.dat"); NO_LOC_STUFF
	memcpy (data[i].occupancyGrid, grid_r2_0421_07, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot2_newColours_20130421\\img00008.dat"); NO_LOC_STUFF
	memcpy (data[i].occupancyGrid, grid_r2_0421_08, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot2_newColours_20130421\\img00009.dat"); NO_LOC_STUFF
	memcpy (data[i].occupancyGrid, grid_r2_0421_09, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot2_newColours_20130421\\img00010.dat"); NO_LOC_STUFF
	memcpy (data[i].occupancyGrid, grid_r2_0421_10, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot2_newColours_20130421\\img00011.dat"); NO_LOC_STUFF
	memcpy (data[i].occupancyGrid, grid_r2_0421_11, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);



#undef NO_LOC_STUFF
#endif // ifdef USE_ROB2_IMAGES_FROM_0421


#ifdef USE_ROB3_IMAGES_FROM_0421
#define NO_LOC_STUFF data[i].obs = 1; data[i].vis = 0; data[i].isLoc = 0;


	// 0 was garbage
	++i;
	strcpy (data[i].imgName, "robot3_newColours_20130421\\img00001.dat"); NO_LOC_STUFF
	memcpy (data[i].occupancyGrid, grid_r3_0421_01, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot3_newColours_20130421\\img00002.dat"); NO_LOC_STUFF
	memcpy (data[i].occupancyGrid, grid_r3_0421_02, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot3_newColours_20130421\\img00003.dat"); NO_LOC_STUFF
	memcpy (data[i].occupancyGrid, grid_r3_0421_03, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot3_newColours_20130421\\img00004.dat"); NO_LOC_STUFF
	memcpy (data[i].occupancyGrid, grid_r3_0421_04, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot3_newColours_20130421\\img00005.dat"); NO_LOC_STUFF
	memcpy (data[i].occupancyGrid, grid_r3_0421_05, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot3_newColours_20130421\\img00006.dat"); NO_LOC_STUFF
	memcpy (data[i].occupancyGrid, grid_r3_0421_06, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	++i;
	strcpy (data[i].imgName, "robot3_newColours_20130421\\img00007.dat"); NO_LOC_STUFF
	memcpy (data[i].occupancyGrid, grid_r3_0421_07, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	// 8 too dark
	++i;
	strcpy (data[i].imgName, "robot3_newColours_20130421\\img00009.dat"); NO_LOC_STUFF
	memcpy (data[i].occupancyGrid, grid_r3_0421_09, CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	// 10 too dark



#undef NO_LOC_STUFF
#endif // ifdef USE_ROB3_IMAGES_FROM_0421


