#include "../Common/RobotDefs.h"

#if defined(IS_WIN) && !defined(ROBOT)

#include "Visualisation.h"
#include "Map/BoardMapProcessing.h"
#include "../Common/Geometry.h"









void Visualisation_displayPoseCov (BoardDatabase *db, Image *img, const Pose *pose)
{
	PointF evec0, evec1;
	float evals[2];
	PointF corners[5];
	const float robL = 5.0f;
	const float robW = 4.0f;
	const int colourCov = 120;
	const int colourLoc = 200;

	if (MIN_FLT == pose->loc.x)
	{
		return;
	}

	// Display robot.
	{
		// Front corners.
		Geometry_rotatePoint (
			pose->orient,
			-robW,
			robL,
			&corners[0].x,
			&corners[0].y);
		Geometry_rotatePoint (
			pose->orient,
			robW,
			robL,
			&corners[1].x,
			&corners[1].y);
		Geometry_rotatePoint (
			pose->orient,
			0,
			robL,
			&corners[4].x,
			&corners[4].y);

		corners[0].x += pose->loc.x;
		corners[1].x += pose->loc.x;
		corners[4].x += pose->loc.x;

		corners[0].y += pose->loc.y;
		corners[1].y += pose->loc.y;
		corners[4].y += pose->loc.y;

		// Back corners.
		Geometry_rotatePoint (
			Geometry_orientSum (pose->orient, PI),
			-robW,
			robL,
			&corners[2].x,
			&corners[2].y);
		Geometry_rotatePoint (
			Geometry_orientSum (pose->orient, PI),
			robW,
			robL,
			&corners[3].x, 
			&corners[3].y);

		corners[2].x += pose->loc.x;
		corners[3].x += pose->loc.x;

		corners[2].y += pose->loc.y;
		corners[3].y += pose->loc.y;

		Image_drawLine (
			img,
			(int)corners[0].x,
			(int)corners[0].y,
			(int)corners[1].x,
			(int)corners[1].y,
			colourLoc,0);
		Image_drawLine (
			img,
			(int)corners[1].x,
			(int)corners[1].y,
			(int)corners[2].x,
			(int)corners[2].y,
			colourLoc,0);
		Image_drawLine (
			img,
			(int)corners[2].x,
			(int)corners[2].y,
			(int)corners[3].x,
			(int)corners[3].y,
			colourLoc,0);
		Image_drawLine (
			img,
			(int)corners[0].x,
			(int)corners[0].y,
			(int)corners[3].x,
			(int)corners[3].y,
			colourLoc,0);

		// Line from robots centre of gravity along line of sight.
		Image_drawLine (
			img,
			(int)corners[4].x,
			(int)corners[4].y,
			(int)pose->loc.x,
			(int)pose->loc.y,
			colourLoc,0);
	}

	// Display covariance.
	{
		evec0 = pose->evec;
		evec1.x = -evec0.y;
		evec1.y = evec0.x;
		evec1.x = evec1.x * evec1.x;
		evec1.y = evec1.y * evec1.y;
		evals[0] = pose->evals[0] * pose->evals[0];
		evals[1] = pose->evals[1] * pose->evals[1];

		corners[0].x = pose->loc.x + evals[0] * evec0.x;
		corners[0].y = pose->loc.y + evals[0] * evec0.y;
		Image_setPixel_check (img, (int)corners[0].x, (int)corners[0].y, colourCov);

		corners[1].x = pose->loc.x - evals[0] * evec0.x;
		corners[1].y = pose->loc.y - evals[0] * evec0.y;
		Image_setPixel_check (img, (int)corners[1].x, (int)corners[1].y, colourCov);
		//Image_drawLine (img, (int)corners[0].x, (int)corners[0].y, (int)corners[1].x, (int)corners[1].y, colourCov, 0);

		corners[0].x = pose->loc.x + evals[1] * evec1.x;
		corners[0].y = pose->loc.y + evals[1] * evec1.y;
		Image_setPixel_check (img, (int)corners[0].x, (int)corners[0].y, colourCov);

		corners[1].x = pose->loc.x - evals[1] * evec1.x;
		corners[1].y = pose->loc.y - evals[1] * evec1.y;
		Image_setPixel_check (img, (int)corners[1].x, (int)corners[1].y, colourCov);
		//Image_drawLine (img, (int)corners[0].x, (int)corners[0].y, (int)corners[1].x, (int)corners[1].y, colourCov, 0);
	}
}

#ifdef SIM_ERROR
void Visualisation_displayActualLoc (BoardDatabase *db,
									 Image *img,
									 const Pose pose,
									 const PointF locOffset)
{
	PointF pt;
	PointI pti;
	const int colour = 0;

	pt.x = pose.loc.x + locOffset.x;
	pt.y = pose.loc.y + locOffset.y;
	pti = PointF_toPointI (pt);

	Image_setPixel_check (img, pti.x, pti.y, colour);
}
#endif

//! Display a robot's target point and link it to the robot
void Visualisation_displayTarget (BoardDatabase *db,
								  Image *img,
								  const PointF target,
								  const PointF loc,
								  const BEHAVIOUR behaviour)
{
	PointI targetI;
	PointI locI;
	int targetBoxLen;

	if (MIN_FLT == target.x ||
		MIN_FLT == target.y)
	{
		return;
	}

	targetI = PointF_toPointI (target);
	locI = PointF_toPointI (loc);

	Image_setPixel_check (img, targetI.x, targetI.y, 200);

	Image_drawLine (img, targetI.x, targetI.y, locI.x, locI.y, 200,0);

	if (EXPLORATION == behaviour || GOTO_EXP_PT == behaviour)
	{
		if (EXPLORATION == behaviour)
		{
			targetBoxLen = EXP_AREA/2;
		}
		else
		{
			targetBoxLen = LOC_MAP_DIMS/2;
		}

		Image_drawRect (
			img,
			targetI.x - targetBoxLen,
			targetI.y - targetBoxLen,
			targetI.x + targetBoxLen,
			targetI.y + targetBoxLen,
			200,
			0);
	}
}


//! Display robots sensor region - check for initial minimum values
void Visualisation_displayLocalMap (BoardDatabase *db,
									Image *img,
									const PointI localMapOrigin)
{
	Image_drawRect (
		img,
		localMapOrigin.x,
		localMapOrigin.y,
		localMapOrigin.x + LOC_MAP_DIMS,
		localMapOrigin.y + LOC_MAP_DIMS,
		240,
		0);
}

extern int simWindowSize[4];
extern int mapWindowSize[4];

void Visualisation_showSim (BoardDatabase *db)
{
	int i;
	Image *simHandle;

	BoardMapProcessing_refreshSim (db);

	simHandle = db->environment.tempMap;

	for (i = 0; i < N_ROBOTS; ++i)
	{
		Visualisation_displayLocalMap (
			db,
			simHandle,
			db->groupData.robots[i].localMapOrigin);
	}

	for (i = 0; i < N_ROBOTS; ++i)
	{
		Visualisation_displayTarget (
			db,
			simHandle,
			db->groupData.robots[i].target,
			db->groupData.robots[i].pose.loc,
			db->groupData.robots[i].behaviour);
	}

	for (i = 0; i < N_ROBOTS; ++i)
	{
		Visualisation_displayPoseCov (
			db,
			simHandle,
			&db->groupData.robots[i].pose);
	}

#ifdef SIM_ERROR
	for (i = 0; i < N_ROBOTS; ++i)
	{
		Visualisation_displayActualLoc (
			db,
			simHandle,
			db->groupData.robots[i].pose,
			db->groupData.robots[i].actualLocOffset);
	}
#endif

	Image_show_givenIplImage (
		simHandle,
		(schar*)"sim",
		db->globMapIplImage,
		simWindowSize);
}

void Visualisation_showMap (BoardDatabase *db)
{
	Image_show_givenIplImage (
		db->environment.map,
		(schar*)"map",
		db->globMapIplImage,
		mapWindowSize);

}

#endif



