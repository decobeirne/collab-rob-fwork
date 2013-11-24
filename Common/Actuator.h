#include "RobotDefs.h"

/*!
\file Actuator.h
\brief Control robot actuators.
*/
#ifndef ACTUATOR_H
#define ACTUATOR_H

#include "BaseTypes.h"
#include "RobotTypes.h"
#include "Image.h"

void Actuator_setupIkConstants (
	const GeometryConstants *geometryConstants,
	IkConstants *ik);

void Actuator_executeMove (
	Pose *pose,
	const Move move,
	const int isExecuted,
	const int iteration,
	FILE *xmlLog);

extern int G_COOP_LOC_DONT_CHECK_LOS;

//! Check if movement from one pose to the next will result in a collision
int checkMove (
	const PointF loc1,
	const PointF loc2,
	const Image *navMap,
	const int navigation,
	const int localMapEdge,
	const int allowOutsideLocalMap);

#endif // ifndef

