#include "../Common/RobotDefs.h"

#if defined(IS_WIN) && !defined(ROBOT)
/*!
\file Visualisation.h
\brief Visualisation of simulation and maps.
*/
#ifndef VISUALISATION_H
#define VISUALISATION_H


#include "Data/BoardDatabase.h"

//! Display robots, obstacles, etc in simulation
void Visualisation_showSim (BoardDatabase *db);

//! Display global map
void Visualisation_showMap (BoardDatabase *db);



#endif // VISUALISATION_H
#endif