#include "../../Common/RobotDefs.h"

#ifndef BOARD

/*!
\file RobotPartnerData.h
\brief Data on robots' partnes during coalitions.
*/
#ifndef PARTNER_DATA_H
#define PARTNER_DATA_H

#include "../../Common/RobotTypes.h"



//! Robot's data on other robots with which it is in coalition.
typedef struct PartnerData_
{
	int coalitionIter; //!< Iteration at which robot joined a coalition
	Coalition explorationCoalition; //!< Coalition with a supervisor robot.
	Coalition explorationCoalitionQueued; //!< If currently locked into a coalition, can "promise" to join another coalition.
	List supervisionCoalitions; //!< Coalitions with explorer robots.
} PartnerData;

//! Constructor
PartnerData initPartnerData();

//! Destructor. Free coalition structs.
void freePartnerData (PartnerData *r);


#endif // ifndef
#endif
