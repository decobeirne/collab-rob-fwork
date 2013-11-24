#include "RobotPartnerData.h"

#ifndef BOARD


PartnerData initPartnerData()
{
	PartnerData r;
	r.coalitionIter = MAX_INT32;
	r.explorationCoalition = initCoalition(-1);
	r.explorationCoalitionQueued = initCoalition(-1);
	r.supervisionCoalitions = initList();

	return r;
}

void freePartnerData (PartnerData *r)
{
	List_clear (&r->supervisionCoalitions, 1);
}
#endif


