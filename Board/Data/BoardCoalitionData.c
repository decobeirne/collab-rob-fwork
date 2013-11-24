#include "../../Common/RobotDefs.h"

#ifndef ROBOT
#include "BoardCoalitionData.h"

BoardCoalitionData initBoardCoalitionData()
{
	int i;
	BoardCoalitionData c;

	c.coalitionIndex = 0;
	c.bids = initList();
	c.proposals = initList();
	c.coalitions = initList();

	for (i = 0; i < N_ROBOTS; ++i)
	{
		c.isBidSuccessful[i] = 0;
		c.isPropSuccessful[i] = 0;
	}
	return c;
}

void freeBoardCoalitionData (BoardCoalitionData *c)
{
	List_clear (&c->bids, 1);
	List_clear (&c->proposals, 1);
	List_clear (&c->coalitions, 1);
}

#endif
