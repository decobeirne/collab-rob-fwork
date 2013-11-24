#include "../Common/RobotDefs.h"

#ifndef ROBOT
#include "BoardManagement.h"


void BoardManagement_checkTimeElapsed (BoardDatabase *db)
{
	static int printedTimeElapsed = 0;
	int i;
	int nStuck = 0;

	if (!db->groupData.experimentStatus.isTimeElapsed)
	{
		for (i = 0; i < N_ROBOTS; ++i)
		{
			nStuck += (int)(db->groupData.robots[i].behaviour == STUCK);
		}
		db->groupData.experimentStatus.isTimeElapsed = (int)(nStuck == N_ROBOTS);
	}
	if (db->groupData.experimentStatus.isTimeElapsed)
	{
		for (i = 0; i < N_ROBOTS; ++i)
		{
			db->sensorData.statusFromBoard[i].isTimeElapsed = 1;
		}
#ifdef PRINT_EVENTS
		if (!printedTimeElapsed)
		{
			fprintf (db->xmlLog, "<TimeElapsed />\n");
			printedTimeElapsed = 1;
		}
#endif
	}
}

void BoardManagement_checkMissionStatus (BoardDatabase *db)
{
#ifdef BOARD
	enterBoardData();
#endif

#ifdef PRINT_EVENTS
	BoardDatabase_printStatus (db);
#endif

	BoardManagement_checkTimeElapsed (db);

#ifdef BOARD
	leaveBoardData();
#endif
}

//! Set flag in grid to indicate coalition between supervisor and explorer.
void allocateCoalition (BoardDatabase *db, Proposal *proposal, Bid *bid)
{
	Coalition *ptr;
	Coalition coalition = initCoalition (db->coalitionData.coalitionIndex);
//	coalition.pt = proposal->pt;
	coalition.area = proposal->area;
	coalition.stdDev = proposal->stdDev;
	coalition.supervisor = bid->supervisor;
	coalition.explorer = bid->explorer;
//	coalition.bid = bid->bid;

	ptr = (Coalition*)malloc (sizeof (Coalition));
	memcpy (ptr, &coalition, sizeof (Coalition));
	List_pushValue (&db->coalitionData.coalitions, ptr);

#ifdef PRINT_PROFIT
//	fprintf (db->xmlLog, "<AllocateCoalition>coalitionIndex=%d supervisor=%d explorer=%d bid=%f</AllocateCoalition>\n",
	fprintf (db->xmlLog, "<AllocateCoalition>coalitionIndex=%d supervisor=%d explorer=%d</AllocateCoalition>\n",
		db->coalitionData.coalitionIndex,
		bid->supervisor,
		bid->explorer);//,
//		bid->bid);
#endif

	++db->coalitionData.coalitionIndex;
}

void failProposal (BoardDatabase *db, Proposal *proposal)
{
#ifdef PRINT_PROFIT
	fprintf (db->xmlLog, "<ProposalFailed>supervisor=%d</ProposalFailed>\n", proposal->supervisor);
#endif
}

//! Remove existing coalitions in which the robot of given id is a member. May select only coalitions where it was an explorer, or where it was explorer or supervisor
void tidyCoalitions (BoardDatabase *db, const int id, const int explorerOrSupervisor)
{
	int removeCoalition;
	ListNode *iter;
	Coalition *coalition;

	iter = db->coalitionData.coalitions.front;
	while (iter)
	{
		coalition = (Coalition*)iter->value;
		removeCoalition = 0;

		if (coalition->explorer == id)
		{
			removeCoalition = 1;
#ifdef PRINT_EVENTS
			fprintf (db->xmlLog, "<RemoveCoalition>coalition=%d explorer=%d</RemoveCoalition>\n", coalition->id, id);
#endif
		}
		if (coalition->supervisor == id && explorerOrSupervisor)
		{
			removeCoalition = 1;
#ifdef PRINT_EVENTS
			fprintf (db->xmlLog, "<RemoveCoalition>coalition=%d supervisor=%d</RemoveCoalition>\n", coalition->id, id);
#endif
		}

		if (removeCoalition)
		{
			List_deleteElement (&db->coalitionData.coalitions, &iter, 1, 1);
		}
		else
		{
			iter = iter->next;
		}
	}
}

//! Remove open proposals made by this robot. May just have had a bid accepted on another proposal. Also remove bids made on this
void tidyProposals (BoardDatabase *db, const int robotId)
{
	ListNode *iter;
	Proposal *proposal;
	iter = db->coalitionData.proposals.front;
	while (iter)
	{
		proposal = (Proposal*)iter->value;
		if (proposal->supervisor == robotId)
		{
#ifdef PRINT_EVENTS
			fprintf (db->xmlLog, "<RemoveProposal>robot=%d</RemoveProposal>\n", robotId);
#endif
			List_deleteElement (&db->coalitionData.proposals, &iter, 1, 1);
		}
		else
		{
			iter = iter->next;
		}
	}
}

//! Remove any bids matching the given robot id. May match against supervisor or explorer in bid.
void tidyBids (BoardDatabase *db, const int robotId, const int matchExplorer)
{
	ListNode *iter;
	Bid *bid;
	int ids[2];
	iter = db->coalitionData.bids.front;
	while (iter)
	{
		bid = (Bid*)iter->value;
		ids[0] = bid->supervisor;
		ids[1] = bid->explorer;

		if (robotId == ids[matchExplorer])
		{
#ifdef PRINT_EVENTS
//			fprintf (db->xmlLog, "<RemoveBid>explorer=%d supervisor=%d bid=%f</RemoveBid>\n", robotId, bid->explorer, bid->bid);
			fprintf (db->xmlLog, "<RemoveBid>explorer=%d supervisor=%d</RemoveBid>\n", robotId, bid->explorer);
#endif
			List_deleteElement (&db->coalitionData.bids, &iter, 1, 1);
		}
		else
		{
			iter = iter->next;
		}
	}
}

void setSuccessFlags (BoardDatabase *db, const Bid bid)
{
	db->coalitionData. isPropSuccessful[bid.supervisor] = 1;
	db->coalitionData.isBidSuccessful[bid.explorer] = 1;
}

int countNRobotsConsideredProposal (const Proposal *proposal)
{
	int i;
	int n = 0;
	for (i = 0; i < N_ROBOTS; ++i)
	{
		n += proposal->considered[i];
	}
	return n;
}

void BoardManagement_manageCoalitions (BoardDatabase *db)
{
	ListNode *iter;
	ListNode *bidIter;
	Proposal *proposal;
	Bid *bid;
	time_t currentTime;
	Bid *minBid;
	Bid winningBid;
	int nRobotsConsideredProposal, successful;

	memset (db->coalitionData.isBidSuccessful, 0, 8);
	memset (db->coalitionData.isPropSuccessful, 0, 8);

	if (db->coalitionData.proposals.size)
	{
		fprintf (db->xmlLog, "<ProcessingProposals>\n");

		iter = db->coalitionData.proposals.front;
		while (iter)
		{
			proposal = (Proposal*)iter->value;
			fprintf (db->xmlLog, "<Prop>area=(%d,%d) stdDev=%f estCloseLoopProfit=%f sup=%d</Prop>\n",
				proposal->area.x, proposal->area.y, proposal->stdDev, proposal->estCloseLoopProfit, proposal->supervisor);

			iter = iter->next;
		}

		fprintf (db->xmlLog, "</ProcessingProposals>\n");
	}

	iter = db->coalitionData.proposals.front;
	while (iter)
	{
		currentTime = time (&currentTime);
		proposal = (Proposal*)iter->value;

		nRobotsConsideredProposal = countNRobotsConsideredProposal (proposal);
		if (currentTime - proposal->postedTime > 10 || nRobotsConsideredProposal == N_ROBOTS - 1)
		{
#ifdef PRINT_EVENTS
			fprintf (db->xmlLog, "<ProcessProposal>reason=");
			if (currentTime - proposal->postedTime > 10)
			{
				fprintf (db->xmlLog, "\"timeout\"");
			}
			else
			{
				fprintf (db->xmlLog, "\"seenByAll\"");
			}
			fprintf (db->xmlLog, " supervisor=%d</ProcessProposal>\n", proposal->supervisor);
#endif

			minBid = 0;
			successful = 0;
			bidIter = db->coalitionData.bids.front;
			while (bidIter)
			{
				bid = (Bid*)bidIter->value;
				if (bid->supervisor == proposal->supervisor && (!minBid || bid->nSteps < minBid->nSteps))
				{
					minBid = bid;
				}
				bidIter = bidIter->next;
			}

			if (minBid)
			{
				winningBid = *minBid; // Copy bid so we can delete from list.

				tidyCoalitions (db, winningBid.explorer, 1);
				tidyCoalitions (db, proposal->supervisor, 0);
				allocateCoalition (db, proposal, &winningBid);

				tidyProposals (db, winningBid.explorer);
				// The supervisors proposal is deleted below (regardless of whether or not it was succesful)

				tidyBids (db, winningBid.explorer, 1); // Bids made by this explorer.
				tidyBids (db, winningBid.explorer, 0); // Bids on proposal by explorer.
				tidyBids (db, proposal->supervisor, 1); // Bids made by this supervisor.
				tidyBids (db, proposal->supervisor, 0); // Bids on proposal by supervisor.

				setSuccessFlags (db, winningBid);
				successful = 1;

#ifdef PRINT_EVENTS
				fprintf (db->xmlLog, "<ProposalResult>result=\"success\"");
//				fprintf (db->xmlLog, " explorer=%d supervisor=%d nSteps=%f nExpCellsMapped=%d stdDev=%f</ProposalResult>\n",
				fprintf (db->xmlLog, " explorer=%d supervisor=%d nSteps=%f stdDev=%f</ProposalResult>\n",
					winningBid.explorer,
					proposal->supervisor,
					winningBid.nSteps,
//					proposal->expCellsMapped,
					proposal->stdDev);
#endif
			}
			else
			{
#ifdef PRINT_EVENTS
				fprintf (db->xmlLog, "<ProposalResult>result=\"fail\" reason=\"noBids\"</ProposalResult>\n");
#endif
			}

			if (!successful)
			{
				tidyBids (db, proposal->supervisor, 0); // Bids on proposal by supervisor.
				failProposal (db, proposal);
			}

			List_deleteElement (&db->coalitionData.proposals, &iter, 1, 1);
		}
		else
		{
			iter = iter->next;
		}
	}
}
#if 0
void BoardManagement_manageCoalitions_OLD (BoardDatabase *db)
{
	ListNode *iter;
	ListNode *bidIter;
	Proposal *proposal;
	Bid *bid;
	time_t currentTime;
	Bid *maxBid;
	Bid winningBid;
	float bidRatio;
	int nRobotsConsideredProposal, successful;

	memset (db->coalitionData.isBidSuccessful, 0, 8);
	memset (db->coalitionData.isPropSuccessful, 0, 8);

	iter = db->coalitionData.proposals.front;
	while (iter)
	{
		currentTime = time (&currentTime);
		proposal = (Proposal*)iter->value;

		nRobotsConsideredProposal = countNRobotsConsideredProposal (proposal);
		if (currentTime - proposal->postedTime > 10 || nRobotsConsideredProposal == N_ROBOTS - 1)
		{
#ifdef PRINT_EVENTS
			fprintf (db->xmlLog, "<ProcessProposal>reason=");
			if (currentTime - proposal->postedTime > 10)
			{
				fprintf (db->xmlLog, "\"timeout\"");
			}
			else
			{
				fprintf (db->xmlLog, "\"seenByAll\"");
			}
			fprintf (db->xmlLog, " supervisor=%d</ProcessProposal>\n", proposal->supervisor);
#endif

			maxBid = 0;
			successful = 0;
			bidIter = db->coalitionData.bids.front;
			while (bidIter)
			{
				bid = (Bid*)bidIter->value;
				if (bid->supervisor == proposal->supervisor && (!maxBid || bid->bid > maxBid->bid))
				{
					maxBid = bid;
				}
				bidIter = bidIter->next;
			}

			if (maxBid)
			{
				// Determine if the supervisor should accept any bid.
				bidRatio = (proposal->totalProfit * maxBid->bid) / proposal->supervisorResources;

				if (bidRatio > proposal->supervisorReserve)
				{
					winningBid = *maxBid; // Copy bid so we can delete from list.

					tidyCoalitions (db, winningBid.explorer, 1);
					tidyCoalitions (db, proposal->supervisor, 0);
					allocateCoalition (db, proposal, &winningBid);

					tidyProposals (db, winningBid.explorer);
					// The supervisors proposal is deleted below (regardless of whether or not it was succesful)

					tidyBids (db, winningBid.explorer, 1); // Bids made by this explorer.
					tidyBids (db, winningBid.explorer, 0); // Bids on proposal by explorer.
					tidyBids (db, proposal->supervisor, 1); // Bids made by this supervisor.
					tidyBids (db, proposal->supervisor, 0); // Bids on proposal by supervisor.

					setSuccessFlags (db, winningBid);
					successful = 1;

#ifdef PRINT_EVENTS
					fprintf (db->xmlLog, "<ProposalResult>result=\"success\"");
#endif
				}
				else
				{
#ifdef PRINT_EVENTS
					fprintf (db->xmlLog, "<ProposalResult>result=\"fail\" reason=\"tooLow\"");
#endif
				}
#ifdef PRINT_EVENTS
				fprintf (db->xmlLog, " bid=%f explorer=%d profitForSup=%f supervisorReserve=%f totalGross=%f supervisorResources=%f</ProposalResult>\n",
					winningBid.bid, winningBid.explorer, bidRatio,
					proposal->supervisorReserve, proposal->totalProfit, proposal->supervisorResources);
#endif
			}
			else
			{
#ifdef PRINT_EVENTS
				fprintf (db->xmlLog, "<ProposalResult>result=\"fail\" reason=\"noBids\"</ProposalResult>\n");
#endif
			}

			if (!successful)
			{
				tidyBids (db, proposal->supervisor, 0); // Bids on proposal by supervisor.
				failProposal (db, proposal);
			}

			List_deleteElement (&db->coalitionData.proposals, &iter, 1, 1);
		}
		else
		{
			iter = iter->next;
		}
	}
}
#endif // 0
#endif
