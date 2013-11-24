#include "../../Common/RobotDefs.h"

#ifndef BOARD

#include "RobotDatabase.h"

extern char experimentDirName[128];

//! Constructor
RobotDatabase initRobotDatabase (
#ifdef SIMULATION
								 BoardDatabase *board,
#endif
#if defined (SIMULATION) || defined (RERUNNING_ROBOT) || defined(RERUNNING_IMAGES_ONLY)
								  IplImage *localMapIplImage,
#endif
								  const int index)
{
	RobotDatabase r;
	schar name[256];
	time_t t;
#if defined (ROBOT) && defined (SIM_ERROR)
	unsigned int seed;
#endif

	time (&t);

#ifdef IS_GUMSTIX
	sprintf (name,"%s/gum_%d_%d.txt", experimentDirName, index, (int)t);
#else
	sprintf ((char*)name, "%s/robot_%d_%d.txt", experimentDirName, index, (int)t);
#endif
	r.xmlLog = fopen ((char*)name, "w");

#ifdef IS_GUMSTIX
	fprintf (r.xmlLog, "<ROBOT_GUMSTIX>\n");
#elif defined(IS_LINUX)
	fprintf (r.xmlLog, "<ROBOT_LINUX>\n");
#elif defined(ROBOT)
	fprintf (r.xmlLog, "<ROBOT_WIN>\n");
#else
	fprintf (r.xmlLog, "<SIMULATION>\n");
#endif

#if defined(USE_CLOUD)
	fprintf (r.xmlLog, "<USE_CLOUD>\n");
#endif

#ifdef SIMULATION
	r.board = board;
#endif
#if defined (SIMULATION) || defined (RERUNNING_ROBOT) || defined(RERUNNING_IMAGES_ONLY)
	r.localMapIplImage = localMapIplImage;
#endif

	r.groupData = initRobotGroupData();
	r.partnerData = initPartnerData();
	r.behaviourData = initBehaviourData();
	r.sensorData = initRobotSensorData();
	r.environment = initRobotEnvironment();
	r.status = initRobotStatus (index);
	r.commData = initCommData();

	List_pushValue (&r.behaviourData.stack, &r.behaviourData.available);

	CamVectors_init (&r.camVectors);
	GeometryConstants_init (
		&r.geometryConstants,
		&r.camVectors,
		r.xmlLog,
		0);
	UncertaintyConstants_init (
		&r.uncertaintyConstants);
	IkConstants_init (
		&r.geometryConstants,
		&r.ikConstants);

#if defined (ROBOT) && defined (SIM_ERROR)
	{
//		seed = (unsigned int)time(0);
		seed = 1357225759;
		srand (seed);
		fprintf (r.xmlLog, "<RandomSeed>%d</RandomSeed>\n", seed);
	}
#endif // defined (ROBOT) && defined (SIM_ERROR)

	return r;
}

void RobotDatabase_dtor (RobotDatabase *r)
{
	RobotStatus_dtor (&r->status);
	RobotEnvironment_dtor (&r->environment);
	RobotSensorData_dtor (&r->sensorData);
	BehaviourData_dtor (&r->behaviourData);
	freePartnerData (&r->partnerData);
	// RobotGroupData has no data

	fclose (r->xmlLog);
}

#endif
