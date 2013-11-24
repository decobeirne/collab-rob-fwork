#include "../Common/RobotDefs.h"

#if defined(ROBOT) && defined(IS_GUMSTIX)

#include "GumstixControl.h"
#include "Robot.h"





//! Run robot around and take imgs when appropriate
int remoteControl()
{
	BYTE *pbyData;
	BYTE byCamX, byCamY;
	int nBufferSize = 76000;
	int isFinished = 0;
	int isHBridge;
	int input;
	int res = 0;
	int count = -1;
	schar str[40];
	FILE *file;
	UINT16 compass;
	float orient;
	struct timeval tv0, tv1;

	printf("remote control\n");
#ifdef IS_H_BRIDGE
	isHBridge = 1;
#else
	isHBridge = 0;
#endif
	printf("hbrigde %d\n", isHBridge);

	if (0 == bCMU_Init())
	{
		printf ("bCMU_Init\n");
		return -1;
	}

	bCMU_calibWhiteBalance();

	pbyData = (BYTE*)malloc (nBufferSize);

	while (0 == isFinished)
	{
		printf ("FWD:0 BCK:1 LFT:2 RT:3 CAP:4 COMP:5 WHITE_ON:6 WHITE_OFF:7 COMP_CALIB_START:8 COMP_CALIB_END:9 QUIT:any >");
		fflush (stdout);
		gets (str);
		fflush (stdin);

		if (str[0] < '0' || str[0] > '9')
		{
			printf ("QUIT\n");
			isFinished = 1;
			res = 1;
			break;
		}

		str[1] = '\0';
		input = atoi (str);

		if (str[0] >= '0' && str[0] <= '3')
		{
			str[1] = '\0';
			input = atoi (str);
			printf ("bCMU_MoveRobot: %d\n", input);

			res = bCMU_MoveRobot (5, input);
		}
		else if (str[0] == '4')
		{
			gettimeofday (&tv0, NULL);
			if (0 == bCMU_GetImage (&byCamX, &byCamY, pbyData, nBufferSize, 1))
			{
				printf ("Error bCMU_GetImage\n");
				return -1;
			}
			gettimeofday (&tv1, NULL);
			tv1.tv_sec -= tv0.tv_sec;
			tv1.tv_usec -= tv0.tv_usec;

			printf ("bCMU_GetImage %d,%d, nBufferSize %d\n", byCamX, byCamY, nBufferSize);


			sprintf(str, "img%05d.dat", ++count);
			file = fopen(str, "wb");
			if (0 == file)
			{
				printf("ERROR fopen\n");
				break;
			}

			printf ("fwrite %d\n", fwrite (pbyData, 1, 176 * 143 * 3, file));
			fclose(file);
			res = 1;
		}
		else if (str[0] == '5')
		{
			if (0 == bCMU_ReadCompass(&compass))
			{
				printf("ERROR read compass\n");
				res = 0;
				break;
			}

			orient = SensorProcessing_calcOrient(compass);
			printf("compass reading %d angle %f\n", compass, orient);
			res = 1;
		}
		else if (str[0] == '6')
		{
			if (0 == bCMU_whiteBalanceOn())
			{
				printf ("Error bCMU_whiteBalanceOn\n");
				return -1;
			}
			res = 1;
		}
		else if (str[0] == '7')
		{
			if (0 == bCMU_whiteBalanceOff())
			{
				printf ("Error bCMU_whiteBalanceOff\n");
				return -1;
			}
			res = 1;
		}
		else if (str[0] == '8')
		{
			if (0 == bCMU_CompassCalibStart())
			{
				printf("ERROR starting compass calib\n");
				res = 0;
				break;
			}
			res = 1;
		}
		else if (str[0] == '9')
		{
			if (0 == bCMU_CompassCalibExit())
			{
				printf("ERROR exiting compass calib\n");
				res = 0;
				break;
			}
			res = 1;
		}

		if (0 == res)
		{
			printf ("Error\n");
			return -1;
		}
	}

	bCMU_StopRobot();
	bCMU_Close();

	free (pbyData);

	return 1;
}


int calibRemoteControl ()
{
	int isFinished = 0;
	int input;
	int res;
	int isHBridge;

	printf("remote control\n");
#ifdef IS_H_BRIDGE
	isHBridge = 1;
#else
	isHBridge = 0;
#endif
	printf("hbrigde %d\n", isHBridge);

	if (0 == bCMU_Init())
	{
		printf ("Error bCMU_Init\n");
		return -1;
	}

	bCMU_calibWhiteBalance();

	while (0 == isFinished)
	{
		printf ("DIST... QUIT:-1    >");
		scanf("%d", &input);

		if (-1 == input)
		{
			printf ("QUIT\n");
			isFinished = 1;
			res = 1;
		}
		else
		{
			printf ("bCMU_MoveRobot: dist: %d dir:0\n", input);
			res = bCMU_MoveRobot (input, 0);
		}

		if (0 == res)
		{
			printf ("Error bCMU_MoveRobot\n");
			return -1;
		}
	}

	bCMU_StopRobot();
	bCMU_Close();

	return 1;
}

#endif 











