/*
* Copyright Dr. Michael Schukat, NUI Galway, 2004
*/


/*  *******************************
*  G E N E R A L   C O M M E N T S
*  *******************************/



/*  *****************************************
*  E X T E R N A L   H E A D E R   F I L E S
*  *****************************************/

#include "cmu.h"



/*  *************************************
*  G L O B A L   D E C L A R A T I O N S
*  *************************************/


/*  *****************************************
*  I N T E R N A L   D E C L A R A T I O N S
*  *****************************************/

static OSAL_COMM eMyPort;

static BYTE byFactorX, byFactorY;  /* Scaling factors of camera. */
static BYTE byResolX, byResolY;    /* Pixel-resolution. */
static BYTE byFrameX, byFrameY;    /* Effective pixel-resolution of frame. */
static BYTE abyInBuffer[76000];


/*  *************************************************
*  L O C A L   F U N C T I O N   P R O T O T Y P E S
*  *************************************************/

static void vOpenSerialPort(void);
static void vSendCommand(BYTE *pbyData);
static void vSendCommand_2(BYTE *pbyData);
static tBOOL bGetAck(void);
static void vGetTail(void);

void OSAL_SysSleep (OSAL_UINT32 ulMicroSeconds)
{
	usleep(ulMicroSeconds);
}


/*  *******************************
*  G L O B A L   F U N C T I O N S
*  *******************************/

/*  -----------------------------------------------------------------------
*  Purpose: Open port & check for camera module.
*
*  Return:  Module there yes/no
*
*  Notes:   none
*
*/
tBOOL bCMU_Init()
{
	vOpenSerialPort();

	OSAL_SysSleep(mCMU_RESET_DELAY);

	vSendCommand((BYTE *) "GV\r\n");  // Get version string.

	if (TRUE != bGetAck())
		return FALSE;

	vGetTail();
	vSendCommand((BYTE *) "RS\r");  // Reset board.

	if (TRUE != bGetAck())
		return FALSE;

	vGetTail();
	vSendCommand((BYTE *) "BM 1\r");  // Single frame mode.

	if (TRUE != bGetAck())
		return FALSE;

	// 32=Auto gain off
	// 33=Auto gain on
	vSendCommand((BYTE *) "CR 19 33\r");
	if (TRUE != bGetAck())
		return FALSE;

	// 40=RGB with white balance off
	// 44=RGB with white balance on
	vSendCommand((BYTE *) "CR 18 40\r");
	if (TRUE != bGetAck())
		return FALSE;

	byFactorX = 1;
	byFactorY = 1;
	byResolX = 176;
	byResolY = 143;
	byFrameX = byResolX / byFactorX;
	byFrameY = byResolY / byFactorY;
	/*
	// Set pan & tilt.
	vSendCommand((BYTE *) "SV 0 128\r"); // horizontal
	bGetAck();
	vGetTail();
	vSendCommand((BYTE *) "SV 1 180\r"); // vertical
	bGetAck();
	vGetTail();
	*/

	if (FALSE == bCMU_StopRobot())
	{
		printf ("X bCMU_StopRobot\n");
		return FALSE;
	}

	return TRUE;
}

// Close/reopen i2c connection when error is encountered
tBOOL bCMU_Reset()
{
	printf("I2C exception caught.\n");
	printf("Shutting down i2c and bringing back up again.\n");

	bCMU_Close(); OSAL_SysSleep(10000);
	system("modprobe -r i2c-dev"); OSAL_SysSleep(10000); // Time in us, 10^-6
	system("modprobe -r i2c-pxa"); OSAL_SysSleep(10000);
	system("modprobe i2c-dev"); OSAL_SysSleep(10000);
	system("modprobe i2c-pxa"); OSAL_SysSleep(10000);
	if (0 == bCMU_Init())
	{
		printf ("Error bCMU_Init - when resetting system due to i2c error\n");
		assert (0);
	}
	OSAL_SysSleep(10000);
	return TRUE;
}

tBOOL bCMU_whiteBalanceOn()
{
	// Auto gain on: required for white balance.
	vSendCommand((BYTE *) "CR 19 33\r");
	if (TRUE != bGetAck())
		return FALSE;

	// RGB mode with white balance.
	vSendCommand((BYTE *) "CR 18 44\r");
	if (TRUE != bGetAck())
		return FALSE;
	return TRUE;
}

tBOOL bCMU_whiteBalanceOff()
{
	// RGB mode without white balance.
	vSendCommand((BYTE *) "CR 18 40\r");
	if (TRUE != bGetAck())
		return FALSE;

	// Leave auto gain on. This is useful in areas that are too dark, and
	// the models should account for differences in brightness anyway
	vSendCommand((BYTE *) "CR 19 33\r"); 
	if (TRUE != bGetAck())
		return FALSE;
	return TRUE;
}

tBOOL bCMU_calibWhiteBalance()
{
#ifdef IGNORE_CAM_DATA
	return TRUE;
#endif

	printf ("Turning on auto gain and white balance for 5 seconds...\n");
	bCMU_whiteBalanceOn();

	sleep (5); // gcc

	bCMU_whiteBalanceOff();
	printf ("Auto gain and white balance are now off\n");

	return TRUE;
}

/*  -----------------------------------------------------------------------
*  Purpose:
*
*  Return:  -
*
*  Notes:   -.
*
*/
tBOOL bCMU_Close()
{
	return OSAL_CommPortClose(eMyPort);
}


/*  -----------------------------------------------------------------------
*  Purpose: Returns downscaling factors.
*
*  Return:  -
*
*  Notes:   -.
*
*/
void vCMU_GetDownSamplingFactor(BYTE *pbyFactorX, BYTE *pbyFactorY)
{
	*pbyFactorX = byFactorX;
	*pbyFactorY = byFactorY;
}


/*  -----------------------------------------------------------------------
*  Purpose: Returns effective frame size.
*
*  Return:  -
*
*  Notes:   -.
*
*/
void vCMU_GetFrameSize(BYTE *pbyFrameX, BYTE *pbyFrameY)
{
	*pbyFrameX = byFrameX;
	*pbyFrameY = byFrameY;
}


/*  -----------------------------------------------------------------------
*  Purpose: Returns camera resolution.
*
*  Return:  -
*
*  Notes:   -.
*
*/
void vCMU_GetCameraResolution(BYTE *pbyResolX, BYTE *pbyResolY)
{
	*pbyResolX = byResolX;
	*pbyResolY = byResolY;
}


/*  -----------------------------------------------------------------------
*  Purpose: Sets downscaling factors.
*
*  Return:  Returns FALSE if values are out of range.
*
*  Notes:   Expected values are between 1 and 9.
*
*/
tBOOL bCMU_SetDownSamplingFactor(BYTE byNewFactorX, BYTE byNewFactorY)
{
	tBOOL bRetVal = TRUE;
	BYTE abyCmd[10];

	if ((byNewFactorX < 1) || (byNewFactorY < 1) || (byNewFactorX > 9) || (byNewFactorY > 9))
	{
		bRetVal = FALSE;
	}
	else
	{
		strcpy((char *) abyCmd, "DS x x\r");
		abyCmd[3] = 48 + byNewFactorX;
		abyCmd[5] = 48 + byNewFactorY;
		vSendCommand_2(abyCmd);
		if (TRUE == bGetAck())
		{
			byFactorX = byNewFactorX;
			byFactorY = byNewFactorY;
		}
		else
		{
			bRetVal = FALSE;
		}
	}
	vGetTail();
	return(bRetVal);
}


/*  -----------------------------------------------------------------------
*  Purpose: Sets camera resolution.
*
*  Return:  Returns FALSE if values are out of range.
*
*  Notes:   Expected values are TRUE or FALSE.
*
*/
tBOOL bCMU_SetCameraResolution(tBOOL bHighRes)
{
	tBOOL bRetVal = TRUE;
	BYTE abyCmd[10];

	if ((bHighRes !=TRUE) && (bHighRes != FALSE))
	{
		bRetVal = FALSE;
	}
	else
	{
		strcpy((char *) abyCmd, "HR x\r");
		abyCmd[3] = 48 + bHighRes;
		vSendCommand_2(abyCmd);

		if (TRUE == bGetAck())
		{
			if (TRUE == bHighRes)
			{
				byResolX = 176;
				byResolY = 255;
			}
			else
			{
				byResolX = 88;
				byResolY = 143;
			}
			byFrameX = byResolX / byFactorX;
			byFrameY = byResolY / byFactorY;
		}
		else
		{
			bRetVal = FALSE;
		}
	}
	vGetTail();
	return(bRetVal);
}


/*  -----------------------------------------------------------------------
*  Purpose: Toggles LED.
*
*  Return:  Returns FALSE if values are out of range.
*
*  Notes:   Expected LED index is 0 or 1.
*
*/
tBOOL bCMU_ToggleLED(BYTE byLedIndex, tBOOL bLedOn)
{
	tBOOL bRetVal = TRUE;
	BYTE abyCmd[10];

	if (((byLedIndex != 0) && (byLedIndex != 1)) || ((bLedOn != TRUE) && (bLedOn != FALSE)))
	{
		bRetVal = FALSE;
	}
	else
	{
		strcpy((char *) abyCmd, "Lx x\r");
		abyCmd[1] = 48 + byLedIndex;
		abyCmd[3] = 48 + bLedOn;

		vSendCommand_2(abyCmd);
		bRetVal = bGetAck();
		vGetTail();
	}
	return(bRetVal);
}

#define TIME_IMAGE
tBOOL bCMU_GetImage(BYTE *pbyFrameX, BYTE *pbyFrameY, BYTE *pbyBuffer, UINT32 un32OutBufferSize, tBOOL b4ByteBoundary)
{
	tBOOL bRetVal = FALSE;
	tBOOL bGoOn = TRUE;
	UINT32 un32RIndex = 0;
	UINT32 un32WIndex = 0;
	UINT32 un32MyBufferIndex = 0;
	UINT32 un32MyBufferSize;
	UINT32 un32CpBufferSize;
	//UINT32 un32Cnt;
	BYTE bySwap;
#ifdef TIME_IMAGE
	struct timeval tv0, tv1;
#endif

	memset(pbyBuffer, 0, un32OutBufferSize);
	memset(abyInBuffer, 0, sizeof(abyInBuffer));

	*pbyFrameX = 0;
	*pbyFrameY = 0;

	vSendCommand_2((BYTE *) "RF\r");  /* Get new frame. */
	if (TRUE != bGetAck())
	{
		return FALSE;
	}
	vGetTail();
	vSendCommand_2((BYTE *) "SF\r");  /* Transfer frame to host. */
	if (TRUE != bGetAck())
	{
		return FALSE;
	}

#ifdef TIME_IMAGE
	gettimeofday (&tv0, NULL);
#endif

	/* Read data in chunks. */
	do
	{
		un32MyBufferSize = min(8192, un32OutBufferSize - un32MyBufferIndex);
		un32CpBufferSize = un32MyBufferSize;
		OSAL_CommPortRecvBuff(eMyPort, abyInBuffer + un32MyBufferIndex, (int *) &un32MyBufferSize, mCMU_SENSOR_RECEIVE_TIMEOUT_SHORT);
		un32MyBufferIndex += un32MyBufferSize;
	} while ((un32MyBufferSize == un32CpBufferSize) && (0 != un32CpBufferSize));

	//printf ("bCMU_GetImage un32MyBufferIndex:%d un32OutBufferSize:%d\n", un32MyBufferIndex, un32OutBufferSize);

	/* Check buffer for last chars to get rid of osal bug. */
	while ((abyInBuffer[un32MyBufferIndex] != 0) && (un32MyBufferIndex < un32OutBufferSize))
	{
		++un32MyBufferIndex;
	}

	if (3 != abyInBuffer[un32MyBufferIndex - 2])  /* Last char is not an eof. */
	{
		return FALSE;
	}

	while ((un32RIndex < un32MyBufferIndex) && (TRUE == bGoOn) && (un32WIndex < un32OutBufferSize))
	{
		switch (abyInBuffer[un32RIndex])
		{
		case 1:  /* Get frame size. */
			//printf ("bCMU_GetImage get size\n");
			un32RIndex++;
			*pbyFrameX = 2 * abyInBuffer[un32RIndex++];
			*pbyFrameY = abyInBuffer[un32RIndex++];
			if (TRUE == b4ByteBoundary)
			{
				while (((*pbyFrameX) % 4) != 0)
				{
					(*pbyFrameX)++;
				}
			}
			break;

		case 2:  /* Ignore new line cmd. */
			//printf ("bCMU_GetImage ignore new line cmd\n");
			un32RIndex++;

			if (TRUE == b4ByteBoundary) /* Adjust width, required for  BMP format */
			{
				while ((un32WIndex % 4) != 0)
				{
					pbyBuffer[un32WIndex++] = 0;
					pbyBuffer[un32WIndex++] = 0;
					pbyBuffer[un32WIndex++] = 0;
				}
			}
			break;

		case 3:  /* Got eof delimiter. */
			//printf ("bCMU_GetImage eof\n");
			bGoOn = FALSE;
			bRetVal = TRUE;
			break;

		default:
			pbyBuffer[un32WIndex++] = abyInBuffer[un32RIndex++]; /* Get RGB values. */
			pbyBuffer[un32WIndex++] = abyInBuffer[un32RIndex++]; /* Get RGB values. */
			pbyBuffer[un32WIndex++] = abyInBuffer[un32RIndex++]; /* Get RGB values. */

			/* Change order of bytes: */
			bySwap = pbyBuffer[un32WIndex - 2];
			pbyBuffer[un32WIndex - 2] = pbyBuffer[un32WIndex - 1];
			pbyBuffer[un32WIndex - 1] = bySwap;

			pbyBuffer[un32WIndex] = pbyBuffer[un32WIndex - 3];
			un32WIndex++;
			pbyBuffer[un32WIndex] = pbyBuffer[un32WIndex - 3];
			un32WIndex++;
			pbyBuffer[un32WIndex] = pbyBuffer[un32WIndex - 3];
			un32WIndex++;
			break;
		}
	}

#ifdef TIME_IMAGE
	gettimeofday (&tv1, NULL);
	tv1.tv_sec -= tv0.tv_sec;
	tv1.tv_usec -= tv0.tv_usec;
	printf ("<GRAB_IMAGE>time=(%lu,%lu)</GRAB_IMAGE>\n", tv1.tv_sec, tv1.tv_usec);
#undef TIME_IMAGES
#endif

	return(bRetVal);
}



/*  ***********************************
*  I N T E R N A L   F U N C T I O N S
*  ***********************************/

/*  -----------------------------------------------------------------------
*  Purpose: Sends out command byte by byte with delay in between.
*
*  Return:  -
*
*  Notes:   We do not care about error returncodes at this stage.
*
*/
static void vSendCommand(BYTE *pbyData)
{
	BYTE byIndex;
	BYTE byLength;
	int i32SendBufSize;

	byLength = strlen((char *) pbyData);

	for (byIndex = 0; byIndex < byLength; byIndex++)
	{
		i32SendBufSize = 1;
		OSAL_CommPortSendBuff(eMyPort, pbyData++, &i32SendBufSize,  mCMU_SENSOR_SEND_TIMEOUT);
		OSAL_SysSleep(mCMU_SEND_DELAY);
	}
}

static void vSendCommand_2(BYTE *pbyData)
{
	int i32SendBufSize = strlen(pbyData);

	OSAL_CommPortSendBuff(eMyPort, pbyData, &i32SendBufSize,  mCMU_SENSOR_SEND_TIMEOUT);
}


/*  -----------------------------------------------------------------------
*  Purpose: Checks for ACK/NACK.
*
*  Return:  -
*
*  Notes:   We do not care about error returncodes at this stage.
*
*/
static tBOOL bGetAck(void)
{
	BYTE abyDataBuffer[4] = {'0', '0', '0', '0'};
	int i32BufferSize = 4;
	tBOOL bRetVal = TRUE;

	OSAL_SysSleep(mCMU_PROCESS_DELAY);
	OSAL_CommPortRecvBuff(eMyPort, abyDataBuffer, &i32BufferSize, mCMU_SENSOR_RECEIVE_TIMEOUT_LONG);

	/* vRecPrintf("Content of receive buffer: ", abyDataBuffer);  */
	//printf("%d<>%s\n", (int) i32BufferSize, abyDataBuffer);

	if (0 == i32BufferSize)
	{
		bRetVal = FALSE;
		printf("CMU: Got no response from camera module.\n");
	}
	/* Check for ACK. */
	else if ((abyDataBuffer[0] == 'A') && (abyDataBuffer[1] == 'C'))
	{
		bRetVal = TRUE;
	}
	else if ((abyDataBuffer[0] == ':') && (abyDataBuffer[1] == 'A'))
	{
		i32BufferSize = 1;
		// printf("Receive one more char\n");
		OSAL_CommPortRecvBuff(eMyPort, abyDataBuffer, &i32BufferSize, mCMU_SENSOR_RECEIVE_TIMEOUT_SHORT);
		bRetVal = TRUE;
	}
	/* Otherwise get last character of NACK. */
	else
	{
		i32BufferSize = 1;
		// printf("Receive 1 more char\n");
		OSAL_CommPortRecvBuff(eMyPort, abyDataBuffer, &i32BufferSize, mCMU_SENSOR_RECEIVE_TIMEOUT_SHORT);
		bRetVal = FALSE;
	}

	return(bRetVal);
}


/*  -----------------------------------------------------------------------
*  Purpose: Open & program serial port.
*
*  Return:  -
*
*  Notes:   PC requires OSAL_CommPort_COM2, Gumstix requires OSAL_CommPort_COM1.
*
*/
static void vOpenSerialPort(void)
{
	/* Set serial port parameters. */
	eMyPort = OSAL_CommPortOpen(OSAL_CommType_Serial, OSAL_CommPort_COM1); // 1

	if (NULL == eMyPort)
	{
		printf("CMU: Error - Com1 (ttyS0) could not be opened.\n");
		exit(1);
	}

	OSAL_CommPortSettings(
		eMyPort, OSAL_CommSpeed_115200, OSAL_CommParity_NOPARITY,
		OSAL_CommStopBits_ONESTOPBIT, 0, OSAL_CommDTR_DISABLE, OSAL_CommRTS_DISABLE);

}


/*  -----------------------------------------------------------------------
*  Purpose: Gets rest of receive buffer (version string, etc.).
*
*  Return:  -
*
*  Notes:   We do not care about error returncodes at this stage.
*
*/
static void vGetTail(void)
{
	BYTE abyDataBuffer[40];
	int i32BufferSize = 40;

	OSAL_CommPortRecvBuff(eMyPort, abyDataBuffer, &i32BufferSize, mCMU_SENSOR_RECEIVE_TIMEOUT_SHORT);
}




/*  -----------------------------------------------------------------------
*  Purpose: Read compass module.
*
*  Return: Status. Value is stored in 16 bit integer.
*
*  Notes: The IS_HMC6352 switch is required for the new HMC6352 compass modile.
*
*/
tBOOL bCMU_ReadCompass(UINT16 *pun16Data)
{
	tBOOL bRet;

	while (1)
	{
#ifdef IS_HMC6352
		bRet = bI2C_WriteByteShort(0x21, 65); // Send 'A' command to grab compass reading
		if (TRUE == bRet)
		{
			OSAL_SysSleep(10000); /* Wait at least 6 ms before data can be accessed. */
			bRet = bI2C_ReadWordShort((BYTE)0x21, pun16Data);
			if (TRUE == bRet)
			{
				return TRUE;
			}
		}
#else
		bRet = bI2C_ReadWord((BYTE)mCMU_COMPASS_ADDRESS, (UINT16)mCMU_COMPASS_REGISTER, pun16Data);
		if (TRUE == bRet)
		{
			return TRUE;
		}
#endif
		bCMU_Reset();
	}
	return(bRet);
}

tBOOL bCMU_CompassCalibStart()
{
	tBOOL bRet;
#ifdef IS_HMC6352
	bRet = bI2C_WriteByteShort(0x21, 67); // Send 'C' command to kick of calibration mode.
#else
	// Return false for this compass module, as we should either implement calibration
	// for this or verify that it is not necessary
	bRet = FALSE;
#endif
	return bRet;
}

tBOOL bCMU_CompassCalibExit()
{
	tBOOL bRet;
#ifdef IS_HMC6352
	bRet = bI2C_WriteByteShort(0x21, 69); // Send 'E' command to exit calibration mode.
#else
	bRet = FALSE;
#endif
	return bRet;
}


/*  -----------------------------------------------------------------------
*  Purpose: Triggers distance measurement..
*
*  Return:  Status.
*
*  Notes:   Measurement takes 65 ms.
*
*/
tBOOL bCMU_TriggerDistanceMeasure(void)
{
	return(bI2C_WriteByte((BYTE) mCMU_DISTANCE_ADDRESS, (UINT16) mCMU_DISTANCE_START_REGISTER, (BYTE) mCMU_DISTANCE_CM));
}


/*  -----------------------------------------------------------------------
*  Purpose: Triggers distance measurement..
*
*  Return:  Status, data and number of echos.
*
*  Notes:   Measurement takes 65 ms. We only consider the first 10 echos.
*
*/
tBOOL bCMU_GetDistanceData(UINT16 *paun16Data, UINT16 *pun16ArrayLen)
{
	UINT16 un16Counter = 0;
	UINT16 un16Register = mCMU_DISTANCE_DATA_REGISTER;
	tBOOL bRetVal = TRUE;

	while ((un16Counter < *pun16ArrayLen) && (un16Counter < 10))
	{
		if (!(bI2C_ReadWord((BYTE) mCMU_DISTANCE_ADDRESS, un16Register, paun16Data)))
		{
			bRetVal = FALSE;
			break;
		}
		else if (0 == *paun16Data)
		{
			bRetVal = TRUE;
			break;
		}
		else
		{
			paun16Data++;
			un16Counter++;
			un16Register += 2;
		}
	}
	*pun16ArrayLen = un16Counter;
	return(bRetVal);
}


/*  -----------------------------------------------------------------------
*  Purpose: Stops robot.
*
*  Return:  Returns FALSE if values are out of range.
*
*  Notes:   .
*
*/
tBOOL bCMU_StopRobot()
{
#ifdef IS_H_BRIDGE
	{
		vSendCommand_2((BYTE *) "SO 4 0\r"); // Disable ouputs

		if (TRUE != bGetAck())
			return FALSE;

		vGetTail();
		vSendCommand_2((BYTE *) "SO 0 0\r"); // Switch off output.

		if (TRUE != bGetAck())
			return FALSE;

		vGetTail();
		vSendCommand_2((BYTE *) "SO 1 0\r"); // Switch off output.

		if (TRUE != bGetAck())
			return FALSE;

		vGetTail();
		vSendCommand_2((BYTE *) "SO 2 0\r"); // Switch off output.

		if (TRUE != bGetAck())
			return FALSE;

		vGetTail();
		vSendCommand_2((BYTE *) "SO 3 0\r"); // Switch off output.

		if (TRUE == bGetAck())
		{
			vGetTail();
			return TRUE;
		}
	}
#else
	{
		vSendCommand_2((BYTE *) "SO 2 0\rSO 3 0\r"); // Switch off servos 0

		if (TRUE == bGetAck())
		{
			vGetTail();
			return TRUE;
		}
	}
#endif

	return TRUE;
}

tBOOL bCMU_MoveRobot(float fDist, INT16 n16Direction)
{
#ifdef IS_H_BRIDGE
	INT16 i;

	char cmd[9] = "SO 0 0\r";
	char nums[5] = "0123"; // char copied to "SO x 0\r"
	char vals[5]; // char copied to "SO 0 x\r"
#if ROBOT_PLATFORM == 2
	int switchMotorValues[4] = {1, 0, 3, 2};
#endif
#if ROBOT_PLATFORM == 3
	int switchMotorValues[4] = {0, 1, 2, 3};
#endif
#endif
	if (FALSE == bCMU_StopRobot())
	{
		printf ("X bCMU_StopRobot\n");
		return FALSE;
	}

#ifdef IS_H_BRIDGE
	{
		// In case you connected the motors the wrong way around.
#if ROBOT_PLATFORM == 2
		if (n16Direction >= 0 && n16Direction <= 3)
		{
			n16Direction = switchMotorValues[n16Direction];
		}
#endif
#if ROBOT_PLATFORM == 3
		if (n16Direction >= 0 && n16Direction <= 3)
		{
			n16Direction = switchMotorValues[n16Direction];
		}
#endif

		// set bool val to be issued to FL, BL, FR, BR
		switch (n16Direction)
		{
		case 0:
			strcpy (vals, "1010"); // fwd
			break;
		case 1:
			strcpy (vals, "0101"); // back
			break;
		case 2:
			strcpy (vals, "0110"); // left
			break;
		case 3:
		default:
			strcpy (vals, "1001"); // right
			break;
		}

		// disable ouputs before issuing cmds
		vSendCommand_2((BYTE *) "SO 4 0\r");

		if (FALSE == bGetAck())
		{
			printf ("X bGetAck\n");
			return FALSE;
		}

		// send cmds before enabling outputs
		for (i = 0; i < 4; i++)
		{
			// copy values into cmd string: "SO x x\r"
			cmd[3] = nums[i];
			cmd[5] = vals[i];

			vGetTail();
			vSendCommand_2((BYTE *) cmd);

			if (FALSE == bGetAck())
			{
				printf ("X bGetAck\n");
				return FALSE;
			}
		}

		// enable
		vGetTail();
		vSendCommand_2((BYTE *) "SO 4 1\r");

		// if the cmds were received
		if (TRUE != bGetAck())
		{
			printf ("X bGetAck\n");
			return FALSE;
		}
	}
#else
	{
		switch (n16Direction)
		{
		case 0:
			vSendCommand_2((BYTE *) "SV 2 118\rSV 3 138\r"); // fwd
			break;
		case 1:
			vSendCommand_2((BYTE *) "SV 2 138\r SV 3 118\r"); // back
			break;
		case 2:
//			vSendCommand_2((BYTE *) "SV 2 118\r SV 3 118\r"); // left
			vSendCommand_2((BYTE *) "SV 2 138\r SV 3 138\r"); // left
			break;
		case 3:
		default:
//			vSendCommand_2((BYTE *) "SV 2 138\r SV 3 138\r"); // right
			vSendCommand_2((BYTE *) "SV 2 118\r SV 3 118\r"); // right
			break;
		}

		if (FALSE == bGetAck())
		{
			printf ("X bGetAck\n");
			return FALSE;
		}

		vGetTail();
	}
#endif

	OSAL_SysSleep(fDist);

	return bCMU_StopRobot();
}




