/*
 * Copyright Dr. Michael Schukat, NUI Galway, 2004
 */

#ifndef __CMU_H
#define __CMU_H

#include <memory.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "serial_sdk.h"
#include "i2c.h"
#include "cmu_cfg.h"

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
tBOOL bCMU_Init();

tBOOL bCMU_Reset();

//! Turn auto exposure and white balance on.
/*!
It is recommended that this be done for circe 5 seconds after turning
the camera on.
*/
tBOOL bCMU_whiteBalanceOn();

//! Turn white balance and auto exposure off.
/*!
It is recommended to turn this off before colour tracking, etc. is carried out.
*/
tBOOL bCMU_whiteBalanceOff();

//! Turn on white balance, wait 5 seconds, turn it off again.
/*!
Auto gain and white balance.
*/
tBOOL bCMU_calibWhiteBalance();

/*  -----------------------------------------------------------------------
 *  Purpose: Close port
 *
 *  Return:  
 *
 *  Notes:   
 *
 */
tBOOL bCMU_Close();


/*  -----------------------------------------------------------------------
 *  Purpose: Returns downscaling factors.
 *
 *  Return:  -
 *
 *  Notes:   -.
 *
 */
void vCMU_GetDownSamplingFactor(BYTE *pbyFactorX, BYTE *pbyFactorY);


/*  -----------------------------------------------------------------------
 *  Purpose: Returns effective frame size.
 *
 *  Return:  -
 *
 *  Notes:   -.
 *
 */
void vCMU_GetFrameSize(BYTE *pbyFrameX, BYTE *pbyFrameY);


/*  -----------------------------------------------------------------------
 *  Purpose: Returns camera resolution.
 *
 *  Return:  -
 *
 *  Notes:   -.
 *
 */
void vCMU_GetCameraResolution(BYTE *pbyResolX, BYTE *pbyResolY);


/*  -----------------------------------------------------------------------
 *  Purpose: Sets downscaling factors.
 *
 *  Return:  Returns FALSE if values are out of range.
 *
 *  Notes:   Expected values are between 1 and 9.
 *
 */
tBOOL bCMU_SetDownSamplingFactor(BYTE byNewFactorX, BYTE byNewFactorY);


/*  -----------------------------------------------------------------------
 *  Purpose: Sets camera resolution.
 *
 *  Return:  Returns FALSE if values are out of range.
 *
 *  Notes:   Expected values are TRUE or FALSE.
 *
 */
tBOOL bCMU_SetCameraResolution(tBOOL bHighRes);


/*  -----------------------------------------------------------------------
 *  Purpose: Toggles LED.
 *
 *  Return:  Returns FALSE if values are out of range.
 *
 *  Notes:   Expected LED index is 0 or 1.
 *
 */
tBOOL bCMU_ToggleLED(BYTE byLedIndex, tBOOL bLedOn);


/*  -----------------------------------------------------------------------
 *  Purpose: Sends image to host.
 *
 *  Return:  Returns FALSE if values are out of range.
 *
 *  Notes:
 *
 */
tBOOL bCMU_GetImage(BYTE *pbyFrameX, BYTE *pbyFrameY, BYTE *pbyBuffer, UINT32 un32OutBufferSize, tBOOL b4ByteBoundary);


tBOOL bCMU_ReadCompass(UINT16 *pun16Data);
tBOOL bCMU_CompassCalibStart();
tBOOL bCMU_CompassCalibExit();


tBOOL bCMU_TriggerDistanceMeasure(void);


tBOOL bCMU_GetDistanceData(UINT16 *paun16Data, UINT16 *pun16ArrayLen);


tBOOL bCMU_StopRobot();


tBOOL bCMU_MoveRobot(float fDist, INT16 n16Direction);


#endif // __CMU_H
