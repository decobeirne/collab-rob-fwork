/*
 * Copyright Dr. Michael Schukat, NUI Galway, 2004
 */


#ifndef __CMU_CFG_H
#define __CMU_CFG_H


/*  *********************************
 *  S Y M B O L   D E F I N T I O N S
 *  *********************************/

#define mCMU_DISTANCE_ADDRESS        0x70
#define mCMU_DISTANCE_START_REGISTER 0x00
#define mCMU_DISTANCE_DATA_REGISTER  0x02
#define mCMU_DISTANCE_CM             0x51

#define mCMU_MOTOR_CONTROL           0x00
#define mCMU_MOTOR_1_L               0x01
#define mCMU_MOTOR_1_R               0x02
#define mCMU_MOTOR_2_L               0x03
#define mCMU_MOTOR_2_R               0x04

#define mCMU_COMPASS_ADDRESS         0x60
#define mCMU_COMPASS_REGISTER        0x01



/*  *****************************************
 *  D A T A   T Y P E   D E F I N I T I O N S
 *  *****************************************/


/*  *********************************
 *  S Y M B O L   D E F I N T I O N S
 *  *********************************/

/* 5 ms delay between two bytes sent out to CMU. */
#define mCMU_SEND_DELAY                     5 * 1000

/* Timeout values required by osal. */
#define mCMU_SENSOR_SEND_TIMEOUT            30
#define mCMU_SENSOR_RECEIVE_TIMEOUT_LONG    40
#define mCMU_SENSOR_RECEIVE_TIMEOUT_SHORT   30

/* Delay on reset before CMU is contacted. */
#define mCMU_RESET_DELAY                    500 * 1000

/* We assume that it takes 100 ms for the CMU to respond. */
#define mCMU_PROCESS_DELAY                  100 * 1000

/* Calibration Constants for robot movement. */
#define mCMU_SERVO_TURN_CAL                 30
#define mCMU_TTL_TURN_CAL                   7
#define mCMU_SERVO_MOVE_CAL                 35
#define mCMU_TTL_MOVE_CAL                   125 

#endif

