#ifndef H_serial_sdk_H
#define H_serial_sdk_H

#ifdef __cplusplus
extern "C" {
#endif

#include "types.h"


/*! \defgroup SerDataIF Sensor Data Communication Interface
*/
/*@{*/


/*! \brief Declare OSAL_COMM handle to the serial port

*/
typedef struct tagOSAL_COMM *OSAL_COMM;


/*------------------------------------------------
Serial communications related types and values     
------------------------------------------------ */

/*! \brief Defines the possible errors returned by the 
communication functions
*/
typedef enum tagOSAL_CommErr
{
   OSAL_CommErr_OK     =  0, /*!< No errors */
   OSAL_CommErr_BADPRM = -1, /*!< Bad parameters */
   OSAL_CommErr_COMM   = -2, /*!< Communication error */
   OSAL_CommErr_NOMEM  = -3, /*!< Not enough memory */
   OSAL_CommErr_UNSUPP = -4, /*!< Not supported */
   OSAL_CommErr_OTHERS = -5, /*!< Some other type of error */
} OSAL_CommErr;


/*! \brief Defines the possible types for the communication ports
*/
typedef enum tagOSAL_CommType
{
   OSAL_CommType_Auto,		/*!< It may be useful to specify an autodetection of the communication port type */
   OSAL_CommType_Serial,	/*!< It indicates a serial type communication port */
   OSAL_CommType_Bluetooth
} OSAL_CommType;


/*! \brief Defines the serial port number
*/
typedef enum tagOSAL_CommPort
{
   OSAL_CommPort_AUTO = 0,	/*!< Maybe used in some application that wants to detect what comm port number to use based on some criteria*/
   OSAL_CommPort_COM1 = 1,	/*!< COM1*/
   OSAL_CommPort_COM2 = 2,	/*!< COM2*/
   OSAL_CommPort_COM3 = 3,	/*!< COM3*/
   OSAL_CommPort_COM4 = 4,	/*!< COM4*/
   OSAL_CommPort_COM5 = 5,	/*!< COM5*/
   OSAL_CommPort_COM6 = 6,	/*!< COM6*/
   OSAL_CommPort_COM7 = 7,	/*!< COM7*/
   OSAL_CommPort_COM8 = 8,	/*!< COM8*/
   OSAL_CommPort_COM9 = 9,	/*!< COM9*/
   OSAL_CommPort_COM10=10,	/*!< COM10*/
   OSAL_CommPort_COM11=11,	/*!< COM11*/
   OSAL_CommPort_COM12=12,	/*!< COM12*/
   OSAL_CommPort_COM13=13,	/*!< COM13*/
   OSAL_CommPort_COM14=14,	/*!< COM14*/
   OSAL_CommPort_COM15=15,	/*!< COM15*/
   OSAL_CommPort_COM16=16,	/*!< COM16*/
   OSAL_CommPort_ABOVE,		/*!< Higher number serial port*/
} OSAL_CommPort;


/*! \brief Defines the possible values for the serial port communication speed. 
This is used by the function #OSAL_CommPortSettings
*/
typedef enum tagOSAL_CommSpeed
{
   OSAL_CommSpeed_SKIP   = -1,		/*!< If this is used, the serial port speed is not affected */
   OSAL_CommSpeed_9600   = 96,		/*!< 9600 bauds/second */
   OSAL_CommSpeed_19200  = 192,		/*!< 19200 bauds/second */
   OSAL_CommSpeed_38400  = 384,		/*!< 38400 bauds/second */
   OSAL_CommSpeed_57600  = 576,		/*!< 57600 bauds/second */
   OSAL_CommSpeed_115200 = 1152,	/*!< 115200 bauds/second */
   OSAL_CommSpeed_230400 = 2304,	/*!< 230400 bauds/second (SUPPORTED BY SOME OS AND SOME SERIAL PORTS ONY) */
} OSAL_CommSpeed;


/*! \brief Defines the possible values for the parity used to 
   transmit data over the serial port. This is used by the function #OSAL_CommPortSettings
*/
typedef enum tagOSAL_CommParity
{
   OSAL_CommParity_SKIP = -1,	/*!< If this is used, the serial port parity is not affected */
   OSAL_CommParity_NOPARITY,	/*!< No parity is used for data communication*/
   OSAL_CommParity_EVENPARITY,	/*!< Even parity is used for data communication*/
   OSAL_CommParity_MARKPARITY,	/*!< MARK parity is used for data communication*/
   OSAL_CommParity_ODDPARITY,	/*!< Odd parity is used for data communication*/
   OSAL_CommParity_SPACEPARITY, /*!< SPACE parity is used for data communication*/
} OSAL_CommParity;


/*! \brief Defines the possible values for the stop bits used to 
   transmit data over the serial port. This is used by the function #OSAL_CommPortSettings
*/
typedef enum tagOSAL_CommStopBits
{
   OSAL_CommStopBits_SKIP         = -1, /*!< If this is used, the stop bits number used by the serial port is not affected*/
   OSAL_CommStopBits_ONESTOPBIT   = 10, /*!< This is the defalut value. One stop bit is used for data transfer */
   OSAL_CommStopBits_ONE5STOPBITS = 15, /*!< One and a half stop bits are used for data transfer */
   OSAL_CommStopBits_TWOSTOPBITS  = 20, /*!< Two stop bits are used for data transfer */
} OSAL_CommStopBits;


/*! \brief Defines the possible values to control the DTR signal. 
This is used by the function #OSAL_CommPortSettings
*/
typedef enum tagOSAL_CommDTR
{
   OSAL_CommDTR_SKIP =-1,	/*!< If this is used, the DTR signal state will not be affected */
   OSAL_CommDTR_DISABLE,	/*!< This is the default value. The DTR signal is disabled*/
   OSAL_CommDTR_ENABLE,		/*!< The DTR signal is enabled*/
   OSAL_CommDTR_HANDSHAKE,	/*!< Hardware handshake is used for transfer using DTR/DSR signals*/
} OSAL_CommDTR;


/*! \brief Defines the possible values to control the RTS signal. 
This is used by the function #OSAL_CommPortSettings
*/
typedef enum tagOSAL_CommRTS
{
   OSAL_CommRTS_SKIP =-1,	/*!< If this is used, the RTS signal state will not be affected */
   OSAL_CommRTS_DISABLE,	/*!< This is the default value. The RTS signal is disabled */
   OSAL_CommRTS_ENABLE,		/*!< The RTS signal is enabled */
   OSAL_CommRTS_HANDSHAKE,	/*!< Hardware handshake is used for transfer using RTS/CTS signals*/
   OSAL_CommRTS_TOGGLE,		/*!< Toggle the value of the signal. If enable, make it disable and the other way arround*/
} OSAL_CommRTS; 


/*! \brief Defines the possible actions for controlling 
the port through the escape function #OSAL_CommPortEscape
*/
typedef enum tagOSAL_CommCtrl
{
   OSAL_CommCtrl_CLRBREAK,	/*!< Clears the BREAK sequence on the communication port */
   OSAL_CommCtrl_SETBREAK,	/*!< Sets the BREAK sequence on the communication port */
   OSAL_CommCtrl_CLRDTR,	/*!< Clears the DTR signal on the communication port */
   OSAL_CommCtrl_SETDTR,	/*!< Sets the DTR signal on the communication port */
   OSAL_CommCtrl_CLRRTS,	/*!< Clears the RTS signal on the communication port */
   OSAL_CommCtrl_SETRTS,	/*!< Sets the RTS signal on the communication port */
} OSAL_CommCtrl;

/*---------------------------------------
Serial Port Function Declaration
-----------------------------------------*/

/*! \brief Opens a serial port handle
 *
	\param eCommType The type of the communication port
	\param eCommPort The number of the communcaiton port 
	\return <ul>
				<li>If it succedes, then the return value will be used as a handle to all
					subsequent serial port related calls. 
				<li>If it fails, then the returned handle is NULL
			</ul>
*/	
OSAL_COMM OSAL_CommPortOpen (OSAL_CommType eCommType, OSAL_CommPort eCommPort);


/*! \brief Close a serial port handle
 *
	\param pPort The handle to the communication port to be closed
	\return One of the error codes defined in #OSAL_CommErr
*/	
OSAL_CommErr OSAL_CommPortClose (OSAL_COMM pPort);


/*! \brief Sets the desired parameters for a communication port
 *
	\param pPort The handle to the communication port
	\param eCommSpeed The speed of the communicaton port as defined by #OSAL_CommSpeed
	\param eCommParity The parity to use for sending the data over the commnucation port as defined by #OSAL_CommParity
	\param eCommStopBits The number of stop bits to use for sending the data over the communcaiton port as defined by #OSAL_CommStopBits
	\param bUseCTS If this flag is 0, then don't use hardware flow control for communication. A value of 1 idicates the usage of hardware flow control
	\param eCommDTR It is used to control the output pin DTR. Values as defined by #OSAL_CommDTR
	\param eCommRTS It is used to control the output pin RTS. Values as defined by #OSAL_CommRTS
	\return If success, it returns OSAL_CommErr_OK. The possible error codes are defined by #OSAL_CommErr
*/	
OSAL_CommErr OSAL_CommPortSettings (
    OSAL_COMM        pPort,
	OSAL_INT32       eCommSpeed,
	OSAL_CommParity   eCommParity,
	OSAL_CommStopBits eCommStopBits,
	OSAL_INT32             bUseCTS,
	OSAL_CommDTR      eCommDTR,
	OSAL_CommRTS      eCommRTS
);


/*! \brief Used to clear all receive or trasmit buffers.
The pending transmit or receive operations are canceled.

	\param pPort The communication port handle
	\return If success, it returns OSAL_CommErr_OK. The possible error codes are defined by #OSAL_CommErr
*/
OSAL_CommErr OSAL_CommPortClear (OSAL_COMM pPort);


/*! \brief Used to pass direct commands to the serial port
 *
	\param pPort The communication port handle
	\param eCommCtrl The action to perform as defined by #OSAL_CommCtrl
	\return If success, it returns OSAL_CommErr_OK. The possible error codes are defined by #OSAL_CommErr
*/
OSAL_CommErr OSAL_CommPortEscape (OSAL_COMM pPort, OSAL_CommCtrl eCommCtrl);


/*! \brief Used to send data over the serial communication line
 *
	\param pPort The communication port handle
	\param pucSend A pointer to the buffer to send.
	\param pnBytes A pointer to a variable that contains the number of bytes 
	to send. When function returns, this variable will contain the number
	of actually sent bytes. 
	\param iTimeOut This parameter specifies the time out for the send operation. The time is specified in miliseconds.
	\return If success, it returns OSAL_CommErr_OK. The possible error codes are defined by #OSAL_CommErr
*/
OSAL_CommErr OSAL_CommPortSendBuff (
    OSAL_COMM       pPort,
    OSAL_UCHAR *pucSend,
	OSAL_INT32           *pnBytes,
	OSAL_INT32            iTimeOut
);


/*! \brief Used to send receive data over the serial communication line
 *
	\param pPort The communication port handle
	\param pucRecv A pointer to the receive buffer. The buffer should be allocated by the application.
	\param pnBytes A pointer to a variable that contains the number of bytes 
	to receive. When function returns, this variable will contain the number
	of actually received bytes. 
	\param iTimeOut This parameter specifies the time out for the receive operation. The time is specified in miliseconds.
	\return If success, it returns OSAL_CommErr_OK. The possible error codes are defined by #OSAL_CommErr
	\warning The function may return after the specified timeout, with no error code, but with no received bytes.
	The variable pointed by pnBytes parameter should be checked by the application to identify a such situation.
*/
OSAL_CommErr OSAL_CommPortRecvBuff (
    OSAL_COMM       pPort,
	OSAL_UCHAR *pucRecv,
	OSAL_INT32           *pnBytes,
	OSAL_INT32            iTimeOut
);

/*@}*/
#ifdef __cplusplus
}
#endif

#endif /* H_serial_sdk_H */
