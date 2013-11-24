
/* Linux Implementation for the serial communication functions */

#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <termio.h> /* termios structure, ioctl values for RTS,DTR... */
#include <unistd.h>  /* select, read, write */

/* select */
#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>

#include "serial_sdk.h"
#include "types.h"


/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
enum tagOSAL_Constants
{
	OSAL_CommSignature = 0x17717711,
};

struct tagOSAL_COMM 
{
	OSAL_LONG eSignature;
	OSAL_INT32 fd;
	struct termios old_settings;	
};


/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
OSAL_CommErr __inline ChkStruc (OSAL_COMM pPort)
{
	if (OSAL_CommSignature == pPort->eSignature)
	{
		/* what's the best way to check for an valid fd? */
		if ( pPort->fd >= 0)
		{
			//if there is a valid file descriptor, return OK
			return OSAL_CommErr_OK;
		}
	}
	return OSAL_CommErr_BADPRM;
}

/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/

OSAL_COMM OSAL_CommPortOpen (OSAL_CommType eCommType, OSAL_CommPort eCommPort)
{
	OSAL_COMM pcs;
	/* 
	* Update MS, 19.01.04.
	*/
	/* char	szComN[] = "/dev/ttyUSB?";*/
	char        szComN[] = "/dev/ttyS?";
	char        szComM[] = "/dev/rfcomm?";
	char *ptr = NULL;
	char *temp;
	int  fd;

	if (OSAL_CommType_Serial == eCommType)
	{
		//Replace the '?' character with the actual serial port number
		temp = strstr(szComN, "?");
		if (temp !=NULL)
		{	
			*temp = ('0' +eCommPort - 1);
			ptr = szComN;
		}
	}
	else if (OSAL_CommType_Bluetooth == eCommType)
	{
		//Replace the '?' character with the actual serial port number
		temp = strstr(szComM, "?");
		if (temp !=NULL)
		{	
			*temp = ('0' +eCommPort - 1);
			ptr = szComM;
		}
	}
	else
	{
		return NULL;
	}

	printf("Port to open: %s\n", ptr);
	if (NULL == ptr) return NULL;
	if ((fd = open (ptr, O_RDWR)) == -1) return NULL ;

	/* do we need to lock the file ??? 
	if (flock (fd, LOCK_EX | LOCK_NB) == -1) return NULL;
	*/

	pcs = (OSAL_COMM) malloc(sizeof(struct tagOSAL_COMM));
	pcs->fd = fd;
	pcs->eSignature = OSAL_CommSignature;

	if (OSAL_CommPortSettings (pcs, 9600, OSAL_CommParity_NOPARITY, OSAL_CommStopBits_ONESTOPBIT,0, OSAL_CommDTR_DISABLE, OSAL_CommRTS_DISABLE) != OSAL_CommErr_OK)
	{
		if (pcs)
		{
			free (pcs);
		}
		pcs = NULL;
	}

	return pcs;
}

/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/

OSAL_CommErr OSAL_CommPortClose (OSAL_COMM pPort)
{
	/* do we need to lock the file ??? 
	if (flock (fd, LOCK_UN) == -1) return NULL;
	*/
	close (pPort->fd);
	pPort->eSignature = 0;
	pPort->fd = -1;
	if (pPort);
	{
		free (pPort);
	}

	return OSAL_CommErr_OK;
}

/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/

OSAL_CommErr OSAL_CommPortSettings (
									OSAL_COMM pPort,
									OSAL_CommSpeed eCommSpeed, OSAL_CommParity eCommParity, OSAL_CommStopBits eCommStopBits,
									OSAL_BOOL bUseCTS, OSAL_CommDTR  eCommDTR, OSAL_CommRTS  eCommRTS
									)
{
	struct termios tio;
	int	speed = 0, stat;
	int iAction;
	extern int errno;

	/* do we need this ????
	* otherwise the SKIP values would not have any effect, no?*/

	if (tcgetattr (pPort->fd, &tio) < 0)
	{
		return OSAL_CommErr_OK;
	}

	if(eCommStopBits != OSAL_CommStopBits_SKIP)
	{
		tio.c_cflag = CREAD | CLOCAL | CS8;

		switch (eCommStopBits)
		{
		case OSAL_CommStopBits_ONESTOPBIT:  tio.c_cflag &= ~CSTOPB; break;
		case OSAL_CommStopBits_TWOSTOPBITS: tio.c_cflag |= CSTOPB;  break;
		case OSAL_CommStopBits_ONE5STOPBITS: 	/* unsuported in Linux ? */
		case OSAL_CommStopBits_SKIP:				/* do nothing */
		default:
			break;
		}
	}

	if (eCommParity != OSAL_CommParity_SKIP)
	{
		tio.c_iflag = IGNBRK | IGNPAR;

		if (eCommParity == OSAL_CommParity_NOPARITY)
		{
			tio.c_iflag |= IGNPAR;
		}
		else
		{
			tio.c_iflag &= ~IGNPAR;
			tio.c_iflag |= INPCK;
			tio.c_cflag |= PARENB;

			switch (eCommParity)
			{
			case OSAL_CommParity_SKIP:break;
			case OSAL_CommParity_ODDPARITY: tio.c_cflag |= PARODD; break;
			case OSAL_CommParity_EVENPARITY: tio.c_cflag &= ~PARODD; break;
			case OSAL_CommParity_MARKPARITY: /*tio.c_iflag |= PARMRK; break;*/
			case OSAL_CommParity_SPACEPARITY: break;
			default:
				break;
			}
		}
	}


	/*  What is the difference between CTS flow, DTR,RTS handshake? */
	if (bUseCTS > 0)
	{
		/* seems to affect tcdrain so that tcdrain doesn't return anymore */ 
		tio.c_cflag |= CRTSCTS;
	}

	errno = 0;
	stat = 0;
	switch (eCommDTR)
	{
	case OSAL_CommDTR_DISABLE:
		iAction = TIOCM_DTR;
		stat = ioctl (pPort->fd, TIOCMBIC, &iAction);
		break;
	case OSAL_CommDTR_ENABLE:
		iAction = TIOCM_DTR /* | TIOCM_CTS */;
		stat = ioctl (pPort->fd, TIOCMBIS,&iAction);
		break;
	case OSAL_CommDTR_HANDSHAKE: 	/*????*/
	case OSAL_CommDTR_SKIP:		/* do nothing */
	default:
		stat = 0;
	}
	if (stat != 0){
		return OSAL_CommErr_OK;


	}
	stat = 0;

	switch (eCommRTS)
	{
	case OSAL_CommRTS_DISABLE:
		iAction = TIOCM_RTS;	
		stat = ioctl (pPort->fd, TIOCMBIC, &iAction);
		break;
	case OSAL_CommRTS_ENABLE:
		iAction = TIOCM_RTS;
		stat = ioctl (pPort->fd, TIOCMBIS, &iAction);
		break;
	case OSAL_CommRTS_HANDSHAKE: 	/*????*/
	case OSAL_CommRTS_TOGGLE:		/*????*/
	case OSAL_CommRTS_SKIP:		/* do nothing */
	default:
		break;

	}
	if (stat != 0) return OSAL_CommErr_OK;

	tio.c_oflag = 0;
	tio.c_lflag = 0;
	tio.c_cc[VMIN] = 1;  /* we might need to change these lines */
	tio.c_cc[VTIME] = 0;


	if (eCommSpeed != OSAL_CommSpeed_SKIP)
	{
		switch (eCommSpeed)
		{
			/* no def for speed 2400 */
		case 2400: case 24:			speed = B2400; break;
		case OSAL_CommSpeed_9600: case 9600:	speed = B9600; break;
		case OSAL_CommSpeed_19200: case 19200:	speed = B19200; break;
		case OSAL_CommSpeed_38400: case 38400:	speed = B38400; break;
		case OSAL_CommSpeed_57600: case 57600:	speed = B57600; break;
		case OSAL_CommSpeed_115200: case 115200:	speed = B115200; break;
		case OSAL_CommSpeed_230400: case 230400:	speed = B230400; break;
		default:
			return OSAL_CommErr_BADPRM; /* is this enough ? */
		}

		if (cfsetispeed (&tio, speed) != 0) return OSAL_CommErr_OK;
		if (cfsetospeed (&tio, speed) != 0) return OSAL_CommErr_OK;
	}

	// is TCSANOW | TCSAFLUSH correct?
	if (tcsetattr (pPort->fd, TCSANOW, &tio) != 0) return OSAL_CommErr_OK;

	return 0;
}

/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/

OSAL_CommErr OSAL_CommPortClear (OSAL_COMM pPort)
{
	OSAL_CommErr rc;
	OSAL_INT32 stat;

	rc = ChkStruc(pPort);
	if(OSAL_CommErr_OK == rc){
		/* 
		* flush both data received but not read, 
		* and data written but not transmitted
		*/
		stat = tcflush(pPort->fd, TCIOFLUSH);
		if (-1 == stat) rc = OSAL_CommErr_COMM;
	}
	return rc;
}

/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/

OSAL_CommErr OSAL_CommPortEscape (OSAL_COMM pPort, OSAL_CommCtrl eCommCtrl)
{
	/* 
	* Win32: communication device performs an extended funciton 
	* CLRDTR, CLRRTS, SETDTR, SETRTS, SETBREAK,CLRBREAK
	*
	* SETXOFF, SETXON are not implemented in Win32 version
	*/

	OSAL_CommErr rc;
	OSAL_INT32 iAction, iRequest = -1;

	rc = ChkStruc(pPort);
	if(OSAL_CommErr_OK == rc){
		switch(eCommCtrl){
case OSAL_CommCtrl_CLRBREAK:
	/*
	* Linux: do nothing
	* We don't need to clear the break state, this is done
	* by tcsendbreak itself after a certain amount of time.
	*/
	break;
case OSAL_CommCtrl_SETBREAK:
	/*
	* The default amount of time in which tcsendbreak sends
	* the break characters seems to be not enough to
	* reset the communication speed to 9600baud for 
	* the Kodak DC210 camera
	*/
	if( 0 > tcsendbreak(pPort->fd, 0))
		rc = OSAL_CommErr_COMM;
	break;
case OSAL_CommCtrl_CLRDTR:	/* TIOCMBIC, TIOCM_DTR */
	iAction = TIOCM_DTR;// | TIOCM_CTS;
	iRequest = TIOCMBIC;
	break;
case OSAL_CommCtrl_SETDTR:	/* TIOCMBIS, TIOCM_DTR */	
	iAction =/* TIOCM_DTR|*/ TIOCM_CTS; // | TIOCM_CTS;
	iRequest = TIOCMBIS;
	break;
case OSAL_CommCtrl_CLRRTS:	/* TIOCMBIC, TIOCM_RTS */
	iAction = TIOCM_RTS;
	iRequest = TIOCMBIC;
	break;
case OSAL_CommCtrl_SETRTS:	/* TIOCMBIS, TIOCM_RTS */
	iAction = TIOCM_RTS;
	iRequest = TIOCMBIS;
	break;
default:
	return OSAL_CommErr_UNSUPP;
		}
		if(eCommCtrl != OSAL_CommCtrl_CLRBREAK && eCommCtrl != OSAL_CommCtrl_SETBREAK){
			if(-1 == ioctl(pPort->fd, iRequest, &iAction)){
				rc = OSAL_CommErr_COMM;
			}
		}


	}

	return rc;
}

/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/

OSAL_CommErr OSAL_CommPortSendBuff (OSAL_COMM pPort, unsigned char *pucSend, OSAL_INT32 *pnBytes, OSAL_INT32 iTimeOut)
{

	OSAL_DWORD dwSent;
	OSAL_CommErr rc;

	rc = ChkStruc(pPort);
	if(OSAL_CommErr_OK != rc)
	{
		return rc;
	}

	/* what about the write timeouts? should we use select aswell? */


	dwSent = write(pPort->fd, pucSend, (size_t)*pnBytes);
	if(dwSent == -1)
	{
		/* debug output */
		return OSAL_CommErr_COMM;
	}
	/*
	* If not all bytes were written return error.
	* Should we make sure that all data is written, like in libgpio?
	*/
	if(dwSent != *pnBytes)
	{
		*pnBytes = 0;
		return OSAL_CommErr_COMM;
	}

	/*
	* Wait till all bytes are really sent.
	* tcdrain seems to take forever on some machines
	*/
	tcdrain(pPort->fd);
	return OSAL_CommErr_OK;
}

/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/

OSAL_CommErr OSAL_CommPortRecvBuff (OSAL_COMM pPort, unsigned char *pucRecv, OSAL_INT32 *pnBytes, OSAL_INT32 iTimeOut)
{

	struct timeval timeout;
	fd_set readfd; /* file descriptor set */
	OSAL_DWORD dwRecv;
	OSAL_INT32 toRead, readen = 0;
	OSAL_CommErr rc;

	int zerocounter = 0;
	// printf("OSAL: Start TIME OUT %d \n",iTimeOut);
	FD_ZERO(&readfd);
	FD_SET(pPort->fd, &readfd);

	/* 
	* Check pPort structure 
	*/
	rc = ChkStruc(pPort);
	if(OSAL_CommErr_OK != rc) return rc;

	toRead = *pnBytes;

	while ((readen < toRead) && (zerocounter < 10))
	{
		/* 
		* set timeout value within input loop 
		* iTimeOut - timeout in milliseconds
		*/
		timeout.tv_usec = iTimeOut * 1000;
		timeout.tv_sec = 0;

		/* wait until data is available or timeout expires*/
		select(pPort->fd +1, &readfd, NULL, NULL, &timeout);
		if(FD_ISSET(pPort->fd, &readfd))
		{
			dwRecv = read(pPort->fd, pucRecv, (size_t) (toRead - readen));
			if(dwRecv < 0)
			{
				/* read failed for some reason */
				printf("OSAL: Error\n");
				return OSAL_CommErr_COMM;
			} 
			else
			{
				readen +=dwRecv;
				pucRecv +=dwRecv;
				// printf("Received some data: %s\n", pucRecv);
				if (dwRecv == 0)
				{
					++zerocounter;
				}
				else
				{
					zerocounter = 0;
				}
			}
		}
		else
		{
			/*
			* select timed out, no data is available
			* return OSAL_CommErr_COMM for now and no data 
			*/
			*pnBytes = 0;
			return OSAL_CommErr_COMM;
		}
	}

	*pnBytes = readen;
	return rc;
}



