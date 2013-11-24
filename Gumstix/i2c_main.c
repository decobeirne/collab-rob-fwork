/*
* Copyright Dr. Michael Schukat, NUI Galway, 2004
*/


/*  ***************************************** 
*  E X T E R N A L   H E A D E R   F I L E S
*  *****************************************/

#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <string.h>
#include <unistd.h>

#include "types.h"
#include "i2c-dev.h"
#include "i2c.h"


/*  *****************************************
*  E X T E R N A L   D E C L A R A T I O N S
*  *****************************************/


/*  ************************************* 
*  G L O B A L   D E C L A R A T I O N S
*  *************************************/


/*  *****************************************
*  I N T E R N A L   D E C L A R A T I O N S 
*  *****************************************/


/*  *************************************************
*  L O C A L   F U N C T I O N   P R O T O T Y P E S
*  *************************************************/



/*  *******************************
*  G L O B A L   F U N C T I O N S
*  *******************************/ 

/*  -----------------------------------------------------------------------
*  Purpose: Read word.
*
*  Return:  Status
*
*  Notes:   none
*
*/
tBOOL bI2C_ReadWord(BYTE byAddress, UINT16 un16Register, UINT16 *pun16Result)
{
	int reg,res,file,address;
	char filename1[20];
	char filename2[20];
	char filename3[20];
	char *filename;
	long funcs;
	tBOOL bRetVal = TRUE;

	address = (int) byAddress;
	reg = (int) un16Register;

	/* Try all three variants and give the correct error message upon failure.*/
	sprintf(filename1,"/dev/i2c-%d",i2cbus);
	sprintf(filename2,"/dev/i2c%d",i2cbus);
	sprintf(filename3,"/dev/i2c/%d",i2cbus);

	if ((file = open(filename1,O_RDWR)) < 0) 
	{
		if ((file = open(filename2,O_RDWR)) < 0) 
		{
			if ((file = open(filename3,O_RDWR)) < 0) 
			{
				fprintf(stderr,"I2C: Could not open file.\n"); 
				bRetVal = FALSE;
			}
			else 
			{
				filename = filename3;
			}
		} 
		else 
		{
			filename = filename2;
		}
	} 
	else 
	{
		filename = filename1;
	}

	if (bRetVal == TRUE)
	{
		/* Check adapter functionality */
		if (ioctl(file,I2C_FUNCS,&funcs) < 0) 
		{
			fprintf(stderr,"I2C: Could not get the adapter functionality matrix.\n");
			bRetVal = FALSE;
		}
		else
		{
			if (!(funcs & I2C_FUNC_SMBUS_READ_WORD_DATA)) 
			{
				fprintf(stderr, "I2C: Adaptor does not have word read capability.\n");
				bRetVal = FALSE;
			}
			else
			{  
				/* Use FORCE so that we can look at registers even when a driver is also running. */
				if (ioctl(file,I2C_SLAVE_FORCE,address) < 0) 
				{
					fprintf(stderr,"I2C: Could not set address to %d.\n",address);
					bRetVal = FALSE;
				}
				else
				{
					res = i2c_smbus_read_word_data(file,reg);
					*pun16Result = (UINT16) res;
				}
			}
		}
		close(file);
	}
	return bRetVal;
}


/*  -----------------------------------------------------------------------
*  Purpose: Write byte.
*
*  Return:  Status	
*
*  Notes:   none		
*
*/
tBOOL bI2C_WriteByte(BYTE byCAddr, BYTE byDAddr, BYTE byData)
{
	int res,file,value,address,daddress;
	char filename1[20];
	char filename2[20];
	char filename3[20];
	char *filename;
	long funcs;
	tBOOL bRetVal = TRUE;

	address=(int) byCAddr;
	daddress=(int) byDAddr;
	value=(int) byData;

	/* Try all three variants and give the correct error message upon failure. */
	sprintf(filename1,"/dev/i2c-%d",i2cbus);
	sprintf(filename2,"/dev/i2c%d",i2cbus);
	sprintf(filename3,"/dev/i2c/%d",i2cbus);

	if ((file = open(filename1,O_RDWR)) < 0) 
	{
		if ((file = open(filename2,O_RDWR)) < 0) 
		{
			if ((file = open(filename3,O_RDWR)) < 0) 
			{
				fprintf(stderr,"I2C: Could not open file.\n"); 
				bRetVal = FALSE;
			} 
			else 
			{
				filename = filename3;
			}
		} 
		else 
		{
			filename = filename2;
		}
	} 
	else 
	{
		filename = filename1;
	}

	if (bRetVal == TRUE)
	{
		/* Check adapter functionality */
		if (ioctl(file,I2C_FUNCS,&funcs) < 0) 
		{
			fprintf(stderr,"I2C: Could not get the adapter functionality matrix.\n");
			bRetVal = FALSE;
		}
		else
		{
			if (!(funcs & (I2C_FUNC_SMBUS_WRITE_WORD_DATA | I2C_FUNC_SMBUS_READ_WORD_DATA))) 
			{
				fprintf(stderr, "I2C: Adaptor does not have word write capability.\n");
				bRetVal = FALSE;
			}
			else
			{
				/* Use FORCE so that we can look at registers even when a driver is also running. */
				if (ioctl(file,I2C_SLAVE_FORCE,address) < 0) 
				{
					fprintf(stderr,"I2C: Could not set address to %d.\n",address);
					bRetVal = FALSE;
				}
				else
				{
					/* Handle data to write. */
					res = i2c_smbus_write_byte_data(file, daddress, value);
					if(res < 0) 
					{
						fprintf(stderr, "I2C: Write failed.\n");
						bRetVal = FALSE;
					}
				}
			}
		}
		close(file);
	}
	return bRetVal;
}


/*  -----------------------------------------------------------------------
*  Purpose: Read word from device byAddress.
*
*  Return:  Status	
*
*  Notes:   none		
*
*/
tBOOL bI2C_ReadWordShort(BYTE byAddress, UINT16 *pun16Result)
{
	int file,address;
	char filename1[20];
	char filename2[20];
	char filename3[20];
	char *filename;
	long funcs;
	tBOOL bRetVal = TRUE;
	UINT16 res;

	address = (int) byAddress;

	/* Try all three variants and give the correct error message upon failure.*/
	sprintf(filename1,"/dev/i2c-%d",i2cbus);
	sprintf(filename2,"/dev/i2c%d",i2cbus);
	sprintf(filename3,"/dev/i2c/%d",i2cbus);

	if ((file = open(filename1,O_RDWR)) < 0) 
	{
		if ((file = open(filename2,O_RDWR)) < 0) 
		{
			if ((file = open(filename3,O_RDWR)) < 0) 
			{
				fprintf(stderr,"I2C: Could not open file.\n"); 
				bRetVal = FALSE;
			} 
			else 
			{
				filename = filename3;
			}
		} 
		else 
		{
			filename = filename2;
		}
	} 
	else 
	{
		filename = filename1;
	}

	if (bRetVal == TRUE)
	{
		/* Check adapter functionality */
		if (ioctl(file,I2C_FUNCS,&funcs) < 0) 
		{
			fprintf(stderr,"I2C: Could not get the adapter functionality matrix.\n");
			bRetVal = FALSE;
		}
		else
		{
			if (!(funcs & I2C_FUNC_SMBUS_READ_WORD_DATA)) 
			{
				fprintf(stderr, "I2C: Adaptor does not have word read capability.\n");
				bRetVal = FALSE;
			}
			else
			{  
				/* Use FORCE so that we can look at registers even when a driver is also running. */
				if (ioctl(file,I2C_SLAVE_FORCE,address) < 0) 
				{
					fprintf(stderr,"I2C: Could not set address to %d.\n",address);
					bRetVal = FALSE;
				}
				else
				{
					res = i2c_smbus_read_word(file);
					*pun16Result = res;
				}
			}
		}
		close(file);
	}
	return bRetVal;
}


/*  -----------------------------------------------------------------------
*  Purpose: Write byte to device byAddress.
*
*  Return:  Status	
*
*  Notes:   none		
*
*/
tBOOL bI2C_WriteByteShort(BYTE byCAddr, BYTE byData)
{
	int res,file,value,address;
	char filename1[20];
	char filename2[20];
	char filename3[20];
	char *filename;
	long funcs;
	tBOOL bRetVal = TRUE;

	address=(int) byCAddr;
	value=(int) byData;

	/* Try all three variants and give the correct error message upon failure. */
	sprintf(filename1,"/dev/i2c-%d",i2cbus);
	sprintf(filename2,"/dev/i2c%d",i2cbus);
	sprintf(filename3,"/dev/i2c/%d",i2cbus);

	if ((file = open(filename1,O_RDWR)) < 0) 
	{
		if ((file = open(filename2,O_RDWR)) < 0) 
		{
			if ((file = open(filename3,O_RDWR)) < 0) 
			{
				fprintf(stderr,"I2C: Could not open file.\n"); 
				bRetVal = FALSE;
			} 
			else 
			{
				filename = filename3;
			}
		} 
		else 
		{
			filename = filename2;
		}
	} 
	else 
	{
		filename = filename1;
	}

	if (bRetVal == TRUE)
	{
		/* Check adapter functionality */
		if (ioctl(file,I2C_FUNCS,&funcs) < 0) 
		{
			fprintf(stderr,"I2C: Could not get the adapter functionality matrix.\n");
			bRetVal = FALSE;
		}
		else
		{
			if (!(funcs & (I2C_FUNC_SMBUS_WRITE_WORD_DATA | I2C_FUNC_SMBUS_READ_WORD_DATA))) 
			{
				fprintf(stderr, "I2C: Adaptor does not have word write capability.\n");
				bRetVal = FALSE;
			}
			else
			{
				/* Use FORCE so that we can look at registers even when a driver is also running. */
				if (ioctl(file,I2C_SLAVE_FORCE,address) < 0) 
				{
					fprintf(stderr,"I2C: Could not set address to %d.\n",address);
					bRetVal = FALSE;
				}
				else
				{
					/* Handle data to write. */
					res = i2c_smbus_write_byte(file, value);
					if(res < 0) 
					{
						fprintf(stderr, "I2C: Write failed.\n");
						bRetVal = FALSE;
					}
				}
			}
		}
		close(file);
	}
	return bRetVal;
}


/*  ***********************************
*  I N T E R N A L   F U N C T I O N S
*  ***********************************/

