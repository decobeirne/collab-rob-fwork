/*
 * Copyright Dr. Michael Schukat, NUI Galway, 2004
 */

#ifndef __I2C_H
#define __I2C_H

#include "../Common/RobotDefs.h"


/*  *********************************
 *  S Y M B O L   D E F I N T I O N S
 *  *********************************/

/* Address of I2C. */
#define i2cbus     0




/*  *************************************
 *  F U N C T I O N   P R O T O T Y P E S 
 *  *************************************/

/*  -----------------------------------------------------------------------
 *  Purpose: Read word.
 *
 *  Return:  Status	
 *
 *  Notes:   none		
 *
 */
tBOOL bI2C_ReadWord(BYTE byAddress, UINT16 un16Register, UINT16 *pun16Result);


/*  -----------------------------------------------------------------------
 *  Purpose: Write byte.
 *
 *  Return:  Status	
 *
 *  Notes:   none		
 *
 */
tBOOL bI2C_WriteByte(BYTE byCAddr, BYTE byDAddr, BYTE byData);

/*  -----------------------------------------------------------------------
 *  Purpose: Read word.
 *
 *  Return:  Status	
 *
 *  Notes:   none		
 *
 */
tBOOL bI2C_ReadWordShort(BYTE byAddress, UINT16 *pun16Result);


/*  -----------------------------------------------------------------------
 *  Purpose: Write byte.
 *
 *  Return:  Status	
 *
 *  Notes:   none		
 *
 */
tBOOL bI2C_WriteByteShort(BYTE byCAddr, BYTE byData);


#endif
