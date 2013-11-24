#include "../Common/RobotDefs.h"

#ifndef GUM_CONTROL_H
#define GUM_CONTROL_H

#if defined(ROBOT) && defined(IS_GUMSTIX) && defined(IS_GUMSTIX)

//! Run robot in remote control mode
int remoteControl();

//! Run function to determine robot movement calibration
int calibRemoteControl();

#endif
#endif
