#ifndef __CONSTANTS_H
#define __CONSTANTS_H

//#include "main.h"
#include "target_constants.h" // To be defined in target include folder

/*
 * For more settings see target_constants.h in a target specific folder
 */


#define SW_VERSION "1.3.13" // Version string
#define MIN_SW_CONFIGURATOR "1.3.10" // Minimal supported configurator version

#define MAX_AXIS 2 // ONLY USE 2 for now else screws HID Reports


#ifndef CANBUS
#undef ODRIVE
#endif

#endif
