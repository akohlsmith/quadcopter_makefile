/* Define to prevent recursive inclusion*/
#ifndef _COMMUNICATION_H
#define _COMMUNICATION_H

#include <stdint.h>

#define CMD_HISTORY_SIZE 20
#define MAX_CMD_SIZE 16

#define RC_SPEED 0 /*!< RC Speed channel */
#define RC_PITCH 1 /*!< RC Pitch channel */
#define RC_ROLL  2 /*!< RC Roll channel */
#define RC_YAW   3 /*!< RC Yaw channel */

void analyseString(uint8_t idx);

#endif /* _COMMUNICATION_H */
