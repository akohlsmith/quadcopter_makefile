#include "communication.h"

#define LX 0xFF
#define LY 0xFE
#define RX 0xFD
#define RY 0xFC
#define ASK_POS 0xFB
#define STOP_POS 0xFA

extern uint16_t rc_channel[4];
extern char rcvd[CMD_HISTORY_SIZE][4];

void analyseString(uint8_t idx){
    switch(rcvd[idx][0]){
    case LX:
      rc_channel[RC_YAW] = rcvd[idx][1];
      break;
    case RX:
      rc_channel[RC_PITCH] = rcvd[idx][1];
      break;
    case ASK_POS:
      //TODO
       break;
    case STOP_POS:
      //TODO
      break;
    }
    switch(rcvd[idx][2]){
    case LY:
      rc_channel[RC_SPEED] = rcvd[idx][3];
      break;
    case RY:
      rc_channel[RC_ROLL] = rcvd[idx][3];
      break;
    }
}
