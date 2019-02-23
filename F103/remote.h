
#ifndef _remote_h_
#define _remote_h_

#include <stdint.h>

#pragma pack (1)
// must follow HID report structure
typedef struct remote{
  int8_t  buttons;
  int16_t  pitch;
  int16_t  roll;
  int16_t  throttle;
  int16_t  yaw;
  int16_t  aux1;
  int16_t  aux2;
}Remote_Type;

void REMORE_Read(Remote_Type *rem);
void REMOTE_Init(void);


#define LOGICAL_MINIMUM 0 //-127
#define LOGICAL_MAXIMUM 2047 // 127

#endif