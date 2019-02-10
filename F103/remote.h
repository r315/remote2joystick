
#ifndef _remote_h_
#define _remote_h_

#include <stdint.h>

// must follow HID report structure
typedef struct remote{
  int8_t  buttons;
  int8_t  pitch;
  int8_t  roll;
  int8_t  throttle;
  int8_t  yaw;
  int8_t  aux1;
  int8_t  aux2;
}Remote_Type;

void REMORE_Read(Remote_Type *rem);
void REMOTE_Init(void);

#endif