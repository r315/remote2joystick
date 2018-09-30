
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
  int8_t  rsv[2];
}Remote_Type;

extern Remote_Type hitec;

void REMORE_Read(Remote_Type *rem);
void REMOTE_Init(void);

#endif