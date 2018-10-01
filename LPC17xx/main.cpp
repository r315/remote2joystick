/* mbed USBJoystick Library Demo
 * Copyright (c) 2012, v01:  Initial version, WH,
 *                           Modified USBMouse code ARM Limited.
 *                           (c) 2010-2011 mbed.org, MIT License
 *               2016, v02:  Updated USBDevice Lib, Added waitForConnect, Updated 32 bits button 
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, inclumosig without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUmosiG BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "mbed.h"
#include "USBJoystick.h"

#include <display.h>
#include <accel.h>
#include "stick.h"

#define JOY_HAT_PORT LPC_GPIO1
#define JOY_HAT_MASK (JOY_HAT_UP_PIN | JOY_HAT_RIGHT_PIN | JOY_HAT_DOWN_PIN | JOY_HAT_LEFT_PIN | JOY_B0_PIN)
#define JOY_HAT_READ (~JOY_HAT_PORT->FIOPIN & JOY_HAT_MASK)

#define JOY_BUTTON_PORT LPC_GPIO1
#define JOY_BUTTON_MASK JOY_B0_PIN
#define JOY_BUTTON_READ (~JOY_BUTTON_PORT->FIOPIN & JOY_BUTTON_MASK)

//#define LANDTIGER 1

//USBMouse mouse;
USBJoystick joystick;

// Variables for Heartbeat and Status monitoring
DigitalOut heartbeatLED(LED1);
//DigitalOut myled1(LED1);
DigitalOut myled2(LED2);
DigitalOut myled3(LED3);

Stick leftctrl;
Stick rightctrl;

Ticker heartbeat;

//Serial pc(USBTX, USBRX); // tx, rx

// Heartbeat monitor
void pulse() {
  heartbeatLED = !heartbeatLED;
}

void heartbeat_start() {
  heartbeatLED=1;
  heartbeat.attach(&pulse, 0.5);
}

void heartbeat_stop() {
  heartbeat.detach();
}

void DelayMs(uint32_t ms){
    wait_ms(ms);
}

uint8_t readHat(void){
uint32_t hat = JOY_HAT_READ;
#if (HAT8 == 1)
    switch(hat){
        case JOY_HAT_UP_PIN:
            return JOY_HAT_UP;
        case JOY_HAT_UP_PIN | JOY_HAT_RIGHT_PIN:
            return JOY_HAT_UP_RIGHT;
        case JOY_HAT_RIGHT_PIN:
            return JOY_HAT_RIGHT;
        case JOY_HAT_RIGHT_PIN | JOY_HAT_DOWN_PIN:
            return JOY_HAT_RIGHT_DOWN;
        case JOY_HAT_DOWN_PIN:
            return JOY_HAT_DOWN;
        case JOY_HAT_DOWN_PIN | JOY_HAT_LEFT_PIN:
            return JOY_HAT_DOWN_LEFT;
        case JOY_HAT_LEFT_PIN:
            return JOY_HAT_LEFT;
        case JOY_HAT_LEFT_PIN | JOY_HAT_UP_PIN:
            return JOY_HAT_LEFT_UP;
        default : break;
    }
#elif (HAT4 == 1)
    if(hat & JOY_HAT_UP_PIN) return JOY_HAT_UP;
    if(hat & JOY_HAT_DOWN_PIN) return JOY_HAT_DOWN;
    if(hat & JOY_HAT_LEFT_PIN) return JOY_HAT_LEFT;
    if(hat & JOY_HAT_RIGHT_PIN) return JOY_HAT_RIGHT;   
#endif
    return JOY_HAT_NEUTRAL;
}

uint8_t readButtons(void){
    uint32_t buttons = JOY_BUTTON_READ;
    if(buttons & JOY_B0_PIN) return JOY_B0;
    return 0;
}

int32_t map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max){
  return ((x - in_min) * (out_max - out_min) / (in_max - in_min)) + out_min;
}


int main() {
  int16_t throttle = 127;
  int16_t yaw = 0;
  int16_t pitch = 0;
  int16_t roll = 0;
  
  int16_t z;
  int16_t rx;

  uint32_t buttons = 0;    
  uint8_t hat;

  int32_t radius = 120;
  int32_t angle = 0;

  Spi_Type spi;
  Accel_Type accel;
      
  //pc.printf("Hello World from Joystick!\n\r");

  heartbeat_start();

  DISPLAY_Init(ON);
  LCD_Rotation(LCD_REVERSE_LANDSCAPE);
  LCD_Clear(BLACK);
  LCD_Bkl(ON);

  spi.freq = 1000000;
  spi.cfg = SPI_MODE0 | SPI_8BIT;
  spi.bus = SPI_BUS0;

  SPI_Init(&spi);

  if( !ACCEL_Init(&spi)){
      DISPLAY_printf("Error fail to initialize Accelerometer\n");
  }

  leftctrl.init(LCD_GetWidth() / 2 - 15 -50, 170);
  rightctrl.init(LCD_GetWidth() / 2 + 15, 170);

  leftctrl.setyrange(-127, 0);
  leftctrl.setxrange(-127, 0);

 

  while (1) {
    // Basic Joystick
    //throttle = (i >> 8) & 0xFF; // value -127 .. 128
    //rudder = (i >> 8) & 0xFF;   // value -127 .. 128
    switch(readHat()){
        case JOY_HAT_UP:
            throttle += 1;
            if(throttle > 127)
                throttle = 127;
            break;
        case JOY_HAT_DOWN:
            throttle -= 1;
            if(throttle < -128)
                throttle = -128;
            break;
        case JOY_HAT_RIGHT:
            yaw += 1;
            if(yaw > 127)
                yaw = 127;
            break;
        case JOY_HAT_LEFT:
            yaw -= 1;
            if(yaw < -128)
                yaw = -128;
            break;

    }

    hat = JOY_HAT_NEUTRAL;

    buttons = readButtons();    

    ACCEL_Read(&accel); 

    if(buttons){
        roll = cos((double)angle*3.14/180.0)*radius;  // value -127 .. 128
        pitch = sin((double)angle*3.14/180.0)*radius;  // value -127 .. 128
        z = pitch;
        rx = roll;
        throttle = 127;
        yaw = 0;
    }
    else{
        roll = map(accel.x, -128, 127, 0, 127);
        pitch = map(accel.y, -128, 127, 0, 127);
        z = map(throttle, -128, 127, 127, 0);
        rx = map(yaw, -128, 127, 0, 127);
    }
    
    angle += 3;    

    DISPLAY_Goto(0,0);
    DISPLAY_printf("Throttle: %5d\n", z);
    DISPLAY_printf("YAW:      %5d\n", rx);
    DISPLAY_printf("Roll:     %5d\n", roll);
    DISPLAY_printf("Pitch:    %5d\n", pitch);    

    leftctrl.update(-rx, -z);
    rightctrl.update(pitch, roll);
    joystick.update(0,0, pitch, roll, z, rx, hat, buttons);
    wait_ms(10);
  }
    
  //pc.printf("Bye World!\n\r");                           
}