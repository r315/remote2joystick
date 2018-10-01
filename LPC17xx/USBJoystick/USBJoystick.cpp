/* mbed USBJoystick Library
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


#include "stdint.h"
#include "USBJoystick.h"

bool USBJoystick::update(int16_t t, int16_t r, int16_t x, int16_t y, int16_t z, int16_t rx, uint8_t hat, uint32_t buttons) {

   _t = t;
   _r = r;   
   _x = x;
   _y = y;
   _z = z;
   _rx = rx;

   _buttons = buttons;

   _hat = hat;

   return update();
}
 
bool USBJoystick::update() {
   HID_REPORT report;

   report.length = 7;
   // Fill the report according to the Joystick Descriptor

   report.data[0] = (_buttons & 0x0f);
   
   report.data[1] = _x & 0xff;            
   report.data[2] = _y & 0xff;

   report.data[3] = _z & 0xff;            
   report.data[4] = _rx & 0xff;
   
   //report.data[7] = _hat;

   report.data[5] = _t & 0xff;            
   report.data[6] = _r & 0xff;
       
   return send(&report);
}

bool USBJoystick::throttle(int16_t t) {
     _t = t;
   return update();
}

bool USBJoystick::rudder(int16_t r) {
   _r = r;
   return update();
}

bool USBJoystick::move(int16_t x, int16_t y) {
   _x = x;
   _y = y;
   return update();
}

bool USBJoystick::buttons(uint32_t buttons) {
   _buttons = buttons;
   return update();
}

bool USBJoystick::hat(uint8_t hat) {
   _hat = hat;
   return update();
}


void USBJoystick::_init() {
   _t = -127;
   _r = -127;    
   _x = 0;                       
   _y = 0;     
   _buttons = 0x00000000;
   _hat = 0x00;              
}

uint8_t * USBJoystick::reportDesc() {    
static uint8_t reportDescriptor[] = {
  USAGE_PAGE(1), 0x01,           // Generic Desktop
  LOGICAL_MINIMUM(1), 0x00,      // Logical_Minimum (0)    
  USAGE(1), 0x04,                // Usage (Joystick)
  COLLECTION(1), 0x01,           // Application

#if 1// 4 Buttons
    USAGE_PAGE(1), 0x09,            // Buttons
    USAGE_MINIMUM(1), 0x01,         // 1
    USAGE_MAXIMUM(1), 0x04,         // 4
    LOGICAL_MINIMUM(1), 0x00,       // 0
    LOGICAL_MAXIMUM(1), 0x01,       // 1
    REPORT_SIZE(1), 0x01,
    REPORT_COUNT(1), 0x04,
    //UNIT_EXPONENT(1), 0x00,         // Unit_Exponent (0)
    //UNIT(1), 0x00,                  // Unit (None)                                           
    INPUT(1), 0x02,                 // Data, Variable, Absolute

    // Padding 4 bits
    REPORT_SIZE(1), 0x01,
    REPORT_COUNT(1), 0x04,
    INPUT(1), 0x01,                 // Constant
#endif

#if 1
    USAGE_PAGE(1), 0x01,            // Generic Desktop
    USAGE(1), 0x04,                 // Usage (Joystick)
    COLLECTION(1), 0x00,              // Physical
      USAGE(1), 0x30,                 // X
      USAGE(1), 0x31,                 // Y
      USAGE(1), 0x32,                 // Z
      USAGE(1), 0x33,                 // Rx
      LOGICAL_MINIMUM(1), 0x81,       // -127
      LOGICAL_MAXIMUM(1), 0x7f,       // 127
      REPORT_SIZE(1), 0x08,
      REPORT_COUNT(1), 0x04,
      INPUT(1), 0x02,                 // Data, Variable, Absolute
    END_COLLECTION(0), 
#endif

#if 0 // 4 Position Hat Switch
    USAGE(1), 0x39,                 // Usage (Hat switch)
    LOGICAL_MINIMUM(1), 0x00,       // 0
    LOGICAL_MAXIMUM(1), 0x03,       // 3
    PHYSICAL_MINIMUM(1), 0x00,      // Physical_Minimum (0)
    PHYSICAL_MAXIMUM(2), 0x0E, 0x01, // Physical_Maximum (270)
    UNIT(1), 0x14,                  // Unit (Eng Rot:Angular Pos)                            
    REPORT_SIZE(1), 0x04,
    REPORT_COUNT(1), 0x01,
    INPUT(1), 0x02,                 // Data, Variable, Absolute

    // Padding 4 bits
    REPORT_SIZE(1), 0x01,
    REPORT_COUNT(1), 0x04,
    INPUT(1), 0x01,                 // Constant
#endif

#if 1 // Throttle and ruddle
    USAGE_PAGE(1), 0x02,            // Simulation Controls
    USAGE(1), 0xBB,                 // Throttle             
    USAGE(1), 0xBA,                 // Rudder               
    LOGICAL_MINIMUM(1), 0x81,       // -127
    LOGICAL_MAXIMUM(1), 0x7f,       // 127
    REPORT_SIZE(1), 0x08,
    REPORT_COUNT(1), 0x02,
    INPUT(1), 0x02,                 // Data, Variable, Absolute  
#endif

  END_COLLECTION(0)
};

  reportLength = sizeof(reportDescriptor);
  return reportDescriptor;
}

