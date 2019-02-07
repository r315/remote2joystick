#include "stm32f103xb.h"
#include "remote.h"
#include "math.h"

/**
 * PA0 <- CH1
 * PA1 <- CH2
 * PA2 <- CH3
 * PA3 <- CH4
 * */

#define RAM_CODE __attribute__((section(".ram_code")))
#define TIM2_CAP_POL(ch) (2 << (ch - 1) * 4)
#define PWM_MAX_PULSE 2000
#define PWM_MIN_PULSE 1000

#define LOGICAL_MINIMUM -127
#define LOGICAL_MAXIMUM  127

int32_t radius = 120;
int32_t angle = 0;

volatile uint16_t chs[8];
volatile uint8_t ready;

Remote_Type hitec;

#if defined(DEMO)
static void remote_demo(Remote_Type *rem){

rem->roll = cos((double)angle*3.14/180.0)*radius;  // value -127 .. 128
rem->pitch = sin((double)angle*3.14/180.0)*radius;  // value -127 .. 128
angle += 3;    

rem->throttle = rem->roll;
rem->yaw = -rem->pitch;

}
#endif


int32_t map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max){
  return ((x - in_min) * (out_max - out_min) / (in_max - in_min)) + out_min;
}

void REMOTE_Init(void){

    RCC->APB1ENR |= (1 << 0); // TIM2EN

    TIM2->CR1 = 0;              // Stop counter

    TIM2->PSC = SystemCoreClock/1000000;      // 1Mhz
    TIM2->CCMR1 = 0x0101;       // CC1-2S = 01, ch1-2 mapped to TI1,TI2 
    TIM2->CCMR2 = 0x0101;       // CC3-4S = 01, map to TI1 
    TIM2->CCER  = 0x1111;       // CC1E, CC1; Configure all channels as input, Capture on rising edge

    TIM2->DIER = (0x0f << 1);   // Enable interrupt on capture for all channels
    NVIC_EnableIRQ(TIM2_IRQn);  // Enable timer 2 interupt
    TIM2->CR1 |= (1 << 0);      // Start counter
}

RAM_CODE void readFilter(int8_t *dst, uint16_t newvalue){

    if(newvalue > PWM_MIN_PULSE && newvalue < PWM_MAX_PULSE)
    {
        *dst = map(newvalue, PWM_MIN_PULSE, PWM_MAX_PULSE, LOGICAL_MINIMUM, LOGICAL_MAXIMUM);
    }
}

RAM_CODE void REMORE_Read(Remote_Type *rem){
    #if defined(DEMO)
    remote_demo(rem);
    #else
    if(ready == 0) return;
    
    readFilter(&hitec.throttle, chs[2]);
    readFilter(&hitec.yaw, chs[3]);
    readFilter(&hitec.pitch, chs[0]);
    readFilter(&hitec.roll, chs[1]);
    #endif
}

/**
 * Handles a radio channel with the cirrespondent captured value from interrupt.
 * Hw capture flag is cleared when the correspondent regiter is read.
 * This is a synchronous algorithm, since the ready flag is set only if all channels were 
 * measured.  
 * */
RAM_CODE void HandleChannel(volatile uint16_t *dst, volatile uint16_t *ccr, uint8_t ch){

    // check if overflow occurred
    if(TIM2->SR & (1 << (8 + ch))){ 
        // if set ignore capture and set capture for rising edge 
        TIM2->CCER &= ~(TIM2_CAP_POL(ch));
        TIM2->SR &= ~((1 << (8 + ch)) | (1 << ch));
        return;
    }

    // If interrupt was from a rising edge, save capture value on the remote channel
    // and change the polarity of the capturing edge for the give channel
    if(!(TIM2->CCER & TIM2_CAP_POL(ch))){
        *dst =  *ccr;
        TIM2->CCER |= TIM2_CAP_POL(ch);
        if(ch == 1) ready = 0;
    }else{            
        // If caused by falling edge, calculate the pulse width and save it on the 
        // remote channel.
        *dst =  (*ccr > *dst) ? *ccr - *dst : (0xFFFF - *dst) + *ccr;
        TIM2->CCER &= ~(TIM2_CAP_POL(ch));
        *(dst-4) = *dst;
        if(ch == 4) ready = 1;
    }    
}

/**
 * Every capture of any channel will generate an interrupt on capture event (rising or falling edge), 
 * then the handler will evaluate which channel caused the interrupt and call the channel handler
 * with the correspondent radio channel buffer
 * */
RAM_CODE void TIM2_IRQHandler(void){
    if(TIM2->SR & (1 << 1)){
        HandleChannel(&chs[4], (uint16_t*)&TIM2->CCR1, 1);
    }
    
    if(TIM2->SR & (1 << 2)){
        HandleChannel(&chs[5], (uint16_t*)&TIM2->CCR2, 2);
    }

    if(TIM2->SR & (1 << 3)){
        HandleChannel(&chs[6], (uint16_t*)&TIM2->CCR3, 3);
    }

    if(TIM2->SR & (1 << 4)){
        HandleChannel(&chs[7], (uint16_t*)&TIM2->CCR4, 4);
    }  
}