#include "stm32f103xb.h"
#include "remote.h"
#include "math.h"


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

void readFilter(int8_t *dst, uint16_t newvalue){

    if(newvalue > 1000 && newvalue < 1900)
    {
        *dst = map(newvalue, 1000, 1900, 0, 127);
    }
}

void REMORE_Read(Remote_Type *rem){
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

void REMOTE_Init(void){

    RCC->APB1ENR |= (1 << 0); // TIM2EN

    TIM2->CR1 = 0;

    TIM2->PSC = SystemCoreClock/1000000;
    TIM2->CCMR1 = 0x0101; // CC1-2S = 01, map to TI1 
    TIM2->CCMR2 = 0x0101; // CC3-4S = 01, map to TI1 
    TIM2->CCER  = 0x1111;  // CC1E, CC1 input, rising edge

    TIM2->DIER = (0x0f << 1); // CC1IE    
    NVIC_EnableIRQ(TIM2_IRQn);
    TIM2->CR1 |= (1 << 0);  // CEN
}


#define TIM2_CAP_POL(ch) (2 << (ch - 1) * 4)
void HandleChannel(volatile uint16_t *dst, volatile uint16_t *ccr, uint8_t ch){

    if(TIM2->SR & (1 << (8 + ch))){ // check over flow
            // if set ignore capture and set capture for rising edge 
            TIM2->CCER &= ~(TIM2_CAP_POL(ch));
            TIM2->SR &= ~((1 << (8 + ch)) | (1 << ch));
            return;
        }
        if(!(TIM2->CCER & TIM2_CAP_POL(ch))){ // if rising edge, capture
            *dst =  *ccr;
            TIM2->CCER |= TIM2_CAP_POL(ch);
            if(ch == 1) ready = 0;
        }else{            
            *dst =  (*ccr > *dst) ? *ccr - *dst : (0xFFFF - *dst) + *ccr;
            TIM2->CCER &= ~(TIM2_CAP_POL(ch));
            *(dst-4) = *dst;
            if(ch == 4) ready = 1;
    }
    // TIM2->SR &= ~(1 << ch);
}

void TIM2_IRQHandler(void){
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