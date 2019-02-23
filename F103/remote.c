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
#define TIM_CAP_POL(ch) (2 << (ch - 1) * 4)
#define PWM_MAX_PULSE 2000
#define PWM_CENTER_PULSE 1500
#define PWM_MIN_PULSE 1000

// in PWM mode half of the buffer is to store
// the first captured value
#define MAX_REMOTE_CHANNELS 8

int32_t radius = 120;
int32_t angle = 0;
#define THROTTLE_OFFSET 200

volatile uint8_t edge_counter;
volatile uint16_t remote_channels[MAX_REMOTE_CHANNELS];
volatile uint8_t ready;

#define PPM_INPUT
#define PPM_CHANNELS 6

#define TEST_PIN (1 << 5)  //PB5
#define TEST_PIN_PORT GPIOB
#define CFG_TEST_PIN TEST_PIN_PORT->CRL &= ~(0x0F << 20); TEST_PIN_PORT->CRL |= (2 << 20);
#define TOGGLE_TEST_PIN TEST_PIN_PORT->ODR ^= TEST_PIN

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

RAM_CODE void readFilter(int16_t *dst, uint16_t newvalue){

    if(newvalue < PWM_MIN_PULSE || newvalue > PWM_MAX_PULSE)
    {
        newvalue = PWM_CENTER_PULSE;
    }

    *dst = map(newvalue, PWM_MIN_PULSE, PWM_MAX_PULSE, LOGICAL_MINIMUM, LOGICAL_MAXIMUM);    
}

RAM_CODE void REMORE_Read(Remote_Type *rem){
    #if defined(DEMO)
    remote_demo(rem);
    #else
    if(ready == 0) 
        return;
    
    readFilter(&rem->pitch, remote_channels[0]);
    readFilter(&rem->roll, remote_channels[1]);
    readFilter(&rem->throttle, remote_channels[2] + THROTTLE_OFFSET);
    readFilter(&rem->yaw, remote_channels[3]);
    readFilter(&rem->aux1, remote_channels[4]);
    readFilter(&rem->aux2, remote_channels[5]); 
    #endif
}

#if !defined (PPM_INPUT)
void REMOTE_Init(void){

    RCC->APB1ENR |= (1 << 0);   // TIM2EN

    TIM2->CR1 = 0;              // Stop counter

    TIM2->PSC = SystemCoreClock/1000000;      // 1Mhz
    TIM2->CCMR1 = 0x0101;       // CC1-2S = 01, ch1-2 mapped to TI1,TI2 
    TIM2->CCMR2 = 0x0101;       // CC3-4S = 01, map to TI1 
    TIM2->CCER  = 0x1111;       // CC1E, CC1; Configure all channels as input, Capture on rising edge

    TIM2->DIER = (0x0f << 1);   // Enable interrupt on capture for all channels
    NVIC_EnableIRQ(TIM2_IRQn);  // Enable timer 2 interupt
    TIM2->CR1 |= (1 << 0);      // Start counter
}

/**
 * Handles a radio channel with the cirrespondent captured value from interrupt.
 * Hw capture flag is cleared when the correspondent regiter is read.
 * This is a synchronous algorithm, since the ready flag is set only if all channels were 
 * measured.  
 * */
RAM_CODE void HandleChannel(volatile uint16_t *dst, volatile uint16_t *ccr, uint8_t ch){

    // check if overcapture occurred
    if(TIM2->SR & (1 << (8 + ch))){ 
        // if set ignore capture and set capture for rising edge 
        TIM2->CCER &= ~(TIM_CAP_POL(ch));
        TIM2->SR &= ~((1 << (8 + ch)) | (1 << ch));
        return;
    }

    // If interrupt was from a rising edge, save capture value on the remote channel
    // and change the polarity of the capturing edge for the give channel
    if(!(TIM2->CCER & TIM_CAP_POL(ch))){
        // First capture, save it on buffer 
        *dst =  *ccr;
        TIM2->CCER |= TIM_CAP_POL(ch);
        if(ch == 1) ready = 0;
    }else{            
        // If caused by falling edge, calculate the pulse width and save it on the 
        // remote channel.
        *dst =  (*ccr > *dst) ? *ccr - *dst : (0xFFFF - *dst) + *ccr;
        TIM2->CCER &= ~(TIM_CAP_POL(ch));
        // The calculated pulse value is stored on the lower half of the buffer
        *(dst-4) = *dst;
        if(ch == 4) ready = 1;
    }    
}

/**
 * Every capture of any channel will generate an interrupt on capture event (rising or falling edge), 
 * then the handler will evaluate which channel caused the interrupt and call the channel handler
 * with the correspondent radio channel buffer.
 * As each remote channel requires two captures we send the address of
 * the position to store the first captured value, then tha handler decides were to store
 * the pulse value
 * */
RAM_CODE void TIM2_IRQHandler(void){
    if(TIM2->SR & (1 << 1)){
        HandleChannel(&remote_channels[4], (uint16_t*)&TIM2->CCR1, 1);
    }
    
    if(TIM2->SR & (1 << 2)){
        HandleChannel(&remote_channels[5], (uint16_t*)&TIM2->CCR2, 2);
    }

    if(TIM2->SR & (1 << 3)){
        HandleChannel(&remote_channels[6], (uint16_t*)&TIM2->CCR3, 3);
    }

    if(TIM2->SR & (1 << 4)){
        HandleChannel(&remote_channels[7], (uint16_t*)&TIM2->CCR4, 4);
    }  
}
#else
/**
 *  Code for PPM input on PB9 (TIM4_CH4)
 * */
void REMOTE_Init(void){

    RCC->APB1ENR |= (1 << 2);   // TIM4EN

    TIM4->CR1 = 0;              // Stop counter

    TIM4->PSC = SystemCoreClock/1000000;      // 1Mhz
    //TIM4->CCMR1 = 0x0101;       // CC1-2S = 01, ch1-2 mapped to TI1,TI2 
    TIM4->CCMR2 = 0x0100;       // ch4 capture PB9 pin, ch3 output
    TIM4->CCER  = 0x1000;       // ch4 as input, Capture on rising edge; ch3 OC3 not active

    TIM4->DIER = (1 << 4);      // Enable interrupt for ch4

    TIM4->CCR3 = 10000;         // 12ms is the maximum period after all ppm pulses
    
    NVIC_EnableIRQ(TIM4_IRQn);  // Enable timer 2 interupt
    edge_counter = 0; 
    CFG_TEST_PIN;
    TIM4->CR1 |= (1 << 0);      // Start counter
}

/**
 * The interrupt is invoked in every rising edge of the pin. If we are expecting the first edge
 * then the capture value is stored, a timeout count is enabled and proceed waiting for the next rising edge.
 * For the follow edges the pulse is calculated and stored on the correspondent buffer location,
 * on the last edge, the edge counter and timer are reset. 
 * */
RAM_CODE void HandleChannel(volatile uint16_t *dst, volatile uint16_t ccr, uint8_t tim_ch){
static uint16_t last_capture;
    // check if overcapture occurred, ignore it
    if(TIM4->SR & (1 << (8 + tim_ch))){        
        TIM4->SR &= ~((1 << (8 + tim_ch)) | (1 << tim_ch)); // Clear flags
    }

    //TOGGLE_TEST_PIN; 
    
    if(edge_counter++ == 0){
        TIM4->DIER |= (1 << 3); // Enable sync timeout
        last_capture = ccr;
        return;
    }

    // remote channel is offset by 2, since the first pulse is for starting counting
    // and the increment happens before this point 
    dst[edge_counter - 2] = (ccr > last_capture) ? ccr - last_capture : (0xFFFF - last_capture) + ccr;
    last_capture = ccr;
    // was the last edge,reset all states
    // ppm adds an extra pulse to last channel
    if(edge_counter == PPM_CHANNELS + 1){
        edge_counter = 0;
        ready = 1;        
        TIM4->CNT = 0;        
    }    
}

RAM_CODE void TIM4_IRQHandler(void){
    if(TIM4->SR & (1 << 4)){              
        HandleChannel(remote_channels, TIM4->CCR4, 4);
    }else if(TIM4->SR & (1 << 3)){
        // Lost sync reset channels and restart counter        
        TIM4->SR &= ~((1 << (8 + 3)) | (1 << 3)); // Clear flags
        edge_counter = 0;
        // Change to failsafe values
        //dst[0] = 0;dst[1] = 0;dst[2] = 0;dst[3] = 0;
        TIM4->CNT = 0;
        TIM4->DIER &= ~(1 << 3); // Disable sync timeout
    }
}

#endif