#include "stick.h"
#include <display.h>

#define STICK_FRAME_W 63
#define STICK_FRAME_H 63

#define STICK_MARK_W   11
#define STICK_MARK_H   11

#define STICK_MARK_COLOR  BLUE
#define STICK_FRAME_COLOR RED

#define STICK_MIN_VAL 0
#define STICK_MAX_VAL 127

int32_t map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max);

void Stick::init(uint16_t positionx, uint16_t positiony){
    posx = positionx;
    posy = positiony;  

    // arbitrary position inside frame for erase
    x = posx + (STICK_FRAME_W / 2);  
    y = posy + (STICK_FRAME_H / 2);

    xmin = STICK_MIN_VAL;
    xmax = STICK_MAX_VAL;

    ymin = STICK_MIN_VAL;
    ymax = STICK_MAX_VAL;

    LCD_Rect(posx, posy, STICK_FRAME_W, STICK_FRAME_H, STICK_FRAME_COLOR);
    LCD_Rect(posx + 1, posy + 1, STICK_FRAME_W - 2, STICK_FRAME_H - 2, STICK_FRAME_COLOR);
    update(0,0);
}

void Stick::draw(void){
    if(x < posx + 2 || x > posx + STICK_FRAME_W - STICK_MARK_W - 1 || 
       y < posy + 2 || y > posy + STICK_FRAME_H - STICK_MARK_H - 1 )
        return;
    LCD_Line(posx + (STICK_FRAME_W/2), posy, posx + (STICK_FRAME_W/2), posy + STICK_FRAME_H, STICK_FRAME_COLOR);
    LCD_Line(posx, posy + (STICK_FRAME_H/2), posx + STICK_FRAME_W, posy + (STICK_FRAME_H/2), STICK_FRAME_COLOR);

    LCD_FillRect(x, y, STICK_MARK_W, STICK_MARK_H, STICK_MARK_COLOR);
}

void Stick::erase(void){
    if(x < posx + 2 || x > posx + STICK_FRAME_W - STICK_MARK_W - 1 || 
       y < posy + 2 || y > posy + STICK_FRAME_H - STICK_MARK_H - 1 )
        return;
    LCD_FillRect(x, y, STICK_MARK_W, STICK_MARK_H, BLACK);    
}

void Stick::update(int16_t newx, int16_t newy){

    newx = map(newx, xmin, xmax, -((STICK_FRAME_W/2) - (STICK_MARK_W/2)) + 2, (STICK_FRAME_W/2) - (STICK_MARK_W/2) - 1);
    newy = map(newy, ymin, ymax, -((STICK_FRAME_H/2) - (STICK_MARK_H/2)) + 2, (STICK_FRAME_H/2) - (STICK_MARK_H/2) - 1);

    newx = posx + (STICK_FRAME_W/2) - (STICK_MARK_W/2) + newx;
    newy = posy + (STICK_FRAME_H/2) - (STICK_MARK_H/2) + newy;
    
    if ((uint16_t)newx == x && (uint16_t)newy == y) {
        return;
    }    

    erase();   

    x = (uint16_t)newx;
    y = (uint16_t)newy;

    draw();

}


void Stick::setxrange(int16_t min, int16_t max){
    xmin = min;
    xmax = max;
}

void Stick::setyrange(int16_t min, int16_t max){
    ymin = min;
    ymax = max;
}


