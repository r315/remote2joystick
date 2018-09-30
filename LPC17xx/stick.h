#ifndef _stick_h_
#define _stick_h_

#ifdef cppplus
extern c "{"
#endif

#include <stdint.h>

class Stick{

    uint16_t x;
    uint16_t y;

    uint16_t posx;
    uint16_t posy; 

    int16_t xmin;
    int16_t xmax;

    int16_t ymin;
    int16_t ymax;

    void draw(void);
    void erase(void);

public:

    void update(int16_t newx, int16_t newy);
    void init(uint16_t positionx, uint16_t positiony);

    void setxrange(int16_t min, int16_t max);
    void setyrange(int16_t min, int16_t max);


};


#ifdef cppplus
"}"
#endif

#endif
