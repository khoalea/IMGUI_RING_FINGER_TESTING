#include "User_code/motion.h"
#include <math.h>

#define TWO_PI 6.28318530718f

MotionCh motion[XL330_MAX_MOTORS];

int32_t Motion_Step(MotionCh *c, uint32_t dt_ms)
{
    /* fmodf chứ không trừ 2pi một lần: dt lớn (vòng lặp bị chặn lâu) vẫn đúng. */
    c->phase = fmodf(c->phase + TWO_PI * (float)c->freq_mhz * (float)dt_ms * 1e-6f, TWO_PI);
    c->goal  = c->center + (int32_t)lroundf((float)c->amp * sinf(c->phase));
    return c->goal;
}
