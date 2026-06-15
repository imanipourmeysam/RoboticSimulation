#ifndef BASE_H
#define BASE_H

#include "raylib.h"

struct Base
{
    float axisLength = 1.0f;
    Base(float _legnth);
    void draw();

};

#endif // BASE_H
