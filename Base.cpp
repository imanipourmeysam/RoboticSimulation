#include "Base.h"

Base::Base(float _length)
{
    axisLength = _length;
}

void Base::draw()
{
    DrawLine3D(
        {0,0,0},
        {axisLength, 0,0},
        RED);

    DrawLine3D(
        {0,0,0},
        {0,axisLength,0},
        GREEN);

    DrawLine3D(
        {0,0,0},
        { 0,0,axisLength},
        BLUE);
}
