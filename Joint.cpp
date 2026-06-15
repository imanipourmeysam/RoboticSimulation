#include "Joint.h"


Joint::Joint()
{

}

Joint::Joint(float _angle, float _targetAngle, const Vector3& _rotationAxis, float _linkLength, Color _color)
{
    angle = _angle;
    targetAngle = _targetAngle;
    rotationAxis = _rotationAxis;
    linkLength = _linkLength;
    color = _color;
}

void Joint::set_angle(float _angle)
{
    angle = _angle;
}

float Joint::get_angle()const
{
    return angle;
}


void Joint::set_rotationAxis(const Vector3& _rotationAxis)
{
    rotationAxis = _rotationAxis;
}

const Vector3& Joint::get_rotationAxis()const
{
    return rotationAxis;
}

void Joint::set_linkLength(float _linkLength)
{
    linkLength = _linkLength;
}
float Joint::get_linkLength()const
{
    return linkLength;
}

void Joint::set_color(const Color& _color)
{
    color= _color;
}
const Color& Joint::get_color()const
{
    return color;
}

void Joint::set_targetAngle(float _targetAngle)
{
    targetAngle = _targetAngle;
}

float Joint::get_targetAngle()const
{
    return targetAngle;
}

