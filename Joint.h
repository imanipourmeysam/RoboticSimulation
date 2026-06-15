#ifndef JOINT_H
#define JOINT_H

#include "raylib.h"

struct Joint
{
    Joint();
    Joint(float _angle, float _targetAngle, const Vector3& _rotationAxis, float _linkLength, Color _color);

    void set_angle(float _angel);
    float get_angle()const;

    void set_rotationAxis(const Vector3& _rotationAxis);
    const Vector3& get_rotationAxis()const;

    void set_linkLength(float _linkLength);
    float get_linkLength()const;

    void set_color(const Color& _color);
    const Color& get_color()const;

    void set_targetAngle(float _targetAngle);
    float get_targetAngle()const;


private:
    float angle = 0.0f;
    float targetAngle = 0.0f;
    Vector3 rotationAxis = {0.0f, 0.0f, 0.0f};
    float linkLength = 0.0f;
    Color color=DARKGRAY;

};
#endif // JOINT_H
