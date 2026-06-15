#ifndef ROBOT_H
#define ROBOT_H

#include <vector>
#include "Joint.h"
#include <Eigen/Dense>
#include "raylib.h"
#include "rlgl.h"
#include <math.h>
#include <iostream>
#include <utility>

using Eigen::MatrixXd;
using Eigen::Matrix3d;
using Eigen::Matrix4d;

using Eigen::VectorXd;
using Eigen::Vector3d;

class Robot
{
public:
    Robot(size_t _jointCount);
    void renderRobot();
    std::vector<Joint>& get_chain();
    size_t get_jointCount()const;
    void printEndEffectorPosition();
    bool animate(double epsilon);
    void set_ActiveIK(bool _state);
    void set_Target(const Vector3d& _target);
    bool get_ActiveIK();
    const Vector3d& get_Target();


private:
    std::vector<Joint> chain;
    size_t jointCount = 0;
    Matrix3d rotationMatrix(float _angle, Vector3 _rotationAxis);
    MatrixXd homoTransformationMatrix(const Joint& _joint);
    std::pair<Eigen::Matrix<double,3, Eigen::Dynamic>, Vector3d> computeJacobian_EndEffector();
    std::pair<VectorXd, Vector3d> getAngleViaIK(const Vector3d& _target);
    double IKSolver();
    bool ActiveIK = false;
    Vector3d target; // This should be changed. Now it works, but it's better to have target not as data member, but as argument provided by user, or triggered via mouse click.

#endif // ROBOT_H
