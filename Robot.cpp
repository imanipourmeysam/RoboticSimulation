#include "Robot.h"

#define PI 3.14159265


Robot::Robot(size_t _jointCount)
{
    jointCount = _jointCount;
    chain.resize(jointCount);
}

void Robot::renderRobot()
{
    if (jointCount == 0) return;
    else
    {
        rlPushMatrix();
        for (size_t i = 0; i < jointCount; i++)
        {
            Joint& j = chain[i];

            // rotate joint
            rlRotatef(j.get_angle(), j.get_rotationAxis().x, j.get_rotationAxis().y, j.get_rotationAxis().z);

            // draw link
            rlPushMatrix();

            rlTranslatef(0,j.get_linkLength() / 2.0f, 0);

            DrawCube(
                (Vector3){0,0,0},
                0.4f,
                j.get_linkLength(),
                0.4f,
                j.get_color()
                );

            rlPopMatrix();

            // move to next joint position
            rlTranslatef(0,j.get_linkLength(), 0);
        }

        rlPopMatrix();
    }
}

std::vector<Joint>& Robot::get_chain()
{
    return chain;
}

size_t Robot::get_jointCount()const
{
    return jointCount;
}

Matrix3d Robot::rotationMatrix(float _angle, Vector3 _rotationAxis)
{

    float c = cos(_angle*PI/180);
    float s = sin(_angle*PI/180);

    Matrix3d R;
    if (_rotationAxis.x) // x axis rotation.
    {
        R <<1, 0, 0,
            0, c, -s,
            0, s, c;
    }
    else if (_rotationAxis.y) // y axis rotation.
    {
        R <<c, 0, s,
            0, 1, 0,
            -s, 0, c;
    }
    else if (_rotationAxis.z)
    {
        R <<c, -s, 0,
            s, c, 0,
            0, 0, 1;
    }
    else
        ;// not considered condition.

    return R;
}


MatrixXd Robot::homoTransformationMatrix(const Joint& _joint)
{
    float angle = _joint.get_angle();
    float length = _joint.get_linkLength();
    Vector3 rotationAxis = _joint.get_rotationAxis();

    Vector3d vec;
    vec << 0, length, 0;

    Matrix3d R = rotationMatrix(angle, rotationAxis);
    vec = R * vec;

    Matrix4d T;
    T.setIdentity();
    T.block<3,3>(0,0) = R;
    T.block<3,1>(0,3) = vec;
    return T;
}

std::pair<Eigen::Matrix<double, 3, Eigen::Dynamic>, Vector3d> Robot::computeJacobian_EndEffector()
{
    Eigen::Matrix<double,3,Eigen::Dynamic>jacobian;
    jacobian.resize(3,jointCount);
    jacobian.setZero();

    //Eigen::Matrix<double,3,jointCount+1>jointsPos_EEPos;
    Eigen::Matrix<double,3,Eigen::Dynamic>jointsPos_EEPos;
    jointsPos_EEPos.resize(3,jointCount+1);
    jointsPos_EEPos.setZero(); // the first column is (0,0,0) for first joint.

    Eigen::Matrix<double, 3, Eigen::Dynamic> rotationAxisInBase;
    rotationAxisInBase.resize(3, jointCount);
    rotationAxisInBase.setZero();

    Matrix4d T = Eigen::Matrix4d::Identity();

    for (size_t i =0 ; i<jointCount; i++)
    {
        T = T * homoTransformationMatrix(chain[i]);

        // in each iteraction, pos is the position of a joint starting from second joint, as the first joint position is (0,0,0).
        // in last iteraction, pos is the end effector position.
        Vector3d pos = T.block<3,1>(0,3);
        jointsPos_EEPos.col(i+1) = pos;

        Matrix3d R = T.block<3,3>(0,0);
        Vector3d localRotation (chain[i].get_rotationAxis().x, chain[i].get_rotationAxis().y, chain[i].get_rotationAxis().z);
        rotationAxisInBase.col(i)= R * localRotation; // resulting in 3 by 1 rotation vector.
    }

    Vector3d endEffectorPosition = jointsPos_EEPos.col(jointCount);
    Eigen::Matrix<double, 3, Eigen::Dynamic> deltaPs;
    deltaPs.resize(3,jointCount);
    deltaPs = endEffectorPosition.replicate(1,jointCount) - jointsPos_EEPos.leftCols(jointCount);

    for (size_t i = 0; i<jointCount; i++)
    {
        jacobian.col(i) = rotationAxisInBase.col(i).cross(deltaPs.col(i));
    }
    return {jacobian, endEffectorPosition};
}

void Robot::printEndEffectorPosition()
{
    std::pair<MatrixXd, Vector3d> res = computeJacobian_EndEffector();
    // End-effector position
    std::cout << "End-effector:\n" << res.second.transpose() << std::endl;

}

std::pair<VectorXd, Vector3d> Robot::getAngleViaIK(const Vector3d& _target)
{
    Eigen::VectorXd currentJointsAngles;
    currentJointsAngles.resize(jointCount);
    for (size_t i = 0; i<jointCount; i++)
    {
        currentJointsAngles(i) = chain[i].get_angle();
    }
    std::pair<MatrixXd, Vector3d> J_Pend = computeJacobian_EndEffector();

    Vector3d deltaTargetEndPos = _target  - J_Pend.second;

    VectorXd deltaJointsAngles;
    deltaJointsAngles.resize(jointCount);

    deltaJointsAngles = J_Pend.first.transpose() * deltaTargetEndPos;
    return {deltaJointsAngles, deltaTargetEndPos};
}

//void Robot::animateIK(const Vector3d& target)
//{
//    double alpha = 0.5;
//    double epsilon = 0.05;
//    size_t max_iter = 1000;
//
//    for (size_t iter = 0; iter < max_iter; iter++)
//    {
//        auto [deltaTheta, error] = getAngleViaIK(target);
//
//        // update joints
//        for (int i = 0; i < jointCount; i++)
//        {
//            double current = chain[i].get_angle();
//            chain[i].set_angle(current + alpha * deltaTheta(i));
//        }
//
//        std::cout << "iter:" << iter << " - error: " << error.norm() << std::endl;
//
//        if (error.norm() < epsilon)
//            break;
//    }
//}
//
double Robot::IKSolver()
{
    double alpha = 0.2;

    auto [deltaTheta, error] = getAngleViaIK(target);

    for (int i = 0; i < jointCount; i++)
    {
        double current = chain[i].get_angle();
        chain[i].set_angle(current + alpha * deltaTheta(i));
    }

    return error.norm();
}

void Robot::set_ActiveIK(bool _state)
{
    ActiveIK = _state;
}

void Robot::set_Target(const Vector3d &_target)
{
    target = _target;
}
bool Robot::get_ActiveIK()
{
    return ActiveIK;
}

const Vector3d& Robot::get_Target()
{
    return target;
}

bool Robot::animate(double epsilon)
{
    double error_norm = IKSolver();
    std::cout << "error:" << error_norm << std::endl;
    return error_norm < epsilon;
}
