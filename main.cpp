#include <iostream>
#include "raylib.h"
#include "rlgl.h"
#include "Joint.h"
#include <Eigen/Dense>
#include "Robot.h"
#include "Base.h"

using Eigen::MatrixXd;

int main(void)
{
for (size_t i = 0; i <10; i++) std::cout << "hi\n";
InitWindow(800, 600, "Data-driven Robot Arm");

MatrixXd m(2, 2);
m(0, 0) = 3;
m(1, 0) = 2.5;
m(0, 1) = -1;
m(1, 1) = m(1, 0) + m(0, 1);
std::cout << m << std::endl;

Camera camera = { 0 };
camera.position = (Vector3){ 8, 8, 8 };
camera.target   = (Vector3){ 0, 3, 0 };
camera.up       = (Vector3){ 0, 1, 0 };
camera.fovy     = 45;
camera.projection = CAMERA_PERSPECTIVE;

//robot chain of 2 joints. (users can define the robot arm with different number of joints).
Robot arm(2);
std::vector<Joint>& chain = arm.get_chain();
chain[0] = (Joint){0,0,(Vector3){0,0,1}, 3.0f, RED};
chain[1] = (Joint){0,0,(Vector3){0,0,1}, 2.5f, VIOLET};

SetTargetFPS(60);
Base base(10);
double iter  = 0;
while (!WindowShouldClose())
{
    UpdateCamera(&camera, CAMERA_FREE);

    // Input key to execute animation via IK solver. (for test and debug).
    if (IsKeyPressed(KEY_I))
    {
        iter = 0;
        arm.set_ActiveIK(true);
        arm.set_Target(Vector3d(-2.5,3,0));
    }
    if (arm.get_ActiveIK())
    {
        iter += 1;
        bool done = arm.animate(0.05);
        std::cout << "iter: " << iter << " -- ";
        if (done)
            arm.set_ActiveIK(false);
    }

    // Drawing.
    BeginDrawing();
    ClearBackground(RAYWHITE);
    BeginMode3D(camera);
    base.draw();
    arm.renderRobot();
    EndMode3D();

    DrawText("0/1/2 rotate joints | R reset", 10, 10, 20, DARKGRAY);

    EndDrawing();
}

CloseWindow();
return 0;
}
