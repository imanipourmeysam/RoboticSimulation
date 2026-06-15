a simple robotic simulation software for prototyping. 
For this project I used Eigen for computation part, including forward kinematic, and Inverse kinematic via jacobian matrix. 
For animation and rendering I used raylib C++.

The user can describe a robot arm via number of joints, rotaiton angle of each joint, and link length. 
The user can simulte an inverse kinematic solver.
Here is a very simple demo for IK simulation which is used jacobian solution to find joint angles required to get to the given target. 
https://github.com/user-attachments/assets/d72b0e7c-1bb9-459f-b38c-4c916c34dcbf

