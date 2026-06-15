A simple robotic simulation software for prototyping. 

For this project I used Eigen for computation part, including forward kinematic, and Inverse kinematic via jacobian matrix. 

For animation and rendering I used raylib C++.

The user can describe a robot arm via number of joints, rotaiton angle of each joint, and link length. 

The user can simulte an inverse kinematic solver.

Here is two simple demos for IK simulation which is used jacobian solution to find joint angles required to get to the given target. 


<img width="800" height="637" alt="Screencastfrom2026-06-1517-49-25-ezgif com-video-to-gif-converter" src="https://github.com/user-attachments/assets/2ba561c5-bd0c-40ae-8a82-0a19f700ee55" />




<img width="800" height="559" alt="demo_2DOF_RobotSim_IK" src="https://github.com/user-attachments/assets/551a7d79-6dab-4356-8ccb-3ddcaf4d1f7a" />
