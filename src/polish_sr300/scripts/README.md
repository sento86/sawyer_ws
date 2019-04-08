This README was written to contain brief descriptions of the programs written and not included in the orignal set of intera examples. This will need to be updated for the other programs that have been included.

### IK_Sample.py - Ramos, Luke 2018
This program was written as an exercise to explore mechanics of the Inverse Kinematic Client Service provided in Intera.
A hardcoded pose is given in terms of a position and a quaternion. The ik service client performs inverse kinematics
based on the given pose. The ik client service will yield a set of joint angles for sawyer, if the point is achievable.
The program then passes on the joint angles to manipulate sawyer into the desired pose.

### IK_cloud_planner.py - Ramos, Luke 2018 
Program written to combine input from PointCloud and inverse kinematics to move robot from pose to pose based on point cloud input.
The aim is to transform Jaime's manipulate_cloud outfrom from the camera frame to the base frame in order to obtain a point in the appropriate frame to perform inverse kinematics.
Main goal for now is just to get the end-effector to move to a point and ignoring the orientation for the time being.
Orientation in relation to the goal position can be constrained at a later date.
This script still has issues that need to be resolved, mainly associated with the transformation of the point cloud.
