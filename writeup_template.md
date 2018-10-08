## Project: Kinematics Pick & Place
### Austin Chun
### October 2018

[//]: # (Image References)

[joints_links]: ./writeup_images/label_joints_links.png
[axis_origins]: ./writeup_images/axis_origins.png
[dh_params]: ./writeup_images/DH_params.png
[joint_dims]: ./writeup_images/joint_dims.png
[trans_mat]: ./writeup_images/trans_mat.png
[corr_mat]: ./writeup_images/corr_mat.png
[theta23]: ./writeup_images/theta23.jpg
[theta456]: ./writeup_images/theta456.jpg
[DisplayPath]: ./writeup_images/DisplayPath.png
[ReachedTarget]: ./writeup_images/ReachedTarget.png
[DisplayDropOff]: ./writeup_images/DisplayDropoff.png
[ReachedDropOff]: ./writeup_images/ReachedDropoff.png
[Tmats]: ./writeup_images/Tmats.png



## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

Following the DH Parameter Assignment algorithm, explained in Lesson 12.13, the first steps are to label all joints and links, then assign the axes (Z first, then X) for each link. The video guide in the KR210 Forward Kinematics 1 section provides the following joint/link labels, and corresponding axes assignments.

![label joints and links][joints_links]
![label axis and origins][axis_origins]

I used these labels for deriving the DH parameters, also provided in the video guide, with the non-zero DH lengths shown below. All theta values are variables (except the gripper link which is rigidly fixed). However there is some variation on alpha (twist angle) values. Focusing on the angle between adjacent Z axes, the non-zero alpha values are alpha1 = alpha3 = alpha5 = -90 and alpha4 = 90.

![DH params][dh_params]

Lastly, the actual dimensions of the links can be extracted from the kr210.urdf.xacro file, which can then be used to give exact values for the non-zero lengths shown previously. 

![joint_dims][joint_dims]

The resulting DH Table is as follows:

#### DH Parameter Table
i   | Links | alpha(i-1) | a(i-1) | d(i)   | theta(i)
--- | ---   | ---        | ---    | ---    | ---
1   | 0->1  | 0          | 0      | 0.75   | q1
2   | 1->2  | - pi/2     | 0.35   | 0      | q2
3   | 2->3  | 0          | 1.25   | 0      | q3
4   | 3->4  | - pi/2     | -0.054 | 1.50   | q4
5   | 4->5  | pi/2       | 0      | 0      | q5
6   | 5->6  | - pi/2     | 0      | 0      | q6
7   | 6->EE | 0          | 0      | 0.303  | 0



#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.


Now that all links have defined axes and DH parameters, the transformation matrices can be setup between adjacent linkages. Since all transformation matrices have the same format, I will just write the matrix from Link0 to Link1. All other transformation matrices have the same format, just with incrementing DH parameters.

![trans_mat][trans_mat]

The simplified transformation matrices between each frame is below.

![Tmats][Tmats]

Thus, to get the transformation matrix from the base_link to the gripper_link, simply multiply the transformation matrices

`T0_G = T0_1*T1_2*T2_3*T3_4*T4_5*T5_6*T6_G`


`T0_G = [
[((sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*cos(q5) + sin(q5)*cos(q1)*cos(q2 + q3))*cos(q6) + (sin(q1)*cos(q4) - sin(q4)*sin(q2 + q3)*cos(q1))*sin(q6), -((sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*cos(q5) + sin(q5)*cos(q1)*cos(q2 + q3))*sin(q6) + (sin(q1)*cos(q4) - sin(q4)*sin(q2 + q3)*cos(q1))*cos(q6), -(sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*sin(q5) + cos(q1)*cos(q5)*cos(q2 + q3), -0.303*(sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*sin(q5) + (1.25*sin(q2) - 0.054*sin(q2 + q3) + 1.5*cos(q2 + q3) + 0.35)*cos(q1) + 0.303*cos(q1)*cos(q5)*cos(q2 + q3)],
[((sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*cos(q5) + sin(q1)*sin(q5)*cos(q2 + q3))*cos(q6) - (sin(q1)*sin(q4)*sin(q2 + q3) + cos(q1)*cos(q4))*sin(q6), -((sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*cos(q5) + sin(q1)*sin(q5)*cos(q2 + q3))*sin(q6) - (sin(q1)*sin(q4)*sin(q2 + q3) + cos(q1)*cos(q4))*cos(q6), -(sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*sin(q5) + sin(q1)*cos(q5)*cos(q2 + q3), -0.303*(sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*sin(q5) + (1.25*sin(q2) - 0.054*sin(q2 + q3) + 1.5*cos(q2 + q3) + 0.35)*sin(q1) + 0.303*sin(q1)*cos(q5)*cos(q2 + q3)],
[                                                               -(sin(q5)*sin(q2 + q3) - cos(q4)*cos(q5)*cos(q2 + q3))*cos(q6) - sin(q4)*sin(q6)*cos(q2 + q3),                                                                  (sin(q5)*sin(q2 + q3) - cos(q4)*cos(q5)*cos(q2 + q3))*sin(q6) - sin(q4)*cos(q6)*cos(q2 + q3),                                     -sin(q5)*cos(q4)*cos(q2 + q3) - sin(q2 + q3)*cos(q5),                                               -0.303*sin(q5)*cos(q4)*cos(q2 + q3) - 0.303*sin(q2 + q3)*cos(q5) - 1.5*sin(q2 + q3) + 1.25*cos(q2) - 0.054*cos(q2 + q3) + 0.75],
[                                                                                                                                                           0,                                                                                                                                                             0,                                                                                        0,                                                                                                                                                                            1]]))
`

To account for the difference in reference frame orientation between the DH convention and the URDF of the gripper link, first apply a body-fixed rotation about the Z-axis by pi, then a rotation about the X-axis by -pi/2. The transformation matrix for this correction can be derived as below:

![corr_mat][corr_mat]

Thus the final total transformation matrix is simply 

`T_tot = T0_G * R_corr`



#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

##### Inverse Position Kinematics
Here we derive the values of q1, q2, and q3, given the end-effector position. 
First, from the end-effector position, we need to determine the location of the wrist-center. Details can be found in the Project Section 15, but in summary,

- First convert the gripper orientation from quaternion to roll-pitch-yaw
- Construct the Rrpy total transformation matrix
- Extract nx, ny, nz 
- Calculate wrist center

    `wx = px - (d6+l)*nx`

    `wy = py - (d6+l)*ny`

    `wz = pz - (d6+l)*nz`

Now with the wrist center `(wx,wy,wz)`, the first angle can be easily calculated, because q1 is simply the angle the arm makes projected to the xy-plane. Thus,

`theta1 = atan2(wy,wx)`

As for theta2 and theta3, the Project page instructs to use the law of cosines. The derivation of the equations are below. The sides of the triangles can be calculated using Pythagorean theorem. 

![theta23][theta23]

##### Inverse Orientation Kinematics
Instructions to derive the orientation kinematics are shown in Project Section 15. 
Extract the total rotation matrix from the base_link to the gripper in terms of roll-pitch-yaw, along with the correction.

`Rrpy = Rot(Z, yaw) * Rot(Y, pitch) * Rot(X, roll) * R_corr`

Which equals

`R0_6 = R0_1*R1_2*R2_3*R3_4*R4_5*R5_6 = Rrpy`

Premultiply by inv(R0_3).

`R3_6 = R3_4*R4_5*R5_6 = inv(R0_3)*Rrpy`

The right side gives a known numerical value (given the first three angles). And the left side can be expressed analytically. The LHS was determined using sympy, multiplying out `R3_4*R4_5*R5_6` Thus the equations for theta4, theta5, and theta6 can be extracted, similar to extracting Euler Angles from a rotation matrix. (Using atan2() to avoid ambiguous results)
![theta456][theta456]



### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 

As far as the Forward Kinematics (lines 33-129) and Inverse Kinematics (lines 159-199), the code is a relatively straightforward application of the analysis. 

As recommended, I moved the Forward Kinematics portion out of the main loop, and actually placed in outside the function (as global variables) to avoid redundant calculations within the code. 

In testing the code, the Inverse Kinematics seems to do pretty well. There are some instances where the planned path and the actual path vary drastically. One possible reason could be ambiguity from using acos() when solving for theta2, theta3 using Law of Cosines. But even when the path is drastically different, the robotic arm still successfully brings the can to the destination. When the path does vary drastically, the can sometimes collides with other objects (the ground, shelf, garbage can) and the gripper drops the can. Also, as mentioned in the Slack, there seems to be a common issue that the gripper either doesn't pick up the can from the shelf, or sometimes it drops the can while mid-path (no collisions). Similarly, it seems like using 'Continue' results in more errors compared to using 'Next'.

Screenshots of succesful pickup
![DisplayPath][DisplayPath]
![ReachedTarget][ReachedTarget]
![DisplayDropOff][DisplayDropOff]
![ReachedDropOff][ReachedDropOff]
