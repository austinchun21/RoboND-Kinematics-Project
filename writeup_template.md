## Project: Kinematics Pick & Place
### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---


**Steps to complete the project:**  


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code. 


[//]: # (Image References)

[image1]: ./misc_images/misc1.png
[image2]: ./misc_images/misc3.png
[image3]: ./misc_images/misc2.png
[joints_links]: ./writeup_images/label_joints_links.png
[axis_origins]: ./writeup_images/axis_origins.png
[dh_params]: ./writeup_images/DH_params.png
[joint_dims]: ./writeup_images/joint_dims.png
[trans_mat]: ./writeup_images/trans_mat.png
[corr_mat]: ./writeup_images/corr_mat.png
[theta23]: ./writeup_images/theta23.jpg

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

Following the DH PArameter Assignment algorithm, explained in Lesson 12.13, the first steps are to label all joints and links, then assign the axes (Z first, then X) for each link. The video guide in the KR210 Forward Kinematics 1 section provides the following joint/link labels, and corresponding axes assignments.

![label joints and links][joints_links]
![label axis and origins][axis_origins]

I used these labels for deriving the DH parameters, also provided in the video guide, with the non-zero DH lengths shown below. All theta values are variables (except the gripper link which is rigidly fixed). However there is some variation on alpha (twist angle) values. Focusing on the angle between adjacent Z axes, the non-zero alpha values are alpha1 = alpha3 = alpha5 = -90 and alpha4 = 90.

![DH params][dh_params]

Lastly, the actual dimensions of the links can be extracted from the kr210.urdf.xacro file, which can then be used to give exact values for the non-zero lengths shown previously. 

![joint_dims][joint_dims]

The resulting DH Table is as follows:

#### \   DH Parameter Table
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

Thus, to get the transformation matrix from the base_link to the gripper_link, simply multiply the transformation matrices

`T0_G = T0_1*T1_2*T2_3*T3_4*T4_5*T5_6*T6_G`

To account for the difference in refrence frame orientation between the DH convention and the URDF of the gripper link, first apply a body-fixed rotation about the Z-axis by pi, then a rotation about the X-axis by -pi/2. The transformation matrix for this correction can be derived as below:

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

Now with the wrist center `(wx,wy,wz)`, the first angle can be easilly calculated, because q1 is simply the angle the arm makes projected to the xy-plane. Thus,

`theta1 = atan2(wy,wx)`

As for theta2 and theta3, the Project page hints to use the law of cosines. The derivation of the equations are below. The sides of the triangles can be calculated using pythagorean theorem

![theta23][theta23]

##### Inverse Orientation Kinematics


### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  


And just for fun, another example image:
![alt text][image3]


