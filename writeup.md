## Project: Kinematics Pick & Place

---


**Steps to complete the project:**  


1. [Set up](https://classroom.udacity.com/nanodegrees/nd209/parts/af07ae99-7d69-4b45-ab98-3fde8b576a16/modules/2919466f-aa2b-4424-b86a-98b0a53ce335/lessons/658c94f5-f806-4273-9001-9e2838e56856/concepts/a777bc7a-95d4-44ca-b4e3-119718e3a213) your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. [Experiment with the forward_kinematics environment](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/669ba823-b6bc-418e-814b-1e2e9938f02e) and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. [Fill in](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/06fd4f65-5706-4147-88b1-20dac983676f) the `IK_server.py` with your Inverse Kinematics code. 


[//]: # (Image References)

[image1]: ./misc_images/misc1.png
[image2]: ./misc_images/misc2.png
[image3]: ./misc_images/misc3.png

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

![alt text][image1]

#### 2. Using kr210.urdf.xacro file to figure out the parameters in the DH parameter table.
The data in kr210.urdf.xacro

```
 <joint name="gripper_joint" type="fixed">
    <parent link="link_6"/>
    <child link="gripper_link"/>
    <origin xyz="0.11 0 0" rpy="0 0 0"/><!--0.087-->
    <axis xyz="0 1 0" />
  </joint>

  <joint name="fixed_base_joint" type="fixed">
    <parent link="base_footprint"/>
    <child link="base_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>
  <joint name="joint_1" type="revolute">
    <origin xyz="0 0 0.33" rpy="0 0 0"/>
    <parent link="base_link"/>
    <child link="link_1"/>
    <axis xyz="0 0 1"/>
    <limit lower="${-185*deg}" upper="${185*deg}" effort="300" velocity="${123*deg}"/>
  </joint>
  <joint name="joint_2" type="revolute">
    <origin xyz="0.35 0 0.42" rpy="0 0 0"/>
    <parent link="link_1"/>
    <child link="link_2"/>
    <axis xyz="0 1 0"/>
    <limit lower="${-45*deg}" upper="${85*deg}" effort="300" velocity="${115*deg}"/>
  </joint>
  <joint name="joint_3" type="revolute">
    <origin xyz="0 0 1.25" rpy="0 0 0"/>
    <parent link="link_2"/>
    <child link="link_3"/>
    <axis xyz="0 1 0"/>
    <limit lower="${-210*deg}" upper="${(155-90)*deg}" effort="300" velocity="${112*deg}"/>
  </joint>
  <joint name="joint_4" type="revolute">
    <origin xyz="0.96 0 -0.054" rpy="0 0 0"/>
    <parent link="link_3"/>
    <child link="link_4"/>
    <axis xyz="1 0 0"/>
    <limit lower="${-350*deg}" upper="${350*deg}" effort="300" velocity="${179*deg}"/>
  </joint>
  <joint name="joint_5" type="revolute">
    <origin xyz="0.54 0 0" rpy="0 0 0"/>
    <parent link="link_4"/>
    <child link="link_5"/>
    <axis xyz="0 1 0"/>
    <limit lower="${-125*deg}" upper="${125*deg}" effort="300" velocity="${172*deg}"/>
  </joint>
  <joint name="joint_6" type="revolute">
    <origin xyz="0.193 0 0" rpy="0 0 0"/>
    <parent link="link_5"/>
    <child link="link_6"/>
    <axis xyz="1 0 0"/>
    <limit lower="${-350*deg}" upper="${350*deg}" effort="300" velocity="${219*deg}"/>
  </joint>
```
--------------
```
a1 = 0.35 = x(joint2)
a2 = 1.25 = z(joint3)
a3 = -0.054 = z(joint4)
d1 = 0.75 = 0.33 + 0.42 = z(joint1) + z(joint2)
d4 = 1.5 = 0.96 + 0.54 = x(joint4) + x(joint5)
d7 = 0.303 = 0.11 + 0.193 = x(gripper_joint) + x(joint6)
```
--------------

i | alpha(i-1) | a (i-1) | d (i) | q (i)
--- | --- | --- | --- | ---
1 | 0| 0 | 0.75| 
2 | -pi/2| 0.35| 0| -pi/2
3 | 0| 1.25| 0|
4 | -pi/2| -0.054| 1.50| 
5 | pi/2| 0|  0|
6 | -pi/2| 0| 0|
7 |  0| 0| 0.303| 0

#### 3. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. 
Create transformation matrices:
```
    T0_1 = Matrix([[		    cos(q1),		 -sin(q1),		0,		a0],
			   [	sin(q1)*cos(alpha0),  cos(q1)*cos(alpha0),   -sin(alpha0), -sin(alpha0)*d1],
			   [	sin(q1)*sin(alpha0),  cos(q1)*sin(alpha0),    cos(alpha0),  cos(alpha0)*d1],
			   [			  0,  		 	0,   		0, 		 1]])

    T1_2 = Matrix([[		    cos(q2),		 -sin(q2),		0,		a1],
			   [	sin(q2)*cos(alpha1),  cos(q2)*cos(alpha1),   -sin(alpha1), -sin(alpha1)*d2],
			   [	sin(q2)*sin(alpha1),  cos(q2)*sin(alpha1),    cos(alpha1),  cos(alpha1)*d2],
			   [			  0,  		 	0,   		0, 		 1]])

    T2_3 = Matrix([[		    cos(q3),		 -sin(q3),		0,		a2],
			   [	sin(q3)*cos(alpha2),  cos(q3)*cos(alpha2),   -sin(alpha2), -sin(alpha2)*d3],
			   [	sin(q3)*sin(alpha2),  cos(q3)*sin(alpha2),    cos(alpha2),  cos(alpha2)*d3],
			   [			  0,  		 	0,   		0, 		 1]])

    T3_4 = Matrix([[		    cos(q4),		 -sin(q4),		0,		a3],
			   [	sin(q4)*cos(alpha3),  cos(q4)*cos(alpha3),   -sin(alpha3), -sin(alpha3)*d4],
			   [	sin(q4)*sin(alpha3),  cos(q4)*sin(alpha3),    cos(alpha3),  cos(alpha3)*d4],
			   [			  0,  		 	0,   		0, 		 1]])

    T4_5 = Matrix([[		    cos(q5),		 -sin(q5),		0,		a4],
			   [	sin(q5)*cos(alpha4),  cos(q5)*cos(alpha4),   -sin(alpha4), -sin(alpha4)*d5],
			   [	sin(q5)*sin(alpha4),  cos(q5)*sin(alpha4),    cos(alpha4),  cos(alpha4)*d5],
			   [			  0,  		 	0,   		0, 		 1]])

    T5_6 = Matrix([[		    cos(q6),		 -sin(q6),		0,		a5],
			   [	sin(q6)*cos(alpha5),  cos(q6)*cos(alpha5),   -sin(alpha5), -sin(alpha5)*d6],
			   [	sin(q6)*sin(alpha5),  cos(q6)*sin(alpha5),    cos(alpha5),  cos(alpha5)*d6],
			   [			  0,  		 	0,   		0, 		 1]])

    T6_G = Matrix([[		    cos(q7),		 -sin(q7),		0,		a6],
			   [	sin(q7)*cos(alpha6),  cos(q7)*cos(alpha6),   -sin(alpha6), -sin(alpha6)*d7],
			   [	sin(q7)*sin(alpha6),  cos(q7)*sin(alpha6),    cos(alpha6),  cos(alpha6)*d7],
			   [			  0,  		 	0,   		0, 		 1]])
```

```
    T0_1 = T0_1.subs(dh)
    T1_2 = T1_2.subs(dh)
    T2_3 = T2_3.subs(dh)
    T3_4 = T3_4.subs(dh)
    T4_5 = T4_5.subs(dh)	    
    T5_6 = T5_6.subs(dh)
    T6_G = T6_G.subs(dh)
```

#### 4. Generating a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.
```
	    T0_2 = simplify(T0_1 * T1_2)
	    T0_3 = simplify(T0_2 * T2_3)
	    T0_4 = simplify(T0_3 * T3_4)
	    T0_5 = simplify(T0_4 * T4_5)
	    T0_6 = simplify(T0_5 * T5_6)
	    T0_G = simplify(T0_6 * T6_G)
```
#### 5. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.
 px,py,pz = end-effector position
 x, y, z, w = end-effector orientation parameters
 roll, pitch, yaw = end-effector orientation
```
(roll, pitch, yaw) = tf.transformations.euler_from_quaternion([x, y, z, w])
```
 Calculate joint angles using Geometric IK method
 
 _calculate l for x, y, z position_
```
    lx = cos(roll) * cos(pitch)
    ly = sin(roll) * cos(pitch)
    lz = -sin(roll)
```
 _calculate wrist center for x, y, z position_
```
    d6 = dh['d6']
    l =  dh['d7']
    Wx = px - (d6 + l) * lx 
    Wy = py - (d6 + l) * ly		
    Wz = pz - (d6 + l) * lz
```
 _calculate beda angle_
```
    s = Wz - dh['d1'] 
    r = sqrt(Wx*Wx+Wy*Wy)
    beta = atan2(s, r)
```
### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  


And just for fun, another example image:
![alt text][image3]


