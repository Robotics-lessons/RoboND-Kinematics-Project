## Project: Kinematics Pick & Place

---

## Steps for submit the Kinematics Pick & Place project 

  Because my RoboND-Kinematics-Project in github.com is under Robotics-lessons organizations group,
and not in my popular repositorieds, I can't submit the github link url directly to project review.
I have to zip the folder and upload this zip file to review.

 1. Go to project root
   cd  ~/catkin_ws/src/RoboND-Kinematics-Project

 2. Execute a zip command
   zip -r RoboND-Kinematics-Project.zip RoboND-Kinematics-Project

 3. Open a browser in VM
   Click browser icon in VM

 4. Open Udacity page for Robotics class to submit the project
   Use upload zip file option 

## Add a new file to github.com directly and snyc with locasl repository.
 1. Create a new file on local PC

 2. Go to the project and folder which saves the file on github.com

 3. Upload the file and commit.

 4. Run git pull origin master command on your local PC
 


## Collect the project testing data and run testing program

 1. Run _roslaunch kuka_arm forward_kinematics.launch_

 2. Go scripts folder
    cd ~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/scripts

 3. Add these data in test.py code
 ```
 # inputdata format: [px, py pz, x, y, z, w]
 inputdata = [[2.3146, 0.11282, 2.1129, -0.24965, 0.41624, -0.11376, 0.86688],
        [2.15, 0, 1.94, 0, 0, 0, 1]]
 # outputdata format: [theta1, theta2, theta3, theta4, theta5, theta6]
 outputdata = [[0.11, 0.27, -0.54, -0.53, 1.19, -0.07],
        [0, 0, 0, 0, 0, 0, 0]]

 ```
 4. Run _roscore_ command first

 5. Run _python test.py_ command

 6. The test code will display calculated angle results with expected angle results
 Example:
```
[INFO] [1500130811.462120]: Theta 1 = 0.1104, expected answer = 0.11
[INFO] [1500130811.462485]: Theta 2 = 0.2671, expected answer = 0.27
[INFO] [1500130811.463203]: Theta 3 = -0.5085, expected answer = -0.54
[INFO] [1500130811.464455]: Theta 4 = -2.6846, expected answer = -0.53
[INFO] [1500130811.464794]: Theta 5 = -0.3820, expected answer = 1.19
[INFO] [1500130811.465021]: Theta 6 = 0.1575, expected answer = -0.07
```

## Start Ubuntu virtual machine 
 1. Click VMware wrkstation icon on PC desktop

 2. Select Ubuntu 64-bit Robo V2.0.1 to start the kinematics project

 3. Use Ctl + Alt + t to invoke a terminal

 4. Use . ./go.sh to set running environemnt and go to project folder




