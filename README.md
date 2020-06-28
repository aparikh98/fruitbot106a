# Fruitbot 106a

Aakash Parikh, Hung-Ju Wang, Seungwoo Son, Charlie Tian, Chris Mitchell

You can find design, implementation, videos and more details at: https://fruitbot.weebly.com/

### Overview: 

Our project aims to detect various classes of fruits and sort them into target locations.

In order to accomplish this, we need to

- Process and segment images into their various objects and orientations
    
- Develop a backend, offline fruit classifier and use said model to classify the fruits
    
- Translate image coordinates and orientations into real-world coordinates and orientations through homography
    
- Plan and execute paths to pick up and place each fruit while taking in account the physical setup to avoid collisions or damaging any items
    

### How to run:

1) Clone this repository https://github.com/pjreddie/darknet inside the fruitbot directory
2) Save your target image within the directory
3) update send.sh to use your lab login account
4) On your local machine, run bash send.sh
5) Enter your password for SCP
6) Set up your ROS workspace with the files in the folder fruitbotWS 
    This is a similar set up to Lab 7
7) Connect to a baxter robot with a gripper on the left arm
    -Some commands for baxter (may vary based on your setup)   \
    -ln -s /scratch/shared/baxter_ws/baxter.sh ~/ros_workspaces/fruitbotWS/   \
    -./baxter.sh [name-of-robot].local  \
    -source devel/setup.bash \
    -rosrun baxter_tools enable_robot.py -e   \
    -rosrun baxter_interface joint_trajectory_action_server.py  \
    -roslaunch baxter_moveit_config demo_baxter.launch left_electric_gripper:=true  \
    -rosrun planning fruitbot_pickup.py   
    
Dependencies: We require python 2.7 and opencv 3.4.2+

