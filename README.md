# fruitbot106a

fruitbot.weebly.com 


How to run:

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
    -rosrun baxter_tools enable_robot.py -e   \
    -roslaunch baxter_moveit_config demo_baxter.launch left_electric_gripper:=true  \
    -rosrun planning fruitbot_pickup.py   
    
Dependencies: We require python 2.7 and opencv 3.4.2+

