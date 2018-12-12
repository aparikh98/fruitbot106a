source activate python27
echo 'Running image recognition'
./fruitbot_image.py
echo 'Sending to remote'
scp output.txt ee106a-ach@cory.eecs.berkeley.edu:~/ros_workspaces/lab7/src/planning/src/
#replace with your lab login
