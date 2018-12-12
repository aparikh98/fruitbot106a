source activate python27
echo 'Running image recognition'
./complete_demo.py
echo 'Sending to remote'
scp output.txt ee106a-ach@cory.eecs.berkeley.edu:~/ros_workspaces/lab7/src/planning/src/
#nT6cK+nA
