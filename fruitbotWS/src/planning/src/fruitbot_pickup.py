#!/usr/bin/env python
"""
Path planning script for fruit bot pick up

Code uses certain components from lab 7 by Valmik Prabhu
"""

import sys
import rospy
import numpy as np

from moveit_msgs.msg import OrientationConstraint
from geometry_msgs.msg import PoseStamped

from path_planner import PathPlanner
from baxter_interface import Limb
# from intera_interface import Limb
from baxter_interface import gripper as robot_gripper
import time
#testing decide orientations below
from tf import transformations
#testing decide orientations above

def euler_to_quaternion(yaw):
    """
    Input: roatation around the z axis
    Output: a 4 element list in quaternion form, which corresponds to the orientation
    of the gripper.
    """
    yaw =  float(yaw)
    quaternion =  transformations.quaternion_from_euler(np.pi, 0, yaw)
    return [quaternion[0], quaternion[1], quaternion[2], quaternion[3]]


def generate_obstacle (x,y):
    obs = PoseStamped()

    obsleft.header.frame_id = "base"

    obsleft.pose.position.x = 0
    obsleft.pose.position.y = 0
    obsleft.pose.position.z = 0.764

    obsleft.pose.orientation.x = 1
    obsleft.pose.orientation.y = 0
    obsleft.pose.orientation.z = 0
    obsleft.pose.orientation.w = 0

    obsleft_size = [max(x-.1, 0), 1, 0.2]
    planner_left.add_box_obstacle(obsleft_size, "left", obsleft)

    obsright.header.frame_id = "base"

    obsright.pose.position.x = x+0.1
    obsright.pose.position.y = 0
    obsright.pose.position.z = 0.764

    obsright.pose.orientation.x = 1
    obsright.pose.orientation.y = 0
    obsright.pose.orientation.z = 0
    obsright.pose.orientation.w = 0

    obsright_size = [min(1-(x+.1),0), 1, 0.2]
    planner_left.add_box_obstacle(obsright_size, "right", obsright)

    obsright.header.frame_id = "base"

    obsdown.pose.position.x = 0
    obsdown.pose.position.y = 0
    obsdown.pose.position.z = 0.764

    obsdown.pose.orientation.x = 1
    obsdown.pose.orientation.y = 0
    obsdown.pose.orientation.z = 0
    obsdown.pose.orientation.w = 0

    obsdown_size = [1, max(x-.1, 0), min(1-(x+.1),0), 1, 0.2]
    planner_left.add_box_obstacle(obsright_size, "down", obsright)

    obsup.header.frame_id = "base"

    obsup.pose.position.x = 0
    obsup.pose.position.y = y + 0.1
    obsup.pose.position.z = 0.764

    obsup.pose.orientation.x = 1
    obsup.pose.orientation.y = 0
    obsup.pose.orientation.z = 0
    obsup.pose.orientation.w = 0

    obsright_size = [0, min(1-(y+.1),0), 0.2]
    planner_left.add_box_obstacle(obsright_size, "up", obsright)


    def remove_obstacle():
        planner_left.remove_obstacle("left")
        planner_left.remove_obstacle("right")
        planner_left.remove_obstacle("up")
        planner_left.remove_obstacle("down")


        return


def main(list_of_class_objs, basket_coordinates, planner_left):
    """
    Main Script
    input:first argument: a list of classX_objs. Ex: [class1_objs, class2_objs] where class1_objs contains the same kind of fruits
         second argument: a list of baskets coordinates
    """

    # Make sure that you've looked at and understand path_planner.py before starting

    planner_left = PathPlanner("left_arm")

    home_coordinates = [0.544, 0.306, 0.3]

    home =  PoseStamped()
    home.header.frame_id = "base"

    #home x,y,z position
    home_x = home_coordinates[0]
    home_y = home_coordinates[1]
    home_z = home_coordinates[2]
    home.pose.position.x = home_x
    home.pose.position.y = home_y
    home.pose.position.z = home_z

    #Orientation as a quaternion
    home.pose.orientation.x = 1.0
    home.pose.orientation.y = 0.0
    home.pose.orientation.z = 0.0
    home.pose.orientation.w = 0.0

    intermediate_obstacle = PoseStamped()
    intermediate_obstacle.header.frame_id = "base"

    intermediate_obstacle.pose.position.x = 0
    intermediate_obstacle.pose.position.y = 0
    intermediate_obstacle.pose.position.z = 0.764

    intermediate_obstacle.pose.orientation.x = 1
    intermediate_obstacle.pose.orientation.y = 0
    intermediate_obstacle.pose.orientation.z = 0
    intermediate_obstacle.pose.orientation.w = 0

    intermediate_size = [1, 1, 0.2]

    left_gripper = robot_gripper.Gripper('left')
    print('Calibrating...')
    left_gripper.calibrate()


    while not rospy.is_shutdown():
        try:
            #GO HOME
            plan = planner_left.plan_to_pose(home, list())


            raw_input("Press <Enter> to move the left arm to home position: ")
            if not planner_left.execute_plan(plan):
                raise Exception("Execution failed")
        except Exception as e:
            print e
        else:
            break

    for i, classi_objs in enumerate(list_of_class_objs):
        #x, y,z, orientation of class(basket)

        print("processing class: " + str(i))


        classi_position = basket_coordinates[i]
        classi = PoseStamped()
        classi.header.frame_id = "base"
        classi_x = classi_position[0]
        classi_y = classi_position[1]
        classi_z = classi_position[2]
        classi.pose.position.x = classi_x
        classi.pose.position.y = classi_y
        classi.pose.position.z = classi_z +.1
        classi.pose.orientation.x = 1.0
        classi.pose.orientation.y = 0.0
        classi.pose.orientation.z = 0.0
        classi.pose.orientation.w = 0.0


        for fruit in classi_objs:
            gripper_yaw = fruit[3]
            fruit_x = fruit[0]
            fruit_y = fruit[1]
            fruit_z = fruit[2]
            gripper_orientation = euler_to_quaternion(gripper_yaw)

            orien_const = OrientationConstraint()
            orien_const.link_name = "left_gripper";
            orien_const.header.frame_id = "base";
            gripper_orientation_x = gripper_orientation[0]
            gripper_orientation_y = gripper_orientation[1]
            gripper_orientation_z = gripper_orientation[2]
            gripper_orientation_w = gripper_orientation[3]
            orien_const.orientation.x = gripper_orientation_x
            orien_const.orientation.y = gripper_orientation_y
            orien_const.orientation.z = gripper_orientation_z
            orien_const.orientation.w = gripper_orientation_w
            orien_const.absolute_x_axis_tolerance = 0.1
            orien_const.absolute_y_axis_tolerance = 0.1
            orien_const.absolute_z_axis_tolerance = 0.1
            orien_const.weight = 1.0


            print('Opening...')
            left_gripper.open()
            rospy.sleep(1.0)

            while not rospy.is_shutdown():
                try:
                    planner_left.add_box_obstacle(intermediate_obstacle, "intermediate", table_pose)

                    #intermidiate_to_fruit stage: move to the top of the fruit location and open the gripper
                    intermidiate_to_fruit = PoseStamped()
                    intermidiate_to_fruit.header.frame_id = "base"

                   #x,y,z position

                    intermidiate_to_fruit.pose.position.x = fruit_x
                    intermidiate_to_fruit.pose.position.y = fruit_y
                    intermidiate_to_fruit.pose.position.z = home_z - .1
                    print("Trying to reach intermeidiate_to_fruit position :" + str(fruit_x) + " " + str(fruit_y) + " " + str(fruit_z))


                    intermidiate_to_fruit.pose.orientation.x = gripper_orientation_x
                    intermidiate_to_fruit.pose.orientation.y = gripper_orientation_y
                    intermidiate_to_fruit.pose.orientation.z = gripper_orientation_z
                    intermidiate_to_fruit.pose.orientation.w = gripper_orientation_w

                    plan = planner_left.plan_to_pose(intermidiate_to_fruit, list())

                    raw_input("Press <Enter> to move the left arm to intermidiate_to_fruit position: ")
                    if not planner_left.execute_plan(plan):
                        raise Exception("Execution failed")
                    planner_left.remove_obstacle("intermediate")
                except Exception as e:
                    print e
                else:
                    break

            while not rospy.is_shutdown():

                try:
                    #go down to the actual height of the fruit and close gripper
                    fruitobs = generate_obstacle(fruit_x, fruit_y)
                    fruit =  PoseStamped()
                    fruit.header.frame_id = "base"

                    #x,y,z position
                    fruit.pose.position.x = fruit_x
                    fruit.pose.position.y = fruit_y
                    fruit.pose.position.z = fruit_z
                    print("Trying to reach fruit position :" + str(fruit_x) + " " + str(fruit_y) + " " + str(fruit_z))

                    #Orientation as a quaternion
                    fruit.pose.orientation.x = gripper_orientation_x
                    fruit.pose.orientation.y = gripper_orientation_y
                    fruit.pose.orientation.z = gripper_orientation_z
                    fruit.pose.orientation.w = gripper_orientation_w

                    plan = planner_left.plan_to_pose(fruit, [orien_const])


                    raw_input("Press <Enter> to move the left arm to fruit position: ")
                    if not planner_left.execute_plan(plan):
                        raise Exception("Execution failed")
                    fruitobs()
                except Exception as e:
                    print e

                else:
                    break

            #close the gripper
            print('Closing...')
            left_gripper.close()
            rospy.sleep(1.0)

            while not rospy.is_shutdown():
                try:
                    fruitobs = generate_obstacle(fruit_x, fruit_y)
                    #intermidiate_to_basket stage1: Lift up to a height higher than the height of the basket

                    firt_intermidiate_to_class_i = PoseStamped()
                    firt_intermidiate_to_class_i.header.frame_id = "base"

                    #x, y, and z position
                    firt_intermidiate_to_class_i.pose.position.x = fruit_x
                    firt_intermidiate_to_class_i.pose.position.y = fruit_y
                    firt_intermidiate_to_class_i.pose.position.z = classi_z + 0.25
                    print("Trying to reach intermidiate_to_class_i position :" + str(fruit_x) + " " + str(fruit_y) + " " + str(classi_position[2] + 0.2))



                    #Orientation as a quaternion
                    firt_intermidiate_to_class_i.pose.orientation.x = 1.0
                    firt_intermidiate_to_class_i.pose.orientation.y = 0.0
                    firt_intermidiate_to_class_i.pose.orientation.z = 0.0
                    firt_intermidiate_to_class_i.pose.orientation.w = 0.0

                    plan = planner_left.plan_to_pose(firt_intermidiate_to_class_i, list())


                    raw_input("Press <Enter> to move the left arm to first_intermidiate_to_class_" + str(i) + "position: ")
                    if not planner_left.execute_plan(plan):
                        raise Exception("Execution failed")
                    fruitobs()
                except Exception as e:
                    print e
                else:
                    break

            while not rospy.is_shutdown():
                try:
                    planner_left.add_box_obstacle(intermediate_obstacle, "intermediate", table_pose)

                    #intermidiate_to_basket stage2: Move to the top of the basket
                    intermidiate_to_class_i = PoseStamped()
                    intermidiate_to_class_i.header.frame_id = "base"

                    #x, y, and z position
                    intermidiate_to_class_i.pose.position.x = classi_x
                    intermidiate_to_class_i.pose.position.y = classi_y
                    intermidiate_to_class_i.pose.position.z = classi_z + 0.2
                    print("Trying to reach intermidiate_to_class_i position :" + str(fruit_x) + " " + str(fruit_y) + " " + str(classi_position[2] + 0.2))


                    #Orientation as a quaternion
                    intermidiate_to_class_i.pose.orientation.x = 1.0
                    intermidiate_to_class_i.pose.orientation.y = 0.0
                    intermidiate_to_class_i.pose.orientation.z = 0.0
                    intermidiate_to_class_i.pose.orientation.w = 0.0

                    plan = planner_left.plan_to_pose(intermidiate_to_class_i, list())


                    raw_input("Press <Enter> to move the left arm to second_intermidiate_to_class_" + str(i) + "position: ")
                    if not planner_left.execute_plan(plan):
                        raise Exception("Execution failed")
                    planner_left.remove_obstacle("intermediate")
                except Exception as e:
                    print e
                else:
                    break

            while not rospy.is_shutdown():
                try:
                    #basket stage: put the fruit in the basket
                    classi_obs = generate_obstacle(classi_x, class_y)
                    plan = planner_left.plan_to_pose(classi, list())
                    raw_input("Press <Enter> to move the left arm to sclass_" + str(i) + "position: ")
                    if not planner_left.execute_plan(plan):
                        raise Exception("Execution failed")
                    classi_obs()
                except Exception as e:
                    print e
                else:
                    break

            #Open the gripper
            print('Opening...')
            left_gripper.open()
            rospy.sleep(1.0)

            while not rospy.is_shutdown():
                try:
                    #intermidiate to home stage: lifting up to the home position height


                    intermidiate_to_home_1 = PoseStamped()
                    intermidiate_to_home_1.header.frame_id = "base"

                    #x, y, and z position
                    intermidiate_to_home_1.pose.position.x = classi_x
                    intermidiate_to_home_1.pose.position.y = classi_y
                    intermidiate_to_home_1.pose.position.z = home_z


                    #Orientation as a quaternion
                    intermidiate_to_home_1.pose.orientation.x = 1.0
                    intermidiate_to_home_1.pose.orientation.y = 0.0
                    intermidiate_to_home_1.pose.orientation.z = 0.0
                    intermidiate_to_home_1.pose.orientation.w = 0.0

                    plan = planner_left.plan_to_pose(intermidiate_to_home_1, list())


                    raw_input("Press <Enter> to move the right arm to intermidiate_to_home_1 position: ")
                    if not planner_left.execute_plan(plan):
                        raise Exception("Execution failed")
                except Exception as e:
                    print e
                else:
                    break

    planner_left.shutdown()



if __name__ == '__main__':
    rospy.init_node('moveit_node')

    planner_left = PathPlanner("left_arm")

    table_pose = PoseStamped()

    table_pose.header.frame_id = "base"

    table_pose.pose.position.x = 0
    table_pose.pose.position.y = 0
    table_pose.pose.position.z = 0

    table_pose.pose.orientation.x = 1
    table_pose.pose.orientation.y = 0
    table_pose.pose.orientation.z = 0
    table_pose.pose.orientation.w = 0

    table_size = [1, 1, 0.764]

    planner_left.add_box_obstacle(table_size, "table", table_pose)


    obstacles = list()
    calibration = false
    if calibration :

        while not rospy.is_shutdown():
            try:

                #intermidiate_to_fruit stage: move to the top of the fruit location and open the gripper
                calib1 = PoseStamped()
                calib1.header.frame_id = "base"

               #x,y,z position

                calib1.pose.position.x = 0.35
                calib1.pose.position.y = 0.75
                calib1.pose.position.z = -.23


                calib1.pose.orientation.x = 1
                calib1.pose.orientation.y = 0
                calib1.pose.orientation.z = 0
                calib1.pose.orientation.w = 0

                plan = planner_left.plan_to_pose(calib1, obstacles)

                raw_input("Press <Enter> to move the left arm to calib1 position: ")
                if not planner_left.execute_plan(plan):
                    raise Exception("Execution failed")
                else:
                    break
            except Exception as e:
                print e
        while not rospy.is_shutdown():
            try:

                #intermidiate_to_fruit stage: move to the top of the fruit location and open the gripper
                calib1 = PoseStamped()
                calib1.header.frame_id = "base"

               #x,y,z position

                calib1.pose.position.x = 0.35
                calib1.pose.position.y = 0.15
                calib1.pose.position.z = -.23


                calib1.pose.orientation.x = 1
                calib1.pose.orientation.y = 0
                calib1.pose.orientation.z = 0
                calib1.pose.orientation.w = 0

                plan = planner_left.plan_to_pose(calib1, obstacles)

                raw_input("Press <Enter> to move the left arm to calib2 position: ")
                if not planner_left.execute_plan(plan):
                    raise Exception("Execution failed")
                else:
                    break
            except Exception as e:
                print e
        while not rospy.is_shutdown():
            try:

                #intermidiate_to_fruit stage: move to the top of the fruit location and open the gripper
                calib1 = PoseStamped()
                calib1.header.frame_id = "base"

               #x,y,z position

                calib1.pose.position.x = 0.77
                calib1.pose.position.y = 0.15
                calib1.pose.position.z = -.23


                calib1.pose.orientation.x = 1
                calib1.pose.orientation.y = 0
                calib1.pose.orientation.z = 0
                calib1.pose.orientation.w = 0

                plan = planner_left.plan_to_pose(calib1, obstacles)

                raw_input("Press <Enter> to move the left arm to calib3 position: ")
                if not planner_left.execute_plan(plan):
                    raise Exception("Execution failed")
                else:
                    break
            except Exception as e:
                print e


        while not rospy.is_shutdown():
            try:

                #intermidiate_to_fruit stage: move to the top of the fruit location and open the gripper
                calib1 = PoseStamped()
                calib1.header.frame_id = "base"

               #x,y,z position

                calib1.pose.position.x = 0.67
                calib1.pose.position.y = 0.65
                calib1.pose.position.z = -.23


                calib1.pose.orientation.x = 1
                calib1.pose.orientation.y = 0
                calib1.pose.orientation.z = 0
                calib1.pose.orientation.w = 0

                plan = planner_left.plan_to_pose(calib1, obstacles)

                raw_input("Press <Enter> to move the left arm to calib4 position: ")
                if not planner_left.execute_plan(plan):
                    raise Exception("Execution failed")
                else:
                    break
            except Exception as e:
                print e

    raw_input("Press Enter to Continue")

    fruits = []
    bowls = []
    realWorldObjects = []
    while True:
        try:
            f = open('src/planning/src/output.txt')
            for line in f:
                inner_list = [elt.strip()for elt in line.split(',')]
                inner_list = [float(inner_list[elt]) if elt < 4 else inner_list[elt][1:-1] for elt in range(len(inner_list))]
                realWorldObjects.append(inner_list)
            f.close()
            break
        except IOError:
            print("IO")
            time.sleep(5)
            continue
    print(realWorldObjects)
    bowls = [obj for obj in realWorldObjects if obj[4] == 'bowl']
    apples = [obj for obj in realWorldObjects if obj[4] == 'apple']
    bananas = [obj for obj in realWorldObjects if obj[4] == 'banana']
    print(bowls)
    print(apples)
    print(bananas)
    main([apples, bananas], bowls, planner_left)
