#!/usr/bin/env python3
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Maciej Wozniak, based on Acorn Pooley, Mike Lautman

## BEGIN_SUB_TUTORIAL imports
##
## To use the Python MoveIt interfaces, we will import the `moveit_commander`_ namespace.
## This namespace provides us with a `MoveGroupCommander`_ class, a `PlanningSceneInterface`_ class,
## and a `RobotCommander`_ class. More on these below. We also import `rospy`_ and some messages that we will use:
##

# Python 2/3 compatibility imports
from __future__ import print_function
import imp
from urllib import response
# from six.moves import input
from rospy import init_node, is_shutdown
import time
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import sensor_msgs.msg
from shape_msgs.msg import Plane
import geometry_msgs.msg
import trajectory_msgs.msg
from std_msgs.msg import Float32
try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))

import math
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from panda_msgs.srv import PandaSimpleService, PandaSimpleServiceResponse, PandaPickUp, PandaPickUpResponse, PandaManyPoses, PandaManyPosesResponse, PandaManyPosesRequest
from panda_msgs.msg import FloatList
from collision_scene_example import CollisionSceneExample 
from scipy.spatial.transform import Rotation
import franka_gripper.msg
import rospy
import sys

# Brings in the SimpleActionClient
import actionlib


# Brings in the messages used by the action, including the
# goal message and the result message.

def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if the values in two lists are within a tolerance of each other.
    For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
    between the identical orientations q and -q is calculated correctly).
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True


class UnityPythonConnector(object):

    def __init__(self):
        super(UnityPythonConnector, self).__init__()

        ## BEGIN_SUB_TUTORIAL setup
        ##
        ## First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize("unity_python_node")
        rospy.init_node("unity_python", anonymous=True)
        joint_state_publisher_Unity = rospy.Publisher("/joint_state_unity", FloatList , queue_size = 10)
        self.plan_publisher = rospy.Publisher(
            "plan_publisher",
            moveit_msgs.msg.RobotTrajectory,
            queue_size=20,
        )
       
        ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        ## kinematic model and the robot's current joint states
        print("loading robot")
        robot = moveit_commander.RobotCommander()
        # group_names = robot.get_group_names()
        print("============ Available Planning Groups:", robot.get_group_names())
        # robot = moveit_msgs.msg.RobotState()
        ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
        ## for getting, setting, and updating the robot's internal understanding of the
        ## surrounding world:
        print("loading planning scene")
        scene = moveit_commander.PlanningSceneInterface()
        tau = 2 * math.pi
        ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        ## to a planning group (group of joints).  In this tutorial the group is the primary
        ## arm joints in the Panda robot, so we set the group's name to "panda_arm".
        ## If you are using a different robot, change this value to the name of your robot
        ## arm planning group.
        ## This interface can be used to plan and execute motions:
        print("loading group")
        group_name = "panda_arm"
        move_group = moveit_commander.MoveGroupCommander(robot.get_group_names()[0], wait_for_servers=150.0)

        ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
        ## trajectories in Rviz:
        # display_trajectory_publisher = rospy.Publisher(
        #     "/move_group/display_planned_path",
        #     moveit_msgs.msg.DisplayTrajectory,
        #     queue_size=20,
        # )
        
        

        ## END_SUB_TUTORIAL

        ## BEGIN_SUB_TUTORIAL basic_info
        ##
        ## Getting Basic Information
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^
        # We can get the name of the reference frame for this robot:
        planning_frame = move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        eef_link = move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        # # We can get a list of all the groups in the robot:
        # group_names = robot.get_group_names()
        # print("============ Available Planning Groups:", robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print("============ Printing robot state")
        # print(robot.get_current_state())
        print("")
        ## END_SUB_TUTORIAL
        self.joint_names = ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5',
                    'panda_joint6','panda_joint7']
        # Misc variables
        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        # self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        # self.group_names = group_names
        # self.add_plane_to_the_scene()
        print("============ publish current joint state")
        print(move_group.get_current_joint_values())
        joint_state_publisher_Unity.publish(move_group.get_current_joint_values())
        print("done")
        self.robot = moveit_msgs.msg.RobotState() # may need to uncomment
        # hand rotation 
        # self.zero_rot = Rotation.from_euler('xyz', [0, 180, 0], degrees=True).as_quat()

        # self.add_plane_to_the_scene()
        # self.add_box() # adding table to the plannig space
        self.stackofplans = [] 

    def generate_JointMsg(self, joints):
        # returns the message with joint states in the form RobotState accepts it 
        newjoint_state = sensor_msgs.msg.JointState()
        for i in range(len(self.joint_names)):
            newjoint_state.name.append(self.joint_names[i])
            newjoint_state.position.append(joints[i])
        return newjoint_state

    def plan_trajectory(self, init_angles, target_pose, verbose = False):
        # plan the traj given angles and target pose 
        self.robot.joint_state = self.generate_JointMsg(init_angles)
        self.move_group.set_start_state(self.robot)     
        self.move_group.set_pose_target(target_pose)
        if verbose:
            plan = self.move_group.plan()[1]
            print(plan)
            return plan
        return self.move_group.plan()[1]
    
    def go_to_pose_goal(self, request):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        # print("service was called")
        # print("pick up cube try")
        # self.move_group.pick("box")
        # print("success?")
        move_group = self.move_group
        # to set up initial joint state we need to define the message as a jointState type msg
        newjoint_state = sensor_msgs.msg.JointState()
        for i in range(len(self.joint_names)):
            newjoint_state.name.append(self.joint_names[i])
            newjoint_state.position.append(request.current_joints[i])
        # setting up initial joint state 
        self.robot.joint_state = newjoint_state
        print(newjoint_state)
        move_group.set_start_state(self.robot)
    
        resp = PandaSimpleServiceResponse()
        move_group.set_pose_target(request.targetpose)
        plan = move_group.plan()
        resp.trajectories.append(plan[1])
        print(resp)
        return resp

    def pickup_plan(self, request):
        # request contain the message of pick up pose and place pose 
        # we need to return path to pre pick up pose
        # plan to pick up 
        # plan pack to pre pick up pose 
        # plan to placing position
        print(request)
        # print(request.pick_pose)
        self.plans = []
        resp = PandaPickUpResponse()
        
        # to set up initial joint state we need to define the message as a jointState type msg       
        # setting up initial joint state 
        # 1 go to pre-pick up pose 
        # pick_pose_rotation = copy.deepcopy(request.pick_pose.orientation)
        pick_pose = copy.deepcopy(request.pick_pose)
        request.pick_pose.position.z = 0.35
        # request.pick_pose.orientation = self.zero_rot
        current_joint_stat = self.move_group.get_current_joint_values()
        plan = self.plan_trajectory(current_joint_stat, request.pick_pose)
        # self.plan_publisher.publish(plan)
        self.plans.append(plan)        
        previous_ending_joint_angles = plan.joint_trajectory.points[-1].positions
        # 2 pick up pose 
        # pick_pose.position.z = 0.23
        plan = self.plan_trajectory(previous_ending_joint_angles, pick_pose)
        # print(plan)
        self.plans.append(plan)   
        previous_ending_joint_angles = plan.joint_trajectory.points[-1].positions
        # pint 2.5 just pick up
        self.plans.append("close")
        # pick_pose.orientation = pick_pose_rotation # rotate to the state of the 
        plan = self.plan_trajectory(previous_ending_joint_angles, pick_pose)
        self.plans.append(plan)
        # 3 come back to pre-place up pose 
        place_pose = copy.deepcopy(request.place_pose)
        request.place_pose.position.z = 0.35
        plan = self.plan_trajectory(previous_ending_joint_angles, request.place_pose)
        self.plans.append(plan)
        previous_ending_joint_angles = plan.joint_trajectory.points[-1].positions
        # 4 come back to pre-pick up pose 
        # request.place_pose.position.z = 0.3
        plan = self.plan_trajectory(previous_ending_joint_angles, place_pose)
        self.plans.append(plan)
        self.plans.append("open")
        # go up
        previous_ending_joint_angles = plan.joint_trajectory.points[-1].positions
        plan = self.plan_trajectory(previous_ending_joint_angles, request.place_pose)
        self.plans.append(plan)

        # go home 
        previous_ending_joint_angles = plan.joint_trajectory.points[-1].positions
        request.pick_pose.position.z = 0.4
        request.pick_pose.position.x = 0.3
        plan = self.plan_trajectory(previous_ending_joint_angles, request.pick_pose)
        self.plans.append(plan)

        resp.trajectories = self.plans
        print("I renturned trajectory")
        # for p in self.plans:
        #     self.plan_publisher.publish(p)
        # try carthesian path
        # wpose = self.move_group.get_current_pose().pose
        # interpose = copy.deepcopy(request.pick_pose)
        # interpose.position.x = 0.26
        # interpose.position.z = -0.363
        # points = [wpose, request.pick_pose, pick_pose, request.pick_pose, interpose, request.place_pose]
        # plan = self.plan_cartesian_path(points)
        # plans = [plan]
        # resp.trajectories = plans
        self.execute_plans(self.plans)
        # return resp

    def knock_down_plan(self, request):
        # request contain the message of pick up pose and place pose 
        # we need to return path to pre pick up pose
        # plan to pick up 
        # plan pack to pre pick up pose 
        # plan to placing position
        print(request)
        # print(request.pick_pose)
        self.plans = []
        resp = PandaPickUpResponse()
        
        # to set up initial joint state we need to define the message as a jointState type msg       
        # setting up initial joint state 
        # 1 go to pre-pick up pose 
        # pick_pose_rotation = copy.deepcopy(request.pick_pose.orientation)
        pick_pose = copy.deepcopy(request.pick_pose)
        request.pick_pose.position.z = 0.35
        # request.pick_pose.orientation = self.zero_rot
        current_joint_stat = self.move_group.get_current_joint_values()
        plan = self.plan_trajectory(current_joint_stat, request.pick_pose)
        # self.plan_publisher.publish(plan)
        self.plans.append(plan)        
        previous_ending_joint_angles = plan.joint_trajectory.points[-1].positions
        # 2 pick up pose 
        # pick_pose.position.z = 0.23
        plan = self.plan_trajectory(previous_ending_joint_angles, pick_pose)
        # print(plan)
        self.plans.append(plan)   
        previous_ending_joint_angles = plan.joint_trajectory.points[-1].positions
        # pint 2.5 just pick up
        self.plans.append("close")
        # pick_pose.orientation = pick_pose_rotation # rotate to the state of the 
        plan = self.plan_trajectory(previous_ending_joint_angles, pick_pose)
        self.plans.append(plan)
        #knockdown
        previous_ending_joint_angles = plan.joint_trajectory.points[-1].positions
        # 4 come back to pre-pick up pose 
        # request.place_pose.position.z = 0.3
        request.place_pose.position.y += 0.11
        request.place_pose.position.y -= 0.1

        plan = self.plan_trajectory(previous_ending_joint_angles, request.place_pose)
        self.plans.append(plan)
        self.plans.append("open")
        # go up
        previous_ending_joint_angles = plan.joint_trajectory.points[-1].positions
        plan = self.plan_trajectory(previous_ending_joint_angles, request.place_pose)
        self.plans.append(plan)

        # go home 
        previous_ending_joint_angles = plan.joint_trajectory.points[-1].positions
        request.pick_pose.position.z = 0.4
        request.pick_pose.position.x = 0.3
        plan = self.plan_trajectory(previous_ending_joint_angles, request.pick_pose)
        self.plans.append(plan)

        resp.trajectories = self.plans
        print("I renturned trajectory")
        self.execute_plans(self.plans)

    def move_gripper(self, movement="open"):
        
        # Creates the SimpleActionClient, passing the type of the action
        client = actionlib.SimpleActionClient('/franka_gripper/move', franka_gripper.msg.MoveAction)

        # Waits until the action server has started up and started
        # listening for goals.
        client.wait_for_server()

        # Creates a goal to send to the action server.
        if movement == "close":
            goal = franka_gripper.msg.MoveGoal(width=0.04, speed=1.0)
        else:
            goal = franka_gripper.msg.MoveGoal(width=0.068, speed=1.0)
        #goal.width = 0.022
        #goal.speed = 1.0
        
        # Sends the goal to the action server.
        client.send_goal(goal)

        # Waits for the server to finish performing the action.
        # client.wait_for_result()
        time.sleep(1)
        # Prints out the result of executing the action
        return client.get_result()  # A move result

    def pick_no_place(self, request):
        """
        request to move around without picking or placing the objects
        """
        response = PandaManyPosesResponse()

        # group_name = "arm"
        # move_group = moveit_commander.MoveGroupCommander(group_name)

        # current_robot_joint_configuration = request.current_joints
        robot_poses = []
        print(f"this is PICK AND PLACE \n request : {request} \n current state: {request.current_joints} ")

        # Pre grasp - position gripper directly above target object
        for i in range(len(request.poses)):
            if (i == 0):
                robot_poses.append(self.plan_trajectory(request.current_joints, request.poses[0]))

                # robot_poses.append(plan_trajectory(move_group, req.poses[0], current_robot_joint_configuration))
            else:
                robot_poses.append(self.plan_trajectory(robot_poses[i-1].joint_trajectory.points[-1].positions, request.poses[i]))
                # robot_poses.append(plan_trajectory(move_group, req.poses[i], robot_poses[i-1].joint_trajectory.points[-1].positions))
        
            if not robot_poses[i].joint_trajectory.points:
                return response
        response.trajectories = robot_poses
        # If trajectory planning worked for all pick and place stages, add plan to response
        print("im good boy i returned responsed")
        return response
    
    def execute_plans(self, plans):
        # executes plans on the real world robot (or rviz robot)
        for plan in plans:
            if isinstance(plan, str):
                self.move_gripper(plan)
                continue
            self.move_group.execute(plan, wait=True)
        # pass
    
def generate_knockover_request(which):
    request = PandaManyPosesRequest()
    
    request.pick_pose.orientation.x =  0.9452608
    request.pick_pose.orientation.y =  -0.3150869
    request.pick_pose.orientation.z =  0.0
    request.pick_pose.orientation.w =  -0.0848662
    request.place_pose.position.z = .13
    request.place_pose.position.y = -0.3
    request.place_pose.position.x = .3

    request.place_pose.orientation = request.pick_pose.orientation

    if which == "none" or which=="":
        request.pick_pose.position.z = 0.11
        request.pick_pose.position.y = -0.1
        request.pick_pose.position.x = 0.53

        
    if which == "one" or which=="1":
        request.pick_pose.position.z = 0.12
        request.pick_pose.position.y = -0.02
        request.pick_pose.position.x = 0.53

    if which == "two" or which=="2":
        request.pick_pose.position.z = 0.12
        request.pick_pose.position.y = -0.02
        request.pick_pose.position.x = 0.43

    if which == "three" or which=="3":
        request.pick_pose.position.z = 0.12
        request.pick_pose.position.y = -0.02
        request.pick_pose.position.x = 0.63

    return request
    
def generate_request(which):
    request = PandaManyPosesRequest()
    
    request.pick_pose.orientation.x =  0.9452608
    request.pick_pose.orientation.y =  -0.3150869
    request.pick_pose.orientation.z =  0.0
    request.pick_pose.orientation.w =  -0.0848662

    request.place_pose.orientation = request.pick_pose.orientation

    if which == "one" or which=="1":
        request.pick_pose.position.z = 0.12
        request.pick_pose.position.y = -0.02
        request.pick_pose.position.x = 0.53

        request.place_pose.position.z = .13
        request.place_pose.position.y = -0.3
        request.place_pose.position.x = .3
    if which == "two" or which=="2":
        request.pick_pose.position.z = 0.12
        request.pick_pose.position.y = -0.02
        request.pick_pose.position.x = 0.43

        request.place_pose.position.z = .17
        request.place_pose.position.y = -0.3
        request.place_pose.position.x = .3
    if which == "three" or which=="3":
        request.pick_pose.position.z = 0.12
        request.pick_pose.position.y = -0.02
        request.pick_pose.position.x = 0.63

        request.place_pose.position.z = .203
        request.place_pose.position.y = -0.3
        request.place_pose.position.x = .3
    return request

def generate_picknoplace(which):
    request = PandaManyPosesRequest()
    
    request.pick_pose.orientation.x =  0.9452608
    request.pick_pose.orientation.y =  -0.3150869
    request.pick_pose.orientation.z =  0.0
    request.pick_pose.orientation.w =  -0.0848662

    request.place_pose.orientation = request.pick_pose.orientation

    if which == "one" or which=="1":
        request.pick_pose.position.z = 0.12
        request.pick_pose.position.y = -0.02
        request.pick_pose.position.x = 0.53

        request.place_pose.position.z = .13
        request.place_pose.position.y = -0.3
        request.place_pose.position.x = .6
    if which == "two" or which=="2":
        request.pick_pose.position.z = 0.12
        request.pick_pose.position.y = -0.02
        request.pick_pose.position.x = 0.43

        request.place_pose.position.z = .13
        request.place_pose.position.y = -0.3
        request.place_pose.position.x = .6
    if which == "three" or which=="3":
        request.pick_pose.position.z = 0.12
        request.pick_pose.position.y = -0.02
        request.pick_pose.position.x = 0.63

        request.place_pose.position.z = .13
        request.place_pose.position.y = -0.3
        request.place_pose.position.x = .6
    return request

def initialize():
    try:
        
        tutorial = UnityPythonConnector()
        # tutorial.add_box()
        # coll = CollisionSceneExample()
        # coll.add_table()
        # add wall behind the robot 
        # pose = [-0.6, 0, 0, 0, 0.707, 0, 0.707]
        # dimensions = [1.2, 1.2, 0.0001]
        # coll.add_table(pose, dimensions, "wall")
        # # # add box to pick up
        # # pose = [0.5, 0.2, 0.1, 0, 0., 0, 1]
        # # dimensions = [0.03, 0.03, 0.03]
        # rot = Rotation.from_euler('xyz', [0, 0, 90], degrees=True)
        # rot = rot.as_quat()
        # pose = [0.38, 0, 0.02, rot[0], rot[1], rot[2], rot[3]] # in robot frame so table is Y 0
        # dimensions = [1.5, 0.28, 0.11] # this are dims for RViz not unity so X,Y,Z (Unity Z, X, Y) - possible to flip it 
        # coll.add_table(pose, dimensions, "carton_box")
        print("Ready to plan")
        s = rospy.Service('moveit_many', PandaSimpleService, tutorial.go_to_pose_goal) # go_to_pose_goal
        s2 = rospy.Service('panda_move', PandaPickUp, tutorial.pickup_plan)
        s3 = rospy.Service('waypoints_service', PandaManyPoses, tutorial.pick_no_place)
        sub1 = rospy.Subscriber('plan_publisher', moveit_msgs.msg.RobotTrajectory, tutorial.execute_plans)
        sub2 = rospy.Subscriber('realrobot_publisher', moveit_msgs.msg.RobotTrajectory, tutorial.execute_plans)

        # s4 = rospy.Service('waypoints_service', moveit_msgs.msg.RobotTrajectory, tutorial.execute_plans)
        # rospy.spin()
        while not is_shutdown():
            print("What do you want to do?")
            action = input()
            if action=="pickup":
                which = input("which one?")
                request = generate_request(which)
                tutorial.pickup_plan(request)
            if action == "gripper":
                # tutorial.move_gripper('close')
                tutorial.move_gripper('open')
            if action == "knockover":
                which = input("which one?")
                request = generate_knockover_request(which)
                # tutorial.pickup_plan(request)
                tutorial.knock_down_plan(request)
            if action=="nopick":
                which = input("which one?")
                request = generate_picknoplace(which)
                tutorial.pickup_plan(request)
        return
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return
    
def main():
    initialize()

if __name__ == "__main__":
    main()