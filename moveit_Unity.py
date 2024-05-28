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
    

    
    def pickup_plan(self, request, drop = False):
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
        pick_pose = copy.deepcopy(request.pick_pose)
        request.pick_pose.position.z = 0.35
        # request.pick_pose.orientation = self.zero_rot
        current_joint_stat = self.move_group.get_current_joint_values()
        (plan, fraction) = self.move_group.compute_cartesian_path(
                                   request.pick_pose,   # waypoints to follow
                                   0.01,        # eef_step
                                   0.0)  
        # plan = self.plan_trajectory(current_joint_stat, request.pick_pose)
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
        if drop:
            self.plans.append("open")
        previous_ending_joint_angles = plan.joint_trajectory.points[-1].positions
        # 4 come back to pre-pick up pose 
        # request.place_pose.position.z = 0.3
        plan = self.plan_trajectory(previous_ending_joint_angles, place_pose)
        self.plans.append(plan)

        if not(drop):
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
        # return resp


    def pickup_plan_cartesian(self, request, drop = False):
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
        pick_pose = copy.deepcopy(request.pick_pose)
        request.pick_pose.position.z = 0.35
        (plan, fraction) = self.move_group.compute_cartesian_path(
                                   [request.pick_pose],   # waypoints to follow
                                   0.01,        # eef_step
                                   0.0)  
        self.execute_plans([plan])        


        # 2 pick up pose 
        # pick_pose.position.z = 0.23
        (plan, fraction) = self.move_group.compute_cartesian_path(
                                   [pick_pose],   # waypoints to follow
                                   0.01,        # eef_step
                                   0.0)  
        self.execute_plans([plan])        
        self.execute_plans(["close"])            
        # pint 2.5 just pick up

        (plan, fraction) = self.move_group.compute_cartesian_path(
                                   [request.pick_pose],   # waypoints to follow
                                   0.01,        # eef_step
                                   0.0)  
        self.execute_plans([plan])        
        # 3 come back to pre-place up pose 
        place_pose = copy.deepcopy(request.place_pose)
        drop_pose = copy.deepcopy(request.place_pose)
        request.place_pose.position.z += 0.1
        if drop:
            drop_pose.position.z = 0.3
            drop_pose.position.y += 0.15
            drop_pose.position.x += 0.15
            (plan, fraction) = self.move_group.compute_cartesian_path(
                                   [drop_pose],   # waypoints to follow
                                   0.01,        # eef_step
                                   0.0)  
            self.execute_plans([plan]) 
            self.execute_plans(["open"]) 
        else: 
            (plan, fraction) = self.move_group.compute_cartesian_path(
                                       [request.place_pose],   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)  
            self.execute_plans([plan])  

            # 4 come back to pre-pick up pose
            (plan, fraction) = self.move_group.compute_cartesian_path(
                                       [place_pose],   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0) 
            self.execute_plans([plan]) 


            self.execute_plans(["open"])
            place_pose.position.z += 0.1
            # 4 come back to pre-pick up pose
            (plan, fraction) = self.move_group.compute_cartesian_path(
                                    [place_pose],   # waypoints to follow
                                    0.01,        # eef_step
                                    0.0) 
            self.execute_plans([plan]) 

        request.pick_pose.position.z = 0.4
        request.pick_pose.position.x = 0.3
        print("I renturned trajectory") 
        (plan, fraction) = self.move_group.compute_cartesian_path(
                                   [request.pick_pose],   # waypoints to follow
                                   0.01,        # eef_step
                                   0.0) 
        self.execute_plans([plan]) 

        

  

    def knock_down_plan(self, request, request2, place):
        # request contain the message of pick up pose and place pose 
        # we need to return path to pre pick up pose
        # plan to pick up 
        # plan pack to pre pick up pose 
        # plan to placing position
        print(request)
        home = copy.deepcopy(request.pick_pose)
        home.position.z = 0.45
        home.position.x = 0.3 
        home.position.y = 0
        # print(request.pick_pose)
        self.plans = []
        resp = PandaPickUpResponse()
        if place == "place":
            pick_pose = copy.deepcopy(request.pick_pose)
            request.pick_pose.position.z = 0.35
            (plan, fraction) = self.move_group.compute_cartesian_path(
                                    [request.pick_pose],   # waypoints to follow
                                    0.01,        # eef_step
                                    0.0)  
            self.execute_plans([plan])        


            # 2 pick up pose 
            # pick_pose.position.z = 0.23
            (plan, fraction) = self.move_group.compute_cartesian_path(
                                    [pick_pose],   # waypoints to follow
                                    0.01,        # eef_step
                                    0.0)  
            self.execute_plans([plan])        
            self.execute_plans(["close"])  
                  
            # pint 2.5 just pick up

            (plan, fraction) = self.move_group.compute_cartesian_path(
                                    [pick_pose],   # waypoints to follow
                                    0.01,        # eef_step
                                    0.0)  
            self.execute_plans([plan])      

        waypoints = []    
        request.pick_pose.position.z = 0.35
        pick_pose = copy.deepcopy(request.pick_pose)
        place_pose = copy.deepcopy(request.post_place_pose)
        place_pose.position.y += 0.03
        if place != "place":
            waypoints.append(request.pick_pose)
        waypoints = [request.post_place_pose]
        (plan, fraction) = self.move_group.compute_cartesian_path(
                                   waypoints,   # waypoints to follow
                                   0.01,        # eef_step
                                   0.0) 
        self.execute_plans([plan]) 
        if place != "place":
            self.execute_plans(["close"])

        waypoints = [place_pose] 
        # pick_pose, request.pick_pose,
        request.place_pose.position.y -= 0.13
        waypoints.append(request.place_pose)
        
        # request.pick_pose.position.z = 0.4
        # request.pick_pose.position.x = 0.3
        # waypoints.append(request.pick_pose)
        (plan, fraction) = self.move_group.compute_cartesian_path(
                                   waypoints,   # waypoints to follow
                                   0.01,        # eef_step
                                   0.0) 
        self.execute_plans([plan])  
        if place != "place":
            self.execute_plans(["open"])
            (plan, fraction) = self.move_group.compute_cartesian_path(
                                   [home],   # waypoints to follow
                                   0.01,        # eef_step
                                   0.0) 
            self.execute_plans([plan])
        if place == "place":
            # (plan, fraction) = self.move_group.compute_cartesian_path(
            #                        [pick_pose],   # waypoints to follow
            #                        0.01,        # eef_step
            #                        0.0)  
            # self.execute_plans([plan])        

            # 4 come back to pre-pick up pose
            
            (plan, fraction) = self.move_group.compute_cartesian_path(
                                    [request2.place_pose],   # waypoints to follow
                                    0.01,        # eef_step
                                    0.0) 
            self.execute_plans([plan]) 

            
            self.execute_plans(["open"])
            request2.place_pose.position.z = 0.3
            # 4 come back to pre-pick up pose
            (plan, fraction) = self.move_group.compute_cartesian_path(
                                    [request2.place_pose],   # waypoints to follow
                                    0.01,        # eef_step
                                    0.0) 
            self.execute_plans([plan]) 

            request2.pick_pose.position.z = 0.4
            request2.pick_pose.position.x = 0.3
            print("I renturned trajectory")
             
            (plan, fraction) = self.move_group.compute_cartesian_path(
                                    [request2.pick_pose],   # waypoints to follow
                                    0.01,        # eef_step
                                    0.0) 
            self.execute_plans([plan]) 

    def move_gripper(self, movement="open"):
        
        # Creates the SimpleActionClient, passing the type of the action
        client = actionlib.SimpleActionClient('/franka_gripper/move', franka_gripper.msg.MoveAction)

        # Waits until the action server has started up and started
        # listening for goals.
        client.wait_for_server()

        # Creates a goal to send to the action server.
        if movement == "close":
            goal = franka_gripper.msg.MoveGoal(width=0.05, speed=.1) #0.0405 close
        else:
            goal = franka_gripper.msg.MoveGoal(width=0.088, speed=1.0) #0.068 open
  
        
        # Sends the goal to the action server.
        client.send_goal(goal)

        # Waits for the server to finish performing the action.
        # client.wait_for_result()
        time.sleep(1)
        # Prints out the result of executing the action
        return client.get_result()  # A move result

    
    def execute_plans(self, plans):
        # executes plans on the real world robot (or rviz robot)
        for plan in plans:
            if isinstance(plan, str):
                self.move_gripper(plan)
                continue
            self.move_group.execute(plan)
        # pass
    
def generate_knockover_request(which):
    request = PandaManyPosesRequest()
    
    request.pick_pose.orientation.x =  0.9452608
    request.pick_pose.orientation.y =  -0.3150869
    request.pick_pose.orientation.z =  0.0
    request.pick_pose.orientation.w =  -0.0848662
    
    request.place_pose.position.z = .16
    request.place_pose.position.y = -0.3 #0.15 for less aggressive 0.3 for more aggressive
    request.place_pose.position.x = .48 #.3

    request.post_place_pose.position.z = 0.16
    request.post_place_pose.position.y = -0.1
    request.post_place_pose.position.x = .48 #.3

    request.place_pose.orientation = request.pick_pose.orientation
    request.post_place_pose.orientation = request.pick_pose.orientation

    if which == "none" or which=="":
        request.pick_pose.position.z = 0.16
        request.pick_pose.position.y = -0.1
        request.pick_pose.position.x = 0.53

        
    if which == "one" or which=="1":
        request.pick_pose.position.z = 0.16
        request.pick_pose.position.y = -0.02
        request.pick_pose.position.x = 0.53

    if which == "two" or which=="2":
        request.pick_pose.position.z = 0.16
        request.pick_pose.position.y = -0.02
        request.pick_pose.position.x = 0.43

    if which == "three" or which=="3":
        request.pick_pose.position.z = 0.16
        request.pick_pose.position.y = -0.02
        request.pick_pose.position.x = 0.63

    return request
    
def generate_request(which, height):
    request = PandaManyPosesRequest()
    
    request.pick_pose.orientation.x =  0.9452608
    request.pick_pose.orientation.y =  -0.3150869
    request.pick_pose.orientation.z =  0.0
    request.pick_pose.orientation.w =  -0.0848662

    request.place_pose.orientation = request.pick_pose.orientation
    
    request.place_pose.position.z = 0.15
    request.place_pose.position.y = -0.3
    request.place_pose.position.x = .48 #.3
    
    if which == "one" or which=="1":
        request.pick_pose.position.z = 0.15
        request.pick_pose.position.y = -0.02
        request.pick_pose.position.x = 0.53

    if which == "two" or which=="2":
        request.pick_pose.position.z = 0.15
        request.pick_pose.position.y = -0.02
        request.pick_pose.position.x = 0.43

    if which == "three" or which=="3":
        request.pick_pose.position.z = 0.15
        request.pick_pose.position.y = -0.02
        request.pick_pose.position.x = 0.63

    if height == "one" or height=="1":
        request.place_pose.position.z = 0.15
    if height == "two" or height=="2":
        request.place_pose.position.z = .23
    if height == "three" or height=="3":
        request.place_pose.position.z = .305 #18
    if height == "four" or height=="4":
        request.place_pose.position.z = .4 #1
    return request

def generate_picknoplace(which):
    request = PandaManyPosesRequest()
    
    request.pick_pose.orientation.x =  0.9452608
    request.pick_pose.orientation.y =  -0.3150869
    request.pick_pose.orientation.z =  0.0
    request.pick_pose.orientation.w =  -0.0848662

    request.place_pose.orientation = request.pick_pose.orientation

    request.place_pose.position.z = .203
    request.place_pose.position.y = -0.3
    request.place_pose.position.x = .48 #.3
    if which == "one" or which=="1":
        request.pick_pose.position.z = 0.16
        request.pick_pose.position.y = -0.1
        request.pick_pose.position.x = 0.53

        
    if which == "two" or which=="2":
        request.pick_pose.position.z = 0.16
        request.pick_pose.position.y = -0.1
        request.pick_pose.position.x = 0.43

       
    if which == "three" or which=="3":
        request.pick_pose.position.z = 0.16
        request.pick_pose.position.y = -0.1
        request.pick_pose.position.x = 0.63

        



    return request

def initialize():
    try:
       
        tutorial = UnityPythonConnector()
        print("Ready to plan")
        s = rospy.Service('moveit_many', PandaSimpleService, tutorial.go_to_pose_goal) # go_to_pose_goal
        s2 = rospy.Service('panda_move', PandaPickUp, tutorial.pickup_plan)
        # s3 = rospy.Service('waypoints_service', PandaManyPoses, tutorial.pick_no_place)
        sub1 = rospy.Subscriber('plan_publisher', moveit_msgs.msg.RobotTrajectory, tutorial.execute_plans)
        sub2 = rospy.Subscriber('realrobot_publisher', moveit_msgs.msg.RobotTrajectory, tutorial.execute_plans)
        coll = CollisionSceneExample()
        # coll.add_table()
        # add wall behind the robot 
        pose = [0, 0, 0, 0, 0, 0, 1.0]
        dimensions = [1.2, 1.2, 0.0001]
        coll.add_table(pose, dimensions, "wall")
        # s4 = rospy.Service('waypoints_service', moveit_msgs.msg.RobotTrajectory, tutorial.execute_plans)
        # rospy.spin()

        print("Do you want to run in debug mode or trial mode? DEBUG/TRIAL")
        mode = input()
        if not mode in {"DEBUG", "TRIAL"}:
              raise Exception("Sorry, not a valid mode.")
        

        while not is_shutdown():

            if mode == "TRIAL":
                print("Type in the current randomized condition, follow by the current turn, e.g., \"A2\", \"a2\", \"C4\"\"c4\".")
                action = input()
                # One block on table, pick up block 1 and place it
                if action in {"A2", "a2", "B2", "b2", "E2", "e2"}:
                    request = generate_request("1", "2")
                    tutorial.pickup_plan_cartesian(request)
                # One block on table, pick up block 1 but drop it
                elif action in {"C2", "c2", "F2", "f2"}:   
                    request = generate_picknoplace("1")
                    tutorial.pickup_plan_cartesian(request, drop = True)
                # one block on table, pickup block 1, knockover  stack, and drop the block you're holding
                elif action in {"D2", "d2", "G2", "g2", "H2", "h2"}:
                    request = generate_knockover_request("1")
                    tutorial.knock_down_plan(request, copy.deepcopy(request), "drop")

                # three blocks on table, pick up block 2 and place it
                elif action in {"A4", "a4"}:
                    request = generate_request("2", "4")
                    tutorial.pickup_plan_cartesian(request)
                # One block on table, pick up block 2 but drop it
                elif action in {"G4", "g4"}:   
                    request = generate_picknoplace("2")
                    tutorial.pickup_plan_cartesian(request, drop = True)
                # some blocks on table, pickup block 2, knockover  stack, and drop the block you're holding
                elif action in {"B4", "b4", "C4", "c4" "D4", "d4"}:
                    request = generate_knockover_request("2")
                    tutorial.knock_down_plan(request, copy.deepcopy(request), "drop")
                # some blocks on table, pickup block 2, knockover  stack, and stack the block you're holding
                elif action in {"E4", "e4" "F4", "f4", "H4", "h4"}:
                    request = generate_knockover_request("2")
                    tutorial.knock_down_plan(request, copy.deepcopy(request), "place")
                else:
                    print("You entered an invalid condition/turn. Conditions are A, B, C, D, E, F, G and H. Turns are 1 and 2. Please try again.")

            if mode == "DEBUG":
                print("What do you want to do? Options: p(ickup), g(ripper), k(nockover), no(place)")
                action = input()
                # pickup and place, first asking which position (1,2,3) we pick up from
                # second one is the height (1,2,3)
                if action=="pickup" or action=="p":
                    print("# pickup and place, first asking which position (1,2,3) we pick up from \\ # second one is the height (1,2,3)")
                    which = input("which one?")
                    height = input("which height?")
                    request = generate_request(which, height)
                    tutorial.pickup_plan_cartesian(request)
                # open or close the gripper
                if action == "gripper" or action == "g":
                    print("# open or close the gripper")
                    which = input("which one?")
                    if which == "close":
                        tutorial.move_gripper('close')
                    else:
                        tutorial.move_gripper('open')
                # knock over the tower pickup first then knock over (1,2,3)
                if action == "knockover" or action == "k":
                    print("# knock over the tower pickup first then knock over (1,2,3)")
                    which = input("which one?")
                    place = input("place or drop")
                    request = generate_knockover_request(which)
                    # tutorial.pickup_plan(request)
                    tutorial.knock_down_plan(request, copy.deepcopy(request), place)
                # pick up and place without changing the height
                # pick up (1,2,3) and drop ('yes' or nothing) from the height
                if action=="noplace" or action=="no":
                    print("# pick up and place without changing the height \\ # pick up (1,2,3) and drop ('yes' or nothing) from the height")
                    which = input("which one?")
                    request = generate_picknoplace(which)
                    drop = True if (input("drop?") == "yes" or input("drop?") == "yes") else False
                    tutorial.pickup_plan_cartesian(request, drop = drop)
        return
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return
    
def main():
    initialize()

if __name__ == "__main__":
    main()
