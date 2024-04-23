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
from panda_msgs.srv import PandaSimpleService, PandaSimpleServiceResponse, PandaPickUp, PandaPickUpResponse, PandaManyPoses, PandaManyPosesResponse
from panda_msgs.msg import FloatList
from collision_scene_example import CollisionSceneExample 
from scipy.spatial.transform import Rotation
## END_SUB_TUTORIAL

#!/usr/bin/env python

from rospy import init_node, is_shutdown

if __name__ == '__main__':

  init_node('input_test')

  while not is_shutdown():

      print("What do you want to do?")
      action = input()
      