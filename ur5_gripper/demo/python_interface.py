#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley, Mike Lautman

import time
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list



def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True


class UR5GripperPythonInteface(object):
  """UR5GripperPythonInteface"""
  def __init__(self):
    super(UR5GripperPythonInteface, self).__init__()


    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('ur5_gripper_python_interface', anonymous=True)

    ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
    ## kinematic model and the robot's current joint states
    robot = moveit_commander.RobotCommander()

    ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
    ## for getting, setting, and updating the robot's internal understanding of the
    ## surrounding world:
    scene = moveit_commander.PlanningSceneInterface()


    ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
    ## trajectories in Rviz:
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)


    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print "============ Available Planning Groups:", robot.get_group_names()

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print "============ Printing robot state"
    print robot.get_current_state()
    print ""
    ## END_SUB_TUTORIAL

    # Misc variables
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.display_trajectory_publisher = display_trajectory_publisher
    self.group_names = group_names
    
    self.arm_group = self.robot.get_group("ur5_arm")
    self.gripper_group = self.robot.get_group("robotiq140gripper")




  def control_gripper(self):
    print "============ Enter joint value for the gripper finger joint ..."
    joint_input = raw_input()
    new_goal = float(joint_input)
    if(new_goal < 0 or new_goal > 0.7):
      print "Gripper joint value should be 0 < and < 0.7!"
      return
    gripper_group = self.gripper_group
    joint_goal = gripper_group.get_current_joint_values()
    joint_goal[0] = new_goal
    gripper_group.go(joint_goal, wait=True)
    gripper_group.stop()
    return 

  def execute_joint_state(self, goal):

    arm_group = self.arm_group
    rospy.sleep(0.3)
    joint_goal = arm_group.get_current_joint_values()
    for i in range(6):
      joint_goal[i] = goal[i]

    arm_group.get_current_joint_values()
    plan = arm_group.plan(joint_goal)
    rospy.sleep(0.1)
    # a = raw_input()
    arm_group.execute(plan, wait=True)
    rospy.sleep(0.3)
    return 

  def go_to_joint_state(self, goal):

    arm_group = self.arm_group

    joint_goal = arm_group.get_current_joint_values()
    for i in range(6):
      joint_goal[i] = goal[i]


    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    arm_group.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    arm_group.stop()

    ## END_SUB_TUTORIAL

    # For testing:
    current_joints = arm_group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)


  def pose_goal(self):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    arm_group = self.arm_group

    ## BEGIN_SUB_TUTORIAL plan_to_pose
    ##
    ## Planning to a Pose Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## We can plan a motion for this group to a desired pose for the
    ## end-effector:
    print "============ Enter x y z qx qy qz qw ..."
    pose_input = raw_input()
    x,y,z,qx, qy, qz, qw = pose_input.split(" ")
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = float(qw)
    pose_goal.orientation.x = float(qx)
    pose_goal.orientation.y = float(qy)
    pose_goal.orientation.z = float(qz)
    pose_goal.position.x = float(x)
    pose_goal.position.y = float(y)
    pose_goal.position.z = float(z)

    arm_group.set_pose_target(pose_goal)

    ## Now, we call the planner to compute the plan and execute it.
    plan = arm_group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    arm_group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    arm_group.clear_pose_targets()

    ## END_SUB_TUTORIAL

    # For testing:
    # Note that since this section of code will not be included in the tutorials
    # we use the class variable rather than the copied state variable
    current_pose = arm_group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)


  def plan_cartesian_path(self, pose, num, scale=1):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    arm_group = self.arm_group

    ## BEGIN_SUB_TUTORIAL plan_cartesian_path
    ##
    ## Cartesian Paths
    ## ^^^^^^^^^^^^^^^
    ## You can plan a Cartesian path directly by specifying a list of waypoints
    ## for the end-effector to go through. If executing  interactively in a
    ## Python shell, set scale = 1.0.
    ##
    waypoints = []
    wpose = arm_group.get_current_pose().pose
    xstep = (pose["x"] - wpose.position.x)/num
    ystep = (pose["y"] - wpose.position.y)/num
    zstep = (pose["z"] - wpose.position.z)/num
    for i in range(num):
        wpose.position.x += xstep
        wpose.position.y += ystep
        wpose.position.z += zstep
        waypoints.append(copy.deepcopy(wpose))

    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0,
    # ignoring the check for infeasible jumps in joint space, which is sufficient
    # for this tutorial.
    (plan, fraction) = arm_group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold

    # Note: We are just planning, not asking move_group to actually move the robot yet:
    return plan, fraction

    ## END_SUB_TUTORIAL


  def display_trajectory(self, plan):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    robot = self.robot
    display_trajectory_publisher = self.display_trajectory_publisher

    ## BEGIN_SUB_TUTORIAL display_trajectory
    ##
    ## Displaying a Trajectory
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## You can ask RViz to visualize a plan (aka trajectory) for you. But the
    ## group.plan() method does this automatically so this is not that useful
    ## here (it just displays the same trajectory again):
    ##
    ## A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
    ## We populate the trajectory_start with our current robot state to copy over
    ## any AttachedCollisionObjects and add our plan to the trajectory.
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    # Publish
    display_trajectory_publisher.publish(display_trajectory);

    ## END_SUB_TUTORIAL


  def execute_plan(self, plan):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    arm_group = self.arm_group

    ## BEGIN_SUB_TUTORIAL execute_plan
    ##
    ## Executing a Plan
    ## ^^^^^^^^^^^^^^^^
    ## Use execute if you would like the robot to follow
    ## the plan that has already been computed:
    arm_group.execute(plan, wait=True)

    ## **Note:** The robot's current joint state must be within some tolerance of the
    ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail
    ## END_SUB_TUTORIAL


  def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = self.box_name
    scene = self.scene

    ## BEGIN_SUB_TUTORIAL wait_for_scene_update
    ##
    ## Ensuring Collision Updates Are Received
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## If the Python node dies before publishing a collision object update message, the message
    ## could get lost and the box will not appear. To ensure that the updates are
    ## made, we wait until we see the changes reflected in the
    ## ``get_attached_objects()`` and ``get_known_object_names()`` lists.
    ## For the purpose of this tutorial, we call this function after adding,
    ## removing, attaching or detaching an object in the planning scene. We then wait
    ## until the updates have been made or ``timeout`` seconds have passed
    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
      # Test if the box is in attached objects
      attached_objects = scene.get_attached_objects([box_name])
      is_attached = len(attached_objects.keys()) > 0

      # Test if the box is in the scene.
      # Note that attaching the box will remove it from known_objects
      is_known = box_name in scene.get_known_object_names()

      # Test if we are in the expected state
      if (box_is_attached == is_attached) and (box_is_known == is_known):
        return True

      # Sleep so that we give other threads time on the processor
      rospy.sleep(0.1)
      seconds = rospy.get_time()

    # If we exited the while loop without returning then we timed out
    return False
    ## END_SUB_TUTORIAL


  def add_box(self, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = self.box_name
    scene = self.scene

    ## BEGIN_SUB_TUTORIAL add_box
    ##
    ## Adding Objects to the Planning Scene
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## First, we will create a box in the planning scene at the location of the left finger:
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "panda_leftfinger"
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.position.z = 0.07 # slightly above the end effector
    box_name = "box"
    scene.add_box(box_name, box_pose, size=(0.1, 0.1, 0.1))

    ## END_SUB_TUTORIAL
    # Copy local variables back to class variables. In practice, you should use the class
    # variables directly unless you have a good reason not to.
    self.box_name=box_name
    return self.wait_for_state_update(box_is_known=True, timeout=timeout)


  def attach_box(self, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = self.box_name
    robot = self.robot
    scene = self.scene
    eef_link = self.eef_link
    group_names = self.group_names

    ## BEGIN_SUB_TUTORIAL attach_object
    ##
    ## Attaching Objects to the Robot
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## Next, we will attach the box to the Panda wrist. Manipulating objects requires the
    ## robot be able to touch them without the planning scene reporting the contact as a
    ## collision. By adding link names to the ``touch_links`` array, we are telling the
    ## planning scene to ignore collisions between those links and the box. For the Panda
    ## robot, we set ``grasping_group = 'hand'``. If you are using a different robot,
    ## you should change this value to the name of your end effector group name.
    grasping_group = 'robotiq140_gripper'
    touch_links = robot.get_link_names(group=grasping_group)
    scene.attach_box(eef_link, box_name, touch_links=touch_links)
    ## END_SUB_TUTORIAL

    # We wait for the planning scene to update.
    return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)


  def detach_box(self, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = self.box_name
    scene = self.scene
    eef_link = self.eef_link

    ## BEGIN_SUB_TUTORIAL detach_object
    ##
    ## Detaching Objects from the Robot
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## We can also detach and remove the object from the planning scene:
    scene.remove_attached_object(eef_link, name=box_name)
    ## END_SUB_TUTORIAL

    # We wait for the planning scene to update.
    return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)


  def remove_box(self, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = self.box_name
    scene = self.scene

    ## BEGIN_SUB_TUTORIAL remove_object
    ##
    ## Removing Objects from the Planning Scene
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## We can remove the box from the world.
    scene.remove_world_object(box_name)

    ## **Note:** The object must be detached before we can remove it from the world
    ## END_SUB_TUTORIAL

    # We wait for the planning scene to update.
    return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)

  def set_velociy_acceleration_factors(self, velocity_factor, acc_factor):
      # print "============ Enter velocity and acceleration scaling factors ..."
      # factor_inp = raw_input()
      # vf, ac = factor_inp.split(" ")
      # velocity_factor = float(vf)
      # acc_factor = float(ac)
      arm_group = self.arm_group
      arm_group.set_max_velocity_scaling_factor(velocity_factor)
      arm_group.set_max_acceleration_scaling_factor(acc_factor)
      # gripper_group = self.gripper_group
      # gripper_group.set_max_velocity_scaling_factor(velocity_factor)
      # gripper_group.set_max_acceleration_scaling_factor(acc_factor)      

  def cartesian_path(self):
    print 'Enter x y z step_number...'
    pose_input = raw_input()
    x,y,z, step_number = pose_input.split(" ")
    pose = {}
    pose["x"] = float(x)
    pose["y"] = float(y)
    pose["z"] = float(z)
    step_number = int(step_number)
    plan, fraction = self.plan_cartesian_path(pose, step_number)
    print "============ Press `Enter` to display a saved trajectory needs rviz (this will replay the Cartesian path)  ..."
    raw_input()
    self.display_trajectory(plan)
    print "============ Press `Enter` to execute a saved path ..."
    raw_input()
    self.execute_plan(plan)

def main():
  try:
    print "============ Press `Enter` to begin moveit_commander ..."
    raw_input()
    ur5_gripper_interface = UR5GripperPythonInteface()
    prev = ""
    while(True):
        print ""
        print "============ Enter Mode ..."
        mode = raw_input()
        if mode not in ["pose_goal", "cartesian_path", "set_factors", "control_gripper", "joint_goal"]:
            mode = prev
        if(mode == "pose_goal"):
            ur5_gripper_interface.pose_goal()
        elif(mode == "cartesian_path"):
            ur5_gripper_interface.cartesian_path()
        elif(mode == "set_factors"):
            ur5_gripper_interface.set_velociy_acceleration_factors()
        elif(mode == "control_gripper"):
          ur5_gripper_interface.control_gripper()
        elif(mode == "joint_goal"):
          inp = raw_input("Enter joint goal(6)\n")
          joint_goal = inp.split(" ")
          joint_goal = [float(i) for i in joint_goal]
          #ur5_gripper_interface.go_to_joint_state(joint_goal)
          ur5_gripper_interface.plan_joint_state(joint_goal)
         
        prev = mode

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()

## BEGIN_TUTORIAL
## .. _moveit_commander:
##    http://docs.ros.org/melodic/api/moveit_commander/html/namespacemoveit__commander.html
##
## .. _MoveGroupCommander:
##    http://docs.ros.org/melodic/api/moveit_commander/html/classmoveit__commander_1_1move__group_1_1MoveGroupCommander.html
##
## .. _RobotCommander:
##    http://docs.ros.org/melodic/api/moveit_commander/html/classmoveit__commander_1_1robot_1_1RobotCommander.html
##
## .. _PlanningSceneInterface:
##    http://docs.ros.org/melodic/api/moveit_commander/html/classmoveit__commander_1_1planning__scene__interface_1_1PlanningSceneInterface.html
##
## .. _DisplayTrajectory:
##    http://docs.ros.org/melodic/api/moveit_msgs/html/msg/DisplayTrajectory.html
##
## .. _RobotTrajectory:
##    http://docs.ros.org/melodic/api/moveit_msgs/html/msg/RobotTrajectory.html
##
## .. _rospy:
##    http://docs.ros.org/melodic/api/rospy/html/
## CALL_SUB_TUTORIAL imports
## CALL_SUB_TUTORIAL setup
## CALL_SUB_TUTORIAL basic_info
## CALL_SUB_TUTORIAL plan_to_joint_state
## CALL_SUB_TUTORIAL plan_to_pose
## CALL_SUB_TUTORIAL plan_cartesian_path
## CALL_SUB_TUTORIAL display_trajectory
## CALL_SUB_TUTORIAL execute_plan
## CALL_SUB_TUTORIAL add_box
## CALL_SUB_TUTORIAL wait_for_scene_update
## CALL_SUB_TUTORIAL attach_object
## CALL_SUB_TUTORIAL detach_object
## CALL_SUB_TUTORIAL remove_object
## END_TUTORIAL