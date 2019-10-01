#!/usr/bin/env python

# Copyright (C) 2019 Shadow Robot Company Ltd - All Rights Reserved.
# Proprietary and Confidential. Unauthorized copying of the
# content in this file, via any medium is strictly prohibited.

import rospy
import actionlib
from smart_manipulation_framework_core.msg import GraspAction, GraspGoal, GraspCommand, StandardisedGrasp
from geometry_msgs.msg import PoseStamped
import tf_conversions
import numpy as np


def generate_grasp_messages():
    """
        Generate messages allowing to execute the different states corresponding to Suzuki's method
    """
    # Define the pregrasp posture (opened hand)
    pre_grasp_posture = [0.69]
    # pre_grasp_posture = [0.6883, -0.6883, 0, 0, -0.6883, 0, -0.6883, -0.6883, 0]
    # Define the grasp posture (closed hand)
    grasp_posture = [0.98]
    # grasp_posture = [0.6883, 0.25, 0.58, 0.00, 0.25, 0.58, -0.6883, 0.25, 0.58]
    # Define the percentage of the maximal torque to be applied at the end of the grasp
    squeeze_direction = [0]
    # squeeze_direction = [0, 0, 0, 0, 0, 0.35, 0, 0, 0]

    # Generating the different messages required to execute the StateMachine
    # Define the message containing the Pose of the end effector for the pregrasp
    pregrasp_pose = PoseStamped()
    # The grasp is defined with "world" as reference frame
    pregrasp_pose.header.frame_id = 'world'
    # Set the coordinates
    pregrasp_pose.pose.position.x = -0.62781
    pregrasp_pose.pose.position.y = -0.038801
    pregrasp_pose.pose.position.z = 1.3564
    # Convert the orientation from euler angle (in radians) to quaternion.
    # /!| The -pi is here only for QMUL setup, since otherwise the wrist would be oriented upward (whereas the table
    # is downward)
    # orientation_quaternion = tf_conversions.transformations.quaternion_from_euler(-np.pi, 0, wrist_angle)
    orientation_quaternion = tf_conversions.transformations.quaternion_from_euler(-np.pi, 0, 0)
    pregrasp_pose.pose.orientation.x = orientation_quaternion[0]
    pregrasp_pose.pose.orientation.y = orientation_quaternion[1]
    pregrasp_pose.pose.orientation.z = orientation_quaternion[2]
    pregrasp_pose.pose.orientation.w = orientation_quaternion[3]

    # Define the message containing the Pose of the end effector for grasping
    grasp_pose = PoseStamped()
    # The grasp is defined with "world" as reference frame
    grasp_pose.header.frame_id = 'world'
    # Set the coordinates
    grasp_pose.pose.position.x = -0.83516
    grasp_pose.pose.position.y = -0.062559
    # The height of the object being unknown, we are using the fact tht the kinect is overhead and thus that the
    # estimated centroid will likely be above the real one. We subtract then the estimated fourth of the height to
    # be sure to grasp properly the object. That way, in the worst case (the estiamated centroid is on the top of
    # the object), the gripper's fingers will grasp the object at its centroid.
    # We add to this value, according to the gripper, the height of the fingers (distance between manipulator's
    # frame and finger tip)
    grasp_pose.pose.position.z = 1.3588
    # Convert the orientation from euler angle (in radians) to quaternion.
    # /!| The -pi is here only for QMUL setup, since otherwise the wrist would be oriented upward (whereas the table
    # is downward)
    orientation_quaternion = tf_conversions.transformations.quaternion_from_euler(-np.pi, 0, 0)
    grasp_pose.pose.orientation.x = orientation_quaternion[0]
    grasp_pose.pose.orientation.y = orientation_quaternion[1]
    grasp_pose.pose.orientation.z = orientation_quaternion[2]
    grasp_pose.pose.orientation.w = orientation_quaternion[3]

    grasp_message = generate_grasp_msg("generated_grasp", "ezgripper", ['left_ezgripper_knuckle_palm_L1_1'],
                                       "fh_manipulator_ee", "automatic", squeeze_direction,
                                       pre_grasp_posture, grasp_posture)
    grasp_message.grasp.pose = grasp_pose
    grasp_message.pregrasp.pose = pregrasp_pose

    return grasp_message


def generate_grasp_msg(grasp_id, hand_id, hand_joint_names, manipulator_id, source, squeeze_direction,
                       pre_grasp_posture, grasp_posture, object_id=""):
    """
        Generate a Grasp Msg that will be used by the grasp controller to move the hand
        @param grasp_id: String determining the name of the grasp. It is strongly advised to keep it unique
        @param hand_id: String determining the name of the hand on which the grasp should be executed
        @param hand_joint_names: List of the name of the joints composing the hand
        @param manipulator_id: Name of the manipulator (often defined in th srdf file)
        @param source: String stating if the grasp posture has been generated manually or automatically
        @param squeeze_direction: List of float stating the percentage of the maximum torque applied to each joint
                                  of the hand
        @param pre_grasp_posture: List of float stating the angle in radians of each joint of the hand for pregrasp
        @param grasp_posture: List of float stating the angle in radians of each joint of the hand for grasp
        @param object_id: Name of the object to be grasped. If unknown, set to empty string
    """
    # Initializing a Grasp Msg and filling the proper fields to execute a grasp
    grasp_msg = StandardisedGrasp()
    grasp_msg.torque_intensity.joint_names = hand_joint_names
    grasp_msg.torque_intensity.torque_intensity = squeeze_direction

    # Fill information related the grasp context (frame, object...)
    grasp_msg.pregrasp.posture.header.stamp.secs = 0
    grasp_msg.pregrasp.posture.header.stamp.nsecs = 1000000000
    grasp_msg.pregrasp.posture.header.frame_id = object_id
    grasp_msg.pregrasp.posture.name = hand_joint_names

    grasp_msg.grasp.posture.header.stamp.secs = 0
    grasp_msg.grasp.posture.header.stamp.nsecs = 10000000
    grasp_msg.grasp.posture.header.frame_id = object_id
    grasp_msg.grasp.posture.name = hand_joint_names

    # Initialize a JointTrajectoryPoint Msg
    # pos = JointTrajectoryPoint()
    # positions = pre_grasp_posture
    # pos.positions.extend(positions)
    # pos.time_from_start.secs = 0
    # pos.time_from_start.nsecs = 10000000
    # Add the pregrasp posture to the message
    grasp_msg.pregrasp.posture.position = pre_grasp_posture
    # Initialize a JointTrajectoryPoint Msg
    # pos = JointTrajectoryPoint()
    # positions = grasp_posture
    # pos.positions.extend(positions)
    # pos.time_from_start.secs = 0
    # pos.time_from_start.nsecs = 10000000
    # Add the grasp posture to the message
    # grasp_msg.grasp.posture.points.append(pos)
    grasp_msg.grasp.posture.position = grasp_posture

    return grasp_msg


# def display_feedback(feedback):
#     """
#     """
#     rospy.loginfo("========== {} executing".format(["PREGRASP", "GRASP"][feedback.grasp_state]))
#     rospy.loginfo("========== current joint state is {}".format(dict(zip(feedback.current_joint_state.name,
#                                                                          feedback.current_joint_state.position))))

if __name__ == '__main__':
    rospy.init_node('start_grasp_action_client', anonymous=True)
    client = actionlib.SimpleActionClient("real_ezgripper", GraspAction)
    client.wait_for_server()

    grasp_goal = GraspGoal()
    grasp_command = GraspCommand()

    grasp_command.grasp = generate_grasp_messages()
    grasp_command.max_torque = 500

    grasp_command.grasp_state = grasp_command.PRE_GRASP_STATE
    # grasp_command.grasp_state = grasp_command.GRASP_STATE
    # Fill the grasp goal command field with the grasp command
    grasp_goal.grasp_command = grasp_command
    # Send the goal and wait for result
    client.send_goal(grasp_goal)  # , feedback_cb=display_feedback)
    client.wait_for_result()
    # Get a result if and only if the goal is in the correct format
    result = client.get_result()
    print(result)

    rospy.spin()
