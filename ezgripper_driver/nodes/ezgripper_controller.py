#!/usr/bin/python

import rospy
from ezgripper_libs.ezgripper import EZGripper
import actionlib
from smart_manipulation_framework_core.msg import GraspResult, GraspAction, GraspFeedback


class EzGripperActionServer(object):

    """
        Class running the actionlib server to execute grasps on the SAKERobotic EZgripper.
    """

    def __init__(self, action_server_name, gripper_id="EZGripper", port_name="/dev/ttyUSB0", baudrate=57600,
                 servo_ids=[1]):
        """
            Initialise the gripper and action server

            @param action_server_name: Name of the action server
            @param gripper_id: Name of the gripper (default is EZGripper)
            @param port_name: Port that should be used to communicate with the gripper (default is /dev/ttyUSB0)
            @param baudrate: Baudrate that should be used to contact the servo motor (default is 57600)
            @param servo_ids: List of id of the servos composing the manipulator (default is [1])
        """
        # Initialise the object allowing to control the EZGripper
        self.gripper = EZGripper(gripper_id, port_name, baudrate, servo_ids)
        # Initialise a GraspResult message
        self.result_message = GraspResult()
        # Initialise the action server
        self.action_server = actionlib.SimpleActionServer(action_server_name, GraspAction, auto_start=False)
        # Set the callback to be executed when a goal is received
        self.action_server.register_goal_callback(self.goal_callback)
        # Set the callback that should be executed when a preempt request is received
        self.action_server.register_preempt_callback(self.preempt_callback)
        # Start the server
        self.action_server.start()
        rospy.loginfo("EzGripper calibrated and ready to receive commands")

    def goal_callback(self):
        """
            Callback executed when a goal is received. Execute the grasp contained in the message
        """
        # Get the grasp command field, containing all information required to execute a grasp
        goal_grasp_command = self.action_server.accept_new_goal().grasp_command
        # Check that the max torque is at least 100, otherwise the gripper doesn't move
        if goal_grasp_command.max_torque < 100:
            rospy.logerr("The provided maximum torque should be at least 100, otherwise the gripper will not move")
            # Fill the message and publishes the result (fail) and exit the callback
            self.result_message.pregrasped.data = False
            self.result_message.grasped.data = False
            self.result_message.postgrasped.data = False
            self.action_server.set_aborted(self.result_message)
            return

        # Set the maximum torque the EZGripper can use (800 is the limit)
        self.gripper.set_max_torque(goal_grasp_command.max_torque)
        # Extract the manipulator state depending on the received grasp state
        if goal_grasp_command.grasp_state == 0:
            manipulator_state = goal_grasp_command.grasp.pregrasp
        elif goal_grasp_command.grasp_state == 1:
            manipulator_state = goal_grasp_command.grasp.grasp
        else:
            manipulator_state = goal_grasp_command.grasp.postgrasp

        # Set the torque to be applied after the gripper achieves the desired joint state
        self.gripper.set_torque_hold(goal_grasp_command.grasp.torque_intensity.torque_intensity[0] * 100)
        # Move the gripper to the joint state contained in the goal received with half the maximum speed.
        # Once the gripper has finished to move torque is applied to hold the object
        self.gripper.go_to_joint_value(manipulator_state.posture.position[0], 100)

        # Initialise and fill a GraspFeedback message
        action_feedback = GraspFeedback()
        action_feedback.current_joint_state.header.stamp = rospy.Time.now()
        action_feedback.current_joint_state.name = ["ezgripper_joint"]
        action_feedback.current_joint_state.position = [self.gripper.get_joint_value()]
        # Publish the message
        self.action_server.publish_feedback(action_feedback)

        # Send a result message containing a success
        self.result_message.pregrasped.data = goal_grasp_command.grasp_state == 0
        self.result_message.grasped.data = goal_grasp_command.grasp_state == 1
        self.result_message.postgrasped.data = goal_grasp_command.grasp_state == 2
        self.action_server.set_succeeded(self.result_message)

    def preempt_callback(self):
        """
            Callback executed when a prrempt request has been received.
        """
        rospy.loginfo("Action preempted")
        self.action_server.set_preempted()

if __name__ == '__main__':
    rospy.init_node('ezgripper_action_server', anonymous=True)
    action_server = EzGripperActionServer(rospy.get_param("action_server_name"), rospy.get_param("manipulator_name"),
                                          rospy.get_param("port_name"), rospy.get_param("baudrate"),
                                          rospy.get_param("servo_ids"))
    rospy.spin()
