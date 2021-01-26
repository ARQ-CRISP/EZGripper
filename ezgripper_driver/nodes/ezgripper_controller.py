#!/usr/bin/python

import rospy
from ezgripper_libs.ezgripper_interface_framework import EZGripper
import actionlib
from ezgripper_driver.msg import ActuateGripperResult, ActuateGripperAction, ActuateGripperFeedback
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import thread


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
        # Initialise a ActuateGripperResult message
        self.result_message = ActuateGripperResult()
        # Initialise the action server
        self.action_server = actionlib.SimpleActionServer(action_server_name, ActuateGripperAction, auto_start=False)
        # Set the callback to be executed when a goal is received
        self.action_server.register_goal_callback(self.goal_callback)
        # Set the callback that should be executed when a preempt request is received
        self.action_server.register_preempt_callback(self.preempt_callback)
        # Start the server
        self.action_server.start()
        # Start another thread in which we publish continuously the joint state if the gripper
        thread.start_new_thread(self.publish_joint_state, ())
        rospy.loginfo("EZGripper ready to receive commands")

    def goal_callback(self):
        """
            Callback executed when a goal is received. Execute the grasp contained in the message
        """
        # Get the graspgripper command field, containing all information required to execute a grasp
        gripper_command = self.action_server.accept_new_goal().input

        # Make sure the command is valid, otherwise return outcome 1
        is_torque_between_0_and_1 = gripper_command.torque_intensity > 0 and gripper_command.torque_intensity <= 1
        is_position_positive = gripper_command.target_joint_state.position[0] > 0
        if not (is_position_positive and is_torque_between_0_and_1):
            # Send a result message containing a failure
            self.result_message.outcome = 1
            self.result_message.returned_object = False
            self.action_server.set_aborted(self.result_message)
            return

        # Set the torque to be applied after the gripper achieves the desired joint state
        self.gripper.set_torque_percentage(gripper_command.torque_intensity)
        # Move the gripper to the joint state contained in the goal received with maximum speed.
        # Once the gripper has finished to move torque is applied to hold the object
        self.gripper.go_to_joint_value(gripper_command.target_joint_state.position[0], 100)

        # Initialise and fill a ActuateGripperFeedback message
        action_feedback = ActuateGripperFeedback()
        action_feedback.current_joint_state.header.stamp = rospy.Time.now()
        action_feedback.current_joint_state.name = ["ezgripper_joint"]
        action_feedback.current_joint_state.position = [self.gripper.get_joint_value()]
        # Publish the message
        self.action_server.publish_feedback(action_feedback)

        # Send a result message containing a success
        self.result_message.outcome = 0
        self.result_message.returned_object = True
        self.action_server.set_succeeded(self.result_message)

    def preempt_callback(self):
        """
            Callback executed when a preempt request has been received.
        """
        rospy.loginfo("Action preempted")
        self.action_server.set_preempted()

    def publish_joint_state(self):
        """
            Publish continuously the gripper's joint state in order to be able to correctly plan and avoid collision
        """
        # Define the publisher (the topic name is standard)
        pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
        # In order to avoid overloading the gripper with too many reading/writing commands, limit the reading at 200Hz
        rate = rospy.Rate(200)
        # Initialise a JointState message with the proper joint name
        joint_state_message = JointState()
        joint_state_message.header = Header()
        joint_state_message.name = ["ezgripper_knuckle_palm_L1_1"]
        joint_state_message.velocity = []
        joint_state_message.effort = []
        # While the node is running publish an updated message
        while not rospy.is_shutdown():
            joint_state_message.header.stamp = rospy.Time.now()
            joint_state_message.position = [self.gripper.get_joint_value()]
            pub.publish(joint_state_message)
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('ezgripper_action_server', anonymous=True)
    action_server = EzGripperActionServer("ezgripper_controller", "EZGripper", "/dev/ttyUSB0", 57600, [1])
    rospy.spin()
