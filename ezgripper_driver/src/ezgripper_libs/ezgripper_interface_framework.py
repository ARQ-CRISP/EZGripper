#!/usr/bin/python

from libezgripper import create_connection, Gripper
import math


class EZGripper(Gripper):

    """
        Home made class providing more ways to control the EZGripper
    """

    def __init__(self, gripper_id="EZGripper", port_name="/dev/ttyUSB0", baudrate=57600, servo_ids=[1]):
        """
            Initialize the gripper and the attributes of the class
            @param gripper_id: Name of the gripper (default is EZGripper)
            @param port_name: Port that should be used to communicate with the gripper (default is /dev/ttyUSB0)
            @param baudrate: Baudrate that should be used to contact the servo motor (default is 57600)
            @param servo_ids: List of id of the servos composing the manipulator (default is [1])
        """
        # Initialise a connection to the servo motor
        connection = create_connection(dev_name=port_name, baudrate=baudrate)
        # Call the constructor of Gripper which connects to the gripper and allows to send command
        Gripper.__init__(self, connection, gripper_id, servo_ids)
        # Calibrate the gripper when starting it
        self.calibrate()
        # We assume that we want to start with an open gripper
        self.open()
        # Joint limit (in radians) of the servo motor
        self.JOINT_LIMIT = 1.94
        # Set teh maximum percentage of the max torque that can be loaded to hold the object without overloading
        self.MAX_TORQUE_HOLD = 50
        # Overwrites the value of TORQUE_HOLD from the Gripper class
        self.TORQUE_HOLD = 50
        # Length of the finger and offset between the base of each finger (in cm) in order
        # to compute the proper command for having a given aperture (the values come from the documentation)
        self.FINGER_LENGTH = 11.157
        self.FINGER_OFFSET = 6.

    def _joint_to_dynamixel_value(self, joint_value):
        """
            Convert a target joint value to the corresponding dynamixel value
            @param joint_value: Joint value to convert (float)

            @return: Float corresponding to the dynamixel value
        """
        return int(round(-(self.GRIP_MAX / self.JOINT_LIMIT) * joint_value + self.GRIP_MAX))

    def get_joint_value(self):
        """
            Read the dynamixel's value and convert it to a joint angle value

            @return: Float between 0 and JOINT_LIMIT
        """
        dynamixel_value = self.servos[0].read_word_signed(36) - self.zero_positions[0]

        return (dynamixel_value - self.GRIP_MAX) * (-self.JOINT_LIMIT / self.GRIP_MAX)

    def go_to_joint_value(self, joint_value, closing_effort):
        """
            Move the gripper to a stated joint angle with a specified closing effort

            @param joint_value: Desired joint value
            @param closing_effort: Effort exerted by the gripper while closing
        """
        servo_position = self._joint_to_dynamixel_value(joint_value)
        # essentially sets velocity of movement, but also sets max_effort for initial half second of grasp.
        self.set_max_effort(closing_effort)
        self._go_to_servo_position(servo_position)
        self.set_max_effort(self.TORQUE_HOLD)

    def _go_to_servo_position(self, position):
        """
            Send a command to move the dynamixel to a certain position

            @param position: Position the Dynamixel should reach (not joint value)
        """
        # Without this, the dynamixel always outputs some overload issue
        self.servos[0].write_address(70, [0])
        # Send the proper dynamixel command
        self.servos[0].write_word(30, self.zero_positions[0] + position)
        # Blocking loop waiting for the hand of the movement
        self.wait_for_stop()

    def wait_for_stop(self):
        """
            Blocking loop waiting for a movement to finish
        """
        counter = 0
        # We consider a movement is finished when the value read is 4 times consecutively the same
        while counter < 5:
            last_value = self.servos[0].read_word_signed(36)
            if last_value - self.servos[0].read_word_signed(36) == 0:
                counter += 1

    def go_to_aperture(self, aperture_width, closing_effort):
        """
            Move the gripper to a given aperture with a specified closing effort

            @param aperture_width: Aperture the gripper should reach
            @param closing_effort: Effort exerted by the gripper while closing
        """
        # We first define the joint value with simple geometry
        joint_value = math.acos((aperture_width - self.FINGER_OFFSET) / self.FINGER_LENGTH)
        # And execute the joint
        self.go_to_joint_value(joint_value, closing_effort)

    def set_torque_intensity(self, torque_intensity):
        """
            Set the intensity of the effort exerted by the gripper to hold the object (after the grasp)

            @param torque_intensity: Intensity (0 - 1) of the max effort the gripper will apply when holding an object
        """
        # To avoid any overload issue, the max TORQUE_HOLD can only be 400 (which is 50% of the max torque).
        # So scale it to that value
        self.TORQUE_HOLD = torque_intensity * self.MAX_TORQUE_HOLD

    def release_torque(self):
        """
            Turn off the torque mode, and get back to the "compliant" mode i.e you can move the fingers manually
        """
        self.servos[0].write_address(24, [0])
