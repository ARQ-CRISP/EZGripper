#!/usr/bin/python

from libezgripper import create_connection, Gripper
import math


class EZGripper(Gripper):

    """

    """

    def __init__(self, gripper_id, port_name, baudrate, servo_ids):
        """

        """
        connection = create_connection(dev_name=port_name, baudrate=baudrate)
        Gripper.__init__(self, connection, gripper_id, servo_ids)
        self.calibrate()
        self.open()
        self.JOINT_LIMIT = 1.94
        self.TORQUE_HOLD = 0
        self.finger_length = -14.25 / (math.cos(self.JOINT_LIMIT) - 1)
        self.finger_offset = 14.25 * (1 + 1 / (math.cos(self.JOINT_LIMIT) - 1))

    def _joint_to_dynamixel_value(self, joint_value):
        """
        """
        return int(-(self.GRIP_MAX / self.JOINT_LIMIT) * joint_value + self.GRIP_MAX)

    def get_joint_value(self):
        """
        """
        dynamixel_value = self.servos[0].read_word_signed(36) - self.zero_positions[0]

        return (dynamixel_value - self.GRIP_MAX) * (-self.JOINT_LIMIT / self.GRIP_MAX)

    def go_to_joint_value(self, joint_value, closing_effort):
        """

        """
        servo_position = self._joint_to_dynamixel_value(joint_value)
        # essentially sets velocity of movement, but also sets max_effort for initial half second of grasp.
        self.set_max_effort(closing_effort)
        self._go_to_servo_position(servo_position)
        self.set_max_effort(self.TORQUE_HOLD)

    def _go_to_servo_position(self, position):
        self.servos[0].write_word(30, self.zero_positions[0] + position)
        self.wait_for_stop()

    def wait_for_stop(self):
        counter = 0
        while counter < 5:
            last_value = self.servos[0].read_word_signed(36)
            if last_value - self.servos[0].read_word_signed(36) == 0:
                counter += 1

    def go_to_aperture(self, aperture_width, closing_effort):
        """

        """
        joint_value = math.acos((aperture_width - self.finger_offset) / self.finger_length)
        self.go_to_joint_value(joint_value, closing_effort)

    def set_torque_hold(self, torque_value):
        """

        """
        self.TORQUE_HOLD = torque_value

    def set_max_torque(self, max_torque):
        """

        """
        self.TORQUE_MAX = max_torque

if __name__ == '__main__':
    gripper = EZGripper("test", "/dev/ttyUSB0", 57600, [1])
    # gripper.open()
    # gripper.go_to_aperture(1.5, 50)
    # gripper.release()
    # gripper.go_to_joint_value(1.7, 50)
    # print(gripper.dynamixel_value_to_joint())
    # print(gripper.zero_positions[0])
    # print(gripper.servos[0].read_word_signed(36))
    # print(gripper.servos[0].read_word_signed(36) - gripper.zero_positions[0])
    # gripper.set_max_effort(100)
