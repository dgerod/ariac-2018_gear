#!/usr/bin/env python

from __future__ import print_function

import sys
import time

from test_example_node import ExampleNodeTester
from ariac_example import ariac_example
import rospy
import rostest


class GripperTester(ExampleNodeTester):

    def test(self):
        self.comp_class = ariac_example.MyCompetitionClass()
        ariac_example.connect_callbacks(self.comp_class)
        time.sleep(1.0)

        # Pre-defined initial pose because sometimes the arm starts "droopy"
        self._send_arm_to_initial_pose()

        # Pre-defined pose that puts the gripper in contact with a part.
        self._send_arm_to_part()

        # Enable the gripper so that it picks up the part.
        self._test_enable_gripper()

        # Move the part over the shipping box using a pre-defined sequence of poses.
        self._send_arm_to_shipping_box()
        self.assertTrue(
            self.comp_class.current_gripper_state.enabled, 'Gripper no longer enabled')
        self.assertTrue(
            self.comp_class.current_gripper_state.attached, 'Product no longer attached')

        # Disable the gripper so that it drops the part.
        self._test_disable_gripper()

        time.sleep(1.0)

    def _test_enable_gripper(self):
        success = self._enable_gripper()
        self.assertTrue(success, 'Gripper not successfully controlled')
        time.sleep(1.0)
        self.assertTrue(
            self.comp_class.current_gripper_state.enabled, 'Gripper not successfully enabled')
        self.assertTrue(
            self.comp_class.current_gripper_state.attached, 'Product not successfully attached')

    def _enable_gripper(self):
        success = ariac_example.control_gripper(True)
        time.sleep(0.5)
        return success

    def _test_disable_gripper(self):
        success = self._disable_gripper()
        self.assertTrue(success, 'Gripper not successfully controlled')
        time.sleep(1.0)
        self.assertFalse(
            self.comp_class.current_gripper_state.enabled, 'Gripper not successfully disabled')
        self.assertFalse(
            self.comp_class.current_gripper_state.attached, 'Product not successfully dettached')

    def _disable_gripper(self):
        success = ariac_example.control_gripper(False)
        time.sleep(0.5)
        return success

    def _send_arm_to_part(self):
        trajectory = [
            [1.5, -0.5, -2.0, 3.1, 4.15, -1.51, 0.0],
            [1.5, -0.5, -1.0, 3.1, 4.15, -1.51, 0.0],
            [1.5, -0.55, 0.0, 3.1, 4.15, -1.51, 0.0],
            [1.5, -0.55, 5.91, 3.1, 4.15, -1.51, 0.0],
        ]
        for positions in trajectory:
            self.comp_class.send_arm_to_state(positions)
            time.sleep(1.5)

    def _send_arm_to_shipping_box(self):
        trajectory = [
            [1.5, -0.5, -1.8, 3.1, 4.15, -1.51, 0.0],
            [1.5, -0.5, -1.8, 0, 4.15, -1.51, 0.0],
            [1.5, -0.5, -0.38, 0, 3.39, -1.51, 0.0],
        ]
        for positions in trajectory:
            self.comp_class.send_arm_to_state(positions)
            time.sleep(1.0)


if __name__ == '__main__':
    rospy.init_node('test_gripper', anonymous=True)

    # Wait until /clock is being published; this can take an unpredictable
    # amount of time when we're downloading models.
    while rospy.Time.now().to_sec() == 0.0:
        print('Waiting for Gazebo to start...')
        time.sleep(1.0)
    # Take an extra nap, to allow plugins to be loaded
    time.sleep(12.0)
    print('OK, starting test.')

    rostest.run('test_ariac', 'test_gripper', GripperTester, sys.argv)
