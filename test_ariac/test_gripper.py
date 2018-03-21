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

        # Pre-defined pose that puts the gripper in contact with a product.
        self._send_arm_to_product()

        # Enable the gripper so that it picks up the product.
        self._test_enable_gripper()

        # Move the product over the shipping box using a pre-defined sequence of poses.
        self._send_arm_to_shipping_box()
        self.assertTrue(
            self.comp_class.current_gripper_state.enabled, 'Gripper no longer enabled')
        self.assertTrue(
            self.comp_class.current_gripper_state.attached, 'Product no longer attached')

        # Disable the gripper so that it drops the product.
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

    def _send_arm_to_product(self):
        trajectory = [
            [-1.272, -1.102, 0.050, 1.112, -1.329, 1.360, 0.902, -0.663],
            [0.444, -1.885, -1.726, 1.945, -0.941, 1.754, -2.380, -0.018],
            [0.025, -1.484, -2.085, 0.046, -1.041, 1.317, -2.134, 0.259],
            [0.100, -1.751, -2.046, 0.010, -1.11, 1.312, -2.088, 0.190],
        ]
        for positions in trajectory:
            self.comp_class.send_arm_to_state(positions)
            time.sleep(1.5)

    def _send_arm_to_shipping_box(self):
        trajectory = [
            [0.216, -1.672, -2.10, 0.584, -1.140, 1.574, -2.380, 0.150],
            [0.678, -2.060, -2.031, 1.876, -1.107, 1.914, -3.020, 0.294],
            [1.601, -1.893, -2.465, 0.800, -0.893, 1.919, -2.572, 0.887],
            [2.795, -2.009, -2.316, 0.556, -0.746, 1.745, -1.215, 0.206],
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
