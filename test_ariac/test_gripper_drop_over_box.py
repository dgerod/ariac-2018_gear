#!/usr/bin/env python

from __future__ import print_function

import sys
import time

from test_gripper import GripperTester
from test_tf_frames import TfTester
from ariac_example import ariac_example
import rospy
import rostest

import tf
import tf2_geometry_msgs  # noqa
import tf2_py as tf2
import tf2_ros



class GripperBoxDropTester(GripperTester, TfTester):

    def test(self):
        self.comp_class = ariac_example.MyCompetitionClass()
        ariac_example.connect_callbacks(self.comp_class)
        time.sleep(1.0)
        self.prepare_tf()

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
        self.assertFalse(
            self.comp_class.current_gripper_state.attached, 'Product is still attached')

        self._test_dropped_product_pose()

        time.sleep(1.0)

    def _test_dropped_product_pose(self):
        self._test_pose(
            [0.15, 0.15, 0.0],
            tf.transformations.quaternion_from_euler(0, 0, 0.2),
            self.camera_above_box0 + '_gear_part_3_frame',
            self.camera_above_box0 + '_shipping_box_0_frame'
        )


if __name__ == '__main__':
    rospy.init_node('test_gripper_box_drop', anonymous=True)

    # Wait until /clock is being published; this can take an unpredictable
    # amount of time when we're downloading models.
    while rospy.Time.now().to_sec() == 0.0:
        print('Waiting for Gazebo to start...')
        time.sleep(1.0)
    # Take an extra nap, to allow plugins to be loaded
    time.sleep(12.0)
    print('OK, starting test.')

    rostest.run('test_ariac', 'test_gripper_box_drop', GripperBoxDropTester, sys.argv)
