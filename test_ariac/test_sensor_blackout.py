#!/usr/bin/env python

from __future__ import print_function

import sys
import time

import geometry_msgs.msg
import rospy
import rostest
from test_example_node import ExampleNodeTester
from test_tf_frames import TfTester

import tf
import tf2_geometry_msgs  # noqa
import tf2_py as tf2
import tf2_ros

from osrf_gear.msg import LogicalCameraImage
from osrf_gear.msg import Proximity
from sensor_msgs.msg import Range
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud

class SensorBlackoutTester(TfTester):

    def test(self):
        self.prepare_tester()

        self.prepare_tf()

        # Starting the competition will cause the sensor blackout period to be triggered
        self._test_start_comp()
        rospy.sleep(2.5)

        self.subscribe_to_sensors()
        start_time = rospy.Time.now()
        rospy.loginfo("sleeping")
	while (rospy.Time.now() - start_time).to_sec() < 5:
            rospy.loginfo("sleeping")
            rospy.sleep(0.1)

    def _callback(self, msg):
        self.assertTrue(False, "Callback called with msg type: {0}".format(type(msg)))

    def subscribe_to_sensors(self):
        logical_camera_sub = rospy.Subscriber(
            "/ariac/logical_camera_1", LogicalCameraImage, self._callback)
        quality_control_sensor_sub = rospy.Subscriber(
            "/ariac/quality_control_sensor_1", LogicalCameraImage, self._callback)
        proximity_sensor_sub = rospy.Subscriber(
            "/ariac/proximity_sensor_1", Range, self._callback)
        depth_camera_sub = rospy.Subscriber(
            "/ariac/depth_camera_1", PointCloud, self._callback)
        laser_profile_sub = rospy.Subscriber(
            "/ariac/laser_profiler_1", LaserScan, self._callback)
        break_beam_sub = rospy.Subscriber(
            "/ariac/break_beam_1", Proximity, self._callback)
	'''
        order_sub = rospy.Subscriber("/ariac/orders", Order, comp_class.order_callback)
        joint_state_sub = rospy.Subscriber(
            "/ariac/joint_states", JointState, comp_class.joint_state_callback)
        gripper_state_sub = rospy.Subscriber(
            "/ariac/gripper/state", VacuumGripperState, comp_class.gripper_state_callback)

    def _test_faulty_products(self):
        quality_control_sensor = 'quality_control_sensor_1'
        # This product is faulty and should be reported as such in an anonymized way.
        self._test_pose(
            [0.1, -0.2, 0.0], tf.transformations.quaternion_from_euler(0, 0, 0),
            quality_control_sensor + '_model_1_frame',
            self.camera_above_box0 + '_shipping_box_0_frame'
        )

        # The model type should not be used in the name anymore (now anonymized).
        with self.assertRaises(tf2.LookupException):
            self._test_pose(
                [0.1, -0.2, 0.0], tf.transformations.quaternion_from_euler(0, 0, 0),
                quality_control_sensor + '_piston_rod_part_1_frame',
                self.camera_above_box0 + '_shipping_box_0_frame'
            )

        # This product is not faulty and should not be found by TF.
        with self.assertRaises(tf2.LookupException):
            self._test_pose(
                [0.1, -0.2, 0.0], tf.transformations.quaternion_from_euler(0, 0, 0),
                quality_control_sensor + '_model_3_frame',
                self.camera_above_box0 + '_shipping_box_0_frame'
            )
	'''


if __name__ == '__main__':
    rospy.init_node('test_sensor_blackout', anonymous=True)

    # Wait until /clock is being published; this can take an unpredictable
    # amount of time when we're downloading models.
    while rospy.Time.now().to_sec() == 0.0:
        print('Waiting for Gazebo to start...')
        time.sleep(1.0)
    # Take an extra nap, to allow plugins to be loaded
    time.sleep(10.0)
    print('OK, starting test.')

    rostest.run('test_ariac', 'test_sensor_blackout', SensorBlackoutTester, sys.argv)
