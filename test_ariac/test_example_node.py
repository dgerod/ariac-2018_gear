#!/usr/bin/env python

from __future__ import print_function

import sys
import time
import unittest

from ariac_example import ariac_example
from osrf_gear.srv import SubmitShipment
from std_msgs.msg import Float32
import rospy
import rostest


class ExampleNodeTester(unittest.TestCase):

    def comp_score_callback(self, msg):
        self.current_comp_score = msg.data

    def prepare_tester(self):
        self.comp_class = ariac_example.MyCompetitionClass()
        ariac_example.connect_callbacks(self.comp_class)

        self.current_comp_score = None
        self.comp_state_sub = rospy.Subscriber(
            "/ariac/current_score", Float32, self.comp_score_callback)

        # Pre-defined initial pose because sometimes the arm starts "droopy"
        self._send_arm_to_initial_pose()

    def test(self):
        self.prepare_tester()
        self._test_send_arm_to_zero_state()

        # Starting the competition will cause products from the order to be spawned on shipping_box_0
        self._test_start_comp()
        time.sleep(5.0)
        self._test_order_reception()

        self._test_conveyor_control(100)
        # Wait for the box to reach the end of the belt
        time.sleep(60)

        # Submit the shipping box
        self._test_drone_control()
        time.sleep(5.0)

        # Check the score
        self._test_comp_end()

    def _test_start_comp(self):
        success = ariac_example.start_competition()
        self.assertTrue(success, 'Failed to start the competition')
        time.sleep(1.5)
        self.assertTrue(
            self.comp_class.current_comp_state == 'go', 'Competition not in "go" state')

    def _test_order_reception(self):
        self.assertEqual(len(self.comp_class.received_orders), 1)

    def _send_arm_to_initial_pose(self):
        self.comp_class.send_arm_to_state([0] * len(self.comp_class.arm_joint_names))
        time.sleep(1.0)

    def _test_send_arm_to_zero_state(self):
        self.comp_class.send_arm_to_state([0] * len(self.comp_class.arm_joint_names))
        # This can be slow if there are a lot of models in the environment
        time.sleep(5.0)
        error = 0
        for position in self.comp_class.current_joint_state.position:
            error += abs(position - 0.0)
        self.assertTrue(error < 0.5, 'Arm was not properly sent to zero state')

    def _test_submit_shipment(self, shipping_box_index=0, shipment_id='order_0_shipment_0'):
        # This is a development mode cheat, so it is not in the ariac example itself.

        rospy.loginfo("Waiting for submit shipment service to be ready...")
        name = '/ariac/submit_shipment'
        rospy.wait_for_service(name)
        rospy.loginfo("Requesting shipment submission...")

        try:
            submit_shipment_srv = rospy.ServiceProxy(name, SubmitShipment)
            response = submit_shipment_srv(
                shipping_box_id='shipping_box_{0}::box_base'.format(shipping_box_index),
                shipment_type=shipment_id)
        except rospy.ServiceException as exc:
            rospy.logerr("Failed to submit the shipment: %s" % exc)
            return False
        if not response.success:
            rospy.logerr("Failed to submit the shipment: %s" % response)
        else:
            rospy.loginfo("Shipment submitted successfully")
        self.assertTrue(response.success, 'Failed to control drone')

    def _test_conveyor_control(self, power):
        success = ariac_example.control_conveyor(power)
        self.assertTrue(success, 'Failed to control conveyor')

    def _test_drone_control(self, shipment_id='order_0_shipment_0'):
        success = ariac_example.control_drone(shipment_id)
        self.assertTrue(success, 'Failed to control drone')

    def _test_comp_end(self):
        num_received_orders = len(self.comp_class.received_orders)
        num_shipments = len(self.comp_class.received_orders[0].shipments)
        if num_received_orders == 1 and num_shipments == 1:
            self.assertTrue(
                self.comp_class.current_comp_state == 'done', 'Competition not in "done" state')
        else:
            # If there were more shipments expected, the order won't be done
            self.assertTrue(
                self.comp_class.current_comp_state == 'go', 'Competition not in "go" state')
        num_products_in_order = len(self.comp_class.received_orders[0].shipments[0].products)
        self.assertTrue(
            # Expect to have a point for each non-faulty product, a point for each non-faulty
            # product's pose, and no all products bonus since there is a faulty product.
            self.current_comp_score == 2 * (num_products_in_order - 1),
            'Something went wrong in the scoring. Current score: ' + str(self.current_comp_score))


if __name__ == '__main__':
    rospy.init_node('test_example_node', anonymous=True)

    # Wait until /clock is being published; this can take an unpredictable
    # amount of time when we're downloading models.
    while rospy.Time.now().to_sec() == 0.0:
        print('Waiting for Gazebo to start...')
        time.sleep(1.0)
    # Take an extra nap, to allow plugins to be loaded
    time.sleep(20.0)
    print('OK, starting test.')

    rostest.run('osrf_gear', 'test_example_node', ExampleNodeTester, sys.argv)
