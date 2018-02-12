#!/usr/bin/env python

from __future__ import print_function

import sys
import time
import unittest

from test_example_node import ExampleNodeTester
from ariac_example import ariac_example
from std_msgs.msg import Float32
import rospy
import rostest


class ScoringTester(ExampleNodeTester):

    def test(self):
        self.prepare_tester()

        # Starting the competition will cause products from the order to be spawned on shipping_box_0
        self._test_start_comp()

        # Submit the shipping box as soon as the products are spawned on the shipping box
        time.sleep(2.5)  # OK, give a little bit of leeway because that's what teams would do
        self._test_submit_shipment()
        time.sleep(5.0)

        # Check the score
        self._test_comp_end()


if __name__ == '__main__':
    rospy.init_node('test_scoring_immediate', anonymous=True)

    # Wait until /clock is being published; this can take an unpredictable
    # amount of time when we're downloading models.
    while rospy.Time.now().to_sec() == 0.0:
        print('Waiting for Gazebo to start...')
        time.sleep(1.0)
    # Take an extra nap, to allow plugins to be loaded
    time.sleep(10.0)
    print('OK, starting test.')

    rostest.run('test_ariac', 'test_scoring_immediate', ScoringTester, sys.argv)
