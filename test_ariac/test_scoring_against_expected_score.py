#!/usr/bin/env python

from __future__ import print_function

import sys
import time

from test_example_node import ExampleNodeTester
import rospy
import rostest


class ScoringTester(ExampleNodeTester):

    def test(self):
        expectedScore = float(sys.argv[1])
        rospy.loginfo('Using expected score of: ' + str(expectedScore))
        if len(sys.argv) > 2:
            self.agv2_first = sys.argv[2] == '--agv2-first'
        self.prepare_tester()

        # Starting the competition will cause parts from the order to be spawned on shipping_box_0
        self._test_start_comp()

        if self.agv2_first:
            ''' commented because only one drone supported
            # Submit the tray on AGV2
            self._test_drone_control(index=2, shipment_id='order_1_shipment_0')
            time.sleep(5.0)
            # Submit the tray on shipping_box_0
            self._test_drone_control()
            time.sleep(5.0)
            '''
        else:
            # Submit the tray on shipping_box_0
            self._test_submit_shipment()
            time.sleep(5.0)
            ''' commented because only one drone supported
            # Submit the tray on AGV2
            self._test_drone_control(index=2, shipment_id='order_1_shipment_0')
            time.sleep(5.0)
            '''

        self.assertTrue(
            self.current_comp_score == expectedScore,
            'Something went wrong in the scoring. Expected score of ' + str(expectedScore) +
                ' but received: ' + str(self.current_comp_score))


if __name__ == '__main__':
    rospy.init_node('test_scoring_against_expected_score', anonymous=True)

    # Wait until /clock is being published; this can take an unpredictable
    # amount of time when we're downloading models.
    while rospy.Time.now().to_sec() == 0.0:
        print('Waiting for Gazebo to start...')
        time.sleep(1.0)
    # Take an extra nap, to allow plugins to be loaded
    time.sleep(10.0)
    print('OK, starting test.')

    rostest.run('test_ariac', 'test_scoring_against_expected_score', ScoringTester, sys.argv)
