cmake_minimum_required(VERSION 2.8.3)
project(test_ariac)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
endif()

find_package(catkin REQUIRED COMPONENTS
  ariac_example
  osrf_gear
)

# Used by test/
set(ROS_PACKAGE_NAME osrf_gear)
catkin_package(
  CATKIN_DEPENDS
  osrf_gear
  ariac_example
)

set(rostests
  ariac_rosapi.test
  ariac_rosapi_development_mode.test
  ariac_example.test
  ariac_tf_frames.test
  gripper.test
  # gripper_unthrottled.test  # this needs a good gripper pose; regression is fixed.
  test_scoring/ariac_scoring_perfect_shipment.test
  test_scoring/ariac_scoring_after_waiting.test
  test_scoring/ariac_scoring_after_waiting_with_state_logging.test
  test_scoring/ariac_scoring_perfect_shipment_flipped.test
  test_scoring/ariac_scoring_correct_parts_incorrect_poses.test
  test_scoring/ariac_scoring_correct_parts_plus_unwanted.test
  test_scoring/ariac_scoring_correct_parts_plus_faulty.test
  test_scoring/ariac_scoring_correct_parts_not_flipped.test
  test_scoring/ariac_scoring_missing_parts.test
  test_scoring/ariac_scoring_low_priority_order.test
  test_scoring/ariac_scoring_interrupted.test
  test_scoring/ariac_scoring_not_interrupted.test
  test_scoring/ariac_scoring_faulty_parts.test
  test_scoring/ariac_scoring_faulty_parts_randomized.test
)

foreach(rostest ${rostests})
  # We don't set a timeout here because we trust rostest to enforce the
  # timeout specified in each .test file.
  add_test(${rostest} rostest ${CMAKE_CURRENT_SOURCE_DIR}/${rostest})
  # Check for test result file and create one if needed.  rostest can fail to
  # generate a file if it throws an exception.
  add_test(check_${rostest} rosrun rosunit check_test_ran.py --rostest ${ROS_PACKAGE_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/${rostest})
endforeach()

# Install test files because we'll run tests from the install tree.  We could
# get away with running most of the test from the source tree, but it's clearer
# and safer to run them from the install tree.
install(FILES
  ${rostests}
  ariac_rosapi.yaml
  ariac_rosapi_development_mode.yaml
  minimal_sample_unthrottled.yaml
  tf_sensors.yaml
  test_scoring/scoring_base_order.yaml
  test_scoring/scoring_base_order_flipped.yaml
  test_scoring/scoring_base_order_interrupted.yaml
  test_scoring/scoring_partial_shipment_both_shipments.yaml
  test_scoring/scoring_perfect_shipment.yaml
  test_scoring/scoring_perfect_shipment_both_shipments.yaml
  test_scoring/scoring_perfect_shipment_flipped.yaml
  test_scoring/scoring_correct_parts_incorrect_poses.yaml
  test_scoring/scoring_correct_parts_plus_unwanted.yaml
  test_scoring/scoring_correct_parts_plus_faulty.yaml
  test_scoring/scoring_correct_parts_not_flipped.yaml
  test_scoring/scoring_missing_parts.yaml
  test_scoring/scoring_faulty_parts.yaml
  test_scoring/scoring_faulty_parts_randomized.yaml
  test_scoring/scoring_perfect_shipment_both_shipments.yaml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/test
)

install(PROGRAMS
  ros_api_checker
  test_example_node.py
  test_gripper.py
  test_gripper_unthrottled.py
  test_scoring_after_waiting.py
  test_scoring_after_waiting.py
  test_scoring_immediate.py
  test_scoring_against_expected_score.py
  test_tf_frames.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/test
)
