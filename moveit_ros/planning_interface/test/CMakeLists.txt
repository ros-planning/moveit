if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  add_executable(test_cleanup test_cleanup.cpp)
  target_link_libraries(test_cleanup moveit_move_group_interface)

  add_rostest_gtest(move_group_interface_cpp_test move_group_interface_cpp_test.test move_group_interface_cpp_test.cpp)
  target_link_libraries(move_group_interface_cpp_test moveit_move_group_interface ${catkin_LIBRARIES})

  add_rostest_gtest(move_group_pick_place_test move_group_pick_place_test.test move_group_pick_place_test.cpp)
  target_link_libraries(move_group_pick_place_test moveit_move_group_interface ${catkin_LIBRARIES})

  add_rostest_gtest(subframes_test subframes_test.test subframes_test.cpp)
  target_link_libraries(subframes_test moveit_move_group_interface ${catkin_LIBRARIES})

  add_rostest(python_move_group.test)
  add_rostest(python_move_group_ns.test)
  add_rostest(robot_state_update.test)
  add_rostest(dual_arm_robot_state_update.test)
  add_rostest(cleanup.test)

endif()
