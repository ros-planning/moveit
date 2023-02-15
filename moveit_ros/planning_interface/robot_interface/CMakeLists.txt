set(MOVEIT_LIB_NAME moveit_robot_interface)

#catkin_lint: ignore undefined_target
pybind_add_module(py${MOVEIT_LIB_NAME} src/wrap_python_robot_interface.cpp)
target_link_libraries(py${MOVEIT_LIB_NAME} PRIVATE ${catkin_LIBRARIES} ${Boost_LIBRARIES}
  moveit_common_planning_interface_objects)

install(TARGETS py${MOVEIT_LIB_NAME} DESTINATION ${CATKIN_GLOBAL_PYTHON_DESTINATION})
