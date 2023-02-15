set(MOVEIT_LIB_NAME moveit_planning_scene_interface)

add_library(${MOVEIT_LIB_NAME} src/planning_scene_interface.cpp)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(${MOVEIT_LIB_NAME} moveit_common_planning_interface_objects ${catkin_LIBRARIES} ${Boost_LIBRARIES})

#catkin_lint: ignore undefined_target
pybind_add_module(py${MOVEIT_LIB_NAME} src/wrap_python_planning_scene_interface.cpp)
target_link_libraries(py${MOVEIT_LIB_NAME} PRIVATE ${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS ${MOVEIT_LIB_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(TARGETS py${MOVEIT_LIB_NAME} DESTINATION ${CATKIN_GLOBAL_PYTHON_DESTINATION})

install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
