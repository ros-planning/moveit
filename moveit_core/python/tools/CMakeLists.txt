set(MOVEIT_LIB_NAME moveit_python_bindings_tools)

add_library(${MOVEIT_LIB_NAME}
  src/ros_msg_typecasters.cpp
  src/roscpp_initializer.cpp
)
add_dependencies(${MOVEIT_LIB_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${PYTHON_LIBRARIES})
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")

install(
  TARGETS ${MOVEIT_LIB_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})

#catkin_lint cannot detect target declarations in functions, here in pybind11_add_module
#catkin_lint: ignore undefined_target

# moveit.binding_tools
pybind_add_module(pymoveit_tools pymoveit_tools.cpp)
target_link_libraries(pymoveit_tools PRIVATE ${MOVEIT_LIB_NAME})

install(TARGETS pymoveit_tools
  LIBRARY DESTINATION ${CATKIN_GLOBAL_PYTHON_DESTINATION})
