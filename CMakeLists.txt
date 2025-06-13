cmake_minimum_required(VERSION 3.8)
project(dwa_planner_ros2)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  ${PROJECT_NAME}/dwa_planner_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME dwa_planner_node
)
install(PROGRAMS
  ${PROJECT_NAME}/fake_sensor_publisher.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME fake_sensor_publisher
)
install(PROGRAMS
  ${PROJECT_NAME}/fake_goal_publisher.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME fake_goal_publisher
)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(FILES resource/${PROJECT_NAME} DESTINATION share/ament_index/resource_index/packages)

ament_package()