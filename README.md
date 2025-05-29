아직 테스트는 하지 않았고 단순히 ros2용으로 바꾸기만 진행했습니다.

refernce : https://github.com/amslabtech/dwa_planner/tree/master/src

----------
How to run  

$ colcon build  
$ source install/setup.bash  
$ ros2 launch dwa_planner_ros2 dwa_planner_launch.py

----------

기본적인 외부에서 받아오는 인풋은 다음과 같습니다.
| 토픽명                      | 메시지 타입                      | 역할           | 누가 퍼블리시해야 하나? |
| ------------------------ | --------------------------- | ------------ | ------------- |
| `/scan`                  | `sensor_msgs/LaserScan`     | 장애물 정보       | 라이다 센서 노드     |
| `/odom`                  | `nav_msgs/Odometry`         | 로봇의 현재 위치/속도 | SLAM 또는 시뮬레이터 |
| `/move_base_simple/goal` | `geometry_msgs/PoseStamped` | 목표 위치 좌표     | RViz 등에서 클릭   |
