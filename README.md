아직 테스트는 하지 않았고 단순히 ros2용으로 바꾸기만 진행했습니다.

refernce : https://github.com/amslabtech/dwa_planner/tree/master/src

----------
How to run  

단일 노드 켜지는지 테스트  
$ colcon build  
$ source install/setup.bash  
$ ros2 run dwa_planner_ros2 dwa_planner_node --ros-args -p config_file:=config/params.yaml

test_avoid_simulation 테스트 (fake input, fake goal이 있을때 장애물 회피 경로 나오는지 테스트)  
$ colcon build  
$ source install/setup.bash  
$ ros2 launch dwa_planner_ros2 test_evasion_launch.py

----------

멀티 노드 테스트 (우리가 실제로 사용하게 될 것)
$ colcon build  
$ source install/setup.bash  
$ ros2 launch dwa_planner_ros2 dwa_planner_launch.py  
