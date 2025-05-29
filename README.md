아직 테스트는 하지 않았고 단순히 ros2용으로 바꾸기만 진행했습니다.

refernce : https://github.com/amslabtech/dwa_planner/tree/master/src

----------
How to run  

단일 노드 켜지는지 테스트
$ colcon build  
$ source install/setup.bash  
$ ros2 launch dwa_planner_ros2 dwa_planner_launch.py

test_avoid_simulation 테스트 (fake input, fake goal이 있을때 장애물 회피 경로 나오는지 테스트)
$ colcon build  
$ source install/setup.bash  
$ ros2 launch dwa_planner_ros2 test_evasion_launch.py

----------
