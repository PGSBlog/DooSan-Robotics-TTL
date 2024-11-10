개발 경로 : 

/home/droppgs/DOOSAN/task/turtlebot3_ws/src/table_service_robot/table_service_robot

navigation_node.py : navigation node => 통신 확인 코드,

navigation_gui.py : navigation gui => 로봇에게 이동, 복귀 명령 코드,

/home/droppgs/DOOSAN/task/turtlebot3_ws/src/table_service_robot/action

Delivery.action : 테이블 정보 확인, 출발, 이동 중, 도착, 복귀에 따른 액션 피드백을 위한 action 코드.

/home/droppgs/DOOSAN/task/turtlebot3_ws/src/table_service_robot/launch

restaurant_world.launch.py : 터틀봇 실행 런치 파일

/home/droppgs/DOOSAN/task/turtlebot3_ws/src/table_service_robot/worlds

restaurant_world.world : 테스트할 주방(SLAM으로 맵 로드하고 저장해서 불러왔는데 인식이 안돼서 새로 만듬)

/home/droppgs/DOOSAN/task/turtlebot3_ws/src/table_service_robot

package.xml : 주방 테이블 및 터틀봇 위치 정의 
setup.py : setup 코드


# 빌드 실행 방식(실행 순서는 꼭 준수하기 터미널1 -> 터미널 4 순으로 실행)

cd ~/[워크스페이스]
colcon build --symlink-install --packages-select table_service_robot

# 터미널 1
source install/setup.bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# 터미널 2
source install/setup.bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True

# 터미널 3
source install/setup.bash
ros2 run table_service_robot navigation_node

# 터미널 4
source install/setup.bash
ros2 run table_service_robot navigation_gui
