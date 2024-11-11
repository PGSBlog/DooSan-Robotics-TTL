cmake_minimum_required(VERSION 3.8)
project(ttb_gui)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Message.action"
  "action/Delivery.action"
)




# Install the Python script
install(
  PROGRAMS 
    scripts/order_manager_gui.py  # scripts 폴더의 Python 파일 지정
    scripts/pub_topic.py          # 새로 추가한 스크립트
    scripts/action_Client.py
    scripts/action_Server.py
    scripts/delivery_order.py
    scripts/delivery_server.py
    scripts/delivery_topic.py
  DESTINATION lib/${PROJECT_NAME}        # 패키지 실행 디렉토리로 설치
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
