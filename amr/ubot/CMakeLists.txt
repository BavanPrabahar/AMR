cmake_minimum_required(VERSION 3.8)
project(ubot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

install(PROGRAMS
  src/Ubot_Realsense_Publisher.py
  src/Ubot_Motor_Controller.py
  src/Ubot_Local_Motor_Controller.py
  src/Ubot_Obstacle_Avoidance.py
  src/Ubot_Safety.py
  src/Ubot_Send_Data.py
  src/Ubot_Receive_Data.py
  src/Ubot_Servo_Subscriber.py
  src/Ubot_Battery_Status.py
  src/end.py
  src/enco_conv.py
  src/revised_rpm.py
  src/diff_motor_controller_2.py
  src/odom_baselink.py
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
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
