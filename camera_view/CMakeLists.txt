cmake_minimum_required(VERSION 3.8)
project(camera_view)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets Core Gui)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

set(dependencies
  rclcpp
  sensor_msgs
  cv_bridge
)

set(cpp_files
  src/main.cpp
  src/undistortionWindow.cpp
  src/undistortedWindow.cpp
  src/mainWindow.cpp
  src/tabs.cpp
  src/projectionWindow.cpp
  src/projectedWindow.cpp
  src/birdviewWindow.cpp
)

set(header_files
  include/camera_view/buffer.hpp
  include/camera_view/undistortionWindow.hpp
  include/camera_view/undistortedWindow.hpp
  include/camera_view/externVariables.hpp
  include/camera_view/mainWindow.hpp
  include/camera_view/settings.hpp
  include/camera_view/util.hpp
  include/camera_view/tabs.hpp
  include/camera_view/videoWidget.hpp
  include/camera_view/videoWindow.hpp
  include/camera_view/undistortionCamera.hpp
  include/camera_view/processThread.hpp
  include/camera_view/birdviewBuffer.hpp
  include/camera_view/projectionWindow.hpp
  include/camera_view/projectedWindow.hpp
  include/camera_view/birdviewWindow.hpp
)

# need to include header files in add_executable for compilation for some reason
add_executable(main ${cpp_files} ${header_files})
ament_target_dependencies(main ${dependencies})
target_link_libraries(main ${OpenCV_LIBRARIES} Qt5::Widgets Qt5::Core Qt5::Gui stdc++fs)

# add_executable(test_buffer_manager src/test_buffer_manager.cpp)
# ament_target_dependencies(test_buffer_manager ${dependencies})
# target_link_libraries(test_buffer_manager ${OpenCV_LIBRARIES})

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

install(TARGETS
  main
  # test_buffer_manager
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
