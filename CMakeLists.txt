cmake_minimum_required(VERSION 3.8)
project(webrtc_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Force use of system OpenCV (avoid CUDA-compiled version in /usr/local)
set(OpenCV_DIR "/usr/lib/x86_64-linux-gnu/cmake/opencv4")

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(x1_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenSSL REQUIRED)

# Fetch dependencies
include(FetchContent)

FetchContent_Declare(
  json
  GIT_REPOSITORY https://github.com/nlohmann/json.git
  GIT_TAG v3.11.3
)
FetchContent_MakeAvailable(json)

FetchContent_Declare(
  libdatachannel
  GIT_REPOSITORY https://github.com/paullouisageneau/libdatachannel.git
  GIT_TAG v0.21.2
)

set(NO_EXAMPLES ON CACHE BOOL "" FORCE)
set(NO_TESTS ON CACHE BOOL "" FORCE)
set(NO_WEBSOCKET OFF CACHE BOOL "" FORCE)
set(NO_MEDIA OFF CACHE BOOL "" FORCE)

FetchContent_MakeAvailable(libdatachannel)

# Include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# Create executable
add_executable(webrtc_bridge_node
  src/main.cpp
  src/webrtc_bridge_node.cpp
  src/signaling_server.cpp
  src/video_encoder.cpp
  src/data_channel_handler.cpp
)

target_link_libraries(webrtc_bridge_node
  datachannel
  nlohmann_json::nlohmann_json
  ${OpenCV_LIBS}
  OpenSSL::SSL
  OpenSSL::Crypto
)

ament_target_dependencies(webrtc_bridge_node
  rclcpp
  std_msgs
  sensor_msgs
  x1_msgs
  cv_bridge
  image_transport
)

# Install
install(TARGETS
  webrtc_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY client/
  DESTINATION share/${PROJECT_NAME}/client
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
