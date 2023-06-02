cmake_minimum_required(VERSION 3.8)
project(telemetry_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/String.msg"
  "msg/Int.msg"
  "msg/Bool.msg"
  "msg/WifiStats.msg"
  "msg/Buzzer.msg"
  "msg/PlayBuzzer.msg"
  "msg/Imu.msg"
  "msg/Encoders.msg"
  "msg/LineSensors.msg"
  "msg/ProximitySensors.msg"
  "msg/Motors.msg"
  "msg/Zumo.msg"
  "msg/Battery.msg"
  "msg/UPSSettings.msg"
  "msg/CompressedImage.msg"
  "msg/CameraSettings.msg"
  "msg/SamplingRate.msg"
  "msg/EnableRead.msg"
)
ament_export_dependencies(rosidl_default_runtime)

ament_package()
