cmake_minimum_required(VERSION 3.8)
project(ld_parser_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Tìm các gói phụ thuộc (BẮT BUỘC)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(serial REQUIRED)

# THÊM MỚI: Cần tìm các gói liên quan đến TF và Odometry mà C++ đang dùng
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# Bạn cần cài đặt: sudo apt install libserial-	dev
#find_library(SERIAL_LIB serial REQUIRED)

# 2. Khai báo file thực thi (Node)
add_executable(raw_data_node src/raw_data_node.cpp)
add_executable(base_controller_node src/base_controller_node.cpp)

# 3. Liên kết thư viện vào Node (Đây là bước giải quyết lỗi No such file của bạn)
ament_target_dependencies(raw_data_node
  rclcpp
  sensor_msgs
  std_msgs
  serial
)

ament_target_dependencies(base_controller_node
  rclcpp
  std_msgs
  serial
  nav_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
)

#target_include_directories(raw_data_node PUBLIC
#  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#  $<INSTALL_INTERFACE:include>)

target_link_libraries(raw_data_node 
  ${rclcpp_LIBRARIES} 
  ${sensor_msgs_LIBRARIES}
  ${SERIAL_LIB}
)

install(TARGETS
  raw_data_node
  base_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
