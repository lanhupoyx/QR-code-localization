cmake_minimum_required(VERSION 2.8.3)
project(ep_qrcode_loc)

#set(CMAKE_BUILD_TYPE "Debug")
#set(CMAKE_BUILD_TYPE "RelWithDebInfo")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "-std=c++14 -O3")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -Wall -g -pthread")

find_package(catkin REQUIRED COMPONENTS
  tf2_geometry_msgs
  tf2_ros
  tf2
  tf
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  message_generation
)

find_package(Boost REQUIRED)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  nav_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  DEPENDS 

  CATKIN_DEPENDS
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  message_runtime
  message_generation
)

# include directories
include_directories(
	include
	${catkin_INCLUDE_DIRS}
  "/opt/libs-x86_64-Linux/eigen/eigen3"
  "/home/zl/work/ep-qrcode-loc/src/ep_qrcode_loc/src/eskf"
  "/home/zl/work/ep-qrcode-loc/src/ep_qrcode_loc/src/eskf/ch3"
  "/home/zl/work/ep-qrcode-loc/src/ep_qrcode_loc/src/eskf/common"
  "/home/zl/work/ep-qrcode-loc/src/ep_qrcode_loc/src/eskf/sophus"
)

# link directories
link_directories(
	include
)

###########
## Build ##
###########

# Range Image Projection
add_executable(${PROJECT_NAME}_node src/QRCodeLoc.cpp)
# add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} )


install(TARGETS
      ${PROJECT_NAME}_node
      RUNTIME DESTINATION deb_package/opt/xmover/ros/melodic/${PROJECT_NAME}/lib/${PROJECT_NAME}
)

install(DIRECTORY 
      launch config
      DESTINATION deb_package/opt/xmover/ros/melodic/${PROJECT_NAME}/share/${PROJECT_NAME}
)  
install(FILES   
      package.xml 
      DESTINATION deb_package/opt/xmover/ros/melodic/${PROJECT_NAME}/share/${PROJECT_NAME}
)

#################
## 打包、安装脚本 ##
#################

install(FILES      install/control   DESTINATION deb_package/DEBIAN)
install(PROGRAMS   install/postinst  DESTINATION deb_package/DEBIAN)
install(PROGRAMS   install/postrm    DESTINATION deb_package/DEBIAN)
install(PROGRAMS   install/preinst   DESTINATION deb_package/DEBIAN)
install(PROGRAMS   install/prerm     DESTINATION deb_package/DEBIAN)
install(FILES   ./../../devel/setup.bash        DESTINATION deb_package/opt/xmover/ros/melodic/${PROJECT_NAME})
install(FILES   ./../../devel/setup.sh          DESTINATION deb_package/opt/xmover/ros/melodic/${PROJECT_NAME})
install(PROGRAMS   ./../../devel/_setup_util.py DESTINATION deb_package/opt/xmover/ros/melodic/${PROJECT_NAME})  
install(PROGRAMS   install/start.sh             DESTINATION deb_package/opt/xmover/ros/melodic/${PROJECT_NAME}/share/${PROJECT_NAME})
install(PROGRAMS   install/start_debug.sh       DESTINATION deb_package/opt/xmover/ros/melodic/${PROJECT_NAME}/share/${PROJECT_NAME})
install(PROGRAMS   install/ep-qrcode-loc.service    DESTINATION deb_package/usr/lib/systemd/system)
install(PROGRAMS   install/make_deb.sh          DESTINATION .)


