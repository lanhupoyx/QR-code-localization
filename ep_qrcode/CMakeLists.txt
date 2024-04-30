cmake_minimum_required(VERSION 2.8.3)
project(ep_qrcode)

#set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_BUILD_TYPE "RelWithDebInfo")
#set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "-std=c++14 -O3")
set(CMAKE_CXX_FLAGS_DEBUG "-O3 -Wall -g -pthread")

find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  message_generation
)

find_package(Boost REQUIRED)

add_message_files(
  DIRECTORY msg
  FILES
  qrcodedebug.msg
)

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
)

# link directories
link_directories(
	include
)

###########
## Build ##
###########

# Range Image Projection
add_executable(ep_qrcode src/main.cpp)
# add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(ep_qrcode ${catkin_LIBRARIES} )


#############
## Install ##
#############

install(TARGETS
    ep_qrcode
    RUNTIME DESTINATION deb_package/opt/xmover/ros/melodic/ep_qrcode/lib/ep_qrcode
)

install(FILES
   launch/ep_qrcode.launch
   DESTINATION deb_package/opt/xmover/ros/melodic/ep_qrcode/share/ep_qrcode/launch
)

install(FILES
   package.xml
   DESTINATION deb_package/opt/xmover/ros/melodic/ep_qrcode/share/ep_qrcode
)

#################
## 打包、安装脚本 ##
#################

install(FILES
   install/start.sh
   DESTINATION deb_package/opt/xmover/ros/melodic/ep_qrcode/share/ep_qrcode
)

install(FILES
   install/control
   DESTINATION deb_package/DEBIAN
)

install(FILES
   install/postinst
   DESTINATION deb_package/DEBIAN
)
      
install(FILES
   install/postrm
   DESTINATION deb_package/DEBIAN
)

install(FILES
   install/preinst
   DESTINATION deb_package/DEBIAN
)

install(FILES
   install/prerm
   DESTINATION deb_package/DEBIAN
)

install(FILES
   install/ep-qrcode.service
   DESTINATION deb_package/usr/lib/systemd/system
)



