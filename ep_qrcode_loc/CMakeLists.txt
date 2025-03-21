cmake_minimum_required(VERSION 2.8.3)
project(ep_qrcode_loc)

#set(CMAKE_BUILD_TYPE "Debug")
#set(CMAKE_BUILD_TYPE "RelWithDebInfo")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "-std=c++11 -O3")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -Wall -g -pthread")

#设置第三方库的根目录
set(THIRD_LIBS_ROOT_DIR /opt/libs-x86_64-Linux)
list(APPEND CMAKE_MODULE_PATH ${THIRD_LIBS_ROOT_DIR}/cmake/modules)

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

add_service_files(
  DIRECTORY srv
  FILES
)

## 配置 msg 源文件
add_message_files(
  FILES
  LocCamera.msg
  CameraFrame_msg.msg
)

# 生成消息时依赖于 std_msgs
generate_messages(
  DEPENDENCIES
  std_msgs
)

#添加依赖库
find_package(Boost REQUIRED)
find_package(EPVcsSdk  MODULE)
find_package(YamlCpp  MODULE)

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
  ${CMAKE_MODULE_PATH}
  ${PROJECT_SOURCE_DIR}/src/mode
  ${PROJECT_SOURCE_DIR}/src/module
  ${PROJECT_SOURCE_DIR}/src/middle
  "/usr/include/x86_64-linux-gnu/"
  "/opt/libs-x86_64-Linux/eigen/eigen3"
  "/home/xun/work/ep-qrcode-loc/src/ep_qrcode_loc/lib/ep_vcs_sdk/common"
  "/home/xun/work/ep-qrcode-loc/src/ep_qrcode_loc/lib/ep_vcs_sdk/config"
  "/home/xun/work/ep-qrcode-loc/src/ep_qrcode_loc/lib/ep_vcs_sdk/listen"
  "/home/xun/work/ep-qrcode-loc/src/ep_qrcode_loc/lib/ep_vcs_sdk/security"
  "/home/xun/work/ep-qrcode-loc/src/ep_qrcode_loc/lib/ep_vcs_sdk"
)


###########
## Build ##
###########

add_executable(${PROJECT_NAME}_node src/main.cpp 
                                    src/middle/QRCodeLoc.cpp
                                    src/module/utilityQloc.cpp 
                                    src/module/logger.cpp
                                    src/module/ParamServer.cpp
                                    src/module/camera.cpp
                                    src/module/wheelOdom.cpp
                                    src/module/qrcodeTable_v3.cpp
                                    src/module/qrcodeTable_v2.cpp
                                    src/module/qrcodeTable_v1.cpp
                                    src/mode/Mode_AssistedDriving.cpp  
                                    src/mode/Mode_CheckCameraHorizon.cpp  
                                    src/mode/Mode_CollectQRCodePose.cpp
                                    src/mode/Mode_ShiHua.cpp
                                    src/mode/Mode_CalYawErr.cpp        
                                    src/mode/Mode_CollectQRCodeIndex.cpp  
                                    src/mode/Mode_GetYaw.cpp             
                                    src/mode/Mode_North.cpp  
                                    src/mode/Mode_TestRun.cpp
                                    )
                                    
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} )

include_directories(${EPCppc_INCLUDE_DIR})
target_link_libraries(${PROJECT_NAME}_node ${EPCppc_LIBRARYS})

include_directories(${Spdlog_INCLUDE_DIR})
target_link_libraries(${PROJECT_NAME}_node ${Spdlog_LIBRARYS})

include_directories(${EPVcsSdk_INCLUDE_DIR})
target_link_libraries(${PROJECT_NAME}_node ${EPVcsSdk_LIBRARYS})


include_directories(${YamlCpp_INCLUDE_DIR})
target_link_libraries(${PROJECT_NAME}_node ${YamlCpp_LIBRARYS})

#############
## Install ##
#############

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

##################
## 打包、安装脚本 ##
##################

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


