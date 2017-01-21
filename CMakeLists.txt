cmake_minimum_required(VERSION 2.8)

project(miivii_orbslam2_ros)

IF(NOT ROS_BUILD_TYPE)
  SET(ROS_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${ROS_BUILD_TYPE})

if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")       
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3   -march=native")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3  -march=native")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3   -march=armv8-a")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3  -march=armv8-a")
endif()

SET(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

execute_process(COMMAND sh build.sh 
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}  
  RESULT_VARIABLE PREBUILD_SCRIPT_RESULT)  

find_package(catkin REQUIRED COMPONENTS roscpp rospy sensor_msgs std_msgs  cv_bridge tf message_filters  genmsg)

add_service_files(DIRECTORY srv FILES  CreateMap.srv DeleteMap.srv LoadMap.srv Relocation.srv SaveMap.srv)
generate_messages(DEPENDENCIES std_msgs)

#find_package(OpenCV REQUIRED)
find_package(Eigen3  REQUIRED)
FIND_PACKAGE(OpenGL)
#find_package(Pangolin REQUIRED)
# PCL #######################                                                           
#find_package(PCL 1.7 REQUIRED COMPONENTS common io  visualization filters compression) 
#find_package(PCL 1.7 REQUIRED )                                                         
#include_directories(${PCL_INCLUDE_DIRS})                                                
#link_directories(${PCL_LIBRARY_DIRS})                                                   
#add_definitions(${PCL_DEFINITIONS})                                                     

include_directories(
${catkin_INCLUDE_DIRS}
${PROJECT_SOURCE_DIR}/include
${PROJECT_SOURCE_DIR}/ORB_SLAM2
${PROJECT_SOURCE_DIR}/ORB_SLAM2/include
${PROJECT_SOURCE_DIR}/ORB_SLAM2/Thirdparty/Pangolin/include
${PROJECT_SOURCE_DIR}/ORB_SLAM2/Thirdparty/Pangolin/build/src/include
${EIGEN3_INCLUDE_DIR}
#${Pangolin_INCLUDE_DIRS}
)
#message(${Pangolin_INCLUDE_DIRS})
#message(${Pangolin_LIBRARIES})
catkin_package()

set(LIBS 
#${OpenCV_LIBS} 
#${EIGEN3_LIBS}
#${Pangolin_LIBRARIES}
#${PCL_LIBRARIES}
${OPENGL_gl_LIBRARY}
${catkin_LIBRARIES}
${PROJECT_SOURCE_DIR}/ORB_SLAM2/Thirdparty/DBoW2/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/ORB_SLAM2/Thirdparty/g2o/lib/libg2o.so
${PROJECT_SOURCE_DIR}/ORB_SLAM2/Thirdparty/Pangolin/build/src/libpangolin.so
${PROJECT_SOURCE_DIR}/ORB_SLAM2/lib/libORB_SLAM2.so
)

# Node for RGB-D camera
add_executable(RGBD
src/ros_rgbd.cc
src/MapPublisher.cpp
src/OdomPublisher.cpp
)

target_link_libraries(RGBD
${catkin_LIBRARIES}
${LIBS}
)


add_executable(create_map_client src/create_map_client.cpp)
target_link_libraries(create_map_client ${catkin_LIBRARIES})
add_dependencies(create_map_client miivii_orbslam2_ros_gencpp)

add_executable(delete_map_client src/delete_map_client.cpp)
target_link_libraries(delete_map_client ${catkin_LIBRARIES})
add_dependencies(delete_map_client miivii_orbslam2_ros_gencpp)

add_executable(load_map_client src/load_map_client.cpp)
target_link_libraries(load_map_client ${catkin_LIBRARIES})
add_dependencies(load_map_client miivii_orbslam2_ros_gencpp)

add_executable(save_map_client src/save_map_client.cpp)
target_link_libraries(save_map_client ${catkin_LIBRARIES})
add_dependencies(save_map_client miivii_orbslam2_ros_gencpp)

add_executable(relocation_client src/relocation_client.cpp) 
target_link_libraries(relocation_client ${catkin_LIBRARIES})
add_dependencies(relocation_client miivii_orbslam2_ros_gencpp)

install(TARGETS RGBD create_map_client delete_map_client load_map_client save_map_client relocation_client ${ADDITIONAL_EXECUTABLES} 
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}                                                      
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}                                                      
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}                                                      
)                                                                                                                  
                                                                                                                    
install(DIRECTORY include/${PROJECT_NAME}/                                                                         
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}                                                          
        FILES_MATCHING PATTERN "*.h"                                                                            
)        
install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
    
install(DIRECTORY Data/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/Data)

install(FILES 
        ${PROJECT_SOURCE_DIR}/ORB_SLAM2/Thirdparty/DBoW2/lib/libDBoW2.so
        ${PROJECT_SOURCE_DIR}/ORB_SLAM2/Thirdparty/g2o/lib/libg2o.so
        ${PROJECT_SOURCE_DIR}/ORB_SLAM2/Thirdparty/Pangolin//build/src/libpangolin.so
        ${PROJECT_SOURCE_DIR}/ORB_SLAM2/lib/libORB_SLAM2.so

        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/../../lib

)                                                       
