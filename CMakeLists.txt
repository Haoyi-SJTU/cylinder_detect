cmake_minimum_required(VERSION 3.0.2)

#SET(CMAKE_C_COMPILER "/usr/bin/gcc-7")
#SET(CMAKE_CXX_COMPILER "/usr/bin/g++-7")

project(rvv C CXX)

#set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/../../debug)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/../../release)
set(CMAKE_BUILD_TYPE "Release")

#dd_compile_options(-std=c++11)
add_compile_options(-std=c++11)
option(USE_OPENCV "Enable samples which depend on OpenCV" OFF)
option(USE_PCL "Enable samples which depend on PCL" OFF)
option(USE_Open3D "Enable samples which depend on Open3D" OFF)

if (RVC_BUILD_SDK)
    set(RVC_INCLUDE_DIRS "${CMAKE_SOURCE_DIR}/Include")
    set(RVC_LIBS RVC)
else()
    if(WIN32)
    else(WIN32)
        set(RVC_ROOT "/opt/RVC")
        set(RVC_INCLUDE_DIRS "${RVC_ROOT}/include")
        set(RVC_LIB_DIR "${RVC_ROOT}/lib")
        file(GLOB RVC_LIBS "${RVC_LIB_DIR}/libRVC.so" ABSOLUTE)
    endif(WIN32)
    list(LENGTH RVC_LIBS RVC_LIBS_LEN)
endif()



macro(disable_samples DEPENDENCY_NAME)
    foreach(SAMPLE ${SAMPLES})
        get_filename_component(SAMPLE_NAME PointCloud NAME)
        set(DEPENDENCY_LIST "${DEPENDENCY_NAME}_DEPENDING")
    endforeach()
endmacro()
# 临时注释掉PCL路径设置
#set(PCL_ROOT "$ENV{HOME}/Rvbust/Install/PCL")
#set(PCL_DIR "${PCL_ROOT}/share/pcl-1.9")

#find_package(PCL 1.2 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  pcl_conversions
  pcl_ros
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    src/${PROJECT_NAME}/include
    ${RVC_INCLUDE_DIRS}
)

if(RVC_EXTERNAL_WITH_OPENCV)
    message(STATUS "using OpenCV library find in external")
else()
    set(OpenCV_ROOT "$ENV{HOME}/Rvbust/Install/OpenCV")
    set(OpenCV_DIR "${OpenCV_ROOT}/lib/cmake/opencv4" CACHE FILEPATH "Set to find OpenCV")
    find_package(OpenCV COMPONENTS imgcodecs)
endif()

add_library(Save_IO ${CMAKE_CURRENT_SOURCE_DIR}/IO/SaveImage.cpp ${CMAKE_CURRENT_SOURCE_DIR}/IO/SavePointMap.cpp ${CMAKE_CURRENT_SOURCE_DIR}/IO/FileIO.cpp)
target_link_libraries(Save_IO ${OpenCV_LIBS})
target_include_directories(Save_IO SYSTEM PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(Save_IO stdc++fs)


# ## Generate services in the 'srv' folder
add_service_files(FILES
  result.srv
)
# ## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES 
  geometry_msgs  
  std_msgs
)
# generate_messages(DEPENDENCIES std_msgs)
catkin_package(
    CATKIN_DEPENDS 
    std_msgs 
    message_runtime
)


link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(PointCloud PointCloud.cpp)
target_link_libraries(PointCloud ${RVC_LIBS} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${catkin_LIBRARIES} Save_IO)
target_include_directories(PointCloud SYSTEM PRIVATE ${RVC_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR} ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

