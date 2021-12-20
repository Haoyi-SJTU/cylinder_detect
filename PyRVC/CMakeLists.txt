cmake_minimum_required(VERSION 2.8.12)
project(PyRVC)

if (RVC_BUILD_SDK)

  set(RVC_INCLUDE_DIRS "${CMAKE_SOURCE_DIR}/Include")
  set(RVC_LIBS RVC)

else()

  if(WIN32)

    set(RVC_ROOT "$ENV{RVC_ROOT}")
    set(RVC_INCLUDE_DIRS "${RVC_ROOT}/include")
    set(RVC_LIB_DIR "${RVC_ROOT}/lib")
    file(GLOB RVC_LIBS "${RVC_LIB_DIR}/*.lib" ABSOLUTE)

  else(WIN32)

    set(RVC_ROOT "/opt/RVC")
    set(RVC_INCLUDE_DIRS "${RVC_ROOT}/include")
    set(RVC_LIB_DIR "${RVC_ROOT}/lib")
    file(GLOB RVC_LIBS "${RVC_LIB_DIR}/libRVC.so" ABSOLUTE)

  endif(WIN32)

  list(LENGTH RVC_LIBS RVC_LIBS_LEN)

  set(pybind11_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Pybind11/share/cmake/pybind11/" CACHE FILEPATH "Set to find pybind11")
  find_package(pybind11 CONFIG REQUIRED)
endif()
    
message(STATUS "Using RVC library in ${RVC_LIBS}.")


pybind11_add_module(PyRVC PyRVC.cpp)
if (RVC_DEVELOPER)
  target_include_directories(PyRVC PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
else()
  target_include_directories(PyRVC PUBLIC ${RVC_INCLUDE_DIRS})
endif()
target_link_libraries(PyRVC PUBLIC ${RVC_LIBS})

