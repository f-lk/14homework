cmake_minimum_required(VERSION 2.8)
set(CMAKE_CXX_FLAGS "-std=c++0x")
project(tt)
#include_directories(/usr/include/eigen3)
#include_directories(/home/flk/yuanma/slambook-master/3rdparty/Pangolin/include)
#include_directories(/home/flk/yuanma/slambook-master/3rdparty/Pangolin/build/src/include)
find_package(Pangolin)
include_directories(${Pangolin_INCLUDE_DIRS})
find_package(Sophus)
include_directories(${Sophus_INCLUDE_DIRS})
add_executable(tt draw_trajectory.cpp)
target_link_libraries(${PROJECT_NAME}
        ${EIGEN3_lib}
        ${Sophus_LIBRARIES}
        ${Pangolin_LIBRARIES})

#PROJECT(drawtrace)
#set(CMAKE_CXX_FLAGS "-std=c++11")
# Check C++11 or C++0x support
#include(CheckCXXCompilerFlag)
#CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
#CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
#if(COMPILER_SUPPORTS_CXX11)
#    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
#    add_definitions(-DCOMPILEDWITHC11)
#    message(STATUS "Using flag -std=c++11.")
#elseif(COMPILER_SUPPORTS_CXX0X)
#    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
#    add_definitions(-DCOMPILEDWITHC0X)
#    message(STATUS "Using flag -std=c++0x.")
#else()
#    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
#endif()
#
#LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

#find_package(OpenCV 3.0 QUIET)
#if(NOT OpenCV_FOUND)
#    find_package(OpenCV 2.4.3 QUIET)
#    if(NOT OpenCV_FOUND)
#        message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
#    endif()
#endif()


#find_package(Sophus)
#include_directories( ${Sophus_INCLUDE_DIRS} )
#
## eigen
#include_directories( "/usr/include/eigen3/" )
#
#
#find_package(Pangolin)
#include_directories( ${Pangolin_INCLUDE_DIRS} )
#
#
#ADD_EXECUTABLE(drawtrace draw_trajectory.cpp)
#target_link_libraries(${PROJECT_NAME}
#        ${EIGEN3_LIBS}
#        ${Pangolin_LIBRARIES}
#        ${Sophus_LIBRARIES}
#        )
