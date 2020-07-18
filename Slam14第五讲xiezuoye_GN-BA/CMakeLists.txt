cmake_minimum_required(VERSION 2.8)
#set(CMAKE_CXX_FLAGS "-std=c++0x")


project( GN-BA)

set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )
set( CMAKE_BUILD_TYPE "Release" )
#list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )
include_directories()
find_package(Pangolin)
include_directories(${Pangolin_INCLUDE_DIRS})
find_package(Sophus)

include_directories(${Sophus_INCLUDE_DIRS})

include_directories(/usr/include/eigen3/Eigen)

find_package(OpenCV 3.1 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})



#find_package(G2O REQUIRED)
#include_directories(${G2O_INCLUDE_DIRS})

list(APPEND CMAKE_MODULE_PATH /home/flk/g2o/cmake_modules)
#set(G2O_ROOT /home/flk/g2o)

find_package( G2O  REQUIRED)
find_package( CSparse REQUIRED )


include_directories(
        ${G2O_INCLUDE_DIRS}
        ${CSPARSE_INCLUDE_DIR}
        "/usr/include/eigen3/"
)

add_executable(${PROJECT_NAME} main.cpp)
target_link_libraries(${PROJECT_NAME}
        ${EIGEN3_lib}
        ${Sophus_LIBRARIES}
        ${Pangolin_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${OpenCV_LIBS}
        ${CSPARSE_LIBRARY}
        g2o_core g2o_stuff g2o_types_sba g2o_csparse_extension
        )

