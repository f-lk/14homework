cmake_minimum_required(VERSION 2.8)
set(CMAKE_CXX_FLAGS "-std=c++0x")
project( directBA)
include_directories()
find_package(Pangolin)
include_directories(${Pangolin_INCLUDE_DIRS})
find_package(Sophus)

include_directories(${Sophus_INCLUDE_DIRS})

include_directories(/usr/include/eigen3/Eigen)

find_package(OpenCV 3 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(Threads)
include_directories(${Threads_INCLUED_DIRS})

add_executable(${PROJECT_NAME} directBA.cpp)
target_link_libraries(${PROJECT_NAME}
        ${EIGEN3_lib}
        ${Sophus_LIBRARIES}
        ${Pangolin_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${CMAKE_THREAD_LIBS_INIT}

        )
