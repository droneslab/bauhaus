cmake_minimum_required(VERSION 2.8)
project(foxglove CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O0 -g")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O0 -g")

set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE}")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE}")


add_library(${PROJECT_NAME} STATIC 
    JSONWriter.cc
    JSONWriter.h
)

target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
${EIGEN3_LIBS}
)

