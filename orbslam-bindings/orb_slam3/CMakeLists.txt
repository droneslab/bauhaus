cmake_minimum_required (VERSION 3.0)
project(orb_slam3)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -g") # -std=c++17
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -g")

# Sofiya: Removed -march=native, based on https://github.com/raulmur/ORB_SLAM2/issues/341 
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} ")
set(CMAKE_CXX_FLAGS_RELEASE"${CMAKE_CXX_FLAGS_RELEASE} ")

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_EXTENSIONS OFF)
add_definitions(-DCOMPILEDWITHC11)

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(OpenCV 4.5)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 4.5 not found.")
   endif()

# There seems to be an issue with MSVC8
# see http://eigen.tuxfamily.org/bz/show_bug.cgi?id=83
if(MSVC90)
  add_definitions(-DEIGEN_DONT_ALIGN_STATICALLY=1)
  message(STATUS "Disabling memory alignment for MSVC8")
endif(MSVC90)


MESSAGE("OPENCV VERSION:")
MESSAGE(${OpenCV_VERSION})

find_package (Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Pangolin REQUIRED)

#include(${CMAKE_CURRENT_SOURCE_DIR}/Sophus/CMakeLists.txt)

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/Sophus
)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

#add_subdirectory(DBoW2)
#add_subdirectory(Sophus)

# file(GLOB dbow2_SRC
#       ${PROJECT_SOURCE_DIR}/DBoW2/
#      "*.h"
#      "*.cpp"
# )

#set( ${PROJECT_SOURCE_DIR}/DBoW2/DUtils/...)
include_directories(../target/cxxbridge)

add_library(${PROJECT_NAME} STATIC
src/CVConvert.cc
src/CVConvert.h
src/ORBextractor.cc
src/ORBextractor.h
src/ORBmatcher.cc
src/ORBmatcher.h
src/TwoViewReconstruction.cc
src/TwoViewReconstruction.h
src/Converter.cc
src/Converter.h
src/GeometricTools.cc
src/GeometricTools.h
../target/cxxbridge/dvos3binding/src/lib.rs.cc
../target/cxxbridge/dvos3binding/src/lib.rs.h
DBoW2/DBoW2/BowVector.h
DBoW2/DBoW2/FORB.h 
DBoW2/DBoW2/FClass.h       
DBoW2/DBoW2/FeatureVector.h
DBoW2/DBoW2/ScoringObject.h   
DBoW2/DBoW2/TemplatedVocabulary.h
DBoW2/DBoW2/BowVector.cpp
DBoW2/DBoW2/FORB.cpp      
DBoW2/DBoW2/FeatureVector.cpp
DBoW2/DBoW2/ScoringObject.cpp
DBoW2/DUtils/Random.h
DBoW2/DUtils/Timestamp.h
DBoW2/DUtils/Random.cpp
DBoW2/DUtils/Timestamp.cpp
DBoW2/DBoW2/ORBVocabulary.cpp
DBoW2/DBoW2/ORBVocabulary.h
)

target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
Eigen3::Eigen
${Pangolin_LIBRARIES}
${PROJECT_SOURCE_DIR}/DBoW2/lib/libDBoW2.so
-lboost_serialization
-lcrypto
)

