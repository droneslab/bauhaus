cmake_minimum_required(VERSION 2.8)
project(orb_slam3)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall   -O0 -g ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O0 -g ")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -march=native -g ")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native -g ")

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_EXTENSIONS OFF)
add_definitions(-DCOMPILEDWITHC11)

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(OpenCV 4.4)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 4.4 not found.")
   endif()

MESSAGE("OPENCV VERSION:")
MESSAGE(${OpenCV_VERSION})

find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)

#include(${CMAKE_CURRENT_SOURCE_DIR}/Sophus/CMakeLists.txt)

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/Sophus
${EIGEN3_INCLUDE_DIR}
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

add_library(${PROJECT_NAME} STATIC
src/DVMat.cc
src/DVMat.h
src/DVConvert.cc
src/DVConvert.h
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

)

target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${PROJECT_SOURCE_DIR}/DBoW2/lib/libDBoW2.so
-lboost_serialization
-lcrypto
)

