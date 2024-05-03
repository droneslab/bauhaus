cmake_minimum_required(VERSION 2.8)

cmake_policy(SET CMP0069 NEW) 
set(CMAKE_POLICY_DEFAULT_CMP0069 NEW)

project(DBoW2)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Sofiya: Removed -march=native, based on https://github.com/raulmur/ORB_SLAM2/issues/341 
# Sofiya: Added -flto=thin to enable link time optimization with Rust: https://doc.rust-lang.org/rustc/linker-plugin-lto.html#toolchain-compatibility

set(CMAKE_CXX_COMPILER "/home/nitin/Downloads/clang+llvm-17.0.6-x86_64-linux-gnu-ubuntu-22.04/bin/clang")
add_link_options(-fuse-ld=/home/nitin/Downloads/clang+llvm-17.0.6-x86_64-linux-gnu-ubuntu-22.04/bin/ld.lld)
set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE)

set(HDRS_DBOW2
  DBoW2/BowVector.h
  DBoW2/FORB.h 
  DBoW2/FClass.h       
  DBoW2/FeatureVector.h
  DBoW2/ScoringObject.h   
  DBoW2/TemplatedVocabulary.h
  DBoW2/ORBVocabulary.h)
set(SRCS_DBOW2
  DBoW2/BowVector.cpp
  DBoW2/FORB.cpp      
  DBoW2/FeatureVector.cpp
  DBoW2/ScoringObject.cpp
  DBoW2/ORBVocabulary.cpp)

set(HDRS_DUTILS
  DUtils/Random.h
  DUtils/Timestamp.h)
set(SRCS_DUTILS
  DUtils/Random.cpp
  DUtils/Timestamp.cpp)


find_package(OpenCV 4.5)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 4.5 not found.")
   endif()

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(${OpenCV_INCLUDE_DIRS})
add_library(DBoW2 SHARED ${SRCS_DBOW2} ${SRCS_DUTILS})
target_link_libraries(DBoW2 ${OpenCV_LIBS})

