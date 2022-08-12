cmake_minimum_required(VERSION 3.2)

project(DenseReconstruction)

set(CMAKE_CXX_STANDARD 11)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

set(CMAKE_INSTALL_PREFIX ${PROJECT_SOURCE_DIR})

set(DIVISIBLE_INSTALL_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include)
set(DIVISIBLE_INSTALL_BIN_DIR ${PROJECT_SOURCE_DIR}/bin)
set(DIVISIBLE_INSTALL_LIB_DIR ${PROJECT_SOURCE_DIR}/lib)

#set(DIVISION_HEADERS_DIR ${PROJECT_SOURCE_DIR}/src/division)

#include_directories(${DIVISIBLE_INSTALL_INCLUDE_DIR})
#include_directories(${DIVISION_HEADERS_DIR})

find_package(OpenCV CONFIG REQUIRED)
find_package(Ceres CONFIG REQUIRED)
find_package(PCL CONFIG REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)
find_package(Sophus CONFIG REQUIRED)
find_package(DBoW2 CONFIG REQUIRED)

include_directories(${PROJECT_SOURCE_DIR}/src)
include_directories(${PROJECT_SOURCE_DIR}/src/common)

if(WIN32)
    set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif(WIN32)


add_subdirectory(src)
add_subdirectory(app)
