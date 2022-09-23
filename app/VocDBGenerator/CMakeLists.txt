cmake_minimum_required(VERSION 3.2)

project(VocDBGenerate)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(DBoW2 CONFIG REQUIRED)
find_package(OpenCV CONFIG REQUIRED)

set(SOURCE_FILES main.cpp)

add_executable(${PROJECT_NAME} ${SOURCE_FILES} ${HEADER_FILES})

target_link_libraries(${PROJECT_NAME} PRIVATE DenseReconstruction_FeatureExtraction)
target_link_libraries(${PROJECT_NAME} PRIVATE ${DBoW2_LIBS})
target_link_libraries(${PROJECT_NAME} PRIVATE ${OpenCV_LIBS})
target_link_libraries(${PROJECT_NAME} PRIVATE argparse::argparse)

