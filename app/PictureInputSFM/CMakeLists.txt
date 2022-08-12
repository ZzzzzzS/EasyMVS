cmake_minimum_required(VERSION 3.2)

set(NAME_POSTFIX PictureSFM)
project(${CMAKE_PROJECT_NAME}_${NAME_POSTFIX})

set(SOURCE_FILES main.cpp)

add_executable(${PROJECT_NAME} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} DenseReconstruction_common)

install(TARGETS ${PROJECT_NAME} DESTINATION ${DIVISIBLE_INSTALL_BIN_DIR})