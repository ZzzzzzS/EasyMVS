cmake_minimum_required(VERSION 3.2)

set(NAME_POSTFIX PoseReconstruction)
project(${CMAKE_PROJECT_NAME}_${NAME_POSTFIX})


set(SOURCE_FILES PoseOptimizer.cpp
                PoseReconstructor.cpp)

set(HEADER_FILES PoseOptimizer.h
                PoseReconstructor.h)

add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES} ${HEADER_FILES})
install(TARGETS ${PROJECT_NAME} DESTINATION ${DIVISIBLE_INSTALL_LIB_DIR})
install(FILES ${HEADER_FILES} DESTINATION ${DIVISIBLE_INSTALL_INCLUDE_DIR}/${NAME_POSTFIX})