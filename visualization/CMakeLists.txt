cmake_minimum_required(VERSION 3.5)

set(NAME_POSTFIX Visualization)
project(${CMAKE_PROJECT_NAME}_${NAME_POSTFIX})

set(SOURCE_FILES QPointCloudViewer.cpp
                 ScrollScale.cpp)

set(HEADER_FILES QPointCloudViewer.h
                 ScrollScale.h)

add_library(${PROJECT_NAME} STATIC ${SOURCE_FILES} ${HEADER_FILES})
target_link_libraries(${PROJECT_NAME} PRIVATE ${PCL_LIBRARIES})
target_link_libraries(${PROJECT_NAME} PRIVATE ${OpenCV_LIBS})
target_link_libraries(${PROJECT_NAME} PRIVATE Qt${QT_VERSION_MAJOR}::Core
                                              Qt${QT_VERSION_MAJOR}::Widgets)

install(TARGETS ${PROJECT_NAME} DESTINATION ${DIVISIBLE_INSTALL_LIB_DIR})
install(FILES ${HEADER_FILES} DESTINATION ${DIVISIBLE_INSTALL_INCLUDE_DIR}/${NAME_POSTFIX})