cmake_minimum_required(VERSION 3.2)

set(NAME_POSTFIX common)
project(${CMAKE_PROJECT_NAME}_${NAME_POSTFIX})

set(SOURCE_FILES CameraObject.cpp
                ConfigurationObject.cpp
                DataFlowObject.cpp
                DenseMapObject.cpp
                FrameObject.cpp
                MapPointObject.cpp
                MVSObject.cpp
                PoseObject.cpp
                WorkFlowObject.cpp)

set(HEADER_FILES CameraObject.h
                ConfigurationObject.h
                DataFlowObject.h
                DenseMapObject.h
                FrameObject.h
                MapPointObject.h
                MVSObject.h
                PoseObject.h
                WorkFlowObject.h
				MVSConfig.h
)


add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES} ${HEADER_FILES})
target_link_libraries(${PROJECT_NAME} PRIVATE ${OpenCV_LIBS} Eigen3::Eigen)
target_link_libraries(${PROJECT_NAME} PRIVATE Qt${QT_VERSION_MAJOR}::Core)
target_link_libraries(${PROJECT_NAME} PRIVATE Sophus::Sophus)
target_link_libraries(${PROJECT_NAME} PRIVATE nlohmann_json::nlohmann_json)

install(TARGETS ${PROJECT_NAME} DESTINATION ${DIVISIBLE_INSTALL_LIB_DIR})
install(FILES ${HEADER_FILES} DESTINATION ${DIVISIBLE_INSTALL_INCLUDE_DIR}/${NAME_POSTFIX})