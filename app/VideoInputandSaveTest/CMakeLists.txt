cmake_minimum_required(VERSION 3.2)

set(NAME_POSTFIX VideoInputAndSave)
project(${CMAKE_PROJECT_NAME}_${NAME_POSTFIX})

set(SOURCE_FILES main.cpp 
				SavetestNode.cpp 
				SavetestNode.h)

add_executable(${PROJECT_NAME} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} PRIVATE 
						DenseReconstruction_common 
						DenseReconstruction_CameraModule
						DenseReconstruction_FeatureExtraction
)
target_link_libraries(${PROJECT_NAME} PRIVATE ${OPENCV_LIBS} Qt${QT_VERSION_MAJOR}::Core Qt${QT_VERSION_MAJOR}::Widgets)
target_link_libraries(${PROJECT_NAME} PRIVATE nlohmann_json::nlohmann_json)
target_link_libraries(${PROJECT_NAME} PRIVATE Sophus::Sophus)


install(TARGETS ${PROJECT_NAME} DESTINATION ${DIVISIBLE_INSTALL_BIN_DIR})