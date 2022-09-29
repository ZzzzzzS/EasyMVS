cmake_minimum_required(VERSION 3.5)

set(NAME_POSTFIX PictureMVS)
project(${CMAKE_PROJECT_NAME}_${NAME_POSTFIX})

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(PROJECT_SOURCES
	main.cpp
)

add_executable(${PROJECT_NAME} ${PROJECT_SOURCES})
target_link_libraries(${PROJECT_NAME} PRIVATE 
						DenseReconstruction_common 
						DenseReconstruction_CameraModule
						DenseReconstruction_DenseReconstruction
						DenseReconstruction_Visualization
)

target_link_libraries(${PROJECT_NAME} PRIVATE 
						Qt${QT_VERSION_MAJOR}::Core 
						Qt${QT_VERSION_MAJOR}::Widgets
)
						
target_link_libraries(${PROJECT_NAME} PRIVATE ${OPENCV_LIBS})
target_link_libraries(${PROJECT_NAME} PRIVATE nlohmann_json::nlohmann_json)
target_link_libraries(${PROJECT_NAME} PRIVATE Sophus::Sophus)
target_link_libraries(${PROJECT_NAME} PRIVATE ${PCL_LIBRARIES})

install(TARGETS ${PROJECT_NAME} DESTINATION ${DIVISIBLE_INSTALL_BIN_DIR})