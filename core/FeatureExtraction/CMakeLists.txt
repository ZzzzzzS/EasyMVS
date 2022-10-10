cmake_minimum_required(VERSION 3.2)

set(NAME_POSTFIX FeatureExtraction)
project(${CMAKE_PROJECT_NAME}_${NAME_POSTFIX})


set(SOURCE_FILES FeatureExtractor.cpp
                FeatureMatcher.cpp
                VocTreeMatcher.cpp
                FSift128.cpp
                ReflectiveStickerExtractor.cpp
                ReflectiveStickerMatcher.cpp
)

set(HEADER_FILES FeatureExtractor.h
                FeatureMatcher.h
                VocTreeMatcher.h
                FSift128.h
                ReflectiveStickerExtractor.h
                ReflectiveStickerMatcher.h
)

add_library(${PROJECT_NAME} STATIC ${SOURCE_FILES} ${HEADER_FILES})
target_link_libraries(${PROJECT_NAME} PRIVATE ${OpenCV_LIBS} Eigen3::Eigen)
target_link_libraries(${PROJECT_NAME} PRIVATE Qt${QT_VERSION_MAJOR}::Core)
target_link_libraries(${PROJECT_NAME} PRIVATE Sophus::Sophus)
target_link_libraries(${PROJECT_NAME} PRIVATE nlohmann_json::nlohmann_json)
target_link_libraries(${PROJECT_NAME} PRIVATE ${DBoW2_LIBS})


install(TARGETS ${PROJECT_NAME} DESTINATION ${DIVISIBLE_INSTALL_LIB_DIR})
install(FILES ${HEADER_FILES} DESTINATION ${DIVISIBLE_INSTALL_INCLUDE_DIR}/${NAME_POSTFIX})