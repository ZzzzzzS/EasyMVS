cmake_minimum_required(VERSION 3.9)

project(doc_doxygen)

find_package(Doxygen)

if (DOXYGEN_FOUND)
    message("Doxygen build started")

    set(DOXYGEN_PROJECT_NAME "EasyMVS")
    set(DOXYGEN_PROJECT_BRIEF "simple 3D reconstruction framework")
    set(DOXYGEN_IMAGE_PATH ${CMAKE_CURRENT_SOURCE_DIR})
    set(DOXYGEN_HTML_EXTRA_STYLESHEET ${CMAKE_CURRENT_SOURCE_DIR}/stylesheet.css)
    if(BUILD_DOC)
        doxygen_add_docs(
        ${PROJECT_NAME}
        ${CMAKE_SOURCE_DIR}/core
        ${CMAKE_CURRENT_SOURCE_DIR}
        ../README.md
        )
    endif(BUILD_DOC)

else (DOXYGEN_FOUND)
  message("Doxygen need to be installed to generate the doxygen documentation")
endif (DOXYGEN_FOUND)


if (DOXYGEN_FOUND)
    if(BUILD_DOC)
	    install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/html DESTINATION ${DIVISIBLE_INSTALL_DOC_DIR})
	endif(BUILD_DOC)
endif (DOXYGEN_FOUND)