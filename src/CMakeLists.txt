aux_source_directory(. srcLists)

add_library(dbow3 SHARED ${srcLists})

# find_package(OpenCV 4 REQUIRED)
# if(OpenCV_FOUND)
    target_include_directories(dbow3 PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
	${OpenCV_INCLUDE_DIRS})
    target_link_libraries(dbow3 PRIVATE ${OpenCV_LIBS})
# else()
#     message(FATAL_ERROR "Opencv4.x not found...nothing will be built.")
# endif()


