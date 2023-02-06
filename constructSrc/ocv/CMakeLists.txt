# set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
aux_source_directory(. srcLists)
add_library(ocv STATIC ${srcLists})

# add_library(ocv SHARED ${srcLists})
message("子目录为${CMAKE_CURRENT_SOURCE_DIR}/include")
target_include_directories(ocv PUBLIC
    ${OpenCV_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/constructSrc)
target_link_libraries(ocv PRIVATE ${OpenCV_LIBS})