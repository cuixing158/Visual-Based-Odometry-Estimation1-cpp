# set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
aux_source_directory(. srcLists)
add_library(ocv SHARED ${srcLists})
target_include_directories(ocv PUBLIC
    ${PROJECT_SOURCE_DIR}/mapSrc
    ${OpenCV_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
    target_link_libraries(ocv PRIVATE ${OpenCV_LIBS})

    

