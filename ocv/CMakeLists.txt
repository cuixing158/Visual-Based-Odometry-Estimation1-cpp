# set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
aux_source_directory(. srcLists)
add_library(ocv SHARED ${srcLists})
target_include_directories(ocv PUBLIC ${PROJECT_SOURCE_DIR})

# add_library(ocv SHARED ${srcLists})
# message("子目录为${CMAKE_CURRENT_SOURCE_DIR}/include")
# find_package(OpenCV 4 REQUIRED)
# if(OpenCV_FOUND)
    target_include_directories(ocv PUBLIC
    ${PROJECT_SOURCE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
    target_link_libraries(ocv PRIVATE ${OpenCV_LIBS})
# else()
#     message(FATAL_ERROR "Opencv4.x not found...nothing will be built.")
# endif()
    

