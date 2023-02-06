aux_source_directory(. constructSrc)
add_subdirectory(ocv)

message(STATUS "subdirectory source:${constructSrc}")
add_library(constructSrc STATIC ${constructSrc})

# add_library(constructSrc SHARED ${constructSrc})
target_include_directories(constructSrc PUBLIC ${OpenCV_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/ocv/include
    ${CMAKE_CURRENT_SOURCE_DIR}/ocv
)

target_link_libraries(constructSrc PUBLIC ocv ${OpenCV_LIBS} -lgomp)
