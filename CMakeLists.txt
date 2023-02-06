# set(CMake_DIR)
cmake_minimum_required(VERSION 3.10)
set(CMAKE_C_COMPILER "/usr/bin/gcc") # "D:/mingw64_v8/bin/x86_64-w64-mingw32-gcc.exe"
set(CMAKE_CXX_COMPILER "/usr/bin/g++") # "D:/mingw64_v8/bin/x86_64-w64-mingw32-g++.exe"

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC") # https://blog.csdn.net/angelazhao620/article/details/123711331
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
project(CONSTRUCTMAP VERSION 1.0)
message(STATUS "Use CMAKE version:${CMAKE_VERSION}")
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR})

set(OpenCV_DIR "/home/cuixingxing/opencv4_6_0/lib/cmake/opencv4")
find_package(OpenCV REQUIRED)

if(OpenCV_FOUND)
    message(STATUS "OpenCV" ${OpenCV_VERSION} Found.)
else()
    message(FATAL_ERROR "Opencv not found...nothing will be built.")
endif()

aux_source_directory(${PROJECT_SOURCE_DIR} srcLists)
add_subdirectory(constructSrc)

message(STATUS ${srcLists})

# add_library(CONSTRUCTMAP MODULE ${srcLists})
add_executable(main ${srcLists})
target_include_directories(main PUBLIC ${OpenCV_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/constructSrc
    ${PROJECT_SOURCE_DIR}/constructSrc/ocv/include
    ${PROJECT_SOURCE_DIR}/constructSrc/ocv
)
target_link_libraries(main ${OpenCV_LIBS} constructSrc ocv)
target_compile_features(main INTERFACE cxx_std_11)

set(CMAKE_INSTALL_PREFIX ${PROJECT_SOURCE_DIR})
install(TARGETS ocv constructSrc main
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib)