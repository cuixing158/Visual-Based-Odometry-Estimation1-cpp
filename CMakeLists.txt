# set(CMake_DIR)
cmake_minimum_required(VERSION 3.10)
#set(CMAKE_C_COMPILER "D:/mingw64_v8/bin/x86_64-w64-mingw32-gcc.exe") # "D:/mingw64_v8/bin/x86_64-w64-mingw32-gcc.exe"
#set(CMAKE_CXX_COMPILER "D:/mingw64_v8/bin/x86_64-w64-mingw32-g++.exe") # "D:/mingw64_v8/bin/x86_64-w64-mingw32-g++.exe"

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC") # https://blog.csdn.net/angelazhao620/article/details/123711331
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
project(main VERSION 0.1.2)
message(STATUS "Use CMAKE version:${CMAKE_VERSION}")

# set(OpenCV_DIR "D:/opencv_4_4_0/opencv/build") 
# set(OpenCV_DIR "/home/cuixingxing/opencv4_6_0/lib/cmake/opencv4")
find_package(OpenCV 4 REQUIRED)
if(NOT OpenCV_FOUND)
    message(FATAL_ERROR "Opencv4.x not found...nothing will be built.")
endif()

add_subdirectory(CXSparse)
add_subdirectory(ocv)
add_subdirectory(src)
add_subdirectory(mapSrc)

add_executable(main main.cpp orbSlam.cpp)
target_compile_features(main INTERFACE cxx_std_11)
target_include_directories(main PRIVATE ${OpenCV_INCLUDE_DIRS} mapSrc)
target_link_libraries(main PRIVATE ${OpenCV_LIBS} mapSrc)

# install
set(CMAKE_INSTALL_PREFIX ${PROJECT_SOURCE_DIR}/install)
set(EXECUTABLE_OUTPUT_PATH  ${PROJECT_SOURCE_DIR})
install(TARGETS  mapSrc ocv CXSparse dbow3
    EXPORT mapLibTargets 
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib)


