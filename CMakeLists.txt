# set(CMake_DIR)
cmake_minimum_required(VERSION 3.10)
#set(CMAKE_C_COMPILER "D:/mingw64_v8/bin/x86_64-w64-mingw32-gcc.exe") # "D:/mingw64_v8/bin/x86_64-w64-mingw32-gcc.exe"
#set(CMAKE_CXX_COMPILER "D:/mingw64_v8/bin/x86_64-w64-mingw32-g++.exe") # "D:/mingw64_v8/bin/x86_64-w64-mingw32-g++.exe"

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC") # https://blog.csdn.net/angelazhao620/article/details/123711331
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
project(constructWorldMap VERSION 0.1.2)
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

aux_source_directory(${PROJECT_SOURCE_DIR} srcLists)
foreach(item ${srcLists})
    message(STATUS "srcLists is ${item}")
endforeach()

# add_library(CONSTRUCTMAP MODULE ${srcLists})
add_executable(constructWorldMap ${srcLists})

set_target_properties(constructWorldMap PROPERTIES
    PREFIX ""
    POSITION_INDEPENDENT_CODE ON)
target_compile_features(constructWorldMap PUBLIC cxx_std_11)
target_compile_definitions(constructWorldMap PRIVATE -DMODEL=constructWorldMap
    -D_CRT_SECURE_NO_WARNINGS)
# target_compile_options(constructWorldMap PRIVATE /wd4251
#     -EHs)

target_include_directories(constructWorldMap PUBLIC 
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/ocv/include
    ${PROJECT_SOURCE_DIR}/ocv

    ${PROJECT_SOURCE_DIR}/src

    ${PROJECT_SOURCE_DIR}/CXSparse/Include
    ${PROJECT_SOURCE_DIR}/CXSparse/SuiteSparse_config
    ${PROJECT_SOURCE_DIR}/CXSparse/CXSparseSupport
)
target_compile_features(constructWorldMap INTERFACE cxx_std_11)
target_include_directories(constructWorldMap PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(constructWorldMap  PUBLIC ${OpenCV_LIBS} ocv dbow3 CXSparse gomp)


find_library(FOUND_LIBM m)
if(FOUND_LIBM)
    target_link_libraries(constructWorldMap INTERFACE m)
endif()
find_library(FOUND_LIBSTDCXX stdc++)
if(FOUND_LIBSTDCXX)
    target_link_libraries(constructWorldMap INTERFACE stdc++)
endif()
find_package(OpenMP REQUIRED)
if(OpenMP_FOUND)
    target_link_libraries(constructWorldMap INTERFACE OpenMP::OpenMP_C)
    target_link_libraries(constructWorldMap INTERFACE OpenMP::OpenMP_CXX)
endif()

set(CMAKE_INSTALL_PREFIX ${PROJECT_SOURCE_DIR})
set(EXECUTABLE_OUTPUT_PATH  ${PROJECT_SOURCE_DIR})
install(TARGETS  constructWorldMap ocv CXSparse dbow3
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib)
