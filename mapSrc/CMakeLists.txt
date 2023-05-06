aux_source_directory(. srcLists)
foreach(item ${srcLists})
    message(STATUS "mapSrc is ${item}")
endforeach()

add_library(mapSrc SHARED ${srcLists})
set_target_properties(mapSrc PROPERTIES
    POSITION_INDEPENDENT_CODE ON)
target_compile_features(mapSrc INTERFACE cxx_std_11)
target_compile_definitions(mapSrc PRIVATE -DMODEL=mapSrc
    -D_CRT_SECURE_NO_WARNINGS)
# target_compile_options(mapSrc PRIVATE /wd4251
#     -EHs)

target_include_directories(mapSrc PRIVATE 
${OpenCV_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/CXSparse/Include
    ${PROJECT_SOURCE_DIR}/CXSparse/SuiteSparse_config
    ${PROJECT_SOURCE_DIR}/CXSparse/CXSparseSupport

    ${PROJECT_SOURCE_DIR}/mapSrc
    ${PROJECT_SOURCE_DIR}/ocv/include
    ${PROJECT_SOURCE_DIR}/ocv

    ${PROJECT_SOURCE_DIR}/src
)
target_link_libraries(mapSrc  PRIVATE ${OpenCV_LIBS} ocv dbow3 CXSparse gomp)


find_library(FOUND_LIBM m)
if(FOUND_LIBM)
    target_link_libraries(mapSrc INTERFACE m)
endif()
find_library(FOUND_LIBSTDCXX stdc++)
if(FOUND_LIBSTDCXX)
    target_link_libraries(mapSrc INTERFACE stdc++)
endif()
find_package(OpenMP REQUIRED)
if(OpenMP_FOUND)
    target_link_libraries(mapSrc INTERFACE OpenMP::OpenMP_C)
    target_link_libraries(mapSrc INTERFACE OpenMP::OpenMP_CXX)
endif()

file(GLOB_RECURSE headerFiles "./*.h")
install(FILES ${headerFiles} DESTINATION include)


