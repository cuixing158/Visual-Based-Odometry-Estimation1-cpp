# set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
aux_source_directory(. CXSparseSrcList)
list(APPEND CXSparseSrcList ./Source/cs_ri_bundle.cpp
./Source/cs_ci_bundle.cpp
./CXSparseSupport/solve_from_lu.cpp
./CXSparseSupport/solve_from_qr.cpp
./CXSparseSupport/makeCXSparseMatrix.cpp
./CXSparseSupport/unpackCXStruct.cpp)

foreach(item ${CXSparseSrcList})
    message(STATUS "CXSparse srcLists is ${item}")
endforeach()

add_library(CXSparse SHARED ${CXSparseSrcList})
# message("子目录为${CMAKE_CURRENT_SOURCE_DIR}/include")
target_include_directories(CXSparse PRIVATE
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/CXSparse/Include
${PROJECT_SOURCE_DIR}/CXSparse/SuiteSparse_config
${PROJECT_SOURCE_DIR}/CXSparse/CXSparseSupport)
# target_link_libraries(ocv PRIVATE ${OpenCV_LIBS})