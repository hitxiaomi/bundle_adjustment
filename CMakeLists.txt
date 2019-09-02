cmake_minimum_required(VERSION 2.8)
project(bundle_adjustment)


set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE  "Debug")


set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O0-march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O0 -march=native")

list(APPEND  CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIR})


include_directories(
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/thirdparty/Sophus/
        ${PROJECT_SOURCE_DIR}/src
)

set(LIBRARY_OUTPUT_PATH  "${PROJECT_SOURCE_DIR}/lib")
add_library(${PROJECT_NAME}_backend  SHARED
        src/base_vatex.cpp
        src/base_edge.cpp
        src/optimizer.cpp
        src/vertex_pose.cpp
        src/vertex_point.cpp
        src/edge_reprojection.cpp
        )


link_directories("${PROJECT_SOURCE_DIR}/lib")


set(EXECUTABLE_OUTPUT_PATH  "${PROJECT_SOURCE_DIR}/bin")
add_executable(test_bundle_adjustment app/tese_bundle_adjustment.cpp)
target_link_libraries(test_bundle_adjustment ${PROJECT_NAME}_backend )