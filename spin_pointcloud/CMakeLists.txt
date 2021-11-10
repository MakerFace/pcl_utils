cmake_minimum_required(VERSION 3.14)
project(spinPointcloud)

find_package(PCL 1.8 REQUIRED COMPONENTS common io filters visualization  segmentation sample_consensus)
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

include_directories(
    ${CMAKE_SOURCE_DIR}
    ${PCL_INCLUDE_DIRS}
)

aux_source_directory(${CMAKE_SOURCE_DIR} src_dir)
add_executable(${PROJECT_NAME} ${src_dir})

target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})