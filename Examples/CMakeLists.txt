add_subdirectory(BA)

add_executable(KITTI_VO KITTI_VO.cpp)
target_link_libraries(KITTI_VO Edrak)
target_compile_definitions(KITTI_VO PUBLIC EDRAK_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../tests/data/")
