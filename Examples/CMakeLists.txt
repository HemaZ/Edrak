add_subdirectory(BA)

add_executable(KITTI_VO KITTI_VO.cpp)
target_link_libraries(KITTI_VO Edrak)
target_compile_definitions(KITTI_VO PUBLIC EDRAK_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../tests/data/")

add_executable(VO_5PTS_ALGORITHM VO/test_PoseEstmation.cpp)
target_compile_definitions(VO_5PTS_ALGORITHM PUBLIC EDRAK_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../tests/data/")
target_link_libraries(VO_5PTS_ALGORITHM Edrak)