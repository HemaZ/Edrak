add_executable(BAExample BA_main.cpp common.cpp)
target_link_libraries(BAExample  ${CONAN_LIBS})
target_compile_definitions(BAExample PUBLIC EDRAK_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/")
