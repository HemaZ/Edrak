cmake_minimum_required(VERSION 3.6)
project(Edrak)

# ensure C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

#export compiler flags for code completion engines
set( CMAKE_EXPORT_COMPILE_COMMANDS 1 )

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})

# Set global compiler warnings
if(MSVC)
    add_compile_options(/W3 /WX)
else()
    add_compile_options(-Wall -pedantic)
endif()

# Src files
set(SRC_FILES_IO Edrak/src/IO/MonoReader.cpp
Edrak/src/IO/Utils.cpp
Edrak/src/Images/TemplateMatching.cpp
)

add_library(Edrak ${SRC_FILES_IO})
target_include_directories(Edrak PUBLIC Edrak/include)

enable_testing()

add_executable(mytest tests/MyTests.cpp tests/Images/Test_TemplateMatching.cpp tests/Types/Test_Transformation.cpp)
target_include_directories(Edrak PRIVATE tests)
target_link_libraries(mytest PRIVATE Edrak)
add_test(test1 mytest)

# OpenCV as a required dependency for mylib
find_package(OpenCV REQUIRED)
target_link_libraries(Edrak PRIVATE "${OpenCV_LIBS}")
target_include_directories(Edrak PRIVATE "${OpenCV_INCLUDE_DIRS}")

# Boost 

find_package(Boost 1.40 COMPONENTS program_options REQUIRED)
target_link_libraries(Edrak PRIVATE Boost::program_options)

# Eigen

find_package (Eigen3 3.3 REQUIRED NO_MODULE)
target_link_libraries(Edrak PRIVATE Eigen3::Eigen)

# For Testing purpose only
add_executable(EdrakMain Edrak/src/main.cpp)
target_link_libraries(EdrakMain Edrak)


# Clang tidy as optional static analyzer
option(Template_USE_CLANG_TIDY "Use clang tidy for static analysis" TRUE)
if (Template_USE_CLANG_TIDY)
    find_program(CLANG_TIDY_EXE NAMES "clang-tidy" "clang-tidy-6.0" "clang-tidy-5.0" "clang-tidy-4.0"
                 DOC "Path to clang-tidy executable")
    if(NOT CLANG_TIDY_EXE)
      message(STATUS "clang-tidy not found.")
    else()
      message(STATUS "clang-tidy found: ${CLANG_TIDY_EXE}")
      set_target_properties(Edrak PROPERTIES CXX_CLANG_TIDY "${CLANG_TIDY_EXE}")
    endif()
endif()


# Setup coverage testing for GCC or Clang
option(Template_ENABLE_COVERAGE "Enable coverage reporting for GCC or Clang" TRUE)
if (Template_ENABLE_COVERAGE)
    if (${CMAKE_CXX_COMPILER_ID} STREQUAL "GNU" OR ${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
        message(STATUS "Configuring with coverage")
        target_compile_options(Edrak PUBLIC --coverage -O0)
        target_link_libraries(Edrak PUBLIC --coverage)
    else()
        message(FATAL_ERROR "GCC or Clang required with Template_ENABLE_COVERAGE: found ${CMAKE_CXX_COMPILER_ID}")
    endif()
endif()
