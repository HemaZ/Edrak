cmake_minimum_required(VERSION 3.6)
project(Edrak)

# ensure C++17
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

#export compiler flags for code completion engines
set( CMAKE_EXPORT_COMPILE_COMMANDS 1 )

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})

# Set global compiler warnings
if(MSVC)
    add_compile_options(/W3 /WX)
else()
    add_compile_options()
endif()

# OpenCV as a required dependency for Edrak
find_package(OpenCV REQUIRED)
include_directories("${OpenCV_INCLUDE_DIRS}")
# Boost 
find_package(Boost 1.40 COMPONENTS program_options REQUIRED)
# Eigen
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

# Find Pangolin (https://github.com/stevenlovegrove/Pangolin)
execute_process(COMMAND "./3rdparty/Pangolin/scripts/install_prerequisites.sh recommended")
set(BUILD_EXAMPLES OFF)
set(BUILD_TESTS OFF)
add_subdirectory(3rdparty/Pangolin/)
find_package(Pangolin 0.4 REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

# Ceres 
find_package(Ceres REQUIRED)

#fmt 
add_subdirectory(3rdparty/fmt/)
include_directories(3rdparty/fmt/include)


# Sophus
set(USE_BASIC_LOGGING ON)
set(BUILD_SOPHUS_TESTS OFF)
set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)
add_subdirectory(3rdparty/Sophus/)
include_directories(3rdparty/Sophus)

# G2o
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/3rdparty/g2o/cmake_modules)
find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIR})




# Src files
set(EDRAK_SRC_FILES Edrak/src/IO/MonoReader.cpp
Edrak/src/IO/Utils.cpp
Edrak/src/Images/TemplateMatching.cpp
Edrak/src/Images/Calibration.cpp
Edrak/src/Visual/3D.cpp
Edrak/src/Evaluation/Trajectory.cpp
Edrak/src/IO/Trajectory.cpp
Edrak/src/Images/RGBD.cpp
Edrak/src/Images/Features.cpp
Edrak/src/VO/2D/PoseEstmation.cpp
)

add_library(Edrak ${EDRAK_SRC_FILES})
target_include_directories(Edrak PUBLIC Edrak/include)
target_link_libraries(Edrak PRIVATE "${OpenCV_LIBS}")
target_link_libraries(Edrak PRIVATE Boost::program_options)
target_link_libraries(Edrak PRIVATE Eigen3::Eigen)
target_link_libraries(Edrak PRIVATE ${Pangolin_LIBRARIES})
target_link_libraries(Edrak PRIVATE ceres)



# For Testing purpose only
add_executable(EdrakMain Edrak/src/main.cpp)
target_link_libraries(EdrakMain Edrak Eigen3::Eigen ${FMT_LIBRARIES})

enable_testing()

set(TESTS_SRCS tests/MyTests.cpp 
tests/Images/Test_TemplateMatching.cpp 
tests/Types/Test_Transformation.cpp 
tests/Visual/test_3D.cpp
tests/IO/test_trajectory.cpp
tests/Images/Test_Calibration.cpp
tests/Images/Test_RGBD.cpp
# tests/3rdparty/ceresCurveFitting.cpp
# tests/3rdparty/g2oCurveFitting.cpp
tests/3rdparty/opencvOrb.cpp
tests/Images/Test_Features.cpp
tests/VO/test_PoseEstmation.cpp
)

add_executable(mytest ${TESTS_SRCS})
target_include_directories(Edrak PRIVATE tests )
target_compile_definitions(mytest PUBLIC EDRAK_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/tests/data/")
target_link_libraries(mytest PRIVATE Edrak "${OpenCV_LIBS}" Eigen3::Eigen ${FMT_LIBRARIES} ${CERES_LIBRARIES} ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY})
add_test(test1 mytest)


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
