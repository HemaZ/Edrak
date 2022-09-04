cmake_minimum_required(VERSION 3.6)
project(Edrak)

# ensure C++20
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

include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
conan_basic_setup()


# Find Pangolin (https://github.com/stevenlovegrove/Pangolin)
# execute_process(COMMAND "./3rdparty/Pangolin/scripts/install_prerequisites.sh recommended")
# set(BUILD_EXAMPLES OFF)
# set(BUILD_TESTS OFF)
# add_subdirectory(3rdparty/Pangolin/)
find_package(Pangolin 0.4 REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

# G2o
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/3rdparty/g2o/cmake_modules)
find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIR})


set(IMGGUI_PATH 3rdparty/imgui)
include_directories( ${IMGGUI_PATH})
include_directories( ${IMGGUI_PATH}/backends)

set(IMGUI_SRCS ${IMGGUI_PATH}/imgui.cpp
${IMGGUI_PATH}/imgui_demo.cpp
${IMGGUI_PATH}/imgui_draw.cpp
${IMGGUI_PATH}/imgui_tables.cpp
${IMGGUI_PATH}/imgui_widgets.cpp
${IMGGUI_PATH}/backends/imgui_impl_glfw.cpp
${IMGGUI_PATH}/backends/imgui_impl_opengl3.cpp
)

add_library(ImgGui ${IMGUI_SRCS})
target_link_libraries(ImgGui ${CONAN_LIBS})


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
Edrak/src/Images/Frame.cpp
Edrak/src/VO/2D/PoseEstmation.cpp
Edrak/src/3D/ICP.cpp
Edrak/src/3D/Landmark.cpp
Edrak/src/SLAM/Map.cpp
)

add_library(Edrak ${EDRAK_SRC_FILES})
target_include_directories(Edrak PUBLIC Edrak/include)
target_link_libraries(Edrak ${CONAN_LIBS})
target_link_libraries(Edrak ${Pangolin_LIBRARIES})
target_link_libraries(Edrak ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY})



# For Testing purpose only
add_executable(EdrakMain Edrak/src/main.cpp)
target_link_libraries(EdrakMain Edrak)

add_executable(ImgGui_Test tests/Visual/Test_GUI.cpp)
target_link_libraries(ImgGui_Test ImgGui Edrak)

# Build Examples
set(EDRAK_BUILD_EXAMPLES ON CACHE BOOL "Build Edrak examples")
if(EDRAK_BUILD_EXAMPLES)
  message(STATUS "Compiling Edrak examples")
  add_subdirectory(Examples/ Examples)
endif(EDRAK_BUILD_EXAMPLES)


enable_testing()

set(TESTS_SRCS tests/MyTests.cpp 
tests/Images/Test_TemplateMatching.cpp 
tests/Types/Test_Transformation.cpp 
tests/Visual/test_3D.cpp
tests/IO/test_trajectory.cpp
tests/Images/Test_Calibration.cpp
# tests/Images/Test_RGBD.cpp
tests/Images/Test_Frame.cpp
tests/3rdparty/ceresCurveFitting.cpp
tests/3rdparty/g2oCurveFitting.cpp
tests/3rdparty/opencvOrb.cpp
tests/Images/Test_Features.cpp
tests/VO/test_PoseEstmation.cpp
tests/3D/Test_utils.cpp
tests/3D/Test_ICP.cpp
tests/3D/Test_Landmark.cpp
)

add_executable(mytest ${TESTS_SRCS})
target_include_directories(Edrak PRIVATE tests )
target_compile_definitions(mytest PUBLIC EDRAK_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/tests/data/")
target_link_libraries(mytest PRIVATE Edrak)
add_test(test1 mytest)
