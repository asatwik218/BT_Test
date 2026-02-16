cmake_minimum_required(VERSION 3.15)
project(behaviour_trees_exploration LANGUAGES CXX)

# Set C++ standard (BehaviorTree.CPP requires at least C++17)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find packages (Conan will provide these)
find_package(BehaviorTree REQUIRED)
find_package(flexiv_rdk REQUIRED)
find_package(fmt REQUIRED)
find_package(spdlog REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(cynlr_gripper REQUIRED)
find_package(cynlr_camera REQUIRED)

# Motor and AMR interfaces use runtime DLL loading (LoadLibrary/GetProcAddress)
# No .lib or header paths needed — struct definitions are local in the node headers

find_package(OpenCV REQUIRED)

# Define your executable
add_executable(${PROJECT_NAME}
    src/main.cpp
    # Sync nodes
    src/nodes/flexiv/flexiv_nodes.cpp
    src/nodes/gripper/gripper_nodes.cpp
    src/nodes/camera/camera_nodes.cpp
    src/nodes/motor/motor_nodes.cpp
    src/nodes/amr/amr_nodes_new.cpp
    src/nodes/vision/vision_nodes.cpp
    # Monitoring nodes (fault detection)
    src/nodes/monitoring/monitoring_nodes.cpp
)

# Include paths: src/ for node headers
target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# Link the libraries
target_link_libraries(${PROJECT_NAME} PUBLIC
    BT::behaviortree_cpp
    flexiv::rdk
    fmt::fmt
    spdlog::spdlog
    Eigen3::Eigen
    cynlr_gripper::cynlr_gripper
    cynlr::camera
    opencv::opencv
)

# Link Windows system library and set stack reserve on Windows
if(WIN32)
    target_link_libraries(${PROJECT_NAME} PRIVATE winmm)
    # Set stack reserve to 8 MiB (required by Flexiv RDK)
    target_link_options(${PROJECT_NAME} PRIVATE "/STACK:8388608")
endif()

# Standalone camera test executable
add_executable(test_camera src/test_camera.cpp)
target_link_libraries(test_camera PRIVATE
    cynlr::camera
    opencv::opencv
)

# Stereo camera live view test
add_executable(test_stereo_view src/test_stereo_view.cpp)
target_link_libraries(test_stereo_view PRIVATE
    cynlr::camera
    opencv::opencv
)
