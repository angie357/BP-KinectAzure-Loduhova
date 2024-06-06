cmake_minimum_required(VERSION 3.8)
project(kinect_control)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

find_package(visualization_msgs REQUIRED)
find_package(rviz2 REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(rviz_visual_tools REQUIRED)
find_package(gripper_srv REQUIRED)

# Include C++ headers
include_directories(include)


#Add the executable
add_executable(main_node
        src/kinect_control.cpp
        src/helper_function.cpp
)

target_link_libraries(main_node
        k4a
        k4abt
)

ament_target_dependencies(main_node
        rclcpp
        moveit_core
        "moveit_ros_planning_interface"
        "rclcpp"
        visualization_msgs
        geometry_msgs
        tf2
        tf2_ros
        tf2_eigen
        tf2_geometry_msgs
        "moveit_visual_tools"
        gripper_srv
        rviz_visual_tools
)

target_include_directories(main_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_compile_features(main_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS
        main_node
        DESTINATION lib/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY
        launch
        DESTINATION share/${PROJECT_NAME}/
)

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # comment the line when a copyright and license is added to all source files
    set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # comment the line when this package is in a git repo and when
    # a copyright and license is added to all source files
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif ()

ament_package()
