//
// Created by collab on 04.06.24.
//

#ifndef LODUHOVA_BP_KINECT_CONTROL_HPP
#define LODUHOVA_BP_KINECT_CONTROL_HPP

#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
#include <memory>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <vector>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cmath>
#include <utility>
#include <string>
#include <tf2_eigen/tf2_eigen.hpp>  // For Eigen conversions
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>  // For geometry_msgs conversions
#include <shape_msgs/msg/solid_primitive.hpp>
#include <moveit_msgs/msg/object_color.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <rviz_visual_tools/rviz_visual_tools.hpp>
#include <gripper_srv/srv/gripper_service.hpp>
// #include <rviz_visual_tools.h>
#include <std_msgs/msg/color_rgba.hpp>
#include <Eigen/Geometry>
#include <unistd.h>
#include "rclcpp/rclcpp.hpp"
#include <future>
#include <functional>
#include "rcl_interfaces/msg/log.hpp"

#include <thread>
#include <geometry_msgs/msg/vector3.hpp>
//transform includes
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/point_stamped.hpp"
#include <mutex>


#endif //LODUHOVA_BP_KINECT_CONTROL_HPP
