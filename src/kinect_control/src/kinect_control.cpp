#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "k4abt.h"
#include "k4a/k4a.h"
#include "iostream"
#include "chrono"
#include "kinect_control/kinect_control.hpp"
#include <fstream>
#include <memory>
#include <vector>
#include <unistd.h>
#include <fstream>
#include "kinect_control/helper_function.h"

using namespace std::chrono_literals;

#define LH_PALM 8
#define RH_PALM 15
#define LH_ELBOW 6
#define RH_ELBOW 13
#define L_SHOULDER 5
#define R_SHOULDER 12
#define CHEST 2
#define DEVIATION_GEST 0.05
#define DEVIATION_CHEST 0.4


#define VERIFY(result, error)                                                                                          \
    if (result != K4A_RESULT_SUCCEEDED)                                                                                 \
    {                                                                                                                  \
        printf("%s \n - (File: %s, Function: %s, Line: %d)\n", error, __FILE__, __FUNCTION__, __LINE__);                                                                                         \
        exit(1);                                                                                                       \
    }

using namespace std::chrono_literals;

class KinectControl : public rclcpp::Node {
public:
    KinectControl() : Node("kinect_control") {
        tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
    }

    void initMoveGroup(std::string gesture);
    void move(geometry_msgs::msg::Pose targetPose);
    void home_position();
    void down_position();

    const std::string PLANNING_GROUP = "ur_manipulator";
    const std::string END_EFFECTOR_LINK = "tool0";
    const std::string POSE_REFERENCE_FRAME = "base_link";
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface{};

private:
    geometry_msgs::msg::Pose targetPose{};
    std::shared_ptr<tf2_ros::Buffer> tf_buffer{};
    std::shared_ptr<tf2_ros::TransformListener> tf_listener{};
};

void KinectControl::move(geometry_msgs::msg::Pose targetPose) {
    // Set a target Pose
    using moveit::planning_interface::MoveGroupInterface;
    auto move_group_interface = MoveGroupInterface(shared_from_this(), PLANNING_GROUP);
    move_group_interface.setPoseTarget(targetPose);

    // Create a plan to that target pose
    auto const [success, plan] = [&move_group_interface] {
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(move_group_interface.plan(msg));
        return std::make_pair(ok, msg);
    }();

    RCLCPP_INFO(this->get_logger(), "\n\nMoveit status %s\n\n", success ? "true" : "false");

    // Execute the plan
    if (success) {
        RCLCPP_INFO(this->get_logger(), "\n\nExecute command sent!\n\n");
        move_group_interface.execute(plan);
    } else {
        RCLCPP_ERROR(this->get_logger(), "\n\nPlanning failed!\n\n");
    }
}

void KinectControl::home_position() {
    geometry_msgs::msg::Pose pose;
    pose.orientation.x = -0.0028119066264480352;
    pose.orientation.y = 0.9999957084655762;
    pose.orientation.z = -0.0007648332393728197;
    pose.orientation.w = -0.00023792324645910412;
    pose.position.x = 1.18;
    pose.position.y = 0.2;

    pose.position.z = 0.5;
    move(pose);
}

void KinectControl::down_position() {
    geometry_msgs::msg::Pose pose;
    pose.orientation.x = -0.0028119066264480352;
    pose.orientation.y = 0.9999957084655762;
    pose.orientation.z = -0.0007648332393728197;
    pose.orientation.w = -0.00023792324645910412;
    pose.position.x = 1.18;
    pose.position.y = 0.2;
    pose.position.z = 0.18;
    move(pose);
}


void KinectControl::initMoveGroup(std::string gesture) {
    using moveit::planning_interface::MoveGroupInterface;
    move_group_interface = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(),
                                                                                            PLANNING_GROUP);
    move_group_interface->setEndEffectorLink(END_EFFECTOR_LINK);
    move_group_interface->setPoseReferenceFrame(POSE_REFERENCE_FRAME);
    move_group_interface->setMaxVelocityScalingFactor(0.8);
    move_group_interface->setMaxAccelerationScalingFactor(0.04);
    move_group_interface->startStateMonitor();
    geometry_msgs::msg::Pose pose;

    //  Home position
    if (gesture == "home") {
        home_position();
    }
        // Down position
    else if (gesture == "down") {
        down_position();
    }
    return;
}


bool gesture_check(std::vector<double> *dist_palm_elbow, std::vector<double> *dist_elbow_shoulder,
                   std::vector<double> *dist_palm_shoulder, std::vector<double> *dist_chest_elbow,
                   std::vector<double> *dist_palm_elbowR, std::vector<double> *dist_elbow_shoulderR,
                   std::vector<double> *dist_palm_shoulderR, std::vector<double> *dist_chest_elbowR,
                   std::string *gesture) {

    float aa_left = dist_palm_elbow->at(0) * dist_palm_elbow->at(0);
    float bb_left = dist_elbow_shoulder->at(0) * dist_elbow_shoulder->at(0);
    float cc_left = dist_palm_shoulder->at(0) * dist_palm_shoulder->at(0);

    float aa_right = dist_palm_elbowR->at(0) * dist_palm_elbowR->at(0);
    float bb_right = dist_elbow_shoulderR->at(0) * dist_elbow_shoulderR->at(0);
    float cc_right = dist_palm_shoulderR->at(0) * dist_palm_shoulderR->at(0);

    // Elbows are not in the same line with the chest
    if (dist_chest_elbow->at(0) > DEVIATION_CHEST && dist_chest_elbowR->at(0) > DEVIATION_CHEST) {
        gesture->assign("NGR");   // No gesture recognized
        return false;
    } else if (dist_chest_elbow->at(0) > DEVIATION_CHEST || dist_chest_elbowR->at(0) > DEVIATION_CHEST) {
        // Left hand
        if (cc_left - DEVIATION_GEST < aa_left + bb_left && aa_left + bb_left < cc_left + DEVIATION_GEST) {
            gesture->assign("home");
            return true;
        }
            // Right hand
        else if (cc_right - DEVIATION_GEST < aa_right + bb_right && aa_right + bb_right < cc_right + DEVIATION_GEST) {
            gesture->assign("down");
            return true;
        } else {
            gesture->assign("NGR");   // No gesture recognized
            return false;
        }
    } else {
        gesture->assign("NGR");   // No gesture recognized
        return false;
    }
}


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    printf("kinect_control_cpp init ros2\n\n\n");
    auto kinect_control = std::make_shared<KinectControl>();
    printf("kinect_control_cpp init KinectControl\n\n\n");

    auto const logger = rclcpp::get_logger("kinect_control");

    std::ofstream myfile("distances.txt");
    std::string gesture;



    // Kinect device initialization ----------------------------
    k4a_device_t device = NULL;
    VERIFY(k4a_device_open(0, &device), "Open K4A Device failed!");

    // Start camera. Make sure depth camera is enabled.
    k4a_device_configuration_t deviceConfig = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    deviceConfig.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
    deviceConfig.color_resolution = K4A_COLOR_RESOLUTION_OFF;
    VERIFY(k4a_device_start_cameras(device, &deviceConfig), "Start K4A cameras failed!");

    k4a_calibration_t sensor_calibration;
    VERIFY(k4a_device_get_calibration(device, deviceConfig.depth_mode, deviceConfig.color_resolution,
                                      &sensor_calibration),
           "Get depth camera calibration failed!");

    k4abt_tracker_t tracker = NULL;
    k4abt_tracker_configuration_t tracker_config = K4ABT_TRACKER_CONFIG_DEFAULT;
    VERIFY(k4abt_tracker_create(&sensor_calibration, tracker_config, &tracker), "Body tracker initialization failed!");

    int frame_count = 0;
    do {
        k4a_capture_t sensor_capture;
        k4a_wait_result_t get_capture_result = k4a_device_get_capture(device, &sensor_capture, K4A_WAIT_INFINITE);
        if (get_capture_result == K4A_WAIT_RESULT_SUCCEEDED) {
            frame_count++;
            k4a_wait_result_t queue_capture_result = k4abt_tracker_enqueue_capture(tracker, sensor_capture,
                                                                                   K4A_WAIT_INFINITE);
            k4a_capture_release(sensor_capture); // Remember to release the sensor capture once you finish using it
            if (queue_capture_result == K4A_WAIT_RESULT_TIMEOUT) {
                // It should never hit timeout when K4A_WAIT_INFINITE is set.
                printf("Error! Add capture to tracker process queue timeout!\n");
                break;
            } else if (queue_capture_result == K4A_WAIT_RESULT_FAILED) {
                printf("Error! Add capture to tracker process queue failed!\n");
                break;
            }

            k4abt_frame_t body_frame = NULL;
            k4a_wait_result_t pop_frame_result = k4abt_tracker_pop_result(tracker, &body_frame, K4A_WAIT_INFINITE);
            if (pop_frame_result == K4A_WAIT_RESULT_SUCCEEDED) {
                // Successfully popped the body tracking result. Start your processing

                size_t num_bodies = k4abt_frame_get_num_bodies(body_frame);
                //printf("%zu bodies are detected!\n", num_bodies);




                if (num_bodies == 1) {
                    k4abt_body_t body;
                    gesture = "";
                    VERIFY(k4abt_frame_get_body_skeleton(body_frame, 0, &body.skeleton),
                           "Get body skeleton failed!");
//                      RCLCPP_INFO(logger, "\n x= %f, y= %f, z= %f \n",
//                           body.skeleton.joints[LH_PALM].position.v[0],
//                           body.skeleton.joints[LH_PALM].position.v[1],
//                           body.skeleton.joints[LH_PALM].position.v[2]);

                    std::vector<double> dist_palm_elbow, dist_elbow_shoulder, dist_palm_shoulder, dist_chest_elbow,
                            dist_palm_elbowR, dist_elbow_shoulderR, dist_palm_shoulderR, dist_chest_elbowR;

                    count_dist(body.skeleton.joints[LH_PALM].position, body.skeleton.joints[LH_ELBOW].position,
                               body.skeleton.joints[L_SHOULDER].position, body.skeleton.joints[CHEST].position,
                               body.skeleton.joints[RH_PALM].position, body.skeleton.joints[RH_ELBOW].position,
                               body.skeleton.joints[R_SHOULDER].position,
                               &dist_palm_elbow, &dist_elbow_shoulder, &dist_palm_shoulder, &dist_chest_elbow,
                               &dist_palm_elbowR, &dist_elbow_shoulderR, &dist_palm_shoulderR, &dist_chest_elbowR);

//                    RCLCPP_INFO(logger,
//                                "\npalm-elbow = %f, elbow-shoulder = %f, palm-shoulder = %f, chest-elbow = %f\n",
//                                dist_palm_elbow[0], dist_elbow_shoulder[0],
//                                dist_palm_shoulder[0], dist_chest_elbow[0]);


                    myfile << "palm-elbow = " << dist_palm_elbow[0] << " elbow-shoulder = "
                           << dist_elbow_shoulder[0] << " palm-shoulder = " << dist_palm_shoulder[0]
                           << " chest-shoulder = " << dist_chest_elbow[0] << "\n";


                    bool gesture_recognized = gesture_check(&dist_palm_elbow, &dist_elbow_shoulder, &dist_palm_shoulder,
                                                            &dist_chest_elbow, &dist_palm_elbowR, &dist_elbow_shoulderR,
                                                            &dist_palm_shoulderR, &dist_chest_elbowR, &gesture);
                    if (gesture_recognized) {
                        RCLCPP_INFO(logger, "\nGesture recognized !!\n\n");
                        myfile << "Gesture recognized !!\n\n";
//                        if (gesture != "NGR") {
//                            kinect_control->initMoveGroup(gesture);
//                        }
                    } else {
                        RCLCPP_INFO(logger, "\nGesture not recognized !!\n\n");
                        myfile << "Gesture not recognized !!\n\n";
                    }
                    kinect_control->initMoveGroup(gesture);
                } else {
                    RCLCPP_INFO(logger, "\nToo many or too few bodies detected !!\n\n");
                    myfile << "Too many or too few bodies detected !!\n\n";
                    kinect_control->initMoveGroup(gesture);
                }
                k4abt_frame_release(body_frame); // Remember to release the body frame once you finish using it
            } else if (pop_frame_result == K4A_WAIT_RESULT_TIMEOUT) {
                //  It should never hit timeout when K4A_WAIT_INFINITE is set.
                printf("Error! Pop body frame result timeout!\n");
                break;
            } else {
                printf("Pop body frame result failed!\n");
                break;
            }
        } else if (get_capture_result == K4A_WAIT_RESULT_TIMEOUT) {
            // It should never hit time out when K4A_WAIT_INFINITE is set.
            printf("Error! Get depth frame time out!\n");
            break;
        } else {
            printf("Get depth capture returned error: %d\n", get_capture_result);
            break;
        }

    } while (frame_count < 120);

    myfile.close();

    printf("Finished body tracking processing!\n");

    k4abt_tracker_shutdown(tracker);
    k4abt_tracker_destroy(tracker);
    k4a_device_stop_cameras(device);
    k4a_device_close(device);

    printf("hello world kinect_control package\n");
    //rclcpp::spin(kinect_control);
    //    // Shutdown ROS
    rclcpp::shutdown();
    return 0;
}
