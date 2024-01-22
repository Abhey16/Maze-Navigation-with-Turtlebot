/**
 * @file navigator_23.hpp
 * @author Group23 (Sachin Jadhav, Abhey Sharma, Abhsihek Avadh)
 * @brief 
 * @version 0.1
 * @date 2023-12-20
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once


#include "geometry_msgs/msg/twist.hpp"
#include <vector>
#include <cmath>
#include <tf2_ros/static_transform_broadcaster.h>
#include <utils.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "tf2_ros/transform_broadcaster.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include "ros2_aruco_interfaces/msg/aruco_markers.hpp"
#include "mage_msgs/msg/advanced_logical_camera_image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include <cmath>
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rosidl_typesupport_cpp/action_type_support.hpp"


using namespace std::chrono_literals;
/**
 * @brief Creating Class Navigation_23
 * 
 */
class Navigation_23 : public rclcpp::Node
{
public:
    // Alliases for the action
    using NavigateThroughPoses = nav2_msgs::action::NavigateThroughPoses;
    using GoalHandleNavigation = rclcpp_action::ClientGoalHandle<NavigateThroughPoses>;
    /**
     * @brief Construct a new Navigation_23 object
     * 
     * @param node_name 
     */
    Navigation_23(std::string node_name) : Node(node_name)
    {
        // parameter to decide whether to execute the broadcaster or not
        RCLCPP_INFO(this->get_logger(), "Broadcaster demo started");

        // Assuming waypoint_params.yaml contains aruco_0 and aruco_1 parameters
        this->declare_parameter<std::vector<std::string>>("my_names", std::vector<std::string>());

        // Retrieve the "my_names" parameter
        std::vector<std::string> names = this->get_parameter("my_names").as_string_array();

        // Declare aruco_0 and aruco_1 parameters dynamically based on retrieved names
        for (const std::string& name : names)
        {
            // Declare aruco_0 parameters
            this->declare_parameter<std::string>("aruco_0." + name + ".type");
            this->declare_parameter<std::string>("aruco_0." + name + ".color");

            // Declare aruco_1 parameters
            this->declare_parameter<std::string>("aruco_1." + name + ".type");
            this->declare_parameter<std::string>("aruco_1." + name + ".color");

            // Retrieve parameters and use them as needed
            std::string battery_0_type = this->get_parameter("aruco_0." + name + ".type").as_string();
            std::string battery_0_color = this->get_parameter("aruco_0." + name + ".color").as_string();
            std::string battery_1_type = this->get_parameter("aruco_1." + name + ".type").as_string();
            std::string battery_1_color = this->get_parameter("aruco_1." + name + ".color").as_string();

            // Use the parameters as needed
            RCLCPP_INFO(this->get_logger(), "Name: %s", name.c_str());
            RCLCPP_INFO(this->get_logger(), "aruco_0_color: %s", battery_0_color.c_str());
            RCLCPP_INFO(this->get_logger(), "aruco_1_color: %s", battery_1_color.c_str());
        }

        // initialize the transform broadcaster
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Load a buffer of transforms
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Creating Subscribers
        rclcpp::QoS qos(10);     qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
        logical_subscription_1= this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>("/mage/camera1/image", 
        qos, std::bind(&Navigation_23::battery_pos1, this, std::placeholders::_1));

        logical_subscription_2= this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>("/mage/camera2/image", 
        qos, std::bind(&Navigation_23::battery_pos2, this, std::placeholders::_1));

        logical_subscription_3= this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>("/mage/camera3/image", 
        qos, std::bind(&Navigation_23::battery_pos3, this, std::placeholders::_1));

        logical_subscription_4= this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>("/mage/camera4/image", 
        qos, std::bind(&Navigation_23::battery_pos4, this, std::placeholders::_1));  

        logical_subscription_5= this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>("/mage/camera5/image", 
        qos, std::bind(&Navigation_23::battery_pos5, this, std::placeholders::_1));    

        aruco_subscription_= this->create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>("aruco_markers", 
        10, std::bind(&Navigation_23::aruco_pos, this, std::placeholders::_1));


        /**
         * @brief Creating the client for the action server
         * 
         */
        client_ = rclcpp_action::create_client<NavigateThroughPoses>(this, "navigate_through_poses");
        // initialize the publisher
        initial_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 10);
        // set the initial pose for navigation
        /**
         * @brief Set the initial pose object
         * 
         */
        set_initial_pose();
        // pause for 5 seconds
        std::this_thread::sleep_for(std::chrono::seconds(5));
 
    }

private:
   
    /*!< Buffer that stores several seconds of transforms for easy lookup by the listener. */
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    /*!< Transform Listener Object */
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    /*!< Broadcaster object */
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr logical_subscription_1;
    /**
     * @brief Declaring the subscriber for the logical camera1
     * 
     * @param msg 
     */
    void battery_pos1(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);

    rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr logical_subscription_2;
    /**
     * @brief Declaring the subscriber for the logical camera2
     * 
     * @param msg 
     */
    void battery_pos2(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);    

    rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr logical_subscription_3;
    /**
     * @brief Declaring the subscriber for the logical camera3
     * 
     * @param msg 
     */
    void battery_pos3(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);
    
    rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr logical_subscription_4;
    /**
     * @brief  Declaring the subscriber for the logical camera4
     * 
     * @param msg 
     */
    void battery_pos4(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);

    rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr logical_subscription_5;
    /**
     * @brief  Declaring the subscriber for the logical camera5
     * 
     * @param msg 
     */
    void battery_pos5(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);

    rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr aruco_subscription_;
    /**
     * @brief Declaring the subscriber for the aruco markers
     * 
     * @param msg 
     */
    void aruco_pos(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg);

    // Assigning the vectors for the battery positions
    std::vector<double> battery_x_vector_;
    std::vector<double> battery_y_vector_;     std::vector<double> battery_z_vector_;    std::vector<double> battery_ox_vector_;
    std::vector<double> battery_oy_vector_;    std::vector<double> battery_oz_vector_;

    std::vector<int> battery_green_vector_x;    std::vector<int> battery_red_vector_x;      std::vector<int> battery_blue_vector_x;
    std::vector<int> battery_orange_vector_x;   std::vector<int> battery_purple_vector_x;   std::vector<int> battery_green_vector_y;
    std::vector<int> battery_red_vector_y;      std::vector<int> battery_blue_vector_y;     std::vector<int> battery_orange_vector_y;
    std::vector<int> battery_purple_vector_y;

    std::vector<int> battery_red_color_vector_;    std::vector<int> battery_blue_color_vector_;
    std::vector<int> battery_green_color_vector_;  std::vector<int> battery_orange_color_vector_;
    std::vector<int> battery_purple_color_vector_;   

    std::vector<int> battery_green_oz_vector_;    std::vector<int> battery_red_oz_vector_;      std::vector<int> battery_blue_oz_vector_;
    std::vector<int> battery_orange_oz_vector_;   std::vector<int> battery_purple_oz_vector_;   


    /**
     * @brief Print the data vector
     * 
     */
    void print_data_vector();
    
    double marker_id;

    /**
     * @brief Publisher to the topic /initialpose
     *
     */
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr  initial_pose_pub_;
    /**
     * @brief Action client for the action server navigate_through_poses
     *
     */
    rclcpp_action::Client<NavigateThroughPoses>::SharedPtr client_;
    //   rclcpp::TimerBase::SharedPtr timer_;
    /**
     * @brief Response from the server after sending the goal
     */
    void goal_response_callback(
        std::shared_future<GoalHandleNavigation::SharedPtr> future);
    /**
     * @brief Feedback received while the robot is driving towards the goal
     *
     * @param feedback
     */
    void feedback_callback(
        GoalHandleNavigation::SharedPtr,
        const std::shared_ptr<const NavigateThroughPoses::Feedback> feedback);
    /**
     * @brief Result after the action has completed
     *
     * @param result
     */
    void result_callback(const GoalHandleNavigation::WrappedResult& result);
    /**
     * @brief Method to build and send a goal using the client
     *
     */
    void send_goal();
    /**
     * @brief Set the initial pose object
     * 
     */
    void set_initial_pose();

    std::string current_color_;

};

