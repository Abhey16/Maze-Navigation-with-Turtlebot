#include <navigator_23.hpp>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <utils.hpp>
#include <deque>
#include <sstream>
#include <tf2/exceptions.h>

using namespace std::chrono_literals;

//===============================================
// Getting the values of aruco marker
void Navigation_23::aruco_pos(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg) {
 if (!msg->poses.empty()) {
 const auto& marker_id_ = msg->marker_ids[0];
 marker_id = marker_id_;
 geometry_msgs::msg::TransformStamped aruco_transform_stamped;
 aruco_transform_stamped.header.stamp = this->get_clock()->now();
 aruco_transform_stamped.header.frame_id = "camera_rgb_optical_frame";
 aruco_transform_stamped.child_frame_id = "aruco_marker_detected";

 RCLCPP_INFO_STREAM(this->get_logger(), "aruco_no: " << marker_id << "\t" );
 
 // Send the transform
 tf_broadcaster_->sendTransform(aruco_transform_stamped);

 } else {
 RCLCPP_WARN(this->get_logger(), "ArucoMarkers message is empty");
 }
}

//===============================================
// Getting the values of BLUE BATTERY from camera positions from camera1
void Navigation_23::battery_pos1(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg) {
 if (!msg->part_poses.empty()) {

 geometry_msgs::msg::PoseStamped target_pose_from_cam;
 geometry_msgs::msg::PoseStamped new_variable1;
 target_pose_from_cam.header.frame_id = "camera1_frame";
 target_pose_from_cam.header.stamp = this->get_clock()->now();
 target_pose_from_cam.pose = msg->part_poses[0].pose;

 try {
 geometry_msgs::msg::TransformStamped target_pose_from_req = tf_buffer_->lookupTransform("map", target_pose_from_cam.header.frame_id, this->get_clock()->now());
 tf2::doTransform(target_pose_from_cam, new_variable1, target_pose_from_req);

 // Check if the color is already present in the vector
 bool colorExists = false;
 for (const auto& color : battery_blue_color_vector_) {
 if (color == msg->part_poses[0].part.color) {
 colorExists = true;
 break;
 }
 }

 // Add the entry to the list only if the color is not already present
 if (!colorExists) {
 battery_blue_color_vector_.push_back(msg->part_poses[0].part.color);
 battery_x_vector_.push_back(new_variable1.pose.position.x);
 battery_y_vector_.push_back(new_variable1.pose.position.y);
 battery_z_vector_.push_back(new_variable1.pose.position.z);
 battery_ox_vector_.push_back(new_variable1.pose.orientation.x);
 battery_oy_vector_.push_back(new_variable1.pose.orientation.y);
 battery_oz_vector_.push_back(new_variable1.pose.orientation.z);
 battery_blue_vector_x.push_back(new_variable1.pose.position.x);
 battery_blue_vector_y.push_back(new_variable1.pose.position.y);
 battery_blue_oz_vector_.push_back(new_variable1.pose.orientation.z);
 RCLCPP_INFO_STREAM(this->get_logger(), "BLUE_BATTERY");
 print_data_vector();
 if (battery_x_vector_.size() == 5) {
 send_goal();
 }
 }
 } catch (const tf2::TransformException& ex) {
 RCLCPP_WARN(this->get_logger(), "Transform failed: %s", ex.what());
 } 
 } else {
 RCLCPP_WARN(this->get_logger(), "BatteryPos message is empty or part_poses is empty");
 }
}

//===============================================
// Getting the values of ORANGE BATTERY from camera positions from camera2

void Navigation_23::battery_pos2(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg) {
 if (!msg->part_poses.empty()) {

 geometry_msgs::msg::PoseStamped target_pose_from_cam;
 geometry_msgs::msg::PoseStamped new_variable2;



 target_pose_from_cam.header.frame_id = "camera2_frame";
 target_pose_from_cam.header.stamp = this->get_clock()->now();
 target_pose_from_cam.pose = msg->part_poses[0].pose;

 try {
 geometry_msgs::msg::TransformStamped target_pose_from_req = tf_buffer_->lookupTransform("map", target_pose_from_cam.header.frame_id, this->get_clock()->now());
 tf2::doTransform(target_pose_from_cam, new_variable2, target_pose_from_req);
 // Check if the color is already present in the vector
 bool colorExists = false;
 for (const auto& color : battery_orange_color_vector_) {
 if (color == msg->part_poses[0].part.color) {
 colorExists = true;
 break;
 }
 }

 // Add the entry to the list only if the color is not already present
 if (!colorExists) {
 battery_orange_color_vector_.push_back(msg->part_poses[0].part.color);
 battery_x_vector_.push_back(new_variable2.pose.position.x);
 battery_y_vector_.push_back(new_variable2.pose.position.y);
 battery_z_vector_.push_back(new_variable2.pose.position.z);
 battery_ox_vector_.push_back(new_variable2.pose.orientation.x);
 battery_oy_vector_.push_back(new_variable2.pose.orientation.y);
 battery_oz_vector_.push_back(new_variable2.pose.orientation.z);
 battery_orange_vector_x.push_back(new_variable2.pose.position.x);
 battery_orange_vector_y.push_back(new_variable2.pose.position.y);
 battery_orange_oz_vector_.push_back(new_variable2.pose.orientation.z);
 RCLCPP_INFO_STREAM(this->get_logger(), "ORANGE_BATTERY");
 print_data_vector();
 if (battery_x_vector_.size() == 5) {
 send_goal();
 }
 }
 } catch (const tf2::TransformException& ex) {
 RCLCPP_WARN(this->get_logger(), "Transform failed: %s", ex.what());
 }
 } else {
 RCLCPP_WARN(this->get_logger(), "BatteryPos message is empty or part_poses is empty");
 }
}

//===============================================
// Getting the values of PURPLE BATTERY from camera positions from camera3

void Navigation_23::battery_pos3(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg) {
 if (!msg->part_poses.empty()) {

 geometry_msgs::msg::PoseStamped target_pose_from_cam;
 geometry_msgs::msg::PoseStamped new_variable3;

 target_pose_from_cam.header.frame_id = "camera3_frame";
 target_pose_from_cam.header.stamp = this->get_clock()->now();
 target_pose_from_cam.pose = msg->part_poses[0].pose;

 try {
 geometry_msgs::msg::TransformStamped target_pose_from_req = tf_buffer_->lookupTransform("map", target_pose_from_cam.header.frame_id, this->get_clock()->now());
 tf2::doTransform(target_pose_from_cam, new_variable3, target_pose_from_req);
 // Check if the color is already present in the vector
 bool colorExists = false;
 for (const auto& color : battery_purple_color_vector_) {
 if (color == msg->part_poses[0].part.color) {
 colorExists = true;
 break;
 }
 }

 // Add the entry to the list only if the color is not already present
 if (!colorExists) {
 battery_purple_color_vector_.push_back(msg->part_poses[0].part.color);
 battery_x_vector_.push_back(new_variable3.pose.position.x);
 battery_y_vector_.push_back(new_variable3.pose.position.y);
 battery_z_vector_.push_back(new_variable3.pose.position.z);
 battery_ox_vector_.push_back(new_variable3.pose.orientation.x);
 battery_oy_vector_.push_back(new_variable3.pose.orientation.y);
 battery_oz_vector_.push_back(new_variable3.pose.orientation.z);
 battery_purple_vector_x.push_back(new_variable3.pose.position.x);
 battery_purple_vector_y.push_back(new_variable3.pose.position.y);
 battery_purple_oz_vector_.push_back(new_variable3.pose.orientation.z);
 RCLCPP_INFO_STREAM(this->get_logger(), "PURPLE_BATTERY");
 print_data_vector();
 if (battery_x_vector_.size() == 5) {
 send_goal();
 }
 }
 } catch (const tf2::TransformException& ex) {
 RCLCPP_WARN(this->get_logger(), "Transform failed: %s", ex.what());
 }
 } else {
 RCLCPP_WARN(this->get_logger(), "BatteryPos message is empty or part_poses is empty");
 }
}

//===============================================
// Getting the values of GREEN BATTERY from camera positions from camera4

void Navigation_23::battery_pos4(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg) {
 if (!msg->part_poses.empty()) {

 geometry_msgs::msg::PoseStamped target_pose_from_cam;
 geometry_msgs::msg::PoseStamped new_variable4;



 target_pose_from_cam.header.frame_id = "camera4_frame";
 target_pose_from_cam.header.stamp = this->get_clock()->now();
 target_pose_from_cam.pose = msg->part_poses[0].pose;

 try {
 geometry_msgs::msg::TransformStamped target_pose_from_req = tf_buffer_->lookupTransform("map", target_pose_from_cam.header.frame_id, this->get_clock()->now());
 tf2::doTransform(target_pose_from_cam, new_variable4, target_pose_from_req);
 // Check if the color is already present in the vector
 bool colorExists = false;
 for (const auto& color : battery_green_color_vector_) {
 if (color == msg->part_poses[0].part.color) {
 colorExists = true;
 break;
 }
 }

 // Add the entry to the list only if the color is not already present
 if (!colorExists) {
 battery_green_color_vector_.push_back(msg->part_poses[0].part.color);
 battery_x_vector_.push_back(new_variable4.pose.position.x);
 battery_y_vector_.push_back(new_variable4.pose.position.y);
 battery_z_vector_.push_back(new_variable4.pose.position.z);
 battery_ox_vector_.push_back(new_variable4.pose.orientation.x);
 battery_oy_vector_.push_back(new_variable4.pose.orientation.y);
 battery_oz_vector_.push_back(new_variable4.pose.orientation.z);
 battery_green_vector_x.push_back(new_variable4.pose.position.x);
 battery_green_vector_y.push_back(new_variable4.pose.position.y);
 battery_green_oz_vector_.push_back(new_variable4.pose.orientation.z);
 RCLCPP_INFO_STREAM(this->get_logger(), "GREEN_BATTERY");
 print_data_vector();
 if (battery_x_vector_.size() == 5) {
 send_goal();
 }
 }
 } catch (const tf2::TransformException& ex) {
 RCLCPP_WARN(this->get_logger(), "Transform failed: %s", ex.what());
 }
 } else {
 RCLCPP_WARN(this->get_logger(), "BatteryPos message is empty or part_poses is empty");
 }
}

//===============================================
// Getting the values of RED BATTERY from camera positions from camera5

void Navigation_23::battery_pos5(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg) {
 if (!msg->part_poses.empty()) {

 geometry_msgs::msg::PoseStamped target_pose_from_cam;
 geometry_msgs::msg::PoseStamped new_variable5;



 target_pose_from_cam.header.frame_id = "camera5_frame";
 target_pose_from_cam.header.stamp = this->get_clock()->now();
 target_pose_from_cam.pose = msg->part_poses[0].pose;

 try {
 geometry_msgs::msg::TransformStamped target_pose_from_req = tf_buffer_->lookupTransform("map", target_pose_from_cam.header.frame_id, this->get_clock()->now());
 tf2::doTransform(target_pose_from_cam, new_variable5, target_pose_from_req);
 // Check if the color is already present in the vector
 bool colorExists = false;
 for (const auto& color : battery_red_color_vector_) {
 if (color == msg->part_poses[0].part.color) {
 colorExists = true;
 break;
 }
 }

 // Add the entry to the list only if the color is not already present
 if (!colorExists) {
 battery_red_color_vector_.push_back(msg->part_poses[0].part.color);
 battery_x_vector_.push_back(new_variable5.pose.position.x);
 battery_y_vector_.push_back(new_variable5.pose.position.y);
 battery_z_vector_.push_back(new_variable5.pose.position.z);
 battery_ox_vector_.push_back(new_variable5.pose.orientation.x);
 battery_oy_vector_.push_back(new_variable5.pose.orientation.y);
 battery_oz_vector_.push_back(new_variable5.pose.orientation.z);
 battery_red_vector_x.push_back(new_variable5.pose.position.x);
 battery_red_vector_y.push_back(new_variable5.pose.position.y);
 battery_red_oz_vector_.push_back(new_variable5.pose.orientation.z);
 RCLCPP_INFO_STREAM(this->get_logger(), "RED_BATTERY");
 print_data_vector();
 if (battery_x_vector_.size() == 5) {
 send_goal();
 }
 }
 } catch (const tf2::TransformException& ex) {
 RCLCPP_WARN(this->get_logger(), "Transform failed: %s", ex.what());
 }
 } else {
 RCLCPP_WARN(this->get_logger(), "BatteryPos message is empty or part_poses is empty");
 }
}

//===============================================
// Printing the values of the Battery positions
void Navigation_23::print_data_vector(){


 std::cout << "Battery Positions [ ";
 std::cout << " detected at xyz = [" << battery_x_vector_.back() << ","<< battery_y_vector_.back() << ","
 << battery_z_vector_.back() << "] rpy = [" << battery_ox_vector_.back() << "," << battery_oy_vector_.back() << "," << battery_oz_vector_.back() << "]"; 
 std::cout << "]" << std::endl;


}

//===============================================
//Nav method to set initial pose
void Navigation_23::set_initial_pose() {
 auto message = geometry_msgs::msg::PoseWithCovarianceStamped();
 message.header.frame_id = "map";
 message.pose.pose.position.x = 1;
 message.pose.pose.position.y = -1.617;
 message.pose.pose.position.z = 0;
 message.pose.pose.orientation.x = 0.0023258279310553356;
 message.pose.pose.orientation.y = 0.0023451902769101034;
 message.pose.pose.orientation.z = -0.7007043986843412;
 message.pose.pose.orientation.w = 0.7134440666733559;
 initial_pose_pub_->publish(message);
}

//===============================================
// Function to plan the trajectory of bot
void Navigation_23::send_goal() {

if (marker_id == 0){
 if( mage_msgs::msg::Part::GREEN && battery_green_color_vector_.back() == 1){
 RCLCPP_INFO(this->get_logger(), "First execute green goal");
 std::this_thread::sleep_for(std::chrono::seconds(1));
 }
 else if( mage_msgs::msg::Part::RED && battery_red_color_vector_.back() == 0){
 RCLCPP_INFO(this->get_logger(), "First execute red goal");
 std::this_thread::sleep_for(std::chrono::seconds(1));
 }
 else if( mage_msgs::msg::Part::ORANGE && battery_orange_color_vector_.back() == 3){
 RCLCPP_INFO(this->get_logger(), "First execute orange goal");
 std::this_thread::sleep_for(std::chrono::seconds(1));
 }
 else if( mage_msgs::msg::Part::PURPLE && battery_purple_color_vector_.back() == 4){
 RCLCPP_INFO(this->get_logger(), "First execute purple goal");
 std::this_thread::sleep_for(std::chrono::seconds(1));
 }
 else if( mage_msgs::msg::Part::BLUE && battery_blue_color_vector_.back() == 2){
 RCLCPP_INFO(this->get_logger(), "First execute blue goal");
 std::this_thread::sleep_for(std::chrono::seconds(1));
 }
 else{
 RCLCPP_INFO(this->get_logger(), "Correct values not detected");
 std::this_thread::sleep_for(std::chrono::seconds(1));
 }


 auto goal_msg = NavigateThroughPoses::Goal();
 goal_msg.poses.resize(5); 
 goal_msg.poses[0].header.frame_id = "map";
 goal_msg.poses[0].pose.position.x = battery_green_vector_x.back();
 goal_msg.poses[0].pose.position.y = battery_green_vector_y.back();
 goal_msg.poses[0].pose.position.z = 0.0;
 goal_msg.poses[0].pose.orientation.x = 0.0;
 goal_msg.poses[0].pose.orientation.y = 0.0;
 goal_msg.poses[0].pose.orientation.z = battery_green_oz_vector_.back();
 goal_msg.poses[0].pose.orientation.w = 0.0;

 goal_msg.poses[1].header.frame_id = "map";
 goal_msg.poses[1].pose.position.x = battery_red_vector_x.back();
 goal_msg.poses[1].pose.position.y = battery_red_vector_y.back();
 goal_msg.poses[1].pose.position.z = 0.0;
 goal_msg.poses[1].pose.orientation.x = 0.0;
 goal_msg.poses[1].pose.orientation.y = 0.0;
 goal_msg.poses[1].pose.orientation.z = battery_red_oz_vector_.back();
 goal_msg.poses[1].pose.orientation.w = 0.0;

 goal_msg.poses[2].header.frame_id = "map";
 goal_msg.poses[2].pose.position.x = battery_orange_vector_x.back();
 goal_msg.poses[2].pose.position.y = battery_orange_vector_y.back();
 goal_msg.poses[2].pose.position.z = 0.0;
 goal_msg.poses[2].pose.orientation.x = 0.0;
 goal_msg.poses[2].pose.orientation.y = 0.0;
 goal_msg.poses[2].pose.orientation.z = battery_orange_oz_vector_.back();
 goal_msg.poses[2].pose.orientation.w = 0.0;

 goal_msg.poses[3].header.frame_id = "map";
 goal_msg.poses[3].pose.position.x = battery_purple_vector_x.back();
 goal_msg.poses[3].pose.position.y = battery_purple_vector_y.back();
 goal_msg.poses[3].pose.position.z = 0.0;
 goal_msg.poses[3].pose.orientation.x = 0.0;
 goal_msg.poses[3].pose.orientation.y = 0.0;
 goal_msg.poses[3].pose.orientation.z = battery_purple_oz_vector_.back();
 goal_msg.poses[3].pose.orientation.w = 0.0;

 goal_msg.poses[4].header.frame_id = "map";
 goal_msg.poses[4].pose.position.x = battery_blue_vector_x.back();
 goal_msg.poses[4].pose.position.y = battery_blue_vector_y.back();
 goal_msg.poses[4].pose.position.z = 0.0;
 goal_msg.poses[4].pose.orientation.x = 0.0;
 goal_msg.poses[4].pose.orientation.y = 0.0;
 goal_msg.poses[4].pose.orientation.z = battery_blue_oz_vector_.back();
 goal_msg.poses[4].pose.orientation.w = 0.0;

 using namespace std::placeholders;

 if (!this->client_->wait_for_action_server()) {
 RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
 rclcpp::shutdown();
 }

 auto send_goal_options = rclcpp_action::Client<NavigateThroughPoses>::SendGoalOptions();
 send_goal_options.goal_response_callback = std::bind(&Navigation_23::goal_response_callback, this, _1);
 send_goal_options.feedback_callback = std::bind(&Navigation_23::feedback_callback, this, _1, _2);
 send_goal_options.result_callback = std::bind(&Navigation_23::result_callback, this, _1);

 client_->async_send_goal(goal_msg, send_goal_options);

}

}

//===============================================
// Goal response callback
void Navigation_23::goal_response_callback(
 std::shared_future<GoalHandleNavigation::SharedPtr> future) {
 auto goal_handle = future.get();
 if (!goal_handle) {
 RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
 rclcpp::shutdown();
 } else {
 RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
 }
}

//===============================================
// Feedback callback
void Navigation_23::feedback_callback(
 GoalHandleNavigation::SharedPtr,
 const std::shared_ptr<const NavigateThroughPoses::Feedback> feedback) {
 RCLCPP_INFO(this->get_logger(), "Robot is driving towards the goal");
}

//===============================================
// Result callback
void Navigation_23::result_callback(
 const GoalHandleNavigation::WrappedResult& result) {
 switch (result.code) {
 case rclcpp_action::ResultCode::SUCCEEDED:
 break;
 case rclcpp_action::ResultCode::ABORTED:
 RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
 return;
 case rclcpp_action::ResultCode::CANCELED:
 RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
 return;
 default:
 RCLCPP_ERROR(this->get_logger(), "Unknown result code");
 return;
 }
 rclcpp::shutdown();
}

//===============================================
// Main function
int main(int argc, char** argv) {
 rclcpp::init(argc, argv);
 auto node = std::make_shared<Navigation_23>("navigator_23");
 rclcpp::spin(node);
 rclcpp::shutdown();
}
