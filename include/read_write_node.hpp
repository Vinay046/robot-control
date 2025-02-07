#ifndef READ_WRITE_NODE_HPP_
#define READ_WRITE_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <dynamixel_sdk/dynamixel_sdk.h>

// Custom message and service interfaces
#include "robot_control_custom_interfaces/msg/set_group_velocities.hpp"
#include "robot_control_custom_interfaces/msg/motor_status.hpp"

class ReadWriteNode : public rclcpp::Node
{
public:
  ReadWriteNode();
  ~ReadWriteNode();

private:
  void setupDynamixel(uint8_t dxl_id);
  void controlLoop();

  // ROS 2 Subscribers & Publishers
  rclcpp::Subscription<robot_control_custom_interfaces::msg::SetGroupVelocities>::SharedPtr set_group_velocities_subscriber_;
  rclcpp::Publisher<robot_control_custom_interfaces::msg::MotorStatus>::SharedPtr motor_status_publisher_;
  rclcpp::TimerBase::SharedPtr control_loop_timer_;

  // Dynamixel SDK Handlers
  dynamixel::PortHandler *portHandler;
  dynamixel::PacketHandler *packetHandler;
  dynamixel::GroupSyncWrite *groupSyncWrite;
  dynamixel::GroupSyncRead *groupSyncRead;

  // Motor IDs and goal velocities
  std::vector<uint8_t> motor_ids;
  std::vector<int32_t> goal_velocities;

  // FIX: Declare missing variables
  int dxl_comm_result;
  uint8_t dxl_error;
};

#endif  // READ_WRITE_NODE_HPP_
