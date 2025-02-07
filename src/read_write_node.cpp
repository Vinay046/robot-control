#include "read_write_node.hpp"

#define ADDR_OPERATING_MODE     11
#define ADDR_TORQUE_ENABLE      64
#define ADDR_GOAL_VELOCITY      104
#define ADDR_PRESENT_VELOCITY   128

#define PROTOCOL_VERSION 2.0
#define BAUDRATE 57600
#define DEVICE_NAME "/dev/ttyACM0"

// Define IDs of the motors
#define MOTOR_IDS {1, 2, 3}

ReadWriteNode::ReadWriteNode()
: Node("read_write_node"), motor_ids(MOTOR_IDS), goal_velocities(motor_ids.size(), 0)
{
  RCLCPP_INFO(this->get_logger(), "Running Sync Read/Write Node for Velocity Control");

  // Initialize PortHandler and PacketHandler
  portHandler = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
  packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  // Open Serial Port
  if (!portHandler->openPort()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open the port!");
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Port opened successfully.");

  // Set Baudrate
  if (!portHandler->setBaudRate(BAUDRATE)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to set baudrate!");
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Baudrate set successfully.");

  // Initialize all motors
  for (uint8_t id : motor_ids) {
    setupDynamixel(id);
  }

  // Initialize Sync Write and Sync Read
  groupSyncWrite = new dynamixel::GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_VELOCITY, 4);
  groupSyncRead = new dynamixel::GroupSyncRead(portHandler, packetHandler, ADDR_PRESENT_VELOCITY, 4);

  // ROS2 Subscriber for velocity control
  set_group_velocities_subscriber_ =
    this->create_subscription<robot_control_custom_interfaces::msg::SetGroupVelocities>(
      "set_group_velocities",
      10,
      [this](const robot_control_custom_interfaces::msg::SetGroupVelocities::SharedPtr msg) -> void
      {
        for (size_t i = 0; i < msg->ids.size(); ++i) {
          auto it = std::find(motor_ids.begin(), motor_ids.end(), msg->ids[i]);
          if (it != motor_ids.end()) {
            size_t index = std::distance(motor_ids.begin(), it);
            goal_velocities[index] = msg->velocities[i];
          }
        }
      });

  // ROS2 Publisher for motor status
  motor_status_publisher_ = this->create_publisher<robot_control_custom_interfaces::msg::MotorStatus>("motor_status", 10);

  // Control loop with 10ms interval
  control_loop_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(10),
    std::bind(&ReadWriteNode::controlLoop, this));
}

ReadWriteNode::~ReadWriteNode()
{
  // Disable Torque on all motors before shutting down
  for (uint8_t id : motor_ids) {
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, id, ADDR_TORQUE_ENABLE, 0, &dxl_error);
  }
  portHandler->closePort();

  // Cleanup Sync Read/Write objects
  delete groupSyncWrite;
  delete groupSyncRead;
}

void ReadWriteNode::setupDynamixel(uint8_t dxl_id)
{
  // Set to Velocity Control Mode
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id, ADDR_OPERATING_MODE, 1, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Failed to set Velocity Control Mode for ID %d", dxl_id);
  }

  // Enable Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, 1, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Failed to enable torque for ID %d", dxl_id);
  }
}

void ReadWriteNode::controlLoop()
{
  // Add parameters to GroupSyncWrite
  groupSyncWrite->clearParam();
  for (size_t i = 0; i < motor_ids.size(); ++i) {
    uint8_t param_goal_velocity[4];
    int32_t velocity = goal_velocities[i];

    param_goal_velocity[0] = DXL_LOBYTE(DXL_LOWORD(velocity));
    param_goal_velocity[1] = DXL_HIBYTE(DXL_LOWORD(velocity));
    param_goal_velocity[2] = DXL_LOBYTE(DXL_HIWORD(velocity));
    param_goal_velocity[3] = DXL_HIBYTE(DXL_HIWORD(velocity));

    bool add_param_result = groupSyncWrite->addParam(motor_ids[i], param_goal_velocity);
    if (!add_param_result) {
      RCLCPP_ERROR(this->get_logger(), "Failed to add velocity for ID %d", motor_ids[i]);
    }
  }

  // Send Sync Write
  dxl_comm_result = groupSyncWrite->txPacket();
  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Sync Write failed: %s", packetHandler->getTxRxResult(dxl_comm_result));
  }

  // Read Present Velocities
  groupSyncRead->clearParam();
  for (uint8_t id : motor_ids) {
    groupSyncRead->addParam(id);
  }
  dxl_comm_result = groupSyncRead->txRxPacket();

  // Publish motor status
  auto status_msg = robot_control_custom_interfaces::msg::MotorStatus();
  for (uint8_t id : motor_ids) {
    int32_t velocity = groupSyncRead->getData(id, ADDR_PRESENT_VELOCITY, 4);
    status_msg.ids.push_back(id);
    status_msg.velocities.push_back(velocity);
  }
  motor_status_publisher_->publish(status_msg);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ReadWriteNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
