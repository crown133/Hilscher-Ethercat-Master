#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "libertech_interface_msgs/msg/hybrid_joint_state.hpp"
#include "can_msgs/msg/frame.hpp"
#include "libertech_messaging/MotorDMInterface.hpp"

#define Joint_Num 2

class HeadControlNode : public rclcpp::Node
{
public:
  HeadControlNode() : Node("head_control_node")
  {
    this->declare_parameter<bool>("dummy", true);
    this->declare_parameter<std::string>("hybrid_joint_cmd_topic", "/hybrid_joint_cmd_head");
    this->declare_parameter<std::string>("joint_state_topic", "/joint_states_head");
    this->declare_parameter<std::string>("from_can_bus_topic", "/from_can_bus");
    this->declare_parameter<std::string>("to_can_bus_topic", "/to_can_bus");

    std::string hybrid_joint_cmd_topic;
    this->get_parameter("hybrid_joint_cmd_topic", hybrid_joint_cmd_topic);
    std::string joint_state_topic;
    this->get_parameter("joint_state_topic", joint_state_topic);
    std::string from_can_bus_topic;
    this->get_parameter("from_can_bus_topic", from_can_bus_topic);
    std::string to_can_bus_topic;
    this->get_parameter("to_can_bus_topic", to_can_bus_topic);

    this->get_parameter("dummy", dummy_);
    std::cout << "head dummy: " << dummy_ << std::endl;

    // /to_can_bus：使用自定义消息类型,每接收一次hybrid joint state就发布一次给/to_can_bus
    hybrid_joint_cmd_subscriber_ = this->create_subscription<libertech_interface_msgs::msg::HybridJointState>(
        hybrid_joint_cmd_topic, 10, std::bind(&HeadControlNode::hybridJointCmdCallback, this, std::placeholders::_1));
    frames_pub_ = this->create_publisher<can_msgs::msg::Frame>(to_can_bus_topic, 10);

    // /from_can_bus：初始化发布器，没接收一次can msg就发布一次给/joint_states_head
    frames_sub_ = this->create_subscription<can_msgs::msg::Frame>(
        from_can_bus_topic, 10, std::bind(&HeadControlNode::fromCanBusCallback, this, std::placeholders::_1));
    joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(joint_state_topic, 10);

    RCLCPP_INFO(this->get_logger(), "Head control node has been started.");
  }

private:
  void hybridJointCmdCallback(const libertech_interface_msgs::msg::HybridJointState::SharedPtr msg)
  {
    // 头部只接收位置，虽然是MIT协议
    for (int i = 0; i < msg->pos.size(); i++)
    {
      // // Calculate the difference between the current and the previous position commands
      // double position_difference = std::abs(msg->pos[i] - cmd[i].pos_d);

      // // TODO: check
      // // Check if the position difference exceeds 30 degrees (approximately 0.5236 radians)
      // if (position_difference > 0.2)
      // {
      //   RCLCPP_WARN(this->get_logger(),
      //               "Position command difference too large for joint %d: %.2f degrees. Command ignored.", i,
      //               position_difference * 180.0 / M_PI);
      //   continue;  // Skip updating this joint's command
      // }
      int dir;
      if (i == 1)
      {
        dir = -1;
      }
      else
      {
        dir = 1;
      }

      cmd[i].pos_d = dir * msg->pos[i];  // TODO:记得改方向
      cmd[i].vel_d = 0;
      cmd[i].Kp_d = 2;  // TODO:记得改
      cmd[i].Kd_d = 1;  // TODO:记得改
      cmd[i].tor_d = 0;

      can_msgs::msg::Frame frame = motor_cmd(i+1, cmd[i]);
      frames_pub_->publish(frame);

      // RCLCPP_INFO(this->get_logger(), "Updated cmd[%d].pos_d to: %f", i, cmd[i].pos_d);
    }
    
  }

  void fromCanBusCallback(const can_msgs::msg::Frame::SharedPtr msg)
  {
    sensor_msgs::msg::JointState joint_state_msg;

    if (dummy_)
    {
      // 直接赋值
      for (int i = 0; i < Joint_Num; i++)
      {
        state[i].pos = cmd[i].pos_d;
        state[i].vel = cmd[i].vel_d;
        state[i].tor = cmd[i].tor_d;
      }
    }
    else
    {
      // 从 HeadInterface 获取状态
      for (int i = 0; i < Joint_Num; i++)
      {
        motor_state_t state_tmp = motor_state(i+1, *msg);
        state[i].pos = state_tmp.pos;
        state[i].vel = state_tmp.vel;
        state[i].tor = state_tmp.tor;
      }
      
    }

    // 设置 JointState 消息的头
    joint_state_msg.header.stamp = this->get_clock()->now();
    joint_state_msg.header.frame_id = "base_link";  // 根据实际情况设置 frame_id

    // 设置关节名字
    joint_state_msg.name = {
      "head_yaw_joint",
      "head_pitch_joint",
    };

    // 设置位置、速度、力矩
    joint_state_msg.position.resize(joint_state_msg.name.size(), 0.0);
    joint_state_msg.velocity.resize(joint_state_msg.name.size(), 0.0);
    joint_state_msg.effort.resize(joint_state_msg.name.size(), 0.0);

    for (size_t i = 0; i < joint_state_msg.name.size(); ++i)
    {
      int dir;
      if (i == 1)
      {
        dir = -1;
      }
      else
      {
        dir = 1;
      }

      joint_state_msg.position[i] = dir * state[i].pos;
      joint_state_msg.velocity[i] = dir * state[i].vel;
      joint_state_msg.effort[i] = dir * state[i].tor;
    }

    // 发布 JointState 消息
    joint_state_publisher_->publish(joint_state_msg);
    // RCLCPP_INFO(this->get_logger(), "Published joint states to /joint_states_head");
  }

  // 发送can msg
  rclcpp::Subscription<libertech_interface_msgs::msg::HybridJointState>::SharedPtr hybrid_joint_cmd_subscriber_;
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr frames_pub_;
  // 接收can msg
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr frames_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;

  bool dummy_;

  motor_cmd_t cmd[2];
  motor_state_t state[2];
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HeadControlNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
