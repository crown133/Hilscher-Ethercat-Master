#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "ethercat_device_configurator/EthercatDeviceConfigurator.hpp"
#include "elmo_ethercat_sdk/Elmo.hpp"
#include "libertech_interface_msgs/msg/hybrid_joint_state.hpp"
#include "libertech_interface_msgs/srv/init_hardware.hpp"
#include <eigen3/Eigen/Dense>

#define NUM_MOTORS 26
#define LEG_MOTORS 12
#define WAIST_MOTORS 0


std::unordered_map<std::string, int> ethercatDevices = {
  { "LeftLegHTAAElmo", 0 },
  { "LeftLegHAAElmo", 1 },
  { "LeftLegHFEElmo", 2 },
  { "LeftLegKFEElmo", 3 },
  { "LeftLegAAAElmo", 4 },
  { "LeftLegAFEElmo", 5 },
  { "RightLegHTAAElmo", 6 },
  { "RightLegHAAElmo", 7 },
  { "RightLegHFEElmo", 8 },
  { "RightLegKFEElmo", 9 },
  { "RightLegAAAElmo", 10 },
  { "RightLegAFEElmo", 11 },

  // { "LeftArmShouderPitchElmo", 12 },
  // { "LeftArmShouderRollElmo", 13 },
  // { "LeftArmShouderYawElmo", 14 },
  // { "LeftArmElbowPitchElmo", 15 },
  // { "LeftArmElbowRollElmo", 16 },
  // { "LeftArmWristPitchElmo", 17 },
  // { "LeftArmWristYawElmo", 18 },
  // { "RightArmShouderPitchElmo", 19 },
  // { "RightArmShouderRollElmo", 20 },
  // { "RightArmShouderYawElmo", 21 },
  // { "RightArmElbowPitchElmo", 22 },
  // { "RightArmElbowRollElmo", 23 },
  // { "RightArmWristPitchElmo", 24 },
  // { "RightArmWristYawElmo", 25 },
};

struct MotorData
{
  double pos, vel, tau;
  double kp, kd;

  MotorData(double pos = 0.0, double vel = 0.0, double tau = 0.0, double kp = 0.0, double kd = 0.0)
    : pos(pos), vel(vel), tau(tau), kp(kp), kd(kd)
  { }
};

struct MotorCmd
{
  double pos_des, vel_des, kp_des, kd_des, ff_des;
  double tau_old, tau_des;

  MotorCmd(double pos_des = 0.0, double vel_des = 0.0, double kp_des = 0.0, double kd_des = 0.0, double ff_des = 0.0,
           double tau_old = 0.0, double tau_des = 0.0)
    : pos_des(pos_des), vel_des(vel_des), kp_des(kp_des), kd_des(kd_des), ff_des(ff_des), tau_old(tau_old), tau_des(tau_des)
  { }
};

// 全身数据结构体 假设每只手有13个自由度
struct RobotData
{
  MotorData motors[LEG_MOTORS + WAIST_MOTORS + 14];
};

struct RobotCmd
{
  MotorCmd motors[LEG_MOTORS + WAIST_MOTORS + 14];
};

class ElmoControlNode : public rclcpp::Node
{
public:
  ElmoControlNode(); 
  ~ElmoControlNode();

private:
  rcl_interfaces::msg::SetParametersResult param_callback(const std::vector<rclcpp::Parameter> &parameters);

  void hybridJointCmdCallback(const libertech_interface_msgs::msg::HybridJointState::SharedPtr msg);
  void timerCallback();
  void init_joint_callback(
      const std::shared_ptr<libertech_interface_msgs::srv::InitHardware::Request> request,
      std::shared_ptr<libertech_interface_msgs::srv::InitHardware::Response> response);

  bool setupEtherCAT();
  
  rclcpp::Subscription<libertech_interface_msgs::msg::HybridJointState>::SharedPtr hybrid_joint_cmd_subscriber_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
  rclcpp::Service<libertech_interface_msgs::srv::InitHardware>::SharedPtr init_joint_server_;

  bool dummy_;
  //******* EtherCAT *******/
  bool abrt_ = false;
  EthercatDeviceConfigurator::SharedPtr configurator_;
  std::string ethercat_config_file_;
  std::thread ethercat_thread_;

  std::vector<double> joint_pos_offset_; // 反馈，绝对值， 增量
  std::vector<bool> actuator_pos_offset_flag_;

  std::vector<double> joint_pos_init_cmd_;
  std::vector<double> joint_pos_integral_error_;
  std::vector<double> joint_pos_min_;
  std::vector<double> joint_pos_max_;
  std::vector<double> joint_vel_max_;
  std::vector<bool> joint_first_flag_;
  double dt = 0.001;

  // 自己定义的位置环
  double vkp_;
  double vkd_;
  double vki_;
  double vmax_;

  // Flag to set the drive state for the elmos on first startup
  bool elmoEnabledAfterStartup = false;

  std::mutex mtx_data_;
  std::mutex mtx_commands_;

  RobotData data_;
  RobotCmd commands_;
};

