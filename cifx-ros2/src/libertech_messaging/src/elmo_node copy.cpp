#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "ethercat_device_configurator/EthercatDeviceConfigurator.hpp"
#include "elmo_ethercat_sdk/Elmo.hpp"
#include "libertech_interface_msgs/msg/hybrid_joint_state.hpp"
#include <eigen3/Eigen/Dense>


std::unordered_map<std::string, int> ethercatDevices = {
  { "WaistYawElmo", 0 },
  { "LeftLegHipYawElmo", 1 },
  { "LeftLegHipRollElmo", 2 },
  { "LeftLegHipPitchElmo", 3 },
  { "LeftLegKneeElmo", 4 },
  { "LeftLegAnkleRollElmo", 5 },
  { "LeftLegAnklePitchElmo", 6 },
  { "RightLegHipYawElmo", 7 },
  { "RightLegHipRollElmo", 8 },
  { "RightLegHipPitchElmo", 9 },
  { "RightLegKneeElmo", 10 },
  { "RightLegAnkleRollElmo", 11 },
  { "RightLegAnklePitchElmo", 12 },

  { "LeftArmShouderPitchElmo", 13 },
  { "LeftArmShouderRollElmo", 14 },
  { "LeftArmShouderYawElmo", 15 },
  { "LeftArmElbowPitchElmo", 16 },
  { "LeftArmElbowRollElmo", 17 },
  { "LeftArmWristPitchElmo", 18 },
  { "LeftArmWristYawElmo", 19 },
  { "RightArmShouderPitchElmo", 20 },
  { "RightArmShouderRollElmo", 21 },
  { "RightArmShouderYawElmo", 22 },
  { "RightArmElbowPitchElmo", 23 },
  { "RightArmElbowRollElmo", 24 },
  { "RightArmWristPitchElmo", 25 },
  { "RightArmWristYawElmo", 26 },
};

struct MotorData
{
  double pos, vel, tau;
  double kp, kd;

  MotorData(double pos = 0.0, double vel = 0.0, double tau = 0.0, double kp = 0.0, double kd = 0.0)
    : pos(pos), vel(vel), tau(tau), kp(kp), kd(kd)
  {
  }
};

struct MotorCmd
{
  double pos_des, vel_des, kp_des, kd_des, ff_des;
  double tau_des;

  MotorCmd(double pos_des = 0.0, double vel_des = 0.0, double kp_des = 0.0, double kd_des = 0.0, double ff_des = 0.0,
           double tau_des = 0.0)
    : pos_des(pos_des), vel_des(vel_des), kp_des(kp_des), kd_des(kd_des), ff_des(ff_des), tau_des(tau_des)
  {
  }
};

// 全身数据结构体 假设每只手有13个自由度
struct RobotData
{
  MotorData motors[13 + 14];
};

struct RobotCmd
{
  MotorCmd motors[13 + 14];
};

// 提前标定好的kp kd
double Kp_[13 + 14] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 80.0, 
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 80.0}; // 示例值
double Kd_[13 + 14] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0}; // 示例值

using Vec6 = typename Eigen::Matrix<double, 6, 1>;
// 6x6 Matrix
using Mat6 = typename Eigen::Matrix<double, 6, 6>;

Vec6 calcQuinticCoeffs(const double q0, const double v0, const double a0, 
                       const double qf, const double vf, const double af, double T)
{
  Vec6 coeffs, states;
  Mat6 M;
  M << 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 1, T, pow(T, 2), pow(T, 3), pow(T, 4), pow(T, 5), 0, 1,
      2 * T, 3 * pow(T, 2), 4 * pow(T, 3), 5 * pow(T, 4), 0, 0, 2, 6 * T, 12 * pow(T, 2), 20 * pow(T, 3);
  states << q0, v0, a0, qf, vf, af;
  coeffs = M.inverse() * states;
  return coeffs;
}

class ElmoControlNode : public rclcpp::Node
{
public:
  ElmoControlNode() : Node("elmo_control_node")
  {
    this->declare_parameter<bool>("dummy", true);
    this->declare_parameter<std::string>("hybrid_joint_cmd_topic", "/hybrid_joint_cmd_elmo");
    this->declare_parameter<std::string>("joint_state_topic", "/joint_states_body");
    this->declare_parameter<std::string>("ethercat_config_file",
                                         "src/libertech_humanoid/libertech_ethercat_driver/config/arm.yaml");

    std::string hybrid_joint_cmd_topic;
    this->get_parameter("hybrid_joint_cmd_topic", hybrid_joint_cmd_topic);
    std::string joint_state_topic;
    this->get_parameter("joint_state_topic", joint_state_topic);
    this->get_parameter("ethercat_config_file", ethercat_config_file_);
    std::cout << "ethercat_config_file: " << ethercat_config_file_ << std::endl;
    this->get_parameter("dummy", dummy_);
    std::cout << "elmo dummy: " << dummy_ << std::endl;
    
    bool flag = setupEtherCAT();

    // 使用自定义消息类型,接收hybrid joint state
    hybrid_joint_cmd_subscriber_ = this->create_subscription<libertech_interface_msgs::msg::HybridJointState>(
        hybrid_joint_cmd_topic, 10, std::bind(&ElmoControlNode::hybridJointCmdCallback, this, std::placeholders::_1));

    // 初始化发布器
    joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(joint_state_topic, 10);

    // ethecat 定时器（/1000Hz）
    timer_ = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&ElmoControlNode::timerCallback, this));

    RCLCPP_INFO(this->get_logger(), "Elmo control node has been started.");
  }

private:
  void hybridJointCmdCallback(const libertech_interface_msgs::msg::HybridJointState::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mtx_commands_);

    for (int i = 0; i < msg->pos.size(); i++)
    {
      // // Calculate the difference between the current and the previous position commands
      // double position_difference = std::abs(msg->pos[i] - commands_.motors[i].pos_des);

      // // TODO: check
      // // Check if the position difference exceeds 30 degrees (approximately 0.5236 radians)
      // if (position_difference > 0.5236)
      // {
      //   RCLCPP_WARN(this->get_logger(),
      //               "Position command difference too large for joint %d: %.2f degrees. Command ignored.", i,
      //               position_difference * 180.0 / M_PI);
      //   continue;  // Skip updating this joint's command
      // }

      // 完全按照ethercatDevices顺序
      // Apply the new command if the difference is within the safe range
      commands_.motors[i].pos_des = msg->pos[i];
      commands_.motors[i].vel_des = msg->vel[i];
      commands_.motors[i].tau_des = msg->tau[i];
      commands_.motors[i].kp_des = msg->kp[i];
      commands_.motors[i].kd_des = msg->kd[i];
      // std::cout << msg->name[i] << " msg: " <<msg->pos[i]<< std::endl;

      commands_.motors[i].kp_des = Kp_[i];
      commands_.motors[i].kd_des = Kd_[i];
    }
  }

  void timerCallback()
  {
    sensor_msgs::msg::JointState joint_state_msg;

    // 设置 JointState 消息的头
    joint_state_msg.header.stamp = this->get_clock()->now();
    joint_state_msg.header.frame_id = "base_link";  // 根据实际情况设置 frame_id

    // 设置关节名字, TODO: 需要腰吗，加入腰的关节会影响吗
    joint_state_msg.name = {
      "waist_yaw",
      "leg_left_HTAA",
      "leg_left_HAA",
      "leg_left_HFE",
      "leg_left_KFE",
      "leg_left_AAA",
      "leg_left_AFE",
      "leg_right_HTAA",
      "leg_right_HAA",
      "leg_right_HFE",
      "leg_right_KFE",
      "leg_right_AAA",
      "leg_right_AFE",
      "arm_left_shoulder_pitch_joint",
      "arm_left_shoulder_roll_joint",
      "arm_left_shoulder_yaw_joint",
      "arm_left_elbow_pitch_joint",
      "arm_left_elbow_roll_joint",
      "arm_left_wrist_pitch_joint",
      "arm_left_wrist_yaw_joint",
      "arm_right_shoulder_pitch_joint",
      "arm_right_shoulder_roll_joint",
      "arm_right_shoulder_yaw_joint",
      "arm_right_elbow_pitch_joint",
      "arm_right_elbow_roll_joint",
      "arm_right_wrist_pitch_joint",
      "arm_right_wrist_yaw_joint",
    };

    // 设置位置、速度、力矩
    joint_state_msg.position.resize(joint_state_msg.name.size(), 0.0);
    joint_state_msg.velocity.resize(joint_state_msg.name.size(), 0.0);
    joint_state_msg.effort.resize(joint_state_msg.name.size(), 0.0);

    {
      std::lock_guard<std::mutex> lock(mtx_data_);
      for (size_t i = 0; i < joint_state_msg.name.size(); ++i)
      {
        joint_state_msg.position[i] = data_.motors[i].pos;
        joint_state_msg.velocity[i] = data_.motors[i].vel;
        joint_state_msg.effort[i] = data_.motors[i].tau;
      }
    }

    // 发布 JointState 消息
    joint_state_publisher_->publish(joint_state_msg);
    // RCLCPP_INFO(this->get_logger(), "Published joint states to /joint_states_body");
  }

  bool setupEtherCAT()
  {
    if (dummy_)
    {
      ethercat_thread_ = std::thread([&]() {
        while (!abrt_)
        {
          for (size_t i = 0; i < sizeof(data_.motors) / sizeof(data_.motors[0]); ++i)
          {
            data_.motors[i].pos = commands_.motors[i].pos_des;
            data_.motors[i].vel = commands_.motors[i].vel_des;
            data_.motors[i].tau = commands_.motors[i].tau_des;
          }
        }
      });
    }
    else
    {
      // a new EthercatDeviceConfigurator object (path to setup.yaml as constructor argument)
      std::cout << ethercat_config_file_ << std::endl;
      configurator_ = std::make_shared<EthercatDeviceConfigurator>(ethercat_config_file_);

      /*
      ** Start all masters.
      ** There is exactly one bus per master which is also started.
      ** All online (i.e. SDO) configuration is done during this call.
      ** The EtherCAT interface is active afterwards, all drives are in Operational
      ** EtherCAT state and PDO communication may begin.
      */
      for (auto& master : configurator_->getMasters())
      {
        if (!master->startup())
        {
          RCLCPP_ERROR(this->get_logger(), "Startup not successful.");
          return EXIT_FAILURE;
        }
      }

      // Start the PDO loop in a new thread.
      // ethercat_thread = std::thread(&setupEtherCAT);
      ethercat_thread_ = std::thread([&]() {
        bool rtSuccess = true;

        for (const auto& master : configurator_->getMasters())
        {
          rtSuccess &= master->setRealtimePriority(49);
        }
        std::cout << "Setting RT Priority: " << (rtSuccess ? "successful." : "not successful. Check user privileges.")
                  << std::endl;

        // Flag to set the drive state for the elmos on first startup
        bool elmoEnabledAfterStartup = false;
        // bool ActuatorPosOffset = true;

        while (!abrt_)
        {
          /*
          ** Update each master.
          ** This sends tha last staged commands and reads the latest readings over EtherCAT.
          ** The StandaloneEnforceRate update mode is used.
          ** This means that average update rate will be close to the target rate (if possible).
          */
          for (const auto& master : configurator_->getMasters())
          {
            master->update(ecat_master::UpdateMode::StandaloneEnforceRate);  // TODO fix the rate compensation (Elmo
                                                                             // reliability problem)!!
          }

          //  Do things with the attached devices.  your lowlevel control input / measurement logic goes here. *
          //  Different logic can be implemented for each device.
          for (const auto& slave : configurator_->getSlaves())
          {
            // Elmo
            if (configurator_->getInfoForSlave(slave).type == EthercatDeviceConfigurator::EthercatSlaveType::Elmo)
            {
              std::shared_ptr<elmo::Elmo> elmo_slave_ptr = std::dynamic_pointer_cast<elmo::Elmo>(slave);
              {
                if (!elmoEnabledAfterStartup)
                  // Set elmos to operation enabled state, do not block the call!
                  elmo_slave_ptr->setDriveStateViaPdo(elmo::DriveState::OperationEnabled, false);

                auto elmo_name = elmo_slave_ptr->getName();
                
                // if (elmo_name == "RightArmWristYawElmo") { // RightArmWristYawElmo RightArmElbowRollElmo
                  
                // }
                // else{
                //   continue;
                // }

                int elmo_id = ethercatDevices[elmo_name];
                auto reading = elmo_slave_ptr->getReading();

                std::cout << "Elmo '" << elmo_name << "': "
                          << "position: " << reading.getActualPosition() << " rad"
                          << " Velocity: " << reading.getActualVelocity() << " rad/s"
                          << " torque:" << reading.getActualTorque() << " A\n";
                // std::cout << " ================================== " << std::endl;
                auto pos_now = reading.getActualPosition();
                auto vel_now = reading.getActualVelocity();

                {
                  std::lock_guard<std::mutex> lock(mtx_data_);

                  data_.motors[elmo_id].pos = reading.getActualPosition();
                  data_.motors[elmo_id].vel = reading.getActualVelocity();
                  data_.motors[elmo_id].tau = reading.getActualTorque();

                  // TODO: check
                  // if (data_.motors[elmo_id].vel > 3)
                  // {
                  //   RCLCPP_ERROR(this->get_logger(),
                  //                "Velocity too large for joint %d: %.2f degrees. Command ignored.",
                  //                elmo_id, data_.motors[elmo_id].vel * 180.0 / M_PI);

                  //   abrt_ = true;
                  //   break;  // 一旦满足条件，直接跳出循环
                  // }
                }

                // set commands if we can
                if (elmo_slave_ptr->lastPdoStateChangeSuccessful() &&
                    elmo_slave_ptr->getReading().getDriveState() == elmo::DriveState::OperationEnabled)
                {
                  std::lock_guard<std::mutex> lock(mtx_commands_);

                  elmo::Command command;
                  if (elmo_id < 13)  // 腿发力
                  {
                    command.setModeOfOperation(elmo::ModeOfOperationEnum::CyclicSynchronousTorqueMode);
                    command.setTargetTorque(0.0);
                    // command.setTargetTorque(data_[leg_index * 5 + joint_index].tau_des_);
                    //   std::cout << "leg:" << leg_index
                    //             << " Joint " << joint_index
                    //             << " KpDes: " << data_[leg_index * 5 + joint_index].kp_
                    //             << " KdDes: " << data_[leg_index * 5 + joint_index].kd_
                    //             << " PosDes: " << jointData_[leg_index * 5 + joint_index].pos_des_
                    //             << " Pos: " << jointData_[leg_index * 5 + joint_index].pos_
                    //             << " TauDes: " << data_[leg_index * 5 + joint_index].tau_des_ << "\n";
                  }
                  else
                  {
                    // 电流环
                    // command.setModeOfOperation(elmo::ModeOfOperationEnum::CyclicSynchronousTorqueMode);
                    // double tau = commands_.motors[elmo_id].kp_des * (commands_.motors[elmo_id].pos_des - pos_now) +
                    //                  commands_.motors[elmo_id].kd_des * (commands_.motors[elmo_id].vel_des - vel_now) + 
                    //                  commands_.motors[elmo_id].tau_des;
                    // std::cout << "tau:" << tau << std::endl;
                    // // std::cout << "elmo_id:" << elmo_id
                    // //           << " pos_des " << commands_.motors[elmo_id].pos_des
                    // //           << " vel_des " << commands_.motors[elmo_id].vel_des
                    // //           << " tau_des " << commands_.motors[elmo_id].tau_des
                    // //           << " kp_des " << commands_.motors[elmo_id].kp_des
                    // //           << " kd_des " << commands_.motors[elmo_id].kd_des << std::endl;
                    // command.setTargetTorque(tau);

                    // 位置环
                    // command.setModeOfOperation(elmo::ModeOfOperationEnum::CyclicSynchronousPositionMode); //ProfiledPositionMode CyclicSynchronousPositionMode
                    // command.setTargetPosition(commands_.motors[elmo_id].pos_des);

                    // 速度环
                    Vec6 coeffs = calcQuinticCoeffs(pos_now, 0., 0., commands_.motors[elmo_id].pos_des, 0., 0., 0.035);
                    double t = 0.001;
                    // double q = coeffs(0) + coeffs(1) * t + coeffs(2) * pow(t, 2) + coeffs(3) * pow(t, 3) + coeffs(4) * pow(t, 4) + coeffs(5) * pow(t, 5);
                    double dq = coeffs(1) + 2 * coeffs(2) * t + 3 * coeffs(3) * pow(t, 2) + 4 * coeffs(4) * pow(t, 3) + 5 * coeffs(5) * pow(t, 4);
                    std::cout << dq << std::endl;
                    command.setModeOfOperation(elmo::ModeOfOperationEnum::CyclicSynchronousVelocityMode);

                    // auto vel = 1 * (commands_.motors[elmo_id].pos_des - pos_now) - 0.2 * vel_now;
                    command.setTargetVelocity(dq);

                    // command.setTargetTorque(0.5);
                    // std::cout << elmo_name << " cmd: " << commands_.motors[elmo_id].pos_des<< std::endl;

                    // std::cout << "modeOfOperation_" << elmo_slave_ptr->get << std::endl;
                  }

                  elmo_slave_ptr->stageCommand(command);
                }
                else
                {
                  MELO_WARN_STREAM("Elmoxx '" << elmo_slave_ptr->getName()
                                              << "': " << elmo_slave_ptr->getReading().getDriveState());
                  std::cout << "stateChange: " << elmo_slave_ptr->lastPdoStateChangeSuccessful() << "\n";
                  // elmo_slave_ptr->setDriveStateViaPdo(elmo::DriveState::OperationEnabled, false);
                }
              }
            }
          }
          elmoEnabledAfterStartup = true;
          // ActuatorPosOffset = false;
        }
      });

      // Wait for a few PDO cycles to pass.
      std::this_thread::sleep_for(std::chrono::milliseconds(100));

      for (auto& slave : configurator_->getSlaves())
      {
        std::cout << " " << slave->getName() << ": " << slave->getAddress() << std::endl;
      }
    }

    return true;
  }

  rclcpp::Subscription<libertech_interface_msgs::msg::HybridJointState>::SharedPtr hybrid_joint_cmd_subscriber_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  bool dummy_;
  //******* EtherCAT *******/
  bool abrt_ = false;
  EthercatDeviceConfigurator::SharedPtr configurator_;
  std::string ethercat_config_file_;
  std::thread ethercat_thread_;

  std::mutex mtx_data_;
  std::mutex mtx_commands_;

  RobotData data_;
  RobotCmd commands_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ElmoControlNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
