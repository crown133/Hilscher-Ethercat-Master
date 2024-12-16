#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "ethercat_device_configurator/EthercatDeviceConfigurator.hpp"
#include "elmo_ethercat_sdk/Elmo.hpp"
#include <std_msgs/msg/float64.hpp> 


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

class ElmoTestNode : public rclcpp::Node
{
public:
  ElmoTestNode() : Node("elmo_test_node")
  {
    this->declare_parameter<std::string>("recv_topic", "/motor_pos");
    this->declare_parameter<std::string>("ethercat_config_file",
                                         "src/libertech_humanoid/ethercat_master/ethercat_device_configurator/example_config/setup.yaml");

    std::string recv_topic;
    this->get_parameter("recv_topic", recv_topic);

    this->get_parameter("ethercat_config_file", ethercat_config_file_);
    std::cout << "ethercat_config_file: " << ethercat_config_file_ << std::endl;

    bool flag = setupEtherCAT();
    if (!flag) {
      RCLCPP_ERROR(this->get_logger(), "Failed to setup EtherCAT");
    }

    // 使用自定义消息类型,接收hybrid joint state
    cmd_subscriber_ = this->create_subscription<std_msgs::msg::Float64>(
        recv_topic, 10, std::bind(&ElmoTestNode::cmdCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Elmo control node has been started.");
  }

private:
  void cmdCallback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mtx_commands_);

    commands_.pos_des = msg->data;
    // RCLCPP_INFO(this->get_logger(), "Received command: %f", msg->data);
  }

  bool setupEtherCAT()
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

                  data_.pos = reading.getActualPosition();
                  data_.vel = reading.getActualVelocity();
                  data_.tau = reading.getActualTorque();
                }

                // set commands if we can
                if (elmo_slave_ptr->lastPdoStateChangeSuccessful() &&
                    elmo_slave_ptr->getReading().getDriveState() == elmo::DriveState::OperationEnabled)
                {
                  std::lock_guard<std::mutex> lock(mtx_commands_);

                  elmo::Command command;
                  
                  
                  // command.setModeOfOperation(elmo::ModeOfOperationEnum::CyclicSynchronousTorqueMode);
                  // double tau = 1.0 * (commands_.pos_des - pos_now) +
                  //              0.01 * (0.0 - vel_now) + 
                  //              0.0;
                  // command.setTargetTorque(tau);
                  // std::cout << "elmo_id:" << elmo_id
                  //           << " pos_des " << commands_.motors[elmo_id].pos_des
                  //           << " vel_des " << commands_.motors[elmo_id].vel_des
                  //           << " tau_des " << commands_.motors[elmo_id].tau_des
                  //           << " kp_des " << commands_.motors[elmo_id].kp_des
                  //           << " kd_des " << commands_.motors[elmo_id].kd_des << std::endl;
                  // command.setTargetTorque(0.0);

                  command.setModeOfOperation(elmo::ModeOfOperationEnum::CyclicSynchronousPositionMode); //ProfiledPositionMode CyclicSynchronousPositionMode
                  command.setTargetPosition(commands_.pos_des);

                  // command.setModeOfOperation(elmo::ModeOfOperationEnum::CyclicSynchronousVelocityMode);
                  // auto vel = 1 * (commands_.motors[elmo_id].pos_des - pos_now) - 0.2 * vel_now;
                  // command.setTargetVelocity(vel);

                  // command.setTargetTorque(0.5);
                  std::cout << elmo_name << " cmd: " << commands_.pos_des<< std::endl;

                  // std::cout << "modeOfOperation_" << elmo_slave_ptr->get << std::endl;

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

    return true;
  }

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr cmd_subscriber_; 

  //******* EtherCAT *******/
  bool abrt_ = false;
  EthercatDeviceConfigurator::SharedPtr configurator_;
  std::string ethercat_config_file_;
  std::thread ethercat_thread_;

  std::mutex mtx_data_;
  std::mutex mtx_commands_;

  MotorData data_;
  MotorCmd commands_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ElmoTestNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
