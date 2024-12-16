#include "libertech_messaging/elmo_node.h"

ElmoControlNode::ElmoControlNode() : Node("elmo_control_node"), 
    joint_pos_offset_(NUM_MOTORS, 0), 
    actuator_pos_offset_flag_(NUM_MOTORS, true),
    joint_first_flag_(NUM_MOTORS, false),
    joint_pos_integral_error_(NUM_MOTORS, 0)
{
  this->declare_parameter<bool>("dummy", false);
  this->declare_parameter<std::string>("hybrid_joint_cmd_topic", "/hybrid_joint_cmds");
  this->declare_parameter<std::string>("joint_state_topic", "/joint_states_body");
  this->declare_parameter<std::string>("ethercat_config_file",
                                        "src/liberman_hardware/libertech_messaging/config/leg.yaml");

  std::string hybrid_joint_cmd_topic;
  this->get_parameter("hybrid_joint_cmd_topic", hybrid_joint_cmd_topic);
  std::string joint_state_topic;
  this->get_parameter("joint_state_topic", joint_state_topic);
  this->get_parameter("ethercat_config_file", ethercat_config_file_);
  std::cout << "ethercat_config_file: " << ethercat_config_file_ << std::endl;
  this->get_parameter("dummy", dummy_);
  std::cout << "elmo dummy: " << dummy_ << std::endl;

  // 
  joint_pos_init_cmd_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
    0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  joint_pos_min_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
    -1.22, -1.57, -2.96, 0.00, -1.57, -1.30, -0.87,
    -1.22, -1.57, -2.96, 0.00, -1.57, -1.30, -0.87};
  joint_pos_max_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
    2.44, 1.39, -0.17, 1.83, 1.57, 1.30, 0.87,
    2.44, 1.39, -0.17, 1.83, 1.57, 1.30, 0.87};
  joint_vel_max_ = {0.0, 9.0, 9.0, 9.0, 9.0, 9.0, 9.0, 9.0, 9.0, 9.0, 9.0, 9.0, 9.0, 
    3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0,
    3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0};

  // 声明参数
  this->declare_parameter("VKP", 2.0);
  this->declare_parameter("VKD", 0.0);
  this->declare_parameter("VKI", 0.0);
  this->declare_parameter("VMAX", 2.0);

  // 注册动态参数回调
  parameter_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&ElmoControlNode::param_callback, this, std::placeholders::_1));

  // 获取初始参数值
  this->get_parameter("VKP", vkp_);
  this->get_parameter("VKD", vkd_);
  this->get_parameter("VKI", vki_);
  this->get_parameter("VMAX", vmax_);

  if(setupEtherCAT())
  RCLCPP_INFO(this->get_logger(), "EtherCAT has been setup.");

  // 使用自定义消息类型,接收hybrid joint state
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
  hybrid_joint_cmd_subscriber_ = this->create_subscription<libertech_interface_msgs::msg::HybridJointState>(
      hybrid_joint_cmd_topic, qos, std::bind(&ElmoControlNode::hybridJointCmdCallback, this, std::placeholders::_1));

  // 初始化发布器
  joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(joint_state_topic, qos);

  // ethecat 定时器（/1000Hz）
  timer_ = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&ElmoControlNode::timerCallback, this));

  init_joint_server_ = this->create_service<libertech_interface_msgs::srv::InitHardware>("init_robot_hardware",
              std::bind(&ElmoControlNode::init_joint_callback, this, std::placeholders::_1, std::placeholders::_2));
  
  RCLCPP_INFO(this->get_logger(), "Elmo control node has been started.");
}

ElmoControlNode::~ElmoControlNode()
{
    /*
    ** Pre shutdown procedure.
    ** The devices execute procedures (e.g. state changes) that are necessary for a
    ** proper shutdown and that must be done with PDO communication.
    ** The communication update loop (i.e. PDO loop) continues to run!
    ** You might thus want to implement some logic that stages zero torque / velocity commands
    ** or simliar safety measures at this point using e.g. atomic variables and checking them
    ** in the communication update loop.
     */
    for(const auto & master: configurator_->getMasters())
    {
        master->preShutdown();
    }

    // stop the PDO communication at the next update of the communication loop
    abrt_ = true;
    if(ethercat_thread_.joinable())
    ethercat_thread_.join();

    /*
    ** Completely halt the EtherCAT communication.
    ** No online communication is possible afterwards, including SDOs.
     */
    for(const auto & master: configurator_->getMasters())
    {
        master->shutdown();
    }

    // Exit this executable
    std::cout << "Shutdown" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    exit(0);
}

rcl_interfaces::msg::SetParametersResult ElmoControlNode::param_callback(const std::vector<rclcpp::Parameter> &parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto &param : parameters)
    {
        if (param.get_name() == "VKP")
        {
            vkp_ = param.as_double();
            RCLCPP_INFO(this->get_logger(), "Updated VKP: %f", vkp_);
        }
        else if (param.get_name() == "VKD")
        {
            vkd_ = param.as_double();
            RCLCPP_INFO(this->get_logger(), "Updated VMAX: %f", vkd_);
        }
        else if (param.get_name() == "VKI")
        {
            vki_ = param.as_double();
            RCLCPP_INFO(this->get_logger(), "Updated VKI: %f", vki_);
        }
        else if (param.get_name() == "VMAX")
        {
            vmax_ = param.as_double();
            RCLCPP_INFO(this->get_logger(), "Updated VMAX: %f", vmax_);
        }
    }
    return result;
}

void ElmoControlNode::hybridJointCmdCallback(const libertech_interface_msgs::msg::HybridJointState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mtx_commands_);

  for (int i = 0; i < msg->pos.size(); i++)
  {
    // 完全按照ethercatDevices顺序
    // Apply the new command if the difference is within the safe range
    commands_.motors[i].pos_des = msg->pos[i];
    commands_.motors[i].vel_des = msg->vel[i];
    commands_.motors[i].ff_des = msg->tau[i];
    commands_.motors[i].kp_des = msg->kp[i];
    commands_.motors[i].kd_des = msg->kd[i];
    // std::cout << msg->actuators_name[i] << " msg: " <<msg->pos[i]<< std::endl;
  }
}

void ElmoControlNode::timerCallback()
{
  sensor_msgs::msg::JointState joint_state_msg;

  // 设置 JointState 消息的头
  joint_state_msg.header.stamp = this->get_clock()->now();
  joint_state_msg.header.frame_id = "base_link";  // 根据实际情况设置 frame_id

  // 设置关节名字, TODO: 需要腰吗，加入腰的关节会影响吗
  joint_state_msg.name = {
    // "waist_yaw",
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
    // "arm_left_shoulder_pitch_joint",
    // "arm_left_shoulder_roll_joint",
    // "arm_left_shoulder_yaw_joint",
    // "arm_left_elbow_pitch_joint",
    // "arm_left_elbow_roll_joint",
    // "arm_left_wrist_pitch_joint",
    // "arm_left_wrist_yaw_joint",
    // "arm_right_shoulder_pitch_joint",
    // "arm_right_shoulder_roll_joint",
    // "arm_right_shoulder_yaw_joint",
    // "arm_right_elbow_pitch_joint",
    // "arm_right_elbow_roll_joint",
    // "arm_right_wrist_pitch_joint",
    // "arm_right_wrist_yaw_joint",
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

void ElmoControlNode::init_joint_callback(
    const std::shared_ptr<libertech_interface_msgs::srv::InitHardware::Request> request,
    std::shared_ptr<libertech_interface_msgs::srv::InitHardware::Response> response) 
{
  if (request->header.frame_id != "liberman") {
    RCLCPP_ERROR(this->get_logger(), "reset request is not for liberman");
    response->is_success = false;
  } else {
    while(!std::all_of(joint_first_flag_.begin(), joint_first_flag_.begin()+LEG_MOTORS+WAIST_MOTORS, [](bool v) {return v;}))
    { }
    response->is_success = true;
    RCLCPP_INFO(this->get_logger(), "robot hardware init finished...");
  }
}

bool ElmoControlNode::setupEtherCAT()
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
        }
        usleep(900);
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
        rtSuccess &= master->setRealtimePriority(80);
      }
      std::cout << "Setting RT Priority: " << (rtSuccess ? "successful." : "not successful. Check user privileges.")
                << std::endl;

      // // Flag to set the drive state for the elmos on first startup
      // bool elmoEnabledAfterStartup = false;
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

              int elmo_id = ethercatDevices[elmo_name];
              auto reading = elmo_slave_ptr->getReading();

              {
                // std::lock_guard<std::mutex> lock(mtx_data_);
                if(elmo_id < LEG_MOTORS + WAIST_MOTORS)
                {
                  if(actuator_pos_offset_flag_[elmo_id] && elmo_slave_ptr->getReading().getDriveState() == elmo::DriveState::OperationEnabled)
                  {
                    if(reading.getAuxiliaryPosition() > 1.3*M_PI) {
                      joint_pos_offset_[elmo_id] =  reading.getActualPosition() - (reading.getAuxiliaryPosition() - 2*M_PI);
                      MELO_WARN_STREAM(elmo_slave_ptr->getName() << " out of range: " 
                                                        << reading.getAuxiliaryPosition());
                    }
                    else if(reading.getAuxiliaryPosition() < -1.3*M_PI) {
                      joint_pos_offset_[elmo_id] =  reading.getActualPosition() - (reading.getAuxiliaryPosition() + 2*M_PI);
                      MELO_WARN_STREAM(elmo_slave_ptr->getName() << " out of range: " 
                                                        << reading.getAuxiliaryPosition());
                    }
                    else {
                      joint_pos_offset_[elmo_id] =  reading.getActualPosition() - reading.getAuxiliaryPosition();
                    }
                    actuator_pos_offset_flag_[elmo_id] = false;
                    MELO_INFO_STREAM(elmo_slave_ptr->getName() << " has set offset value");
                  }

                  data_.motors[elmo_id].pos = reading.getActualPosition() - joint_pos_offset_[elmo_id];
                  data_.motors[elmo_id].vel = reading.getActualVelocity();
                  data_.motors[elmo_id].tau = reading.getActualTorque();
                } else
                {
                  data_.motors[elmo_id].pos = reading.getActualPosition();
                  data_.motors[elmo_id].vel = reading.getActualVelocity();
                  data_.motors[elmo_id].tau = reading.getActualTorque();
                }
                
              }

              // set commands if we can
              if (elmo_slave_ptr->lastPdoStateChangeSuccessful() &&
                  elmo_slave_ptr->getReading().getDriveState() == elmo::DriveState::OperationEnabled)
              {
                // std::lock_guard<std::mutex> lock(mtx_commands_);
                if (!joint_first_flag_[elmo_id]) {
                  commands_.motors[elmo_id].pos_des = data_.motors[elmo_id].pos;
                  joint_first_flag_[elmo_id] = true;
                }

                elmo::Command command;
                if (elmo_id < LEG_MOTORS + WAIST_MOTORS)  // 腿发力
                {
                  command.setModeOfOperation(elmo::ModeOfOperationEnum::ProfiledTorqueMode);
                  command.setTorqueSlopeRaw(10000000);
                  commands_.motors[elmo_id].tau_old = 0.3*(commands_.motors[elmo_id].tau_old) +
                                                      0.7*(commands_.motors[elmo_id].kp_des * (commands_.motors[elmo_id].pos_des - data_.motors[elmo_id].pos) +
                                                            commands_.motors[elmo_id].kd_des * (commands_.motors[elmo_id].vel_des - data_.motors[elmo_id].vel));
                  commands_.motors[elmo_id].tau_des = commands_.motors[elmo_id].tau_old + commands_.motors[elmo_id].ff_des;
                  if(elmo_id == 4 || elmo_id ==5 || elmo_id == 10 || elmo_id ==11)
                  {
                  command.setTargetTorque(commands_.motors[elmo_id].tau_des);
                    // std::cout << elmo_id
                    //           << " targetTorque:" <<  commands_.motors[elmo_id].tau_des
                    //           << " PosDes:" << commands_.motors[elmo_id].pos_des
                    //           << " VelDes:" << commands_.motors[elmo_id].vel_des
                    //           << " TorDes:" <<  commands_.motors[elmo_id].ff_des
                    //           << " Kp:" << commands_.motors[elmo_id].kp_des
                    //           << " Kd:" << commands_.motors[elmo_id].kd_des << std::endl;
                  }
                  else
                  command.setTargetTorque(0);
                }
                else
                {
                  // 位置环
                  // command.setModeOfOperation(elmo::ModeOfOperationEnum::CyclicSynchronousPositionMode); //ProfiledPositionMode CyclicSynchronousPositionMode
                  // command.setTargetPosition(commands_.motors[elmo_id].pos_des);

                  // 速度环
                  command.setModeOfOperation(elmo::ModeOfOperationEnum::CyclicSynchronousVelocityMode);
                  // double pos_des = commands_.motors[elmo_id].pos_des;
                  // 限位，TODO： 目前只给臂加了
                  double pos_des = std::clamp(commands_.motors[elmo_id].pos_des, joint_pos_min_[elmo_id], joint_pos_max_[elmo_id]);
                  
                  double error = pos_des - data_.motors[elmo_id].pos;
                  joint_pos_integral_error_[elmo_id] += error * dt;
                  joint_pos_integral_error_[elmo_id] = std::clamp(joint_pos_integral_error_[elmo_id], -50.0, 50.0);
                  double vel = vkp_ * error + vki_ * joint_pos_integral_error_[elmo_id] + commands_.motors[elmo_id].vel_des - vkd_ * data_.motors[elmo_id].vel;
                  vel = std::clamp(vel, -vmax_, vmax_);
                  command.setTargetVelocity(0);
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


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ElmoControlNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
