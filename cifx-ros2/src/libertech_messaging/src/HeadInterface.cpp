#include "libertech_messaging/HeadInterface.hpp"
#include <chrono>

HeadInterface::HeadInterface()
{
}

HeadInterface::~HeadInterface()
{
  pause();
  usleep(100000);

  // disableMotor();
  // usleep(100000);

  // 除收发函数外，其它的函数调用前后，最好加个毫秒级的延时，即不影响程序的运行，又可以让USBCAN设备有充分的时间处理指令。
  usleep(100000);                   // 延时100ms。
  // VCI_ResetCAN(VCI_USBCAN2, 0, 0);  // 复位CAN1/CAN2通道。
  usleep(100000);                   // 延时100ms。
}

int HeadInterface::init(int cid)
{
  cid_ = cid;

  if (cid_ == 0 || cid_ == 1)
  {

  }
  else{
    return 1;
  }


  printf(">>init\n");
  // 初始化USBCAN设备
  vciNum_ = VCI_FindUsbDevice2(vciPInfoAll_);
  printf(">>USBCAN DEVICE NUM:");
  printf("%d", vciNum_);
  printf(" PCS");

  for (int i = 0; i < vciNum_; i++)
  {
    printf("Device:");
    printf("%d", i);
    printf("\n");
    printf(">>Get VCI_ReadBoardInfo success!\n");

    printf(">>Serial_Num:%c", vciPInfoAll_[i].str_Serial_Num[0]);
    printf("%c", vciPInfoAll_[i].str_Serial_Num[1]);
    printf("%c", vciPInfoAll_[i].str_Serial_Num[2]);
    printf("%c", vciPInfoAll_[i].str_Serial_Num[3]);
    printf("%c", vciPInfoAll_[i].str_Serial_Num[4]);
    printf("%c", vciPInfoAll_[i].str_Serial_Num[5]);
    printf("%c", vciPInfoAll_[i].str_Serial_Num[6]);
    printf("%c", vciPInfoAll_[i].str_Serial_Num[7]);
    printf("%c", vciPInfoAll_[i].str_Serial_Num[8]);
    printf("%c", vciPInfoAll_[i].str_Serial_Num[9]);
    printf("%c", vciPInfoAll_[i].str_Serial_Num[10]);
    printf("%c", vciPInfoAll_[i].str_Serial_Num[11]);
    printf("%c", vciPInfoAll_[i].str_Serial_Num[12]);
    printf("%c", vciPInfoAll_[i].str_Serial_Num[13]);
    printf("%c", vciPInfoAll_[i].str_Serial_Num[14]);
    printf("%c", vciPInfoAll_[i].str_Serial_Num[15]);
    printf("%c", vciPInfoAll_[i].str_Serial_Num[16]);
    printf("%c", vciPInfoAll_[i].str_Serial_Num[17]);
    printf("%c", vciPInfoAll_[i].str_Serial_Num[18]);
    printf("%c", vciPInfoAll_[i].str_Serial_Num[19]);
    printf("\n");

    printf(">>hw_Type:%c", vciPInfoAll_[i].str_hw_Type[0]);
    printf("%c", vciPInfoAll_[i].str_hw_Type[1]);
    printf("%c", vciPInfoAll_[i].str_hw_Type[2]);
    printf("%c", vciPInfoAll_[i].str_hw_Type[3]);
    printf("%c", vciPInfoAll_[i].str_hw_Type[4]);
    printf("%c", vciPInfoAll_[i].str_hw_Type[5]);
    printf("%c", vciPInfoAll_[i].str_hw_Type[6]);
    printf("%c", vciPInfoAll_[i].str_hw_Type[7]);
    printf("%c", vciPInfoAll_[i].str_hw_Type[8]);
    printf("%c", vciPInfoAll_[i].str_hw_Type[9]);
    printf("\n");

    printf(">>Firmware Version:V");
    printf("%x", (vciPInfoAll_[i].fw_Version & 0xF00) >> 8);
    printf(".");
    printf("%x", (vciPInfoAll_[i].fw_Version & 0xF0) >> 4);
    printf("%x", vciPInfoAll_[i].fw_Version & 0xF);
    printf("\n");

    // 打开第一个USBCAN，理论上只有一个
    if (VCI_OpenDevice(VCI_USBCAN2, 0, 0) == 1)  // 打开设备
    {
      printf(">>open deivce success!\n");  // 打开设备成功
    }
    else
    {
      printf(">>open deivce error!\n");
      // exit(1);
    }
    if (VCI_ReadBoardInfo(VCI_USBCAN2, 0, &vciPInfo_) == 1)  // 读取设备序列号、版本等信息。
    {
      printf(">>Get VCI_ReadBoardInfo success!\n");

      printf(">>Serial_Num:%c", vciPInfo_.str_Serial_Num[0]);
      printf("%c", vciPInfo_.str_Serial_Num[1]);
      printf("%c", vciPInfo_.str_Serial_Num[2]);
      printf("%c", vciPInfo_.str_Serial_Num[3]);
      printf("%c", vciPInfo_.str_Serial_Num[4]);
      printf("%c", vciPInfo_.str_Serial_Num[5]);
      printf("%c", vciPInfo_.str_Serial_Num[6]);
      printf("%c", vciPInfo_.str_Serial_Num[7]);
      printf("%c", vciPInfo_.str_Serial_Num[8]);
      printf("%c", vciPInfo_.str_Serial_Num[9]);
      printf("%c", vciPInfo_.str_Serial_Num[10]);
      printf("%c", vciPInfo_.str_Serial_Num[11]);
      printf("%c", vciPInfo_.str_Serial_Num[12]);
      printf("%c", vciPInfo_.str_Serial_Num[13]);
      printf("%c", vciPInfo_.str_Serial_Num[14]);
      printf("%c", vciPInfo_.str_Serial_Num[15]);
      printf("%c", vciPInfo_.str_Serial_Num[16]);
      printf("%c", vciPInfo_.str_Serial_Num[17]);
      printf("%c", vciPInfo_.str_Serial_Num[18]);
      printf("%c", vciPInfo_.str_Serial_Num[19]);
      printf("\n");

      printf(">>hw_Type:%c", vciPInfo_.str_hw_Type[0]);
      printf("%c", vciPInfo_.str_hw_Type[1]);
      printf("%c", vciPInfo_.str_hw_Type[2]);
      printf("%c", vciPInfo_.str_hw_Type[3]);
      printf("%c", vciPInfo_.str_hw_Type[4]);
      printf("%c", vciPInfo_.str_hw_Type[5]);
      printf("%c", vciPInfo_.str_hw_Type[6]);
      printf("%c", vciPInfo_.str_hw_Type[7]);
      printf("%c", vciPInfo_.str_hw_Type[8]);
      printf("%c", vciPInfo_.str_hw_Type[9]);
      printf("\n");

      printf(">>Firmware Version:V");
      printf("%x", (vciPInfo_.fw_Version & 0xF00) >> 8);
      printf(".");
      printf("%x", (vciPInfo_.fw_Version & 0xF0) >> 4);
      printf("%x", vciPInfo_.fw_Version & 0xF);
      printf("\n");
    }
    else
    {
      printf(">>Get VCI_ReadBoardInfo error!\n");
      // exit(1);
    }
  }

  vciConfig_.AccCode = 0;
  vciConfig_.AccMask = 0xFFFFFFFF;
  vciConfig_.Filter = 1;     // 接收所有帧
  vciConfig_.Timing0 = 0x00; /*波特率1000k*/
  vciConfig_.Timing1 = 0x14;
  vciConfig_.Mode = 0;  // 正常模式

  if (VCI_InitCAN(VCI_USBCAN2, 0, cid_, &vciConfig_) != 1)
  {
    printf(">>Init CAN%d error\n", cid_);
    VCI_CloseDevice(VCI_USBCAN2, 0);
  }

  if (VCI_StartCAN(VCI_USBCAN2, 0, cid_) != 1)
  {
    printf(">>Start CAN%d error\n", cid_);
    VCI_CloseDevice(VCI_USBCAN2, 0);
  }

  usleep(100000);

  enableMotor();
  usleep(1000000);
  
  runSendReceiveThread();
  usleep(100000);

  return 1;
}

int HeadInterface::doSendReceive()
{
  auto start = std::chrono::high_resolution_clock::now();

  VCI_CAN_OBJ send[Joint_Num];
  for (int i = 0; i < Joint_Num; i++)
  {
    int id = i + 1;
    uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tau_tmp;

    {
      std::lock_guard<std::mutex> lock(mtx_);
      pos_tmp = float_to_uint(motor_cmd_[i].pos_d, P_MIN, P_MAX, 16);
      vel_tmp = float_to_uint(motor_cmd_[i].vel_d, V_MIN, V_MAX, 12);
      kp_tmp = float_to_uint(motor_cmd_[i].Kp, KP_MIN, KP_MAX, 12);
      kd_tmp = float_to_uint(motor_cmd_[i].Kd, KD_MIN, KD_MAX, 12);
      tau_tmp = float_to_uint(motor_cmd_[i].tau_d, T_MIN, T_MAX, 12);
    }

    send[i].SendType = 0;
    send[i].RemoteFlag = 0;
    send[i].ExternFlag = 0;
    send[i].ID = id;
    send[i].DataLen = 8;
    send[i].Data[0] = (pos_tmp >> 8);
    send[i].Data[1] = (pos_tmp & 0xFF);
    send[i].Data[2] = ((vel_tmp >> 4) & 0xFF);
    send[i].Data[3] = ((((vel_tmp & 0xF) << 4) & 0xFF) | ((kp_tmp >> 8) & 0xFF));
    send[i].Data[4] = (kp_tmp & 0xFF);
    send[i].Data[5] = ((kd_tmp >> 4) & 0xFF);
    send[i].Data[6] = ((((kd_tmp & 0xF) << 4) & 0xFF) | ((tau_tmp >> 8) & 0xFF));
    send[i].Data[7] = (tau_tmp & 0xFF);
  }
  if (VCI_Transmit(VCI_USBCAN2, 0, cid_, send, Joint_Num) != 1)
  {
    // return 0; // 发送失败，返回0
  }

  usleep(200);  // TODO: TEST 100

  VCI_CAN_OBJ rec[3000];
  int reclen = 0;

  if ((reclen = VCI_Receive(VCI_USBCAN2, 0, cid_, rec, 3000, 100)) > 0)  // 调用接收函数，如果有数据，进行数据处理显示。
  {
    for (int j = 0; j < reclen; j++)
    {
      // printf("CAN%d RX ID:0x%08X ", idCAN + 1, rec[j].ID);  // ID
      auto data = rec[j].Data;

      int id = (uint16_t)(data[0]) & 0x0F;

      motor_state_[id - 1].state = (uint16_t)(data[0]) >> 4;

      uint16_t pos_hex = (uint16_t)(data[2] | (data[1] << 8));
      uint16_t vel_hex = (uint16_t)((data[4] >> 4) | (data[3] << 4));
      uint16_t t_hex = (uint16_t)((data[5] | (data[4] & 0x0F) << 8));

      {
        std::lock_guard<std::mutex> lock(mtx_);
        if (id == 1 || id == 2)
        {
          motor_state_[id - 1].pos = uint_to_float(pos_hex, P_MIN, P_MAX, 16);
          motor_state_[id - 1].vel = uint_to_float(vel_hex, V_MIN, V_MAX, 12);
          motor_state_[id - 1].tau = uint_to_float(t_hex, T_MIN, T_MAX, 12);
          motor_state_[id - 1].Tmos = data[6];
          motor_state_[id - 1].Tcoil = data[7];
        }
        else
        {
          std::cout << "Control loop has undefined motors!" << std::endl;
        }
      }
    }
  }

  auto finish = std::chrono::high_resolution_clock::now();
  auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(finish - start);
  std::cout << "time: " << elapsed.count() << "ms" << std::endl;

  return 1;
}

void HeadInterface::runSendReceive()
{
  // TODO: 只有位置模式
  while (is_running_)
  {
    int ret = doSendReceive();
    // usleep(1000);
  }
}

void HeadInterface::runSendReceiveThread()
{
  is_running_ = true;
  thread_ = std::thread([this] { runSendReceive(); });  // 使用 Lambda 表达式捕获 this
}

int HeadInterface::pause()
{
  is_running_ = false;
  if (thread_.joinable())
  {
    thread_.join();
  }
  usleep(100000);
  return 1;
}

int HeadInterface::enableMotor()
{
  VCI_CAN_OBJ send[Joint_Num];  // 使用局部变量send
  for (int i = 0; i < Joint_Num; i++)
  {
    int id = i + 1;
    send[i].SendType = 0;
    send[i].RemoteFlag = 0;
    send[i].ExternFlag = 0;
    send[i].ID = id;
    send[i].DataLen = 8;
    send[i].Data[0] = 0xFF;
    send[i].Data[1] = 0xFF;
    send[i].Data[2] = 0xFF;
    send[i].Data[3] = 0xFF;
    send[i].Data[4] = 0xFF;
    send[i].Data[5] = 0xFF;
    send[i].Data[6] = 0xFF;
    send[i].Data[7] = 0xFC;
  }
  if (VCI_Transmit(VCI_USBCAN2, 0, cid_, send, Joint_Num) != 1)
  {
    // return 0; // 发送失败，返回0
  }

  usleep(100000);
  return 1;
}

int HeadInterface::disableMotor()
{
  VCI_CAN_OBJ send[Joint_Num];  // 使用局部变量send
  for (int i = 0; i < Joint_Num; i++)
  {
    int id = i + 1;
    send[i].SendType = 0;
    send[i].RemoteFlag = 0;
    send[i].ExternFlag = 0;
    send[i].ID = id;
    send[i].DataLen = 8;
    send[i].Data[0] = 0xFF;
    send[i].Data[1] = 0xFF;
    send[i].Data[2] = 0xFF;
    send[i].Data[3] = 0xFF;
    send[i].Data[4] = 0xFF;
    send[i].Data[5] = 0xFF;
    send[i].Data[6] = 0xFF;
    send[i].Data[7] = 0xFD;
  }
  if (VCI_Transmit(VCI_USBCAN2, 0, cid_, send, Joint_Num) != 1)
  {
    // return 0; // 发送失败，返回0
  }

  usleep(100000);
  return 1;
}

int HeadInterface::zeroMotor()
{
  VCI_CAN_OBJ send[Joint_Num];  // 使用局部变量send
  for (int i = 0; i < Joint_Num; i++)
  {
    int id = i + 1;
    send[i].SendType = 0;
    send[i].RemoteFlag = 0;
    send[i].ExternFlag = 0;
    send[i].ID = id;
    send[i].DataLen = 8;
    send[i].Data[0] = 0xFF;
    send[i].Data[1] = 0xFF;
    send[i].Data[2] = 0xFF;
    send[i].Data[3] = 0xFF;
    send[i].Data[4] = 0xFF;
    send[i].Data[5] = 0xFF;
    send[i].Data[6] = 0xFF;
    send[i].Data[7] = 0xFE;
  }
  if (VCI_Transmit(VCI_USBCAN2, 0, cid_, send, Joint_Num) != 1)
  {
    // return 0; // 发送失败，返回0
  }

  return 1;
}

int HeadInterface::resetMotor()
{
  VCI_CAN_OBJ send[Joint_Num];  // 使用局部变量send
  for (int i = 0; i < Joint_Num; i++)
  {
    int id = i + 1;
    send[i].SendType = 0;
    send[i].RemoteFlag = 0;
    send[i].ExternFlag = 0;
    send[i].ID = id;
    send[i].DataLen = 8;
    send[i].Data[0] = 0xFF;
    send[i].Data[1] = 0xFF;
    send[i].Data[2] = 0xFF;
    send[i].Data[3] = 0xFF;
    send[i].Data[4] = 0xFF;
    send[i].Data[5] = 0xFF;
    send[i].Data[6] = 0xFF;
    send[i].Data[7] = 0xFB;
  }
  if (VCI_Transmit(VCI_USBCAN2, 0, cid_, send, Joint_Num) != 1)
  {
    // return 0; // 发送失败，返回0
  }

  return 1;
}

int HeadInterface::setCmd(motor_cmd_t* cmd)
{
  std::lock_guard<std::mutex> lock(mtx_);

  for (int i = 0; i < Joint_Num; i++)
  {
    motor_cmd_[i].pos_d = cmd[i].pos_d;
    motor_cmd_[i].vel_d = cmd[i].vel_d;
    motor_cmd_[i].Kp = cmd[i].Kp;
    motor_cmd_[i].Kd = cmd[i].Kd;
    motor_cmd_[i].tau_d = cmd[i].tau_d;
  }
  return 1;
}

int HeadInterface::getState(motor_state_t* state)
{
  std::lock_guard<std::mutex> lock(mtx_);

  for (int i = 0; i < Joint_Num; i++)
  {
    state[i].pos = motor_state_[i].pos;
    state[i].vel = motor_state_[i].vel;
    state[i].tau = motor_state_[i].tau;
  }
  return 1;
}