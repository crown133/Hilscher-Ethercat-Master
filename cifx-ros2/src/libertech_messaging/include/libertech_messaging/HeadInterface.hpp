#ifndef ARM_SDK_H
#define ARM_SDK_H

#include <vector>
#include <iostream>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <pthread.h>
#include <thread>
#include <ctime>
#include <cstdlib>
#include "unistd.h"
#include <string>
#include <iomanip>
// #include "math/mathTypes.h"
#include <shared_mutex>
#include <mutex>
#include <map>

#include "controlcan.h"
#include "math_ops.h"

#define Joint_Num 2
#define Belt_Reduction 1.24

#define KP_MAX 500
#define KP_MIN 0
#define KD_MAX 5
#define KD_MIN 0

// #define P_MAX_6006 12.5
// #define P_MIN_6006 -12.5
// #define V_MAX_6006 45.0
// #define V_MIN_6006 -45.0
// #define T_MAX_6006 20.0
// #define T_MIN_6006 -20.0

// #define P_MAX_8006 12.5
// #define P_MIN_8006 -12.5
// #define V_MAX_8006 45.0
// #define V_MIN_8006 -45.0
// #define T_MAX_8006 20.0
// #define T_MIN_8006 -20.0

#define P_MAX 12.5
#define P_MIN -12.5
#define V_MAX 30.0
#define V_MIN -30.0
#define T_MAX 10.0
#define T_MIN -10.0

typedef struct
{
  uint16_t state;
  float pos;
  float vel;
  float tau;
  float Kp;
  float Kd;
  float Tmos;
  float Tcoil;
} motor_state_t;

typedef struct
{
  float pos_d;
  float vel_d;
  float tau_d;
  float Kp;
  float Kd;
} motor_cmd_t;

class HeadInterface
{
public:
  HeadInterface();
  // HeadInterface(int cid);  // Constructor
  ~HeadInterface();

  // 运行
  int init(int cid);
  // VCI_CAN_OBJ* createCANFrames();
  int doSendReceive();
  void runSendReceive();
  void runSendReceiveThread();
  int pause();

  int enableMotor();
  int disableMotor();
  int zeroMotor();
  int resetMotor();

  int setCmd(motor_cmd_t* cmd);
  int getState(motor_state_t* state);

private:
  // USB2CAN
  VCI_BOARD_INFO vciPInfo_;
  VCI_BOARD_INFO vciPInfoAll_[50];
  VCI_INIT_CONFIG vciConfig_;
  int vciNum_;
  int cid_;

  // ARM
  std::thread thread_;
  bool is_running_;
  std::mutex mtx_;

  motor_state_t motor_state_[Joint_Num];
  motor_cmd_t motor_cmd_[Joint_Num];
};

#endif /* ARM_SDK_H */
