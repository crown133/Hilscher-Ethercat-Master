#ifndef MOTOT_DM_INTERFACE_H
#define MOTOT_DM_INTERFACE_H

#include <iostream>
#include "math_ops.h"
#include "can_msgs/msg/frame.hpp"

#define KP_MAX 500
#define KP_MIN 0
#define KD_MAX 5
#define KD_MIN 0

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
  float tor;
  float Kp;
  float Kd;
  float Tmos;
  float Tcoil;
} motor_state_t;

typedef struct
{
  float pos_d;
  float vel_d;
  float tor_d;
  float Kp_d;
  float Kd_d;
} motor_cmd_t;

can_msgs::msg::Frame motor_enable(uint8_t ID)
{
  can_msgs::msg::Frame csd;
  csd.id = ID;
  csd.dlc = 0x08;
  csd.data[0] = 0xFF;
  csd.data[1] = 0xFF;
  csd.data[2] = 0xFF;
  csd.data[3] = 0xFF;
  csd.data[4] = 0xFF;
  csd.data[5] = 0xFF;
  csd.data[6] = 0xFF;
  csd.data[7] = 0xFC;

  return csd;
}

can_msgs::msg::Frame motor_disable(uint8_t ID)
{
  can_msgs::msg::Frame csd;
  csd.id = ID;
  csd.dlc = 0x08;
  csd.data[0] = 0xFF;
  csd.data[1] = 0xFF;
  csd.data[2] = 0xFF;
  csd.data[3] = 0xFF;
  csd.data[4] = 0xFF;
  csd.data[5] = 0xFF;
  csd.data[6] = 0xFF;
  csd.data[7] = 0xFD;

  return csd;
}

can_msgs::msg::Frame motor_zero(uint8_t ID)
{
  can_msgs::msg::Frame csd;
  csd.id = ID;
  csd.dlc = 0x08;
  csd.data[0] = 0xFF;
  csd.data[1] = 0xFF;
  csd.data[2] = 0xFF;
  csd.data[3] = 0xFF;
  csd.data[4] = 0xFF;
  csd.data[5] = 0xFF;
  csd.data[6] = 0xFF;
  csd.data[7] = 0xFE;

  return csd;
}

can_msgs::msg::Frame motor_reset(uint8_t ID)
{
  can_msgs::msg::Frame csd;
  csd.id = ID;
  csd.dlc = 0x08;
  csd.data[0] = 0xFF;
  csd.data[1] = 0xFF;
  csd.data[2] = 0xFF;
  csd.data[3] = 0xFF;
  csd.data[4] = 0xFF;
  csd.data[5] = 0xFF;
  csd.data[6] = 0xFF;
  csd.data[7] = 0xFB;

  return csd;
}

can_msgs::msg::Frame motor_cmd(uint8_t ID, motor_cmd_t cmd)
{
  can_msgs::msg::Frame csd;
  uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;

  pos_tmp = float_to_uint(cmd.pos_d, P_MIN, P_MAX, 16);
  vel_tmp = float_to_uint(cmd.vel_d, V_MIN, V_MAX, 12);
  kp_tmp = float_to_uint(cmd.Kp_d, KP_MIN, KP_MAX, 12);
  kd_tmp = float_to_uint(cmd.Kd_d, KD_MIN, KD_MAX, 12);
  tor_tmp = float_to_uint(cmd.tor_d, T_MIN, T_MAX, 12);

  csd.id = ID;
  csd.dlc = 0x08;

  csd.data[0] = (pos_tmp >> 8);
  csd.data[1] = (pos_tmp & 0xFF);
  csd.data[2] = ((vel_tmp >> 4) & 0xFF);
  csd.data[3] = ((((vel_tmp & 0xF) << 4) & 0xFF) | ((kp_tmp >> 8) & 0xFF));
  csd.data[4] = (kp_tmp & 0xFF);
  csd.data[5] = ((kd_tmp >> 4) & 0xFF);
  csd.data[6] = ((((kd_tmp & 0xF) << 4) & 0xFF) | ((tor_tmp >> 8) & 0xFF));
  csd.data[7] = (tor_tmp & 0xFF);

  return csd;
}

motor_state_t motor_state(uint8_t ID, can_msgs::msg::Frame msg)
{
  motor_state_t state;

  int id = (uint16_t)(msg.data[0]) & 0x0F;

  state.state = (uint16_t)(msg.data[0]) >> 4;

  uint16_t pos_hex = (uint16_t)(msg.data[2] | (msg.data[1] << 8));
  uint16_t vel_hex = (uint16_t)((msg.data[4] >> 4) | (msg.data[3] << 4));
  uint16_t t_hex = (uint16_t)((msg.data[5] | (msg.data[4] & 0x0F) << 8));

  if (id == ID)
  {
    state.pos = uint_to_float(pos_hex, P_MIN, P_MAX, 16);
    state.vel = uint_to_float(vel_hex, V_MIN, V_MAX, 12);
    state.tor = uint_to_float(t_hex, T_MIN, T_MAX, 12);
    state.Tmos = msg.data[6];
    state.Tcoil = msg.data[7];
  }
  else
  {
    // std::cout << "Control loop has undefined motors!" << std::endl;
  }

  return state;
}

#endif /* MOTOT_DM_INTERFACE_H */
