#include "libertech_messaging/HeadInterface.hpp"
#include <fstream>
#include <numeric>

int main()
{
  HeadInterface head_sdk;
  head_sdk.init(0);
  sleep(1);

  motor_cmd_t cmd[2];
  cmd[0].pos_d = 0.5;
  cmd[0].vel_d = 0;
  cmd[0].Kp = 2;
  cmd[0].Kd = 1;
  cmd[0].tau_d = 0;

  head_sdk.setCmd(cmd);

  while (1)
  {
    sleep(1);
  }

  head_sdk.pause();
  return 0;
}
