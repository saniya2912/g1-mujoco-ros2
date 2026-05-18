/*****************************************************************
 Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
******************************************************************/

#ifndef G1_SIM_MOTOR_CRC_HG_H_
#define G1_SIM_MOTOR_CRC_HG_H_

#include <stdint.h>
#include <array>

#include "rclcpp/rclcpp.hpp"
#include "unitree_hg/msg/low_cmd.hpp"
#include "unitree_hg/msg/motor_cmd.hpp"

typedef struct {
  uint8_t mode;
  float q;
  float dq;
  float tau;
  float Kp;
  float Kd;
  uint32_t reserve = 0;
} MotorCmdRaw;

typedef struct {
  uint8_t modePr;
  uint8_t modeMachine;
  std::array<MotorCmdRaw, 35> motorCmd;
  std::array<uint32_t, 4> reserve;
  uint32_t crc;
} LowCmdRaw;

uint32_t crc32_core(uint32_t *ptr, uint32_t len);
void get_crc(unitree_hg::msg::LowCmd &msg);

#endif  // G1_SIM_MOTOR_CRC_HG_H_
