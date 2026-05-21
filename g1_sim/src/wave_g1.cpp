/**
 * Arm-wave demo for Unitree G1 Edu U8 (29-DOF) in MuJoCo simulation.
 *
 * Phase 1 (3 s): smooth ramp from initial pose to "ready" — legs and waist held
 * at their start positions, arms ramped to neutral (q=0).
 * Phase 2 (forever): legs and waist still held; shoulders + elbows oscillate
 * in a sine wave so both arms wave. Mirrors the structure of Unitree's
 * g1_low_level_example.py (zero-init then sinusoidal motion).
 **/

#include <cmath>
#include <array>

#include "g1_sim/motor_crc_hg.h"
#include "rclcpp/rclcpp.hpp"
#include "unitree_hg/msg/low_cmd.hpp"
#include "unitree_hg/msg/low_state.hpp"
#include "unitree_hg/msg/motor_cmd.hpp"

constexpr int G1_NUM_MOTOR = 29;

enum G1JointIndex {
  LEFT_HIP_PITCH    = 0,
  LEFT_HIP_ROLL     = 1,
  LEFT_HIP_YAW      = 2,
  LEFT_KNEE         = 3,
  LEFT_ANKLE_PITCH  = 4,
  LEFT_ANKLE_ROLL   = 5,
  RIGHT_HIP_PITCH   = 6,
  RIGHT_HIP_ROLL    = 7,
  RIGHT_HIP_YAW     = 8,
  RIGHT_KNEE        = 9,
  RIGHT_ANKLE_PITCH = 10,
  RIGHT_ANKLE_ROLL  = 11,
  WAIST_YAW         = 12,
  WAIST_ROLL        = 13,
  WAIST_PITCH       = 14,
  LEFT_SHOULDER_PITCH  = 15,
  LEFT_SHOULDER_ROLL   = 16,
  LEFT_SHOULDER_YAW    = 17,
  LEFT_ELBOW           = 18,
  LEFT_WRIST_ROLL      = 19,
  LEFT_WRIST_PITCH     = 20,
  LEFT_WRIST_YAW       = 21,
  RIGHT_SHOULDER_PITCH = 22,
  RIGHT_SHOULDER_ROLL  = 23,
  RIGHT_SHOULDER_YAW   = 24,
  RIGHT_ELBOW          = 25,
  RIGHT_WRIST_ROLL     = 26,
  RIGHT_WRIST_PITCH    = 27,
  RIGHT_WRIST_YAW      = 28,
};

static float kp_for(int i) {
  if (i < 12) return 100.0f;
  if (i < 15) return 60.0f;
  return 40.0f;
}

static float kd_for(int i) {
  if (i < 12) return 2.5f;
  if (i < 15) return 1.5f;
  return 1.0f;
}

static double clamp(double v, double lo, double hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

static double smooth(double t) {
  const double k = 3.0;
  return std::tanh(k * t) / std::tanh(k);
}

class G1ArmWaveDemo : public rclcpp::Node {
 public:
  G1ArmWaveDemo() : Node("g1_arm_wave_demo") {
    cmd_pub_ = this->create_publisher<unitree_hg::msg::LowCmd>("/lowcmd", 10);

    state_sub_ = this->create_subscription<unitree_hg::msg::LowState>(
        "/lowstate", 10,
        [this](const unitree_hg::msg::LowState::SharedPtr msg) {
          mode_machine_ = static_cast<int>(msg->mode_machine);
          for (int i = 0; i < G1_NUM_MOTOR; i++) {
            last_q_[i] = msg->motor_state[i].q;
          }
          lowstate_count_++;
          if (!state_received_) {
            for (int i = 0; i < G1_NUM_MOTOR; i++) {
              start_pos_[i] = msg->motor_state[i].q;
            }
            state_received_ = true;
            RCLCPP_INFO(this->get_logger(),
                        "State received (mode_machine=%d). Starting arm-wave demo...",
                        mode_machine_);
          }
        });

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(dt_ * 1000)),
        [this] { tick(); });

    init_cmd();
    RCLCPP_INFO(this->get_logger(),
                "G1 arm-wave demo ready. Waiting for /lowstate from simulator...");
  }

 private:
  void init_cmd() {
    low_cmd_.mode_pr = 0;
    low_cmd_.mode_machine = 0;
    for (int i = 0; i < 35; i++) {
      low_cmd_.motor_cmd[i].mode = 0x01;
      low_cmd_.motor_cmd[i].q   = 0.0f;
      low_cmd_.motor_cmd[i].dq  = 0.0f;
      low_cmd_.motor_cmd[i].kp  = 0.0f;
      low_cmd_.motor_cmd[i].kd  = 0.0f;
      low_cmd_.motor_cmd[i].tau = 0.0f;
    }
  }

  // Hold-pose for joint i during/after the ramp.
  // Legs (0..11) and waist (12..14) stay at start_pos; arms (15..28) ramp to 0.
  double ready_pos(int i) const {
    if (i < 15) return start_pos_[i];
    return 0.0;
  }

  void tick() {
    if (!state_received_) return;

    time_ += dt_;
    low_cmd_.mode_machine = static_cast<uint8_t>(mode_machine_);

    const double ramp_dur = 3.0;
    const double period   = 2.0;
    const double A_sh     = 0.6;  // shoulder pitch amplitude (rad)
    const double A_el     = 0.4;  // elbow amplitude (rad)
    const double bias_el  = 0.8;  // elbow bias so it doesn't hyperextend

    for (int i = 0; i < G1_NUM_MOTOR; i++) {
      double target = ready_pos(i);

      if (time_ < ramp_dur) {
        // Phase 1: smooth ramp from start_pos to ready_pos
        double ratio = smooth(clamp(time_ / ramp_dur, 0.0, 1.0));
        target = start_pos_[i] + ratio * (ready_pos(i) - start_pos_[i]);

      } else {
        // Phase 2: arms wave, everything else holds ready pose
        const double t = time_ - ramp_dur;
        const double w = 2.0 * M_PI / period;
        const double s_shoulder = std::sin(w * t);
        const double s_elbow    = std::sin(w * t + M_PI / 2.0);

        switch (i) {
          case LEFT_SHOULDER_PITCH:
          case RIGHT_SHOULDER_PITCH:
            target = A_sh * s_shoulder;
            break;
          case LEFT_ELBOW:
          case RIGHT_ELBOW:
            target = bias_el + A_el * s_elbow;
            break;
          default:
            target = ready_pos(i);
            break;
        }
      }

      low_cmd_.motor_cmd[i].q   = static_cast<float>(target);
      low_cmd_.motor_cmd[i].dq  = 0.0f;
      low_cmd_.motor_cmd[i].kp  = kp_for(i);
      low_cmd_.motor_cmd[i].kd  = kd_for(i);
      low_cmd_.motor_cmd[i].tau = 0.0f;

      if (i == LEFT_SHOULDER_PITCH) last_shoulder_tgt_ = target;
      if (i == LEFT_ELBOW)          last_elbow_tgt_    = target;
    }

    get_crc(low_cmd_);
    cmd_pub_->publish(low_cmd_);

    if (time_ >= ramp_dur) {
      RCLCPP_INFO_THROTTLE(
          this->get_logger(), *this->get_clock(), 500,
          "t=%.2f  lowstate_rx=%lu  L_SHO_P tgt=%+.3f act=%+.3f  L_ELBOW tgt=%+.3f act=%+.3f",
          time_, lowstate_count_,
          last_shoulder_tgt_, last_q_[LEFT_SHOULDER_PITCH],
          last_elbow_tgt_,    last_q_[LEFT_ELBOW]);
    }
  }

  rclcpp::Publisher<unitree_hg::msg::LowCmd>::SharedPtr cmd_pub_;
  rclcpp::Subscription<unitree_hg::msg::LowState>::SharedPtr state_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  unitree_hg::msg::LowCmd low_cmd_;
  double dt_    = 0.002;
  double time_  = 0.0;
  int mode_machine_ = 0;

  bool state_received_ = false;
  std::array<double, G1_NUM_MOTOR> start_pos_{};
  std::array<double, G1_NUM_MOTOR> last_q_{};
  uint64_t lowstate_count_ = 0;

  double last_shoulder_tgt_ = 0.0;
  double last_elbow_tgt_    = 0.0;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<G1ArmWaveDemo>());
  rclcpp::shutdown();
  return 0;
}
