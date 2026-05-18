/**
 * Stand demo for Unitree G1 Edu U8 (29-DOF) in MuJoCo simulation.
 *
 * Reads actual joint positions from /lowstate, then smoothly drives all joints
 * to q=0 (upright standing pose) over 3 seconds using state feedback.
 * After standing, demonstrates a gentle knee bend and recovery.
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

// Upright standing: all joints at zero
static const double kStandPos[G1_NUM_MOTOR] = {};

// Knee-bend demo target (legs only, rest stay at 0)
static const double kKneeBendPos[G1_NUM_MOTOR] = {
    0.3,  0.0, 0.0, 0.6, -0.3, 0.0,   // left leg: hip-pitch, knee, ankle
    0.3,  0.0, 0.0, 0.6, -0.3, 0.0,   // right leg
    0.0,  0.0, 0.0,                    // waist
    0.0,  0.3, 0.0, 0.4, 0.0, 0.0, 0.0,  // left arm
    0.0, -0.3, 0.0, 0.4, 0.0, 0.0, 0.0,  // right arm
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
  // S-curve: tanh-based, maps [0,1] -> [0,1]
  const double k = 3.0;
  return std::tanh(k * t) / std::tanh(k);
}

class G1StandDemo : public rclcpp::Node {
 public:
  G1StandDemo() : Node("g1_stand_demo") {
    cmd_pub_ = this->create_publisher<unitree_hg::msg::LowCmd>("/lowcmd", 10);

    state_sub_ = this->create_subscription<unitree_hg::msg::LowState>(
        "/lowstate", 10,
        [this](const unitree_hg::msg::LowState::SharedPtr msg) {
          mode_machine_ = static_cast<int>(msg->mode_machine);
          if (!state_received_) {
            // Snapshot starting joint positions on first message
            for (int i = 0; i < G1_NUM_MOTOR; i++) {
              start_pos_[i] = msg->motor_state[i].q;
            }
            state_received_ = true;
            RCLCPP_INFO(this->get_logger(),
                        "State received. Starting stand-up sequence...");
          }
        });

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(dt_ * 1000)),
        [this] { tick(); });

    init_cmd();
    RCLCPP_INFO(this->get_logger(),
                "G1 stand demo ready. Waiting for /lowstate from simulator...");
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

  void tick() {
    if (!state_received_) return;  // wait until we know where the robot is

    time_ += dt_;
    low_cmd_.mode_machine = static_cast<uint8_t>(mode_machine_);

    const double phase1_dur = 3.0;  // stand up
    const double hold_dur   = 2.0;  // hold standing
    const double phase2_dur = 2.0;  // knee bend
    const double phase3_dur = 2.0;  // recover

    for (int i = 0; i < G1_NUM_MOTOR; i++) {
      double target = 0.0;

      if (time_ < phase1_dur) {
        // Phase 1: from actual start position -> standing (q=0)
        double ratio = smooth(clamp(time_ / phase1_dur, 0.0, 1.0));
        target = start_pos_[i] + ratio * (kStandPos[i] - start_pos_[i]);

      } else if (time_ < phase1_dur + hold_dur) {
        // Phase 2: hold standing
        target = kStandPos[i];

      } else if (time_ < phase1_dur + hold_dur + phase2_dur) {
        // Phase 3: standing -> knee bend
        double ratio = smooth(clamp((time_ - phase1_dur - hold_dur) / phase2_dur, 0.0, 1.0));
        target = kStandPos[i] + ratio * (kKneeBendPos[i] - kStandPos[i]);

      } else if (time_ < phase1_dur + hold_dur + phase2_dur + phase3_dur) {
        // Phase 4: knee bend -> standing
        double ratio = smooth(clamp((time_ - phase1_dur - hold_dur - phase2_dur) / phase3_dur, 0.0, 1.0));
        target = kKneeBendPos[i] + ratio * (kStandPos[i] - kKneeBendPos[i]);

      } else {
        // Hold final standing pose
        target = kStandPos[i];
        if (!done_logged_) {
          RCLCPP_INFO(this->get_logger(), "Demo complete. Holding standing pose.");
          done_logged_ = true;
        }
      }

      low_cmd_.motor_cmd[i].q   = static_cast<float>(target);
      low_cmd_.motor_cmd[i].dq  = 0.0f;
      low_cmd_.motor_cmd[i].kp  = kp_for(i);
      low_cmd_.motor_cmd[i].kd  = kd_for(i);
      low_cmd_.motor_cmd[i].tau = 0.0f;
    }

    get_crc(low_cmd_);
    cmd_pub_->publish(low_cmd_);
  }

  rclcpp::Publisher<unitree_hg::msg::LowCmd>::SharedPtr cmd_pub_;
  rclcpp::Subscription<unitree_hg::msg::LowState>::SharedPtr state_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  unitree_hg::msg::LowCmd low_cmd_;
  double dt_    = 0.002;  // 500 Hz
  double time_  = 0.0;
  int mode_machine_ = 0;

  bool state_received_ = false;
  bool done_logged_    = false;
  std::array<double, G1_NUM_MOTOR> start_pos_{};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<G1StandDemo>());
  rclcpp::shutdown();
  return 0;
}
