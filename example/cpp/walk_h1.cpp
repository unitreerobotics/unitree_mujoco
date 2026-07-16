#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <string>
#include <thread>

#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

namespace {

constexpr int kMotorCount = 20;
constexpr auto kControlPeriod = std::chrono::microseconds(2000);
constexpr double kPi = 3.141592653589793;

// H1 actuator order from unitree_robots/h1/h1.xml. The leg angles match the
// model's home keyframe and form a slightly crouched standing pose.
constexpr std::array<float, kMotorCount> kStandPose = {
    0.0F, -0.4F, 0.8F,  // right hip roll/pitch, knee
    0.0F, -0.4F, 0.8F,  // left hip roll/pitch, knee
    0.0F,                // torso yaw
    0.0F, 0.0F, 0.0F,   // left hip yaw, right hip yaw, unused
    -0.4F, -0.4F,        // left ankle, right ankle
    0.0F, 0.0F, 0.0F, 0.0F,  // right arm
    0.0F, 0.0F, 0.0F, 0.0F   // left arm
};

uint32_t Crc32(uint32_t* words, uint32_t length) {
  uint32_t crc = 0xFFFFFFFF;
  constexpr uint32_t polynomial = 0x04c11db7;
  for (uint32_t i = 0; i < length; ++i) {
    const uint32_t data = words[i];
    for (uint32_t bit = 0x80000000; bit != 0; bit >>= 1) {
      const bool top = (crc & 0x80000000) != 0;
      crc <<= 1;
      if (top) crc ^= polynomial;
      if (data & bit) crc ^= polynomial;
    }
  }
  return crc;
}

class H1WalkController {
 public:
  void Init() {
    command_.head()[0] = 0xFE;
    command_.head()[1] = 0xEF;
    command_.level_flag() = 0xFF;
    command_.gpio() = 0;

    publisher_ = std::make_shared<unitree::robot::ChannelPublisher<
        unitree_go::msg::dds_::LowCmd_>>("rt/lowcmd");
    publisher_->InitChannel();

    subscriber_ = std::make_shared<unitree::robot::ChannelSubscriber<
        unitree_go::msg::dds_::LowState_>>("rt/lowstate");
    subscriber_->InitChannel(
        [this](const void* message) {
          const auto& state =
              *static_cast<const unitree_go::msg::dds_::LowState_*>(message);
          if (!received_state_.exchange(true)) {
            for (int i = 0; i < kMotorCount; ++i) {
              initial_pose_[i] = state.motor_state()[i].q();
            }
          }

          const uint64_t sample = ++state_samples_;
          if (sample % 2000 == 0) {
            std::cout << "IMU rpy=[" << state.imu_state().rpy()[0] << ", "
                      << state.imu_state().rpy()[1] << ", "
                      << state.imu_state().rpy()[2] << "]" << std::endl;
          }
        },
        1);
  }

  void Run() {
    std::cout << "Waiting for the H1 simulator..." << std::endl;
    while (!received_state_) std::this_thread::sleep_for(kControlPeriod);
    std::cout << "H1 detected: standing, then walking forward." << std::endl;

    const auto start = std::chrono::steady_clock::now();
    auto next = start;
    while (true) {
      const double elapsed = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - start).count();
      auto target = StandingTarget(elapsed);
      if (elapsed > 3.0) ApplyWalkingGait(elapsed - 3.0, target);
      Publish(target);

      next += kControlPeriod;
      std::this_thread::sleep_until(next);
    }
  }

 private:
  std::array<float, kMotorCount> StandingTarget(double elapsed) const {
    const double phase = std::min(elapsed / 2.0, 1.0);
    const double blend = 0.5 - 0.5 * std::cos(kPi * phase);
    auto target = kStandPose;
    for (int i = 0; i < kMotorCount; ++i) {
      target[i] = static_cast<float>(initial_pose_[i] * (1.0 - blend) +
                                     target[i] * blend);
    }
    return target;
  }

  static void ApplyWalkingGait(
      double time, std::array<float, kMotorCount>& target) {
    constexpr double kStrideSeconds = 1.6;
    const float stride = static_cast<float>(
        std::sin(2.0 * kPi * time / kStrideSeconds));
    const float right_lift = std::max(stride, 0.0F);
    const float left_lift = std::max(-stride, 0.0F);

    // Legs swing in opposite phases. The forward-swinging leg bends its knee
    // for ground clearance while the ankle compensates for foot pitch.
    target[1] -= 0.25F * stride;
    target[2] += 0.34F * right_lift;
    target[11] -= 0.10F * right_lift - 0.08F * stride;
    target[4] += 0.25F * stride;
    target[5] += 0.34F * left_lift;
    target[10] -= 0.10F * left_lift + 0.08F * stride;

    // Keep roll and yaw neutral and swing the arms opposite to the legs.
    target[0] = target[3] = target[6] = 0.0F;
    target[12] = -0.35F * stride;
    target[13] = -0.12F;
    target[15] = 0.45F;
    target[16] = 0.35F * stride;
    target[17] = 0.12F;
    target[19] = 0.45F;
  }

  void Publish(const std::array<float, kMotorCount>& target) {
    for (int i = 0; i < kMotorCount; ++i) {
      auto& motor = command_.motor_cmd()[i];
      motor.mode() = 0x01;
      motor.q() = target[i];
      motor.dq() = 0.0F;
      motor.kp() = i < 12 ? 120.0F : 35.0F;
      motor.kd() = i < 12 ? 5.0F : 2.0F;
      motor.tau() = 0.0F;
    }
    command_.crc() = Crc32(reinterpret_cast<uint32_t*>(&command_),
                           (sizeof(command_) >> 2) - 1);
    publisher_->Write(command_);
  }

  unitree_go::msg::dds_::LowCmd_ command_{};
  std::array<float, kMotorCount> initial_pose_{};
  std::atomic<bool> received_state_{false};
  std::atomic<uint64_t> state_samples_{0};
  unitree::robot::ChannelPublisherPtr<unitree_go::msg::dds_::LowCmd_>
      publisher_;
  unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_>
      subscriber_;
};

}  // namespace

int main(int argc, char** argv) {
  int domain_id = 1;
  std::string network = "lo";
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if ((arg == "-i" || arg == "--domain_id") && i + 1 < argc) {
      domain_id = std::stoi(argv[++i]);
    } else if ((arg == "-n" || arg == "--network") && i + 1 < argc) {
      network = argv[++i];
    } else {
      std::cerr << "Usage: walk_h1 [-i domain_id] [-n network_interface]"
                << std::endl;
      return 2;
    }
  }

  unitree::robot::ChannelFactory::Instance()->Init(domain_id, network);
  H1WalkController controller;
  controller.Init();
  controller.Run();
}
