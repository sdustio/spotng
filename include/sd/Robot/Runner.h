#include <chrono>
#include <string>

namespace sd::robot
{
  namespace ros
  {
    const std::string NODE_NS = "sd";
    const std::string NODE_NAME = "Robot";
    const std::string TOPIC_CMD = "sd/robot_cmd";
    const std::string TOPIC_MOTION = "sd/robot_motion";
  }

  namespace ctrldt
  {
    constexpr std::chrono::milliseconds SPI(int(1.0 / 0.04));      //0.04kHz
    constexpr std::chrono::microseconds IMU(int(1000 * 1.0 / 10)); //10kHz
    constexpr std::chrono::milliseconds DYN(int(1.0 / 0.5));       //0.5kHz
    constexpr float SPIsec = SPI.count() / 1'000;
    constexpr float IMUsec = IMU.count() / 1'000'000;
    constexpr float DYNsec = DYN.count() / 1'000;
  }
}
