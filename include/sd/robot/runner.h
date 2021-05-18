#include <chrono>
#include <string>

namespace sd::robot
{
  namespace ros
  {
    const std::string kNodeNs = "sd";
    const std::string kNodeName = "Robot";
    const std::string kTopicCmd = "sd/robot_cmd";
    const std::string kTopicMotion = "sd/robot_motion";
  }

  namespace ctrldt
  {
    constexpr std::chrono::milliseconds kSPI(int(1.0 / 0.04));      //0.04kHz
    constexpr std::chrono::microseconds kIMU(int(1000 * 1.0 / 10)); //10kHz
    constexpr std::chrono::milliseconds kDYN(int(1.0 / 0.5));       //0.5kHz
    constexpr float SPIsec = kSPI.count() / 1'000;
    constexpr float IMUsec = kIMU.count() / 1'000'000;
    constexpr float DYNsec = kDYN.count() / 1'000;
  }
}
