#include <memory>

#include "sd/Robot/Runner.h"
#include "Robot/Interface/Hardware/Interface.hpp"

int main(int argc, char* argv[]){
  using namespace sd::robot::interface;
  std::shared_ptr<Interface> itf = std::make_shared<hardware::Interface>();
  printf("Run Node");
  return sd::robot::Run(itf, argc, argv);
}
