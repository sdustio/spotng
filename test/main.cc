#include <iostream>

namespace sdquadx::test {
void RunExample();
}  // namespace sdquadx::test

int main([[maybe_unused]] int argc, [[maybe_unused]] char **argv) {
  std::cout << "Start Test!!!" << std::endl;
  sdquadx::test::RunExample();
  std::cout << "End Test!!!" << std::endl;
  return 0;
}
