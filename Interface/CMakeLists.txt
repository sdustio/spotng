option(BUILD_HW "whether to build hardware interface" ON)
option(BUILD_SIM "whether to build simulator interface" ON)

if(BUILD_HW)
  add_subdirectory(Hardware)
endif(BUILD_HW)

if(BUILD_SIM)
  add_subdirectory(Simulator)
endif(BUILD_SIM)
