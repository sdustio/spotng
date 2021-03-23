add_library(Robot STATIC
  src/Model.cpp
)
target_include_directories(Robot PUBLIC include)

install(TARGETS
  Robot
  DESTINATION lib/${PROJECT_NAME})
