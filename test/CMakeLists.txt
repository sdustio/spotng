add_executable(sdtest)
target_sources(sdtest PRIVATE
  echo_act_itf.cc
  main.cc
)
target_include_directories(sdtest PRIVATE
  ${CMAKE_BINARY_DIR}/include
  ${CMAKE_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(sdtest ${PROJECT_NAME})
