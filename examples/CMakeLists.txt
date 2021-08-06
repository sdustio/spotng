set(EXAMPLE_NAMES
  example1
)

foreach(elem ${EXAMPLE_NAMES})
  add_executable(${elem})
  target_sources(${elem} PRIVATE
      ${elem}.cc
      echo_act_itf.cc
    )
  target_include_directories(${elem} PRIVATE
    ${CMAKE_BINARY_DIR}/include
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}
    )
  target_link_libraries(${elem} ${PROJECT_NAME})
endforeach()
