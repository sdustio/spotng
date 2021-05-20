list(APPEND srcs
  fb_model.cc
)

list(TRANSFORM srcs PREPEND "${CMAKE_CURRENT_SOURCE_DIR}/")

set(SOURCES ${SOURCES} ${srcs} PARENT_SCOPE)
