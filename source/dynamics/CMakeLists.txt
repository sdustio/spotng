list(APPEND srcs
  inertia.cc
  rotation.cc
  spatial.cc
  fb_model.cc
  bspline.cc
  math.cc
)

list(TRANSFORM srcs PREPEND "${CMAKE_CURRENT_SOURCE_DIR}/")

set(SOURCES ${SOURCES} ${srcs} PARENT_SCOPE)
