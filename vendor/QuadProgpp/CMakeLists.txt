# This software may be modified and distributed under the terms
# of the MIT license.  See the LICENSE file for details.

file(GLOB sources *.cc)
file(GLOB headers *.hh)
set(library_sources ${sources})
list(REMOVE_ITEM library_sources ${CMAKE_CURRENT_SOURCE_DIR}/main.cc)
add_library(quadprog STATIC ${library_sources} ${headers})
