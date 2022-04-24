cmake_minimum_required(VERSION 3.1)
project(walnut-javascript)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

SET(WALNUT_USE_VTK OFF CACHE BOOL "Build targets that depend on VTK")
add_subdirectory(walnut EXCLUDE_FROM_ALL)

add_executable(walnut-javascript interface.cpp)
set_target_properties(walnut-javascript PROPERTIES
                      PUBLIC_HEADER "interface.h"
                      OUTPUT_NAME "walnut")
target_link_libraries(walnut-javascript walnut)
target_link_options(walnut-javascript
  PRIVATE
  "SHELL:-s INITIAL_MEMORY=1610612736"
  "SHELL:-s MODULARIZE=1"
  "SHELL:-s EXPORT_NAME=walnut"
  )

configure_file(example/CSG2Object3D.js . COPYONLY)
configure_file(example/CsgToBuffers.js . COPYONLY)
configure_file(example/index.html . COPYONLY)
configure_file(example/jscad-modeling.min.js . COPYONLY)
configure_file(example/jscad-modeling.min.js.map . COPYONLY)
configure_file(example/main.css . COPYONLY)
configure_file(example/Three.jscad.js . COPYONLY)
configure_file(example/Three.jscad.js.map . COPYONLY)
