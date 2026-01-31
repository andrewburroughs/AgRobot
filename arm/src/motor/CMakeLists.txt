cmake_minimum_required(VERSION 3.8)
project(clearpath_sc_ros2)

# Default to Release like -O3 (similar intent to your Makefile)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(clearpath_sc_node src/clearpath_sc_ros2_node.cpp)
ament_target_dependencies(clearpath_sc_node rclcpp)

# ---- Teknic sFoundation include dir (equivalent to -I../../inc/inc-pub) ----
# Option A (recommended): pass it in via -DTEKNIC_INCLUDE_DIR=...
set(TEKNIC_INCLUDE_DIR "" CACHE PATH "Path to Teknic inc-pub directory (contains pubSysCls.h)")
if(TEKNIC_INCLUDE_DIR STREQUAL "")
  message(FATAL_ERROR "TEKNIC_INCLUDE_DIR not set. Example: -DTEKNIC_INCLUDE_DIR=../../inc/inc-pub")
endif()

target_include_directories(clearpath_sc_node PRIVATE
  ${TEKNIC_INCLUDE_DIR}
)

# ---- Teknic library dir + linking (equivalent to -lsFoundation20 -lpthread) ----
# If libs are in a non-standard path, pass it in like your RPATH usage
set(TEKNIC_LIB_DIR "" CACHE PATH "Path containing libsFoundation20.so (or .a)")
if(NOT TEKNIC_LIB_DIR STREQUAL "")
  target_link_directories(clearpath_sc_node PRIVATE ${TEKNIC_LIB_DIR})

  # Optional: embed runtime search path like your -Wl,-rpath
  set_target_properties(clearpath_sc_node PROPERTIES
    BUILD_RPATH "${TEKNIC_LIB_DIR}"
    INSTALL_RPATH "${TEKNIC_LIB_DIR}"
  )
endif()

# Link the same libs as Makefile
target_link_libraries(clearpath_sc_node
  sFoundation20
  pthread
)

# Warnings similar to your DEBUG_CXXFLAGS (optional)
target_compile_options(clearpath_sc_node PRIVATE
  -Wall -Wextra -pedantic
)

install(TARGETS clearpath_sc_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
