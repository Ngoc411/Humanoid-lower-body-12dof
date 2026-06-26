cmake_minimum_required(VERSION 3.8)
project(ws1_cpp)

# --------------------------------------------------
# Compiler options
# --------------------------------------------------
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --------------------------------------------------
# Dependencies
# --------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# libgpiod
find_package(PkgConfig REQUIRED)
pkg_check_modules(GPIOD REQUIRED libgpiod)

set(COMMON_DEPS 
  rclcpp 
  std_msgs
)

# --------------------------------------------------
# External libs
# --------------------------------------------------

# Find onnxruntime
set(ONNXRUNTIME_DIR "/home/ngoc/C++/lib/onnxruntime-linux-aarch64-1.26.0")
include_directories(${ONNXRUNTIME_DIR}/include)
link_directories(${ONNXRUNTIME_DIR}/lib)

# --------------------------------------------------
# Executables list
# --------------------------------------------------
set(NODES
  main_node
  sensor_node
  action_node
  test_model_node
  # node1
  # node2
)

# --------------------------------------------------
# Create executables automatically
# --------------------------------------------------
foreach(node ${NODES})

  if(${node} STREQUAL "main_node")
    add_executable(${node} src/main.cpp)
  else()
    add_executable(${node} ${node}/${node}.cpp)
  endif()

  # Header search paths for each executable
  if(${node} STREQUAL "main_node")
    target_include_directories(${node}
      PRIVATE src sensor_node
    )
  else()
    target_include_directories(${node}
      PRIVATE ${node}
    )
  endif()

  # ROS 2 dependencies for all executables
  ament_target_dependencies(${node}
    ${COMMON_DEPS}
  )

endforeach()

# --------------------------------------------------
# Link external libs
# --------------------------------------------------

# onnxruntime → main_node
target_link_libraries(main_node
  onnxruntime
)

set_target_properties(main_node PROPERTIES
  BUILD_RPATH "${ONNXRUNTIME_DIR}/lib"
  INSTALL_RPATH "${ONNXRUNTIME_DIR}/lib"
  BUILD_WITH_INSTALL_RPATH FALSE
  INSTALL_RPATH_USE_LINK_PATH TRUE
)

# onnxruntime → test_model_node
target_link_libraries(test_model_node
  onnxruntime
)

set_target_properties(test_model_node PROPERTIES
  BUILD_RPATH "${ONNXRUNTIME_DIR}/lib"
  INSTALL_RPATH "${ONNXRUNTIME_DIR}/lib"
  BUILD_WITH_INSTALL_RPATH FALSE
  INSTALL_RPATH_USE_LINK_PATH TRUE
)

# libgpiod → sensor_node
target_link_libraries(sensor_node
  ${GPIOD_LIBRARIES}
)
target_include_directories(sensor_node
  PRIVATE ${GPIOD_INCLUDE_DIRS}
)

# libgpiod → action node
target_link_libraries(action_node
  ${GPIOD_LIBRARIES}
)
target_include_directories(action_node
  PRIVATE ${GPIOD_INCLUDE_DIRS}
)

# --------------------------------------------------
# Install all executables
# --------------------------------------------------
install(TARGETS
  main_node
  sensor_node
  action_node
  test_model_node
  # node1
  # node2
  DESTINATION lib/${PROJECT_NAME}
)

# --------------------------------------------------
# Testing (optional)
# --------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# --------------------------------------------------
# Package export
# --------------------------------------------------
ament_package()
