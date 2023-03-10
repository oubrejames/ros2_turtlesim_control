cmake_minimum_required(VERSION 3.8)
project(turtle_interfaces)

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

########## OUBRE ADDED START #########
# OUBRE - Added this to generate my custom service
find_package(rosidl_default_generators REQUIRED)

# OUBRE - Custom service uses turtlesim/Pose so must get turtlesim pkg
find_package(turtlesim REQUIRED)

# OUBRE - Say what files to generate messages/services and dependencies
rosidl_generate_interfaces(${PROJECT_NAME}
     "srv/Waypoints.srv" 
     DEPENDENCIES turtlesim 
     )
# Export a dependency on the =ros_idl_runtime=
ament_export_dependencies(rosidl_default_runtime)

########## OUBRE ADDED END #########


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
