# Overview
Trajectory generator for the MIRRAX reconfigurable robot. The trajectory generation has been adapted from the [mav_trajectory_generation](https://github.com/ethz-asl/mav_trajectory_generation) package for this robot. 

## Build
First, clone and build the `mav_trajectory_generation` ROS package:

```bash
mkdir -p ~/catkin_trajectory/src && cd ~/catkin_trajectory
catkin init
catkin config --extend /opt/ros/melodic 
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release 
catkin config --merge-devel
cd src
wstool init
wstool set --git mav_trajectory_generation git@github.com:ethz-asl/mav_trajectory_generation.git -y
wstool update
wstool merge mav_trajectory_generation/install/mav_trajectory_generation_https.rosinstall
wstool update -j8
source ~/catkin_trajectory/devel/setup.bash
source /opt/ros/melodic/setup.bash
catkin build mav_trajectory_generation_ros
```

If there is an error due to yaml, replace lines 11-20 in `mav_trajectory_generation/CMakeLists.txt` with the following:
```bash
set(ENV{PKG_CONFIG_PATH} "$ENV{PKG_CONFIG_PATH}:${CATKIN_DEVEL_PREFIX}/lib/pkgconfig")
find_package(PkgConfig)
pkg_check_modules(yaml_cpp yaml-cpp REQUIRED)
# Resolve system dependency on yaml-cpp, which apparently does not
# provide a CMake find_package() module.
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
find_path(YAML_CPP_INCLUDE_DIR
  NAMES yaml_cpp.h
  PATHS ${YAML_CPP_INCLUDE_DIRS}
)
find_library(YAML_CPP_LIBRARY
  NAMES YAML_CPP
  PATHS ${YAML_CPP_LIBRARY_DIRS}
)
link_directories(${YAML_CPP_LIBRARY_DIRS})

if(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")
add_definitions(-DHAVE_NEW_YAMLCPP)
endif(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")
```

Finally, clone the `mirrax_trajectory_generator` and build,

```bash
git clone 
catkin build mirrax_trajectory_generator
source ~/catkin_trajectory/devel/setup.bash
```

## How to use
Run the launch file with the `frame_id` corresponding to your desired frame,

    roslaunch mirrax_trajectory_generator trajectory_generator.launch frame_id:=world

The trajectory node is executed and waits for setpoints. The initial start state, waypoints and goal state are all expected to be provided via ROS topic to the callback `/waypoints`. The trajectory can be visualized in RViZ under the `trajectory_markers` topic.

A example script of waypoints is available at:

    rosrun mirrax_trajectory_generator test_poly.py

Setting the parameter `zero_waypoint_velocity` switches the trajectory generator to either have C1 (True) or C3 (False) continuity.
