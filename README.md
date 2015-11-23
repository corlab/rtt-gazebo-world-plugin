# rtt-gazebo-world-plugin
Gazebo world plugin to instantiate a RTT environment inside the Gazebo world. Inspired by https://github.com/ahoarau/rtt_gazebo/tree/master/rtt_gazebo_deployer.

## Install

`mkdir build`

`cd build`

// for orocos plugins

`export PKG_CONFIG_PATH=$insert-prefix-here/lib/pkgconfig`

`cmake -DRSC-CMake_DIR=/vol/toolkit/nightly/trusty/x86_64/last/share/rsc-cmake0.13 -DRST_DIR=/vol/toolkit/nightly/trusty/x86_64/last/share/rst0.13 -DRSB_DIR=/vol/toolkit/nightly/trusty/x86_64/last/share/rsb0.13 -DRSC_DIR=/vol/toolkit/nightly/trusty/x86_64/last/share/rsc0.13 -Dgazebo_DIR=$insert-prefix-here/lib/cmake/gazebo -Dignition-math2_DIR=$insert-prefix-here/lib/cmake/ignition-math2 -DSDFormat_DIR=$insert-prefix-here/lib/cmake/sdformat -DOROCOS-RTT_DIR=$insert-prefix-here/lib/cmake/orocos-rtt ..`

`make -j 4`

# Launch Gazebo and Spread

// launch spread

`spread -n localhost`

// add plugin to gazebo scope

`export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:/include-path-to/rtt-gazebo-world-plugin/build`

// use your gazebo 6

`export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/$insert-prefix-here/lib`

`gzserver -s /include-path-to/rtt-gazebo-clock-plugin/build/librtt_gazebo_system.so /include-path-to/rtt-gazebo-world-plugin/build/simple.world`

// optionally you can launch the clinet for visualization too

`gzclient`

## TODO

- Exclude Eigen folder
