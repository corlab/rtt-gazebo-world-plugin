# rtt-gazebo-world-plugin
Gazebo world plugin to instantiate a RTT environment inside the Gazebo world. Inspired by https://github.com/ahoarau/rtt_gazebo/tree/master/rtt_gazebo_deployer.

## Install

`mkdir build`

`cd build`

// for orocos plugins

`export PKG_CONFIG_PATH=$insert-prefix-here/lib/pkgconfig`

`cmake -DRSC-CMake_DIR=/vol/toolkit/nightly/trusty/x86_64/last/share/rsc-cmake0.13 -DRST_DIR=/vol/toolkit/nightly/trusty/x86_64/last/share/rst0.13 -DRSB_DIR=/vol/toolkit/nightly/trusty/x86_64/last/share/rsb0.13 -DRSC_DIR=/vol/toolkit/nightly/trusty/x86_64/last/share/rsc0.13 -Dgazebo_DIR=$insert-prefix-here/lib/cmake/gazebo -Dignition-math2_DIR=$insert-prefix-here/lib/cmake/ignition-math2 -DSDFormat_DIR=$insert-prefix-here/lib/cmake/sdformat -DOROCOS-RTT_DIR=$insert-prefix-here/lib/cmake/orocos-rtt ..`

`make -j 4`

## TODO

- Exclude Eigen folder
