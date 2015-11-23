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

## Usage

// upload robot model (URDF)

`rsb-toolscl0.13 call 'spread:/GazeboDeployerWorldPlugin/spawnModel("/include-path-to/robotOutput.urdf")'`

// load rtt-gazebo-lwr-integration and bind to kuka-lwr model

`rsb-toolscl0.13 call -l /$insert-prefix-here/share/rst0.13/proto/sandbox/rst/cogimon/ModelComponentConfig.proto 'spread:/GazeboDeployerWorldPlugin/deployRTTComponentWithModel(pb:.rst.cogimon.ModelComponentConfig:{component_name:"lwr_gazebo" component_type:"LWRGazeboComponent" component_package:"rtt_lwr_gazebo" model_name:"kuka-lwr" script:"/include-path-to/rtt-gazebo-lwr-integration/scripts/lwr_gazebo.ops"})'`

// load position controller template (RTT component)

`rsb-toolscl0.13 call 'spread:/GazeboDeployerWorldPlugin/launchScriptFromFile("/include-path-to/rtt-lwr-controller-template/scripts/rtt_controller.ops")'`

## TODO

- Exclude Eigen folder
