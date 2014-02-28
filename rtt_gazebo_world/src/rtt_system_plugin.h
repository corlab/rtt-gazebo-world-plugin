#ifndef __RTT_GAZEBO_DEPLOYER_RTT_SYSTEM_H
#define __RTT_GAZEBO_DEPLOYER_RTT_SYSTEM_H

#include <cstdlib>

// Boost
#include <boost/bind.hpp>

// Gazebo
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

// Orocos
#include <rtt/deployment/ComponentLoader.hpp>
#include <ocl/DeploymentComponent.hpp>
#include <ocl/TaskBrowser.hpp>
#include <ocl/LoggingService.hpp>
#include <rtt/Logger.hpp>


#include <rtt/os/startstop.h>
#include <rtt/scripting/Scripting.hpp>
#include <rtt/transports/corba/corba.h>
#include <rtt/transports/corba/TaskContextServer.hpp>

#include <rtt_rosclock/rtt_rosclock.h>

namespace rtt_gazebo_world {

  class RTTSystemPlugin : public gazebo::WorldPlugin
  {
  public:
    //! Initialize RTT (__os_init), rtt_rosclock sim clock, and CORBA
    RTTSystemPlugin();
    //! Disconnect the world event and cleanup CORBA
    ~RTTSystemPlugin();

    //! Set the gazebo world (as a time/trigger source)
    void connectWorld(gazebo::physics::WorldPtr world);

    /**
     * \brief Update the RTT clock from the gazebo clock
     *
     * This queries the Gazebo time, and then uses the rtt_rosclock Orocos
     * plugin to both set the RTT::os::TimeService time and trigger any
     * periodic sim clock components using SimClockActivity.
     */
    void updateClock();

    void Load(gazebo::physics::WorldPtr world, sdf::ElementPtr sdf);

  private:

    void initialize();

    //! Gazebo world to get time from
    gazebo::physics::WorldPtr world_;

    //! Event connection to the world update
    gazebo::event::ConnectionPtr update_connection_;
  };

}

#endif // ifndef __RTT_GAZEBO_DEPLOYER_RTT_SYSTEM_H
