/*
 * Copyright (C) 2022 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#ifndef ROS_GZ_EXAMPLE_GAZEBO__FULL_SYSTEM_HH_
#define ROS_GZ_EXAMPLE_GAZEBO__FULL_SYSTEM_HH_

// The only required include in the header is this one.
// All others will depend on what your plugin does.
#include <gz/sim/EventManager.hh>
#include <gz/sim/System.hh>
#include <gz/msgs/twist.pb.h>
#include <gz/transport/Node.hh>   
#include <gz/sim/Model.hh>        // For creating gazebo transport sub pub
#include <gz/sim/Entity.hh>
#include <gz/sim/components.hh>

using namespace gz;
using namespace sim;
using namespace sim;

namespace ros_gz_example_gazebo
{

    // Wrapper
  class DriveRequest {
    public:
      std::string motor {""};
      gz::msgs::Twist request_state;
  };

  const std::vector<std::string> DRIVE_UNITS{"back_left_steering_joint",
                                              "back_right_steering_joint",
                                              "front_left_steering_joint",
                                              "front_right_steering_joint",
                                              "back_left_wheel_joint",
                                              "back_right_wheel_joint",
                                              "front_left_wheel_joint",
                                              "front_right_wheel_joint"};

  const std::set<std::string> STEERING_UNITS{"back_left_steering_joint"};

  // PID constants
  const double SUCCESS_THRESH = 0.1;
  const double CONTROL_LOOP_SPEED = 1000; // Speed in hz
  const double PID_P = 20.0;
  const double PID_I = 0.1;
  const double PID_D = 0.1;

  // This is the main plugin's class. It must inherit from System and at least
  // one other interface.
  // Here we use `ISystemPostUpdate`, which is used to get results after
  // physics runs. The opposite of that, `ISystemPreUpdate`, would be used by
  // plugins that want to send commands.
  class SwerveSystem:
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate,
    public gz::sim::ISystemUpdate,
    public gz::sim::ISystemPostUpdate
  {
    // Plugins inheriting ISystemConfigure must implement the Configure 
    // callback. This is called when a system is initially loaded. 
    // The _entity variable contains the entity that the system is attached to
    // The _element variable contains the sdf Element with custom configuration
    // The _ecm provides an interface to all entities and components
    // The _eventManager provides a mechanism for registering internal signals
    public: void Configure(
                const gz::sim::Entity &_entity,
                const std::shared_ptr<const sdf::Element> &_element,
                gz::sim::EntityComponentManager &_ecm,
                gz::sim::EventManager &_eventManager) override;

    // Plugins inheriting ISystemPreUpdate must implement the PreUpdate
    // callback. This is called at every simulation iteration before the physics
    // updates the world. The _info variable provides information such as time,
    // while the _ecm provides an interface to all entities and components in
    // simulation.
    public: void PreUpdate(const gz::sim::UpdateInfo &_info,
                gz::sim::EntityComponentManager &_ecm) override;

    // Plugins inheriting ISystemUpdate must implement the Update
    // callback. This is called at every simulation iteration before the physics
    // updates the world. The _info variable provides information such as time,
    // while the _ecm provides an interface to all entities and components in
    // simulation.
    public: void Update(const gz::sim::UpdateInfo &_info,
                gz::sim::EntityComponentManager &_ecm) override;

    // Plugins inheriting ISystemPostUpdate must implement the PostUpdate
    // callback. This is called at every simulation iteration after the physics
    // updates the world. The _info variable provides information such as time,
    // while the _ecm provides an interface to all entities and components in
    // simulation.
    public: void PostUpdate(const gz::sim::UpdateInfo &_info,
                const gz::sim::EntityComponentManager &_ecm) override;

    private:
      void handle_command(const gz::msgs::Twist &_msg, std::string _mt, std::string _rot_mt);

      void update_motor_state(gz::sim::EntityComponentManager &_ecm);

      // Object that acts as subscriber and publisher for all inter
      // ROS communication 
      gz::transport::Node node;

      // Reference to the current model stored during the config state
      gz::sim::Model model{ignition::gazebo::v6::kNullEntity};

      std::map<std::string, double> motor_name_to_velocity {};

      std::map<std::string, double> motor_name_to_rot {};
      
      std::vector<DriveRequest> rover_request;

      double divider_accum;
  };
}
#endif
