// We'll use a string and the gzmsg command below for a brief example.
// Remove these includes if your plugin doesn't need them.
#include <string>
#include <gz/common/Console.hh>

// This header is required to register plugins. It's good practice to place it
// in the cc file, like it's done here.
#include <gz/plugin/Register.hh>

// Don't forget to include the plugin's header.
#include "ros_gz_example_gazebo/SwerveSystem.hh"

// This is required to register the plugin. Make sure the interfaces match
// what's in the header.
IGNITION_ADD_PLUGIN(
    ros_gz_example_gazebo::SwerveSystem,
    gz::sim::System,
    ros_gz_example_gazebo::SwerveSystem::ISystemConfigure,
    ros_gz_example_gazebo::SwerveSystem::ISystemPreUpdate,
    ros_gz_example_gazebo::SwerveSystem::ISystemUpdate,
    ros_gz_example_gazebo::SwerveSystem::ISystemPostUpdate
)

using namespace std::placeholders;

namespace ros_gz_example_gazebo 
{

void SwerveSystem::Configure(const gz::sim::Entity &_entity,
                const std::shared_ptr<const sdf::Element> &_element,
                gz::sim::EntityComponentManager &_ecm,
                gz::sim::EventManager &_eventManager)
{
  std::cout << "ros_gz_example_gaebo::SwerveSystem::Configure on entity: " << _entity << std::endl;

  // The subscribe function requires a ref to this own object as handle_command is non static
  // and thus requires to be tied to an object
  using handle_back_left_command = std::function<void(const gz::msgs::Twist &_msg)>;
  using handle_back_right_command = std::function<void(const gz::msgs::Twist &_msg)>;
  using handle_front_left_command = std::function<void(const gz::msgs::Twist &_msg)>;
  using handle_front_right_command = std::function<void(const gz::msgs::Twist &_msg)>;

  handle_back_left_command handle_bl_command = std::bind(&SwerveSystem::handle_command, this, std::placeholders::_1, "back_left_wheel_joint", "back_left_steering_joint");
  handle_back_right_command handle_br_command = std::bind(&SwerveSystem::handle_command, this, std::placeholders::_1, "back_right_wheel_joint", "back_right_steering_joint");
  handle_front_left_command handle_fl_command = std::bind(&SwerveSystem::handle_command, this, std::placeholders::_1, "front_left_wheel_joint", "front_left_steering_joint");
  handle_front_right_command handle_fr_command = std::bind(&SwerveSystem::handle_command, this, std::placeholders::_1, "front_right_wheel_joint", "front_right_steering_joint");

  this->node.Subscribe("/Direction/BackLeft", handle_bl_command);
  this->node.Subscribe("/Direction/BackRight", handle_br_command);
  this->node.Subscribe("/Direction/FrontLeft", handle_fl_command);
  this->node.Subscribe("/Direction/FrontRight", handle_fr_command);

  this->model = gz::sim::Model(_entity);

  // Initialize joint to vel map
  for (std::string drive_name : DRIVE_UNITS) {
    this->motor_name_to_velocity[drive_name] = 0;
  }

  for (std::string drive_name : STEERING_UNITS) {
    this->motor_name_to_rot[drive_name] = 0;
  }
}

void SwerveSystem::PreUpdate(const gz::sim::UpdateInfo &_info,
                           gz::sim::EntityComponentManager &_ecm)
{
  if (!_info.paused && _info.iterations % 1000 == 0)
  {
    igndbg << "ros_gz_example_gazebo::SwerveSystem::PreUpdate" << std::endl;
  }

  // Motor control refresh
    if (divider_accum >= 1 / CONTROL_LOOP_SPEED) {
      divider_accum = 0;
    this->update_motor_state(_ecm);
  }
  else {
    divider_accum += std::chrono::duration<double>(_info.dt).count();
  }

  // Update velocity
  for (std::string drive_unit : DRIVE_UNITS) {

    gz::sim::Entity drive_unit_e = this->model.JointByName(_ecm, drive_unit);


    if (drive_unit_e != ignition::gazebo::v6::kNullEntity) {

      auto drive_value = this->motor_name_to_velocity.find(drive_unit);

      _ecm.SetComponentData<components::JointForceCmd>(drive_unit_e,
      {drive_value->second});
    }
  }
}

// TODO: To implement better motor control
void SwerveSystem::update_motor_state(gz::sim::EntityComponentManager &_ecm) {
  for (std::string drive_unit : STEERING_UNITS) {

    // Get sim object for drive unit
    gz::sim::Entity drive_unit_e = this->model.JointByName(_ecm, drive_unit);

    auto desired_rot = this->motor_name_to_rot.find(drive_unit);

    if (drive_unit_e != ignition::gazebo::v6::kNullEntity 
        && desired_rot != this->motor_name_to_velocity.end()) {

      // If of type steering, perform control loop logic to set direction
      ignition::gazebo::v6::components::JointPosition *curr_joint_pos = 
        _ecm.Component<components::JointPosition>(drive_unit_e);

      if (curr_joint_pos && !curr_joint_pos->Data().empty()) {
        double encoder_val = curr_joint_pos->Data()[0];
        double target_angle = desired_rot->second;

        // std::cout << "Current angle " << encoder_val << std::endl;
        // std::cout << "Desired angle " << target_angle << std::endl;

        // Normalize diff to get optimal direction to travel
        double angular_dir = std::fmod(std::fmod(encoder_val, 2 * M_PI) + 2 * M_PI, 2 * M_PI);
        angular_dir = target_angle - angular_dir;

        double desired_vel = PID_P * angular_dir;
        desired_vel = desired_vel;

        this->motor_name_to_velocity[drive_unit] = desired_vel;
      }
      else {
        _ecm.CreateComponent(drive_unit_e, components::JointPosition());
      }
    }
  }
}

void SwerveSystem::Update(const gz::sim::UpdateInfo &_info,
                        gz::sim::EntityComponentManager &_ecm)
{
  if (!_info.paused && _info.iterations % 1000 == 0)
  {
    igndbg << "ros_gz_example_gazebo::SwerveSystem::Update" << std::endl;
  }
}

void SwerveSystem::PostUpdate(const gz::sim::UpdateInfo &_info,
                            const gz::sim::EntityComponentManager &_ecm) 
{
  if (!_info.paused && _info.iterations % 1000 == 0)
  {
    igndbg << "ros_gz_example_gazebo::SwerveSystem::PostUpdate" << std::endl;
  }
}

void SwerveSystem::handle_command(const gz::msgs::Twist &_msg, std::string _mt, std::string _rot_mt) {

  std::cout << "getting here" << std::endl;
    
  // If drive request on motor drive unit that does not exist:
  if (motor_name_to_velocity.find(_mt) == motor_name_to_velocity.end()) 
    return;

  this->motor_name_to_velocity[_mt] = _msg.linear().x();
  this->motor_name_to_rot[_rot_mt] = _msg.angular().x();
}
}  // namespace ros_gz_example_gazebo
