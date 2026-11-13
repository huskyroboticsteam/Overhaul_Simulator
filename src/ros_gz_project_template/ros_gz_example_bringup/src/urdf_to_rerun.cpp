// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Stream a URDF robot (from /robot_description + TF) to a Rerun viewer.
//
// Each unique mesh is sent to the viewer once; every place the URDF uses it becomes
// an instance pose, so large CAD exports with heavily reused meshes stay cheap.
// Instance poses are re-sent only when the TF of a link that uses the mesh changes.
// The Rerun C++ SDK version should match the viewer's.

#include <cctype>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rerun.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <urdf/model.h>

namespace
{

struct Instance
{
  std::string link;
  tf2::Transform link_to_visual;
  rerun::components::Scale3D scale;
};

struct MeshEntity
{
  std::string entity_path;
  std::vector<Instance> instances;
};

std::string resolve_uri(const std::string & uri)
{
  const std::string package_prefix = "package://";
  const std::string file_prefix = "file://";
  if (uri.rfind(package_prefix, 0) == 0) {
    const auto rest = uri.substr(package_prefix.size());
    const auto slash = rest.find('/');
    const auto pkg = rest.substr(0, slash);
    const auto rel = slash == std::string::npos ? "" : rest.substr(slash + 1);
    return ament_index_cpp::get_package_share_directory(pkg) + "/" + rel;
  }
  if (uri.rfind(file_prefix, 0) == 0) {
    return uri.substr(file_prefix.size());
  }
  return uri;
}

std::string entity_name(const std::string & path, std::set<std::string> & used)
{
  std::string base = std::filesystem::path(path).stem().string();
  for (auto & c : base) {
    if (!std::isalnum(static_cast<unsigned char>(c)) && c != '_') {
      c = '_';
    }
  }
  if (base.empty()) {
    base = "mesh";
  }
  std::string name = base;
  for (int i = 1; used.count(name); ++i) {
    name = base + "_" + std::to_string(i);
  }
  used.insert(name);
  return name;
}

tf2::Transform to_tf2(const urdf::Pose & pose)
{
  return tf2::Transform(
    tf2::Quaternion(pose.rotation.x, pose.rotation.y, pose.rotation.z, pose.rotation.w),
    tf2::Vector3(pose.position.x, pose.position.y, pose.position.z));
}

bool nearly_equal(const tf2::Transform & a, const tf2::Transform & b)
{
  constexpr double eps = 1e-6;
  return a.getOrigin().distance2(b.getOrigin()) < eps * eps &&
         std::abs(std::abs(a.getRotation().dot(b.getRotation())) - 1.0) < eps;
}

}  // namespace

class UrdfToRerun : public rclcpp::Node
{
public:
  UrdfToRerun()
  : Node("urdf_to_rerun"),
    rec_(declare_parameter<std::string>("application_id", "robot"))
  {
    const auto url = declare_parameter<std::string>(
      "rerun_url", "rerun+http://host.docker.internal:9876/proxy");
    fixed_frame_ = declare_parameter<std::string>("fixed_frame", "map");
    entity_root_ = declare_parameter<std::string>("entity_root", "robot");
    const auto rate = declare_parameter<double>("update_rate", 20.0);

    if (auto err = rec_.connect_grpc(url); err.is_err()) {
      throw std::runtime_error("Failed to connect to Rerun at " + url + ": " + err.description);
    }
    rec_.log_static("/", rerun::ViewCoordinates::RIGHT_HAND_Z_UP);
    RCLCPP_INFO(get_logger(), "Streaming to Rerun viewer at %s", url.c_str());

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    description_sub_ = create_subscription<std_msgs::msg::String>(
      "robot_description", rclcpp::QoS(1).transient_local(),
      [this](const std_msgs::msg::String & msg) {on_description(msg.data);});
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate), [this]() {on_timer();});
  }

private:
  void on_description(const std::string & urdf_xml)
  {
    // Several publishers (or a re-latched message) can deliver the same description;
    // don't resend hundreds of MB of meshes for it.
    if (urdf_xml == last_description_) {
      return;
    }
    last_description_ = urdf_xml;

    urdf::Model model;
    if (!model.initString(urdf_xml)) {
      RCLCPP_ERROR(get_logger(), "Failed to parse robot_description");
      return;
    }

    meshes_.clear();
    links_.clear();
    last_link_pose_.clear();
    rec_.log_static(entity_root_ + "/meshes", rerun::Clear::RECURSIVE);
    rec_.log_static(entity_root_ + "/shapes", rerun::Clear::RECURSIVE);

    // Group every mesh visual by the mesh file it uses; primitives are logged directly
    std::map<std::string, std::vector<Instance>> by_path;
    size_t total = 0;
    size_t shapes = 0;
    for (const auto & [link_name, link] : model.links_) {
      for (const auto & visual : link->visual_array) {
        if (!visual || !visual->geometry) {
          continue;
        }
        if (visual->geometry->type != urdf::Geometry::MESH) {
          if (log_primitive(link_name, *visual, shapes)) {
            ++shapes;
          }
          continue;
        }
        const auto mesh = std::static_pointer_cast<urdf::Mesh>(visual->geometry);
        std::string path;
        try {
          path = resolve_uri(mesh->filename);
        } catch (const std::exception & e) {
          RCLCPP_WARN(get_logger(), "Cannot resolve %s: %s", mesh->filename.c_str(), e.what());
          continue;
        }
        by_path[path].push_back(Instance{
            link_name, to_tf2(visual->origin),
            rerun::components::Scale3D(
              static_cast<float>(mesh->scale.x), static_cast<float>(mesh->scale.y),
              static_cast<float>(mesh->scale.z))});
        ++total;
      }
    }

    RCLCPP_INFO(
      get_logger(), "Sending %zu meshes (%zu instances) and %zu primitive shapes to Rerun...",
      by_path.size(), total, shapes);

    std::set<std::string> used;
    for (auto & [path, instances] : by_path) {
      auto asset = rerun::Asset3D::from_file_path(path);
      if (asset.is_err()) {
        RCLCPP_WARN(
          get_logger(), "Skipping mesh %s: %s", path.c_str(), asset.error.description.c_str());
        continue;
      }
      const auto entity = entity_root_ + "/meshes/" + entity_name(path, used);
      rec_.log_static(entity, asset.value);
      for (const auto & inst : instances) {
        links_.insert(inst.link);
      }
      meshes_.push_back(MeshEntity{entity, std::move(instances)});
    }
    RCLCPP_INFO(get_logger(), "Geometry sent; waiting for TF to place it.");
  }

  // Box / cylinder / sphere: logged once, centred on the visual origin, then placed
  // through the same instance-pose path as meshes.
  bool log_primitive(const std::string & link_name, const urdf::Visual & visual, size_t index)
  {
    rerun::Color color(200, 200, 200);
    if (visual.material) {
      const auto & c = visual.material->color;
      color = rerun::Color(
        static_cast<uint8_t>(c.r * 255), static_cast<uint8_t>(c.g * 255),
        static_cast<uint8_t>(c.b * 255), static_cast<uint8_t>(c.a * 255));
    }

    const auto entity = entity_root_ + "/shapes/" + std::to_string(index);
    switch (visual.geometry->type) {
      case urdf::Geometry::BOX: {
          const auto & d = std::static_pointer_cast<urdf::Box>(visual.geometry)->dim;
          rec_.log_static(
            entity, rerun::Boxes3D::from_half_sizes(
              {{static_cast<float>(d.x / 2), static_cast<float>(d.y / 2),
                static_cast<float>(d.z / 2)}})
            .with_colors(color)
            .with_fill_mode(rerun::components::FillMode::Solid));
          break;
        }
      case urdf::Geometry::CYLINDER: {
          const auto cyl = std::static_pointer_cast<urdf::Cylinder>(visual.geometry);
          // Rerun cylinders, like URDF ones, are centred and run along local Z
          rec_.log_static(
            entity, rerun::Cylinders3D::from_lengths_and_radii(
              {static_cast<float>(cyl->length)}, {static_cast<float>(cyl->radius)})
            .with_colors(color)
            .with_fill_mode(rerun::components::FillMode::Solid));
          break;
        }
      case urdf::Geometry::SPHERE: {
          const auto r = static_cast<float>(
            std::static_pointer_cast<urdf::Sphere>(visual.geometry)->radius);
          rec_.log_static(
            entity, rerun::Ellipsoids3D::from_radii({r})
            .with_colors(color)
            .with_fill_mode(rerun::components::FillMode::Solid));
          break;
        }
      default:
        return false;
    }
    links_.insert(link_name);
    meshes_.push_back(MeshEntity{
        entity, {Instance{link_name, to_tf2(visual.origin), rerun::components::Scale3D(1.0f)}}});
    return true;
  }

  void on_timer()
  {
    if (meshes_.empty()) {
      return;
    }

    std::unordered_map<std::string, tf2::Transform> poses;
    std::vector<std::string> missing;
    for (const auto & link : links_) {
      try {
        const auto tf = tf_buffer_->lookupTransform(fixed_frame_, link, tf2::TimePointZero);
        const auto & t = tf.transform.translation;
        const auto & r = tf.transform.rotation;
        poses[link] = tf2::Transform(
          tf2::Quaternion(r.x, r.y, r.z, r.w), tf2::Vector3(t.x, t.y, t.z));
      } catch (const tf2::TransformException &) {
        missing.push_back(link);
      }
    }
    if (!missing.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Waiting for TF %s -> %zu links (e.g. %s)",
        fixed_frame_.c_str(), missing.size(), missing.front().c_str());
      return;
    }

    std::set<std::string> changed;
    for (const auto & [link, pose] : poses) {
      const auto it = last_link_pose_.find(link);
      if (it == last_link_pose_.end() || !nearly_equal(it->second, pose)) {
        changed.insert(link);
      }
    }
    if (changed.empty()) {
      return;
    }
    if (last_link_pose_.empty()) {
      RCLCPP_INFO(get_logger(), "Placed all %zu links in Rerun.", poses.size());
    }
    for (const auto & link : changed) {
      last_link_pose_[link] = poses[link];
    }

    rec_.set_time_timestamp_nanos_since_epoch("ros_time", now().nanoseconds());
    for (const auto & mesh : meshes_) {
      bool affected = false;
      for (const auto & inst : mesh.instances) {
        if (changed.count(inst.link)) {
          affected = true;
          break;
        }
      }
      if (!affected) {
        continue;
      }

      std::vector<rerun::components::Translation3D> translations;
      std::vector<rerun::components::RotationQuat> rotations;
      std::vector<rerun::components::Scale3D> scales;
      translations.reserve(mesh.instances.size());
      rotations.reserve(mesh.instances.size());
      scales.reserve(mesh.instances.size());
      for (const auto & inst : mesh.instances) {
        const tf2::Transform world = poses[inst.link] * inst.link_to_visual;
        const auto & o = world.getOrigin();
        const auto q = world.getRotation();
        translations.emplace_back(
          static_cast<float>(o.x()), static_cast<float>(o.y()), static_cast<float>(o.z()));
        rotations.emplace_back(rerun::datatypes::Quaternion::from_xyzw(
            static_cast<float>(q.x()), static_cast<float>(q.y()),
            static_cast<float>(q.z()), static_cast<float>(q.w())));
        scales.push_back(inst.scale);
      }
      rec_.log(
        mesh.entity_path,
        rerun::InstancePoses3D()
        .with_translations(translations)
        .with_quaternions(rotations)
        .with_scales(scales));
    }
  }

  rerun::RecordingStream rec_;
  std::string fixed_frame_;
  std::string entity_root_;
  std::string last_description_;

  std::vector<MeshEntity> meshes_;
  std::set<std::string> links_;
  std::unordered_map<std::string, tf2::Transform> last_link_pose_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr description_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UrdfToRerun>());
  rclcpp::shutdown();
  return 0;
}
