#pragma once

#include <Eigen/Core>
#include <pinocchio/multibody/geometry.hpp>
#include <pinocchio/multibody/model.hpp>
#include <string>
#include <utility>
#include <vector>

namespace agimus_controller {

/// Parameters to load the robot models (Python RobotModelParameters).
///
/// Python accepts a Path or an XML str for the URDF, the environment URDF and
/// the SRDF. Here each is a pair of strings, `*_xml` and `*_path`: at most one
/// of the pair is set, and an empty string means "not set".
/// Fill the inputs, then call finalize().
struct RobotModelParameters {
  /// Initial full configuration; empty means the neutral configuration.
  Eigen::VectorXd q0;
  bool free_flyer = false;
  std::vector<std::string> moving_joint_names;

  std::string robot_urdf_xml;
  std::string robot_urdf_path;
  std::string env_urdf_xml;   // Optional environment.
  std::string env_urdf_path;  // Optional environment.
  std::string srdf_xml;       // Optional.
  std::string srdf_path;      // Optional.
  /// Frame of the environment the robot is attached to; empty: the universe.
  std::string robot_attachment_frame;
  /// Directory where the URDF's package:// meshes are searched, besides the
  /// ROS package paths. Empty: not set.
  std::string urdf_meshes_dir;
  /// Replace each cylinder `<base>_<i>` of the collision model and its two
  /// end spheres `<base>_<i+1>`, `<base>_<i+2>` by a coal capsule, and drop
  /// the meshes.
  bool collision_as_capsule = false;
  /// Extra collision pairs, as geometry object names.
  std::vector<std::pair<std::string, std::string>> collision_pairs;
  /// Add all collision pairs, minus the ones disabled in the SRDF.
  bool self_collision = false;
  /// Armature of the moving joints; empty: zeros.
  Eigen::VectorXd armature;
  Eigen::Vector4d collision_color =
      Eigen::Vector4d(249.0, 136.0, 126.0, 125.0) / 255.0;

  /// Setup: fills the default armature and validates the inputs (Python
  /// __post_init__). Throws std::invalid_argument for a bad value and
  /// std::runtime_error for a missing file or directory, naming the item.
  void finalize();
};

/// Parses the robot model, reduces it and filters the collision model.
/// Setup only: the models are built by the constructor and never change.
class RobotModels {
 public:
  /// Calls `params.finalize()`, then loads and prepares the models. Throws
  /// std::invalid_argument (bad input) or std::runtime_error (missing file).
  explicit RobotModels(RobotModelParameters params);

  const RobotModelParameters& params() const { return params_; }
  /// Robot, with the environment appended, before locking joints.
  const pinocchio::Model& full_robot_model() const { return full_robot_model_; }
  /// Robot model reduced to the moving joints.
  const pinocchio::Model& robot_model() const { return robot_model_; }
  const pinocchio::GeometryModel& collision_model() const {
    return collision_model_;
  }
  const pinocchio::GeometryModel& visual_model() const { return visual_model_; }
  const Eigen::VectorXd& armature() const { return params_.armature; }

 private:
  RobotModelParameters params_;
  pinocchio::Model full_robot_model_;
  pinocchio::Model robot_model_;
  pinocchio::GeometryModel collision_model_;
  pinocchio::GeometryModel visual_model_;
};

}  // namespace agimus_controller
