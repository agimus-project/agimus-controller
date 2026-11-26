// Shared fixtures of test_robot_models.cpp and test_robot_models_more.cpp,
// compiled once so that each test file stays under the compile-RAM limit.
#pragma once

#include <cstdlib>
#include <filesystem>
#include <pinocchio/multibody/geometry.hpp>
#include <pinocchio/multibody/model.hpp>
#include <string>
#include <vector>

#include "agimus_controller/robot_models.hpp"

namespace robot_models_test {

namespace fs = std::filesystem;
using agimus_controller::RobotModelParameters;

/// Directory of agimus_franka_description under AMENT_PREFIX_PATH, or "".
std::string franka_description_dir();
/// agimus_franka_description and xacro are both found.
bool franka_description_found();
std::string read_file(const fs::path& path);
/// Runs `xacro <args>` and returns its output.
std::string run_xacro(const std::string& args);

// Example robot data's panda.
extern const fs::path kModelDir;
extern const fs::path kPandaUrdf;
extern const fs::path kPandaSrdf;
extern const fs::path kMeshesDir;
/// The Python environment.xacro as pinocchio reads it.
extern const char* const kEnvironmentUrdf;
std::string environment_urdf_path();
std::vector<std::string> panda_moving_joints();
RobotModelParameters panda_params();

/// Empties the variables where pinocchio searches for meshes, and restores
/// them.
class ScopedNoPackagePaths {
 public:
  ScopedNoPackagePaths() {
    for (const char* name : kNames) {
      const char* value = std::getenv(name);
      saved_.push_back(value == nullptr ? std::string() : value);
      had_.push_back(value != nullptr);
      setenv(name, "", 1);
    }
  }
  ~ScopedNoPackagePaths() {
    for (std::size_t i = 0; i < saved_.size(); ++i) {
      if (had_[i]) {
        setenv(kNames[i], saved_[i].c_str(), 1);
      } else {
        unsetenv(kNames[i]);
      }
    }
  }

 private:
  static constexpr const char* kNames[2] = {"ROS_PACKAGE_PATH",
                                            "AMENT_PREFIX_PATH"};
  std::vector<std::string> saved_;
  std::vector<bool> had_;
};

std::vector<std::string> geometry_names(const pinocchio::GeometryModel& m);

/// example-robot-data's ur5 (full model, as the Python warm-start test).
/// Loaded once.
const pinocchio::Model& ur5_model();

/// A tiny robot, for what the panda does not cover.
extern const char* const kTinyUrdf;
RobotModelParameters tiny_params();

/// agimus_franka_description's FER with the Python test environment.
RobotModelParameters franka_params();

}  // namespace robot_models_test
