#include "robot_models_fixtures.hpp"

#include <doctest/doctest.h>

#include <cstdio>
#include <fstream>
#include <memory>
#include <pinocchio/parsers/urdf.hpp>
#include <sstream>

namespace robot_models_test {

// --- Optional agimus_franka_description (and xacro) -------------------------

/// Directory of agimus_franka_description under AMENT_PREFIX_PATH, or "".
std::string franka_description_dir() {
  const char* prefixes = std::getenv("AMENT_PREFIX_PATH");
  if (prefixes == nullptr) return "";
  std::stringstream stream(prefixes);
  std::string prefix;
  while (std::getline(stream, prefix, ':')) {
    const auto dir = fs::path(prefix) / "share" / "agimus_franka_description";
    if (!prefix.empty() && fs::is_directory(dir)) return dir.string();
  }
  return "";
}

bool xacro_found() {
  const char* path = std::getenv("PATH");
  if (path == nullptr) return false;
  std::stringstream stream(path);
  std::string dir;
  while (std::getline(stream, dir, ':')) {
    if (!dir.empty() && fs::is_regular_file(fs::path(dir) / "xacro")) {
      return true;
    }
  }
  return false;
}

bool franka_description_found() {
  return !franka_description_dir().empty() && xacro_found();
}

std::string read_file(const fs::path& path) {
  std::ifstream file(path);
  REQUIRE_MESSAGE(file.is_open(), "cannot open " << path);
  std::stringstream buffer;
  buffer << file.rdbuf();
  return buffer.str();
}

/// Runs `xacro <args>` and returns its output.
std::string run_xacro(const std::string& args) {
  std::unique_ptr<FILE, int (*)(FILE*)> pipe(
      popen(("xacro " + args).c_str(), "r"), pclose);
  REQUIRE_MESSAGE(pipe != nullptr, "cannot run xacro");
  std::string output;
  char buffer[4096];
  while (std::fgets(buffer, sizeof(buffer), pipe.get()) != nullptr) {
    output += buffer;
  }
  return output;
}

// --- Example robot data's panda -------------------------------------------

const fs::path kModelDir = AGIMUS_EXAMPLE_ROBOT_DATA_DIR;
const fs::path kPandaUrdf =
    kModelDir / "robots" / "panda_description" / "urdf" / "panda.urdf";
const fs::path kPandaSrdf =
    kModelDir / "robots" / "panda_description" / "srdf" / "panda.srdf";
// package://example-robot-data/... is resolved against its parent.
const fs::path kMeshesDir = kModelDir.parent_path();

// The Python environment.xacro as pinocchio reads it: the xacro macro is an
// unknown element and ignored, so the obstacle only has a visual.
const char* const kEnvironmentUrdf = R"(<?xml version="1.0"?>
<robot name="environment">
  <material name="grey"><color rgba=".5 .5 .5 1"/></material>
  <link name="obstacle1">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="0.6"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0.0155" rpy="0 0 0"/>
      <geometry><box size="0.597 0.397 0.001"/></geometry>
      <material name="grey"/>
    </visual>
  </link>
</robot>
)";

std::string environment_urdf_path() {
  static const std::string path = [] {
    const auto file =
        fs::temp_directory_path() /
        ("agimus_test_environment_" + std::to_string(std::rand()) + ".urdf");
    std::ofstream(file) << kEnvironmentUrdf;
    return file.string();
  }();
  return path;
}

std::vector<std::string> panda_moving_joints() {
  std::vector<std::string> names;
  for (int i = 3; i < 8; ++i)
    names.push_back("panda_joint" + std::to_string(i));
  return names;
}

RobotModelParameters panda_params() {
  RobotModelParameters p;
  p.q0 = Eigen::VectorXd::Zero(9);
  p.free_flyer = false;
  p.moving_joint_names = panda_moving_joints();
  p.robot_urdf_path = kPandaUrdf.string();
  p.env_urdf_path = environment_urdf_path();
  p.srdf_path = kPandaSrdf.string();
  p.urdf_meshes_dir = kMeshesDir.string();
  p.collision_as_capsule = true;
  p.self_collision = true;
  p.armature = Eigen::VectorXd::LinSpaced(5, 0.1, 0.9);
  return p;
}

std::vector<std::string> geometry_names(const pinocchio::GeometryModel& m) {
  std::vector<std::string> names;
  for (const auto& object : m.geometryObjects) names.push_back(object.name);
  return names;
}

const pinocchio::Model& ur5_model() {
  static const pinocchio::Model model = [] {
    pinocchio::Model m;
    pinocchio::urdf::buildModel(
        (kModelDir / "robots" / "ur_description" / "urdf" / "ur5_robot.urdf")
            .string(),
        m);
    return m;
  }();
  return model;
}

// --- A tiny robot, for what the panda does not cover ------------------------

const char* const kTinyUrdf = R"(<?xml version="1.0"?>
<robot name="tiny">
  <link name="base"/>
  <link name="link1">
    <inertial><mass value="1"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial>
    <collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision>
    <collision><origin xyz="0 0 0.2"/><geometry><cylinder radius="0.05" length="0.3"/></geometry></collision>
    <collision><origin xyz="0 0 0.05"/><geometry><sphere radius="0.05"/></geometry></collision>
    <collision><origin xyz="0 0 0.35"/><geometry><sphere radius="0.05"/></geometry></collision>
  </link>
  <link name="link2">
    <inertial><mass value="1"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial>
    <collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision>
  </link>
  <joint name="j1" type="revolute">
    <parent link="base"/><child link="link1"/><axis xyz="0 0 1"/>
    <limit lower="-1" upper="1" effort="1" velocity="1"/>
  </joint>
  <joint name="j2" type="revolute">
    <parent link="link1"/><child link="link2"/><origin xyz="0 0 1"/><axis xyz="0 1 0"/>
    <limit lower="-1" upper="1" effort="1" velocity="1"/>
  </joint>
</robot>
)";

RobotModelParameters tiny_params() {
  RobotModelParameters p;
  p.robot_urdf_xml = kTinyUrdf;
  p.moving_joint_names = {"j1", "j2"};
  return p;
}

const char* const kEnvironmentXacro = R"(<?xml version='1.0' encoding='utf-8'?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="environment">
    <xacro:include filename="$(find agimus_franka_description)/robots/common/utils.xacro" />
    <material name="grey">
        <color rgba=".5 .5 .5 1" />
    </material>
    <link name="obstacle1">
        <inertial>
            <origin xyz="0.0 0.0 0.0" rpy="0 0 0" />
            <mass value="0.6" />
            <inertia ixx="0.001" ixy="0.0" ixz="0.0"
                iyy="0.001" iyz="0.0"
                izz="0.001" />
        </inertial>
        <visual>
            <origin xyz="0 0 0.0155" rpy="0 0 0" />
            <geometry>
                <box size="0.597 0.397 0.001" />
            </geometry>
            <color name="grey" />
        </visual>
        <xacro:collision_capsule xyz="1.535 0 0.43" direction="x" radius="0.1"
            length="0.4" />
    </link>
</robot>
)";

RobotModelParameters franka_params() {
  static const RobotModelParameters params = [] {
    const fs::path dir = franka_description_dir();
    const fs::path fer = dir / "robots" / "fer";
    const auto env_xacro =
        fs::temp_directory_path() /
        ("agimus_test_environment_" + std::to_string(std::rand()) + ".xacro");
    std::ofstream(env_xacro) << kEnvironmentXacro;

    RobotModelParameters p;
    p.robot_urdf_xml =
        run_xacro((fer / "fer.urdf.xacro").string() + " with_sc:=true");
    p.env_urdf_xml = run_xacro(env_xacro.string());
    p.srdf_xml = read_file(fer / "fer.srdf");
    // Hack for the moving joint names: the whole robot is zeros.
    pinocchio::Model model;
    pinocchio::urdf::buildModelFromXML(p.robot_urdf_xml, model);
    p.q0 = Eigen::VectorXd::Zero(model.nq);
    p.free_flyer = false;
    for (int i = 1; i <= 7; ++i) {
      p.moving_joint_names.push_back("fer_joint" + std::to_string(i));
    }
    p.collision_as_capsule = true;
    p.self_collision = true;
    p.urdf_meshes_dir = dir.string();
    p.armature = Eigen::VectorXd::Constant(7, 0.1);
    return p;
  }();
  return params;
}
}  // namespace robot_models_test
