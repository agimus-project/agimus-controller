// Port of agimus_controller/tests/test_robot_models.py, plus tests of the
// behaviour the Python tests do not cover (capsules, collision pairs, the
// xml/path pairs).

#include <coal/shape/geometric_shapes.h>
#include <doctest/doctest.h>

#include <Eigen/Core>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <memory>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "agimus_controller/robot_models.hpp"

using agimus_controller::RobotModelParameters;
using agimus_controller::RobotModels;
namespace fs = std::filesystem;

namespace {

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

std::vector<std::string> geometry_names(const pinocchio::GeometryModel& m) {
  std::vector<std::string> names;
  for (const auto& object : m.geometryObjects) names.push_back(object.name);
  return names;
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

}  // namespace

// --- TestRobotModelParameters ---------------------------------------------

namespace {
RobotModelParameters valid_parameters() {
  RobotModelParameters p = panda_params();
  p.q0 = Eigen::Vector3d(0.0, 1.0, 2.0);
  return p;
}
}  // namespace

TEST_CASE("RobotModelParameters: valid initialization") {
  const auto args = valid_parameters();
  auto params = args;
  params.finalize();
  CHECK(params.free_flyer == args.free_flyer);
  CHECK(params.moving_joint_names == args.moving_joint_names);
  CHECK(params.robot_urdf_path == args.robot_urdf_path);
  CHECK(params.srdf_path == args.srdf_path);
  CHECK(params.urdf_meshes_dir == args.urdf_meshes_dir);
  CHECK(params.collision_as_capsule);
  CHECK(params.self_collision);
  CHECK((params.q0 == args.q0));
  CHECK((params.armature == args.armature));
}

TEST_CASE("RobotModelParameters: armature default value") {
  auto params = valid_parameters();
  params.armature.resize(0);
  params.finalize();
  CHECK((params.armature == Eigen::VectorXd::Zero(5)));
}

TEST_CASE("RobotModelParameters: mismatched armature size throws") {
  auto params = valid_parameters();
  params.armature = Eigen::Vector2d(0.1, 0.2);
  CHECK_THROWS_WITH_AS(params.finalize(), doctest::Contains("armature"),
                       std::invalid_argument);
}

TEST_CASE("RobotModelParameters: invalid URDF throws") {
  auto params = valid_parameters();
  params.robot_urdf_path = "invalid_path";
  CHECK_THROWS_WITH_AS(params.finalize(), doctest::Contains("invalid_path"),
                       std::runtime_error);
  params.robot_urdf_path = "";  // Neither the XML nor the path.
  CHECK_THROWS_WITH_AS(params.finalize(), doctest::Contains("Robot URDF"),
                       std::invalid_argument);
}

TEST_CASE("RobotModelParameters: invalid SRDF path throws") {
  auto params = valid_parameters();
  params.srdf_path = "invalid_path";
  CHECK_THROWS_WITH_AS(params.finalize(), doctest::Contains("SRDF"),
                       std::runtime_error);
}

TEST_CASE("RobotModelParameters: invalid URDF meshes directory throws") {
  auto params = valid_parameters();
  params.urdf_meshes_dir = "invalid_path";
  CHECK_THROWS_WITH_AS(params.finalize(), doctest::Contains("urdf_meshes_dir"),
                       std::runtime_error);
}

TEST_CASE("RobotModelParameters: XML and path both set throws") {
  auto params = valid_parameters();
  params.robot_urdf_xml = "<robot/>";
  CHECK_THROWS_WITH_AS(params.finalize(), doctest::Contains("Robot URDF"),
                       std::invalid_argument);
  params = valid_parameters();
  params.env_urdf_xml = "<robot/>";
  CHECK_THROWS_WITH_AS(params.finalize(), doctest::Contains("Environment URDF"),
                       std::invalid_argument);
  params = valid_parameters();
  params.srdf_xml = "<robot/>";
  CHECK_THROWS_WITH_AS(params.finalize(), doctest::Contains("SRDF"),
                       std::invalid_argument);
}

// --- TestRobotModelsAgainstExampleRobotData -------------------------------

TEST_CASE("RobotModels (panda): no meshes") {
  auto params = panda_params();
  params.urdf_meshes_dir = "";
  ScopedNoPackagePaths no_paths;
  CHECK_THROWS_AS(RobotModels{params}, std::invalid_argument);
}

TEST_CASE("RobotModels (panda): load URDF from string") {
  const RobotModels from_path(panda_params());
  auto params = panda_params();
  params.robot_urdf_xml = read_file(kPandaUrdf);
  params.robot_urdf_path = "";
  const RobotModels from_string(params);
  CHECK((from_string.full_robot_model() == from_path.full_robot_model()));
  CHECK((from_string.robot_model() == from_path.robot_model()));
  CHECK(from_string.collision_model().ngeoms ==
        from_path.collision_model().ngeoms);
  CHECK(from_string.visual_model().ngeoms == from_path.visual_model().ngeoms);
}

TEST_CASE("RobotModels (panda): initial configuration") {
  // The locked joints are frozen at q0; an empty q0 is the neutral
  // configuration, which is zeros for the panda.
  const RobotModels explicit_q0(panda_params());
  auto params = panda_params();
  params.q0.resize(0);
  const RobotModels neutral_q0(params);
  CHECK((neutral_q0.robot_model() == explicit_q0.robot_model()));
  // A q0 that moves a locked joint changes the reduced model.
  params.q0 = Eigen::VectorXd::Zero(9);
  params.q0[0] = 0.5;
  const RobotModels moved_q0(params);
  CHECK_FALSE((moved_q0.robot_model() == explicit_q0.robot_model()));
  // The size is checked against the full model.
  params.q0 = Eigen::VectorXd::Zero(3);
  CHECK_THROWS_WITH_AS(RobotModels{params}, doctest::Contains("q0"),
                       std::invalid_argument);
}

TEST_CASE("RobotModels (panda): load models populates models") {
  const RobotModels models(panda_params());
  CHECK(models.full_robot_model().nq > 0);
  CHECK(models.visual_model().ngeoms > 0);
  CHECK(models.collision_model().ngeoms > 0);
}

TEST_CASE("RobotModels (panda): reduced robot model") {
  const auto params = panda_params();
  const RobotModels models(params);
  CHECK(models.robot_model().nq ==
        static_cast<int>(params.moving_joint_names.size()));
}

TEST_CASE("RobotModels (panda): invalid joint name throws") {
  auto params = panda_params();
  params.moving_joint_names = {"InvalidJoint"};
  params.armature.resize(0);
  CHECK_THROWS_WITH_AS(RobotModels{params}, doctest::Contains("InvalidJoint"),
                       std::invalid_argument);
}

TEST_CASE("RobotModels (panda): armature property") {
  const auto params = panda_params();
  const RobotModels models(params);
  CHECK((models.armature() == params.armature));
}

TEST_CASE("RobotModels (panda): collision pairs") {
  auto params = panda_params();
  params.collision_as_capsule = false;
  const RobotModels models(params);
  // Number of collision pairs in the panda model.
  CHECK(models.collision_model().collisionPairs.size() == 44);
}

TEST_CASE("RobotModels (panda): rnea") {
  const RobotModels models(panda_params());
  const auto& model = models.robot_model();
  auto data = pinocchio::Data(model);
  const Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nq);
  const Eigen::VectorXd v = Eigen::VectorXd::Zero(model.nv);
  const Eigen::VectorXd a = Eigen::VectorXd::Zero(model.nv);
  pinocchio::rnea(model, data, q, v, a);
  CHECK(data.tau.size() == model.nv);
}

TEST_CASE("RobotModels (panda): collision models") {
  const RobotModels models(panda_params());
  const std::vector<std::string> expected_names = {
      "panda_leftfinger_0",  "panda_leftfinger_1",  "panda_leftfinger_2",
      "panda_leftfinger_3",  "panda_rightfinger_0", "panda_rightfinger_1",
      "panda_rightfinger_2", "panda_rightfinger_3"};
  CHECK(geometry_names(models.collision_model()) == expected_names);
  for (const auto& object : models.collision_model().geometryObjects) {
    CHECK_MESSAGE(
        std::dynamic_pointer_cast<const coal::Box>(object.geometry) != nullptr,
        object.name << " is not a box");
  }
}

// --- Tests of the C++ port that Python lacks ------------------------------

TEST_CASE("RobotModels (tiny): cylinder triplet becomes a capsule") {
  auto params = tiny_params();
  params.collision_as_capsule = true;
  const RobotModels models(params);
  // The capsule is appended after the objects that remain.
  const std::vector<std::string> expected = {"link1_0", "link2_0",
                                             "link1_capsule_0"};
  CHECK(geometry_names(models.collision_model()) == expected);
  const auto& capsule = models.collision_model().geometryObjects[2];
  const auto shape =
      std::dynamic_pointer_cast<const coal::Capsule>(capsule.geometry);
  REQUIRE(shape != nullptr);
  CHECK(shape->radius == doctest::Approx(0.05));
  // Bug A17 (ledger), ported as in Python until ruled: coal::Capsule takes
  // the full length, but gets the cylinder's half length (0.15), so the
  // capsule is half as long as the cylinder.
  CHECK(shape->halfLength == doctest::Approx(0.075));
  CHECK((capsule.meshColor == params.collision_color));
}

TEST_CASE("RobotModels (tiny): explicit collision pairs") {
  auto params = tiny_params();
  params.collision_pairs = {{"link1_0", "link2_0"}};
  const RobotModels models(params);
  CHECK(models.collision_model().collisionPairs.size() == 1);

  params.collision_pairs = {{"link1_0", "nothing"}};
  CHECK_THROWS_WITH_AS(RobotModels{params}, doctest::Contains("nothing"),
                       std::invalid_argument);
}

TEST_CASE("RobotModels (tiny): self collision filtered by the SRDF XML") {
  auto params = tiny_params();
  params.self_collision = true;
  const RobotModels all_pairs(params);
  CHECK(all_pairs.collision_model().collisionPairs.size() == 4);
  params.srdf_xml =
      R"(<robot name="tiny"><disable_collisions link1="link1" link2="link2" reason="Adjacent"/></robot>)";
  const RobotModels filtered(params);
  CHECK(filtered.collision_model().collisionPairs.empty());
}

TEST_CASE("RobotModels (tiny): free flyer and locked joint") {
  auto params = tiny_params();
  params.free_flyer = true;
  params.moving_joint_names = {"root_joint", "j1"};
  params.armature.resize(0);
  const RobotModels models(params);
  CHECK(models.full_robot_model().nq == 7 + 2);
  CHECK(models.robot_model().nq == 7 + 1);
}

TEST_CASE("RobotModels (tiny): environment attachment frame") {
  auto params = tiny_params();
  params.env_urdf_xml = kEnvironmentUrdf;
  const RobotModels models(params);
  CHECK(models.full_robot_model().nq == 2);
  CHECK(models.visual_model().ngeoms == 1);  // The obstacle's box.

  params.robot_attachment_frame = "obstacle1";
  const RobotModels attached(params);
  CHECK(attached.robot_model().nq == 2);

  params.robot_attachment_frame = "no_such_frame";
  CHECK_THROWS_WITH_AS(RobotModels{params}, doctest::Contains("no_such_frame"),
                       std::invalid_argument);
}

TEST_CASE("RobotModels (tiny): a broken URDF throws") {
  auto params = tiny_params();
  params.robot_urdf_xml = "<robot>";
  CHECK_THROWS_WITH_AS(RobotModels{params}, doctest::Contains("robot URDF"),
                       std::invalid_argument);
}

// --- TestRobotModelsAgainstFrankaDescription ------------------------------
// Need agimus_franka_description under AMENT_PREFIX_PATH and xacro on PATH.

namespace {

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

}  // namespace

TEST_CASE("RobotModels (franka): no meshes" *
          doctest::skip(!franka_description_found())) {
  auto params = franka_params();
  params.urdf_meshes_dir = "";
  ScopedNoPackagePaths no_paths;
  CHECK_THROWS_AS(RobotModels{params}, std::invalid_argument);
}

TEST_CASE("RobotModels (franka): initial configuration" *
          doctest::skip(!franka_description_found())) {
  const RobotModels explicit_q0(franka_params());
  auto params = franka_params();
  params.q0.resize(0);
  const RobotModels neutral_q0(params);
  CHECK((neutral_q0.robot_model() == explicit_q0.robot_model()));
}

TEST_CASE("RobotModels (franka): load models populates models" *
          doctest::skip(!franka_description_found())) {
  const RobotModels models(franka_params());
  CHECK(models.full_robot_model().nq > 0);
  CHECK(models.visual_model().ngeoms > 0);
  CHECK(models.collision_model().ngeoms > 0);
}

TEST_CASE("RobotModels (franka): reduced robot model" *
          doctest::skip(!franka_description_found())) {
  const auto params = franka_params();
  const RobotModels models(params);
  CHECK(models.robot_model().nq ==
        static_cast<int>(params.moving_joint_names.size()));
}

TEST_CASE("RobotModels (franka): armature property" *
          doctest::skip(!franka_description_found())) {
  const auto params = franka_params();
  const RobotModels models(params);
  CHECK((models.armature() == params.armature));
}

TEST_CASE("RobotModels (franka): collision pairs" *
          doctest::skip(!franka_description_found())) {
  const RobotModels models(franka_params());
  // Number of collision pairs in the fer model.
  CHECK(models.collision_model().collisionPairs.size() == 142);
}

TEST_CASE("RobotModels (franka): rnea" *
          doctest::skip(!franka_description_found())) {
  const RobotModels models(franka_params());
  const auto& model = models.robot_model();
  auto data = pinocchio::Data(model);
  const Eigen::VectorXd zeros = Eigen::VectorXd::Zero(model.nv);
  pinocchio::rnea(model, data, zeros, zeros, zeros);
  CHECK(data.tau.size() == model.nv);
}

TEST_CASE("RobotModels (franka): collision models" *
          doctest::skip(!franka_description_found())) {
  const RobotModels models(franka_params());
  const std::vector<std::string> expected_names = {
      "fer_leftfinger_0",       "fer_leftfinger_1",
      "fer_leftfinger_2",       "fer_leftfinger_3",
      "fer_rightfinger_0",      "fer_rightfinger_1",
      "fer_rightfinger_2",      "fer_rightfinger_3",
      "obstacle1_capsule_0",    "fer_link0_sc_capsule_0",
      "fer_link1_sc_capsule_0", "fer_link2_sc_capsule_0",
      "fer_link3_sc_capsule_0", "fer_link4_sc_capsule_0",
      "fer_link5_sc_capsule_0", "fer_link5_sc_capsule_1",
      "fer_link6_sc_capsule_0", "fer_hand_sc_capsule_0",
      "fer_hand_sc_capsule_1",  "fer_link7_sc_capsule_0",
      "fer_link7_sc_capsule_1"};
  CHECK(geometry_names(models.collision_model()) == expected_names);

  // The first 8 are boxes, the following ones capsules. The Python test
  // zips against a list one shorter than the names: same here.
  const auto& objects = models.collision_model().geometryObjects;
  for (std::size_t i = 0; i < std::min<std::size_t>(objects.size(), 20); ++i) {
    if (i < 8) {
      CHECK(std::dynamic_pointer_cast<const coal::Box>(objects[i].geometry) !=
            nullptr);
    } else {
      CHECK(std::dynamic_pointer_cast<const coal::Capsule>(
                objects[i].geometry) != nullptr);
    }
  }
}
