// Port of agimus_controller/tests/test_robot_models.py, plus tests of the
// behaviour the Python tests do not cover (capsules, collision pairs, the
// xml/path pairs).

#include <coal/shape/geometric_shapes.h>
#include <doctest/doctest.h>

#include <Eigen/Core>
#include <cstdlib>
#include <filesystem>
#include <pinocchio/algorithm/rnea.hpp>
#include <stdexcept>
#include <string>
#include <vector>

#include "agimus_controller/robot_models.hpp"
#include "robot_models_fixtures.hpp"

using agimus_controller::RobotModelParameters;
using agimus_controller::RobotModels;

using namespace robot_models_test;

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
