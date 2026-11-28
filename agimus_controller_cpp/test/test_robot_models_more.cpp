// RobotModels tests on a tiny URDF and on agimus_franka_description (split
// from test_robot_models.cpp to keep each file under the compile-RAM limit).

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
