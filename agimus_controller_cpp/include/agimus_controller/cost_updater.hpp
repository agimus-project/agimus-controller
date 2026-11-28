#pragma once

#include <Eigen/Core>
#include <crocoddyl/core/fwd.hpp>
#include <cstddef>
#include <optional>
#include <pinocchio/multibody/fwd.hpp>
#include <string_view>
#include <vector>

#include "agimus_controller/status.hpp"
#include "agimus_controller/trajectory_point.hpp"

namespace agimus_controller {

/// A frame pair (parent, child) whose transform the OCP needs as input
/// (visual servoing). Setup only.
struct TransformInput {
  FrameName parent, child;
};

/// One entry per OcpModels::input_transforms, sized at setup. An empty
/// optional is a cleared transform (Python None). std::optional<SE3> never
/// allocates.
using InputTransforms = std::vector<std::optional<pinocchio::SE3>>;

/// How a cost follows the reference trajectory (Python: the `update` methods
/// of PY/ocp/ocp_croco_generic.py). See design.md, "Cost updaters".
enum class UpdateRule {
  kState,             // ResidualModelState: [q; qdot], weights [w_q; w_qdot]
  kControl,           // ResidualModelControl: robot_effort, w_robot_effort
  kControlGrav,       // ResidualModelControlGrav: weights only (B4)
  kFramePlacement,    // ResidualModelFramePlacement: ee.pose, ee.w_pose
  kFrameTranslation,  // ResidualModelFrameTranslation: translation, w_pose[:3]
  kFrameRotation,     // ResidualModelFrameRotation: rotation, w_pose[3:]
  kFrameVelocity,     // ResidualModelFrameVelocity: ee.twist, ee.w_twist
  kVisualServoing,    // ResidualModelFramePlacement: T * ee.pose, ee.w_pose
  kCollisionWeight,   // any residual: the item's weight w_collision_avoidance
  kSoftContactForce,  // P8, not implemented
};

/// Updates one cost of one node. Everything is resolved at setup by
/// make_cost_updater(); apply() only writes through the cached pointers.
struct CostUpdater {
  UpdateRule rule = UpdateRule::kState;
  crocoddyl::CostItem* item = nullptr;  // weight and active flag
  crocoddyl::ResidualModelAbstract* residual = nullptr;  // static_cast per rule
  /// nullptr if the activation is not weighted-quad: weights are then ignored.
  crocoddyl::ActivationModelWeightedQuad* activation = nullptr;
  /// Fixed frame: the key into ee_inputs (for kVisualServoing, the robot
  /// frame + "_vs"). Frame from the trajectory: the name of the residual's
  /// current frame, used as a cache key.
  FrameName frame_name;
  pinocchio::FrameIndex frame_id = 0;
  bool frame_from_trajectory = false;  // Python non-Static residuals
  std::size_t transform_index = 0;     // kVisualServoing
  /// Model whose frames a trajectory-driven frame name is looked up in. It
  /// must outlive the updater.
  const pinocchio::Model* model = nullptr;
  /// B10 may change `item->active`. False when the cost was inactive at
  /// setup (the OCP file's choice): the updater then never activates it.
  bool toggles_active = true;
  /// Storage sized at setup: crocoddyl's setters take `const VectorXd&`, so
  /// fixed-size or concatenated values are assembled here (no temporaries).
  Eigen::VectorXd reference_buffer;  // kState: [q; qdot]
  Eigen::VectorXd weights_buffer;    // the weights the rule writes
};

/// The updaters of one node (one per updated cost).
struct NodeUpdater {
  std::vector<CostUpdater> costs;
};

/// Setup: builds the updater of the cost `cost_name` of `costs`.
///
/// - `model` is the pinocchio model of the residual's state (pass
///   `*state->get_pinocchio()`); it must outlive the updater.
/// - Frame rules (kFramePlacement, kFrameTranslation, kFrameRotation,
///   kFrameVelocity): give either `frame` (Python `*Static`, the residual
///   must already point at that frame) or `frame_from_trajectory = true`
///   (Python non-Static: the point holds one end effector, whose frame the
///   residual follows) with an empty `frame`.
/// - kVisualServoing: `frame` is the robot frame of the residual; the input
///   read is `frame + "_vs"`, transformed by `transform_index`.
/// - Other rules take no frame.
///
/// B10 at setup: an active cost whose effective weight is zero (scalar weight
/// 0, or all its weighted-quad weights 0) is made inactive with
/// `changeCostStatus`. A cost already inactive is never activated.
///
/// Throws std::invalid_argument naming the cost if it does not exist, if its
/// residual does not match the rule, or if the frame arguments are wrong.
CostUpdater make_cost_updater(UpdateRule rule, crocoddyl::CostModelSum& costs,
                              std::string_view cost_name,
                              const pinocchio::Model& model,
                              std::string_view frame = {},
                              bool frame_from_trajectory = false,
                              std::size_t transform_index = 0);

/// Writes the references and weights of `point` into the costs of one node,
/// then applies B10: a cost whose effective weight is zero is made inactive
/// through `CostItem::active` (never `changeCostStatus`, which allocates),
/// and active again once a weight is non-zero.
///
/// Returns at the first failing cost (the costs before it are updated):
/// - kUnknownFrame: a fixed frame is not in `point.ee_inputs`, or a
///   trajectory frame is not in the model;
/// - kSizeMismatch: a frame from the trajectory with not exactly one
///   ee_input, or a point vector not sized like the residual;
/// - kMissingTransform: kVisualServoing with a cleared transform and
///   non-zero weights (Python's assert).
/// The failing cost itself is left unchanged.
/// @rt apply
Status apply(NodeUpdater& node, const WeightedTrajectoryPoint& point,
             const InputTransforms& transforms) noexcept;

}  // namespace agimus_controller
