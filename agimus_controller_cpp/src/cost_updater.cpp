#include "agimus_controller/cost_updater.hpp"

#include <crocoddyl/core/activations/weighted-quadratic.hpp>
#include <crocoddyl/core/costs/cost-sum.hpp>
#include <crocoddyl/core/residuals/control.hpp>
#include <crocoddyl/multibody/residuals/control-gravity.hpp>
#include <crocoddyl/multibody/residuals/frame-placement.hpp>
#include <crocoddyl/multibody/residuals/frame-rotation.hpp>
#include <crocoddyl/multibody/residuals/frame-translation.hpp>
#include <crocoddyl/multibody/residuals/frame-velocity.hpp>
#include <crocoddyl/multibody/residuals/state.hpp>
#include <crocoddyl/multibody/states/multibody.hpp>
#include <stdexcept>
#include <string>

namespace agimus_controller {

namespace {

const char* rule_name(UpdateRule rule) {
  switch (rule) {
    case UpdateRule::kState:
      return "kState";
    case UpdateRule::kControl:
      return "kControl";
    case UpdateRule::kControlGrav:
      return "kControlGrav";
    case UpdateRule::kFramePlacement:
      return "kFramePlacement";
    case UpdateRule::kFrameTranslation:
      return "kFrameTranslation";
    case UpdateRule::kFrameRotation:
      return "kFrameRotation";
    case UpdateRule::kFrameVelocity:
      return "kFrameVelocity";
    case UpdateRule::kVisualServoing:
      return "kVisualServoing";
    case UpdateRule::kCollisionWeight:
      return "kCollisionWeight";
    case UpdateRule::kSoftContactForce:
      return "kSoftContactForce";
  }
  return "unknown rule";
}

bool is_frame_rule(UpdateRule rule) {
  return rule == UpdateRule::kFramePlacement ||
         rule == UpdateRule::kFrameTranslation ||
         rule == UpdateRule::kFrameRotation ||
         rule == UpdateRule::kFrameVelocity;
}

// Index of the first frame named `name`, or model.nframes. Allocation-free:
// used in the tick when a trajectory-driven frame changes.
pinocchio::FrameIndex find_frame(const pinocchio::Model& model,
                                 std::string_view name) noexcept {
  for (std::size_t i = 0; i < model.frames.size(); ++i) {
    if (std::string_view(model.frames[i].name) == name) {
      return i;
    }
  }
  return model.frames.size();
}

// The residual's current frame id. Setup only (dynamic_cast).
pinocchio::FrameIndex residual_frame_id(UpdateRule rule,
                                        crocoddyl::ResidualModelAbstract* r) {
  switch (rule) {
    case UpdateRule::kFramePlacement:
    case UpdateRule::kVisualServoing:
      return static_cast<crocoddyl::ResidualModelFramePlacement*>(r)->get_id();
    case UpdateRule::kFrameTranslation:
      return static_cast<crocoddyl::ResidualModelFrameTranslation*>(r)
          ->get_id();
    case UpdateRule::kFrameRotation:
      return static_cast<crocoddyl::ResidualModelFrameRotation*>(r)->get_id();
    case UpdateRule::kFrameVelocity:
      return static_cast<crocoddyl::ResidualModelFrameVelocity*>(r)->get_id();
    default:
      return 0;
  }
}

// True if the residual has the type the rule writes to.
bool residual_matches(UpdateRule rule, crocoddyl::ResidualModelAbstract* r) {
  switch (rule) {
    case UpdateRule::kState:
      return dynamic_cast<crocoddyl::ResidualModelState*>(r) != nullptr;
    case UpdateRule::kControl:
      return dynamic_cast<crocoddyl::ResidualModelControl*>(r) != nullptr;
    case UpdateRule::kControlGrav:
      return dynamic_cast<crocoddyl::ResidualModelControlGrav*>(r) != nullptr;
    case UpdateRule::kFramePlacement:
    case UpdateRule::kVisualServoing:
      return dynamic_cast<crocoddyl::ResidualModelFramePlacement*>(r) !=
             nullptr;
    case UpdateRule::kFrameTranslation:
      return dynamic_cast<crocoddyl::ResidualModelFrameTranslation*>(r) !=
             nullptr;
    case UpdateRule::kFrameRotation:
      return dynamic_cast<crocoddyl::ResidualModelFrameRotation*>(r) != nullptr;
    case UpdateRule::kFrameVelocity:
      return dynamic_cast<crocoddyl::ResidualModelFrameVelocity*>(r) != nullptr;
    case UpdateRule::kCollisionWeight:
      return true;  // colmpc's distance residuals; only the weight is written
    case UpdateRule::kSoftContactForce:
      return false;
  }
  return false;
}

// Size of the weights the rule writes into a weighted-quad activation.
Eigen::Index rule_weights_size(UpdateRule rule,
                               const crocoddyl::ResidualModelAbstract& r) {
  switch (rule) {
    case UpdateRule::kState:
      return static_cast<Eigen::Index>(r.get_state()->get_ndx());
    case UpdateRule::kControl:
    case UpdateRule::kControlGrav:
      return static_cast<Eigen::Index>(r.get_nu());
    case UpdateRule::kFramePlacement:
    case UpdateRule::kFrameVelocity:
    case UpdateRule::kVisualServoing:
      return 6;
    case UpdateRule::kFrameTranslation:
    case UpdateRule::kFrameRotation:
      return 3;
    default:
      return 0;
  }
}

bool all_zero(const Eigen::Ref<const Eigen::VectorXd>& w) noexcept {
  return (w.array() == 0.0).all();
}

// B10: an item is active iff its scalar weight is non-zero and, when the
// rule wrote activation weights, one of them is non-zero.
void set_active(CostUpdater& u, bool weights_non_zero) noexcept {
  if (u.toggles_active) {
    u.item->active = u.item->weight != 0.0 && weights_non_zero;
  }
}

// Writes `w` (already in u.weights_buffer or a point vector) as activation
// weights and applies B10. Without a weighted-quad activation, Python drops
// the weights, so only the scalar weight decides.
void write_weights(CostUpdater& u, const Eigen::VectorXd& w) noexcept {
  if (u.activation == nullptr) {
    set_active(u, true);
    return;
  }
  u.activation->set_weights(w);
  set_active(u, !all_zero(w));
}

// The end effector a frame rule reads, or a non-kOk status. For a frame from
// the trajectory it also re-targets the residual when the frame changed.
template <typename Residual>
Status frame_input(CostUpdater& u, const WeightedTrajectoryPoint& p,
                   const EndEffectorInput*& ee) noexcept {
  if (!u.frame_from_trajectory) {
    ee = find_ee_input(p, u.frame_name.view());
    return ee == nullptr ? Status::kUnknownFrame : Status::kOk;
  }
  if (p.ee_inputs.size() != 1) {
    return Status::kSizeMismatch;
  }
  ee = &p.ee_inputs[0];
  if (ee->frame_name != u.frame_name) {
    const pinocchio::FrameIndex id =
        find_frame(*u.model, ee->frame_name.view());
    if (id >= u.model->frames.size()) {
      return Status::kUnknownFrame;
    }
    u.frame_name = ee->frame_name;
    u.frame_id = id;
    static_cast<Residual*>(u.residual)->set_id(id);
  }
  return Status::kOk;
}

Status apply_state(CostUpdater& u, const WeightedTrajectoryPoint& p) noexcept {
  const Eigen::Index nx = u.reference_buffer.size();
  const Eigen::Index nv = u.weights_buffer.size() / 2;
  const Eigen::Index nq = nx - nv;
  if (p.q.size() != nq || p.qdot.size() != nv || p.w_q.size() != nv ||
      p.w_qdot.size() != nv) {
    return Status::kSizeMismatch;
  }
  u.reference_buffer.head(nq) = p.q;
  u.reference_buffer.tail(nv) = p.qdot;
  static_cast<crocoddyl::ResidualModelState*>(u.residual)
      ->set_reference(u.reference_buffer);
  u.weights_buffer.head(nv) = p.w_q;
  u.weights_buffer.tail(nv) = p.w_qdot;
  write_weights(u, u.weights_buffer);
  return Status::kOk;
}

Status apply_control(CostUpdater& u, const WeightedTrajectoryPoint& p,
                     bool with_reference) noexcept {
  const Eigen::Index nu = static_cast<Eigen::Index>(u.residual->get_nu());
  if (p.robot_effort.size() != nu || p.w_robot_effort.size() != nu) {
    return Status::kSizeMismatch;
  }
  if (with_reference) {
    static_cast<crocoddyl::ResidualModelControl*>(u.residual)
        ->set_reference(p.robot_effort);
  }
  write_weights(u, p.w_robot_effort);
  return Status::kOk;
}

Status apply_one(CostUpdater& u, const WeightedTrajectoryPoint& p,
                 const InputTransforms& transforms) noexcept {
  const EndEffectorInput* ee = nullptr;
  Status status = Status::kOk;
  switch (u.rule) {
    case UpdateRule::kState:
      return apply_state(u, p);
    case UpdateRule::kControl:
      return apply_control(u, p, true);
    case UpdateRule::kControlGrav:
      // B4: crocoddyl's ControlGrav residual has no reference.
      return apply_control(u, p, false);
    case UpdateRule::kFramePlacement:
      status = frame_input<crocoddyl::ResidualModelFramePlacement>(u, p, ee);
      if (status != Status::kOk) return status;
      static_cast<crocoddyl::ResidualModelFramePlacement*>(u.residual)
          ->set_reference(ee->pose);
      u.weights_buffer = ee->w_pose;
      write_weights(u, u.weights_buffer);
      return Status::kOk;
    case UpdateRule::kFrameTranslation:
      status = frame_input<crocoddyl::ResidualModelFrameTranslation>(u, p, ee);
      if (status != Status::kOk) return status;
      static_cast<crocoddyl::ResidualModelFrameTranslation*>(u.residual)
          ->set_reference(ee->pose.translation());
      u.weights_buffer = ee->w_pose.head<3>();
      write_weights(u, u.weights_buffer);
      return Status::kOk;
    case UpdateRule::kFrameRotation:
      status = frame_input<crocoddyl::ResidualModelFrameRotation>(u, p, ee);
      if (status != Status::kOk) return status;
      static_cast<crocoddyl::ResidualModelFrameRotation*>(u.residual)
          ->set_reference(ee->pose.rotation());
      u.weights_buffer = ee->w_pose.tail<3>();
      write_weights(u, u.weights_buffer);
      return Status::kOk;
    case UpdateRule::kFrameVelocity:
      status = frame_input<crocoddyl::ResidualModelFrameVelocity>(u, p, ee);
      if (status != Status::kOk) return status;
      static_cast<crocoddyl::ResidualModelFrameVelocity*>(u.residual)
          ->set_reference(ee->twist);
      u.weights_buffer = ee->w_twist;
      write_weights(u, u.weights_buffer);
      return Status::kOk;
    case UpdateRule::kVisualServoing: {
      ee = find_ee_input(p, u.frame_name.view());
      if (ee == nullptr) return Status::kUnknownFrame;
      if (u.transform_index >= transforms.size()) {
        return Status::kMissingTransform;
      }
      const std::optional<pinocchio::SE3>& transform =
          transforms[u.transform_index];
      if (!transform.has_value() && !all_zero(ee->w_pose)) {
        return Status::kMissingTransform;
      }
      auto* residual =
          static_cast<crocoddyl::ResidualModelFramePlacement*>(u.residual);
      if (transform.has_value()) {
        residual->set_reference(*transform * ee->pose);
      } else {
        residual->set_reference(ee->pose);
      }
      u.weights_buffer = ee->w_pose;
      write_weights(u, u.weights_buffer);
      return Status::kOk;
    }
    case UpdateRule::kCollisionWeight:
      u.item->weight = p.w_collision_avoidance;
      set_active(u, true);
      return Status::kOk;
    case UpdateRule::kSoftContactForce:
      break;  // Rejected by make_cost_updater.
  }
  return Status::kOk;
}

}  // namespace

CostUpdater make_cost_updater(UpdateRule rule, crocoddyl::CostModelSum& costs,
                              std::string_view cost_name,
                              const pinocchio::Model& model,
                              std::string_view frame,
                              bool frame_from_trajectory,
                              std::size_t transform_index) {
  const std::string name(cost_name);
  const std::string where =
      "make_cost_updater: cost '" + name + "' (" + rule_name(rule) + "): ";
  if (rule == UpdateRule::kSoftContactForce) {
    throw std::invalid_argument(where + "not implemented until P8.");
  }
  const auto it = costs.get_costs().find(name);
  if (it == costs.get_costs().end()) {
    throw std::invalid_argument(where + "no such cost.");
  }
  CostUpdater u;
  u.rule = rule;
  u.item = it->second.get();
  u.residual = u.item->cost->get_residual().get();
  if (u.residual == nullptr || !residual_matches(rule, u.residual)) {
    throw std::invalid_argument(where +
                                "the residual does not match the rule.");
  }
  u.activation = dynamic_cast<crocoddyl::ActivationModelWeightedQuad*>(
      u.item->cost->get_activation().get());
  u.model = &model;
  u.transform_index = transform_index;

  const bool frame_rule = is_frame_rule(rule);
  if (frame_from_trajectory && (!frame_rule || !frame.empty())) {
    throw std::invalid_argument(
        where + "frame_from_trajectory needs a frame rule and no frame.");
  }
  if (frame_rule || rule == UpdateRule::kVisualServoing) {
    const pinocchio::FrameIndex id = residual_frame_id(rule, u.residual);
    if (id >= model.frames.size()) {
      throw std::invalid_argument(where +
                                  "the residual's frame is not in the "
                                  "model.");
    }
    const std::string& residual_frame = model.frames[id].name;
    if (!frame_from_trajectory && residual_frame != frame) {
      throw std::invalid_argument(where + "frame '" + std::string(frame) +
                                  "' differs from the residual's frame '" +
                                  residual_frame + "'.");
    }
    u.frame_id = id;
    u.frame_from_trajectory = frame_from_trajectory;
    u.frame_name =
        FrameName(rule == UpdateRule::kVisualServoing ? residual_frame + "_vs"
                                                      : residual_frame);
  } else if (!frame.empty()) {
    throw std::invalid_argument(where + "this rule takes no frame.");
  }

  if (rule == UpdateRule::kState) {
    const auto& state = *u.residual->get_state();
    u.reference_buffer = Eigen::VectorXd::Zero(state.get_nx());
  }
  const Eigen::Index nw = rule_weights_size(rule, *u.residual);
  if (u.activation != nullptr && u.activation->get_weights().size() != nw) {
    throw std::invalid_argument(where +
                                "the activation does not have the rule's "
                                "number of weights.");
  }
  u.weights_buffer = Eigen::VectorXd::Zero(nw);

  // B10 at setup.
  u.toggles_active = u.item->active;
  if (u.item->active) {
    const bool weights_zero = u.activation != nullptr &&
                              rule != UpdateRule::kCollisionWeight &&
                              all_zero(u.activation->get_weights());
    if (u.item->weight == 0.0 || weights_zero) {
      costs.changeCostStatus(name, false);
    }
  }
  return u;
}

Status apply(NodeUpdater& node, const WeightedTrajectoryPoint& point,
             const InputTransforms& transforms) noexcept {
  for (CostUpdater& u : node.costs) {
    const Status status = apply_one(u, point, transforms);
    if (status != Status::kOk) {
      return status;
    }
  }
  return Status::kOk;
}

}  // namespace agimus_controller
