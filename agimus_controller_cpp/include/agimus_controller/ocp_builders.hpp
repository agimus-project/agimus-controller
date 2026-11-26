#pragma once

#include "agimus_controller/ocp.hpp"
#include "agimus_controller/ocp_params.hpp"
#include "agimus_controller/robot_models.hpp"

namespace agimus_controller {

/// Setup: the models of Python OCPCrocoGoalReaching
/// (PY/ocp/ocp_croco_goal_reaching.py).
///
/// Running nodes (one object each, step `params.timesteps[i]`) hold the costs
/// `stateReg` (state, weights of size ndx), `ctrlReg` and `goalTracking`
/// (frame placement); the terminal node (step 0) holds `stateReg` and
/// `goalTracking`. Every cost has weight 1 and a weighted-quad activation.
/// Every differential model gets `robot_models.armature()`.
///
/// The updaters follow set_reference_weighted_trajectory (lines 125-212):
/// `stateReg` takes [q; qdot] and [w_q; w_qdot], `ctrlReg` takes
/// robot_effort and w_robot_effort, and `goalTracking` follows the only end
/// effector of each point (pose, w_pose), at every node including the
/// terminal one (behaviour changes B12 and B14: Python reuses the last
/// running node's frame at the terminal node and sizes the state weights
/// nq + nv).
///
/// `params` must be finalized. Throws std::invalid_argument otherwise, or if
/// the armature does not have the size of the model's nv.
OcpModels build_goal_reaching_models(const RobotModels& robot_models,
                                     const OcpParams& params);

}  // namespace agimus_controller
