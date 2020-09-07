/*
 * Copyright (c) 2019, The Regents of the University of California (Regents).
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *    1. Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *
 *    2. Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *
 *    3. Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Please contact the author(s) of this library if you have any questions.
 * Authors: David Fridovich-Keil   ( dfk@eecs.berkeley.edu )
 */

///////////////////////////////////////////////////////////////////////////////
//
// Base class specifying the problem interface for managing calls to the core
// ILQGame solver. Specific examples will be derived from this class.
//
///////////////////////////////////////////////////////////////////////////////

#include <ilqgames/solver/game_solver.h>
#include <ilqgames/solver/problem.h>
#include <ilqgames/utils/relative_time_tracker.h>
#include <ilqgames/utils/solver_log.h>
#include <ilqgames/utils/strategy.h>
#include <ilqgames/utils/types.h>

#include <glog/logging.h>
#include <algorithm>
#include <memory>
#include <vector>

// Time horizon and step.
DEFINE_double(time_horizon, 10.0, "Total time horizon (s).");
DEFINE_double(time_step, 0.1, "Length of discrete time step (s).");

namespace ilqgames {

Problem::Problem()
    : time_horizon_(FLAGS_time_horizon),
      time_step_(FLAGS_time_step),
      num_time_steps_(static_cast<size_t>(
          (constants::kSmallNumber + FLAGS_time_horizon) / FLAGS_time_step)),
      initialized_(false) {}

void Problem::SetUpNextRecedingHorizon(const VectorXf& x0, Time t0,
                                       Time planner_runtime) {
  CHECK(initialized_);
  CHECK_GE(planner_runtime, 0.0);
  CHECK_LE(planner_runtime + t0, InitialTime() + TimeHorizon());
  CHECK_GE(t0, operating_point_->t0);

  // Integrate x0 forward from t0 by approximately planner_runtime to get
  // actual initial state. Integrate up to the next discrete timestep, then
  // integrate for an integer number of discrete timesteps until by the *next*
  // timestep at least 'planner_runtime' has elapsed (done by rounding).
  constexpr float kRoundingError = 0.9;
  const Time relative_t0 = t0 - operating_point_->t0;
  size_t current_timestep = static_cast<size_t>(relative_t0 / time_step_);
  Time remaining_time_this_step =
      (current_timestep + 1) * time_step_ - relative_t0;
  if (remaining_time_this_step < kRoundingError * time_step_) {
    current_timestep += 1;
    remaining_time_this_step = time_step_ - remaining_time_this_step;
  }

  CHECK_LT(remaining_time_this_step, time_step_);

  // Initially, set x to the integrated version of x0 at the next timestep.
  VectorXf x = dynamics_->IntegrateToNextTimeStep(t0, x0, *operating_point_,
                                                  *strategies_);
  operating_point_->t0 = t0 + remaining_time_this_step;
  if (remaining_time_this_step <= planner_runtime) {
    const size_t num_steps_to_integrate = static_cast<size_t>(
        constants::kSmallNumber +  // Add to avoid truncation error.
        (planner_runtime - remaining_time_this_step) / time_step_);
    const size_t last_integration_timestep =
        current_timestep + num_steps_to_integrate;

    x = dynamics_->Integrate(current_timestep + 1, last_integration_timestep, x,
                             *operating_point_, *strategies_);
    operating_point_->t0 += time_step_ * num_steps_to_integrate;
  }

  // Find index of nearest state in the existing plan to this state.
  const auto nearest_iter =
      std::min_element(operating_point_->xs.begin(), operating_point_->xs.end(),
                       [this, &x](const VectorXf& x1, const VectorXf& x2) {
                         return dynamics_->DistanceBetween(x, x1) <
                                dynamics_->DistanceBetween(x, x2);
                       });

  // Set initial time to first timestamp in new problem.
  const size_t first_timestep_in_new_problem =
      std::distance(operating_point_->xs.begin(), nearest_iter);

  // Set initial state to this state.
  x0_ = dynamics_->Stitch(*nearest_iter, x);

  // Update all costs to have the correct initial time.
  RelativeTimeTracker::ResetInitialTime(operating_point_->t0);

  // Set final timestep to consider in current operating point.
  const size_t after_final_timestep =
      first_timestep_in_new_problem + NumTimeSteps();
  const size_t timestep_iterator_end =
      std::min(after_final_timestep, operating_point_->xs.size());

  // Populate strategies and opeating point for the remainder of the
  // existing plan, reusing the old operating point when possible.
  for (size_t kk = first_timestep_in_new_problem; kk < timestep_iterator_end;
       kk++) {
    const size_t kk_new_problem = kk - first_timestep_in_new_problem;

    // Set current state and controls in operating point.
    operating_point_->xs[kk_new_problem].swap(operating_point_->xs[kk]);
    operating_point_->us[kk_new_problem].swap(operating_point_->us[kk]);
    CHECK_EQ(operating_point_->us[kk_new_problem].size(),
             dynamics_->NumPlayers());

    // Set current stategy.
    for (auto& strategy : *strategies_) {
      strategy.Ps[kk_new_problem].swap(strategy.Ps[kk]);
      strategy.alphas[kk_new_problem].swap(strategy.alphas[kk]);
    }
  }

  // Make sure operating point is the right size.
  CHECK_GE(operating_point_->xs.size(), NumTimeSteps());
  if (operating_point_->xs.size() > NumTimeSteps()) {
    operating_point_->xs.resize(NumTimeSteps());
    operating_point_->us.resize(NumTimeSteps());
    for (PlayerIndex ii = 0; ii < dynamics_->NumPlayers(); ii++) {
      (*strategies_)[ii].Ps.resize(NumTimeSteps());
      (*strategies_)[ii].alphas.resize(NumTimeSteps());
    }
  }

  // Set new operating point controls and strategies to zero and propagate
  // state forward accordingly.
  for (size_t kk = timestep_iterator_end - first_timestep_in_new_problem;
       kk < NumTimeSteps(); kk++) {
    operating_point_->us[kk].resize(dynamics_->NumPlayers());
    for (size_t ii = 0; ii < dynamics_->NumPlayers(); ii++) {
      (*strategies_)[ii].Ps[kk].setZero(dynamics_->UDim(ii), dynamics_->XDim());
      (*strategies_)[ii].alphas[kk].setZero(dynamics_->UDim(ii));
      operating_point_->us[kk][ii].setZero(dynamics_->UDim(ii));
    }

    operating_point_->xs[kk] = dynamics_->Integrate(
        InitialTime() + ComputeRelativeTimeStamp(kk - 1), time_step_,
        operating_point_->xs[kk - 1], operating_point_->us[kk - 1]);
  }

  // Invariants.
  CHECK_EQ(operating_point_->xs.size(), NumTimeSteps());
  CHECK_LE(std::abs(t0 + planner_runtime - operating_point_->t0), time_step_);
}

size_t Problem::NumPrimals() const {
  CHECK(initialized_);

  const auto horizon = NumTimeSteps();
  const auto xdim = dynamics_->XDim();
  const auto udim = dynamics_->TotalUDim();

  // Start by computing the number of shared variables - states and controls.
  size_t total = xdim * (horizon + 1) + udim * horizon;

  // Accumulate players' individual strategy parameters.
  for (PlayerIndex ii = 0; ii < dynamics_->NumPlayers(); ii++)
    total += (*strategies_)[ii].NumParameters();

  return total;
}

size_t Problem::NumDuals() const {
  CHECK(initialized_);

  const auto horizon = NumTimeSteps();
  const auto xdim = dynamics_->XDim();

  // Start by computing the number of initial state multipliers and dynamics
  // multipliers and feedback multipliers (which should be the same).
  size_t total = dynamics_->NumPlayers() * xdim * (2 * horizon + 1);

  for (PlayerIndex ii = 0; ii < dynamics_->NumPlayers(); ii++) {
    const auto& player_cost = player_costs_[ii];

    // Accumulate the constraint multipliers for each state constraint.
    total += player_cost.NumStateConstraints() * xdim * horizon;

    // Accumulate the control constraint multipliers
    for (const auto& pair : player_cost.ControlConstraints())
      total += dynamics_->UDim(pair.first) * horizon;
  }

  return total;
}

void Problem::OverwriteSolution(const OperatingPoint& operating_point,
                                const std::vector<Strategy>& strategies) {
  CHECK(initialized_);

  *operating_point_ = operating_point;
  *strategies_ = strategies;
}

}  // namespace ilqgames
