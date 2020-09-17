/*
 * Copyright (c) 2020, The Regents of the University of California (Regents).
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
// (Time-varying) dynamic constraint,
// i.e., 0.5*||x_{t+1} - f(t, x_t, us_t)||^2 = 0.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef ILQGAMES_CONSTRAINT_DYNAMIC_CONSTRAINT_H
#define ILQGAMES_CONSTRAINT_DYNAMIC_CONSTRAINT_H

#include <ilqgames/dynamics/multi_player_dynamical_system.h>
#include <ilqgames/utils/linear_dynamics_approximation.h>
#include <ilqgames/utils/quadratic_constraint_approximation.h>
#include <ilqgames/utils/relative_time_tracker.h>
#include <ilqgames/utils/types.h>

#include <glog/logging.h>
#include <memory>
#include <string>

namespace ilqgames {

class DynamicConstraint : public RelativeTimeTracker {
 public:
  ~DynamicConstraint() {}
  DynamicConstraint(const MultiPlayerDynamicalSystem* dynamics,
                    const std::string& name = "")
      : RelativeTimeTracker(name), dynamics_(dynamics) {
    CHECK_NOTNULL(dynamics_);
  }

  // Check if this constraint is satisfied, and optionally return the constraint
  // value, which equals zero if the constraint is satisfied.
  bool IsSatisfied(Time t, const VectorXf& x, const std::vector<VectorXf>& us,
                   const VectorXf& next_x, float* level) const {
    const float value =
        0.5 * (next_x - dynamics_->Evaluate(t, x, us)).squaredNorm();
    if (*level) *level = value;

    return std::abs(value) < constants::kSmallNumber;
  }

  // Quadraticize the constraint value. Do *not* keep a running sum since we
  // keep separate multipliers for each constraint.
  // NOTE: this truncates the dynamics derivatives at first order, i.e., it
  // uses only a linearization of the dynamics, which it assumes to be correct
  // for the given arguments.
  // NOTE: Hessian cross terms are all duplicated for the transposes, i.e., this
  // function assumes that the underlying references are for symmetric parts of
  // the big matrix this is populating.
  void Quadraticize(Time t, const VectorXf& x, const std::vector<VectorXf>& us,
                    const VectorXf& next_x,
                    const LinearDynamicsApproximation& lin,
                    Eigen::Ref<MatrixXf> hess_x,
                    std::vector<std::vector<Eigen::Ref<MatrixXf>>>& hess_us,
                    Eigen::Ref<MatrixXf> hess_nextx,
                    std::vector<Eigen::Ref<MatrixXf>>& hess_xus,
                    std::vector<Eigen::Ref<MatrixXf>>& hess_usx,
                    Eigen::Ref<MatrixXf> hess_xnextx,
                    Eigen::Ref<MatrixXf> hess_nextxx,
                    std::vector<Eigen::Ref<MatrixXf>>& hess_usnextx,
                    std::vector<Eigen::Ref<MatrixXf>>& hess_nextxus,
                    Eigen::Ref<VectorXf> grad_x,
                    std::vector<Eigen::Ref<VectorXf>>& grad_us,
                    Eigen::Ref<VectorXf> grad_nextx) const {
    // NOTE: assuming that all the dimensions are correct, just because checking
    // would be a lot of unnecessary operations, but eventually these should be
    // a factored into DCHECKs.

    // Compute mismatch vector.
    const VectorXf error = next_x - dynamics_->Evaluate(t, x, us);

    // Handle x terms.
    hess_x = lin.A.transpose() * lin.A;
    grad_x = -lin.A.transpose() * error;

    // Handle us terms.
    for (PlayerIndex ii = 0; ii < dynamics_->NumPlayers(); ii++) {
      for (PlayerIndex jj = ii; jj < dynamics_->NumPlayers(); jj++) {
        hess_us[ii][jj] = lin.Bs[ii].transpose() * lin.Bs[jj];
        if (ii != jj) hess_us[jj][ii] = hess_us[ii][jj].transpose();
      }

      grad_us[ii] = -lin.Bs[ii].transpose() * error;
    }

    // Handle nextx terms.
    hess_nextx.setIdentity();
    grad_nextx = error;

    // Handle x cross terms.

  }

 private:
  // Dynamics of the underlying game.
  const MultiPlayerDynamicalSystem* dynamics_;
};  //\class DynamicConstraint

}  // namespace ilqgames

#endif
