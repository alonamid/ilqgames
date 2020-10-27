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
// Three player intersection example. Ordering is given by the following:
// (P1, P2, P3) = (Car 1, Car 2, Pedestrian).
//
///////////////////////////////////////////////////////////////////////////////

#include <ilqgames/constraint/polyline2_signed_distance_constraint.h>
#include <ilqgames/constraint/proximity_constraint.h>
#include <ilqgames/constraint/single_dimension_constraint.h>
#include <ilqgames/cost/curvature_cost.h>
#include <ilqgames/cost/final_time_cost.h>
#include <ilqgames/cost/locally_convex_proximity_cost.h>
#include <ilqgames/cost/nominal_path_length_cost.h>
#include <ilqgames/cost/proximity_cost.h>
#include <ilqgames/cost/quadratic_cost.h>
#include <ilqgames/cost/quadratic_polyline2_cost.h>
#include <ilqgames/cost/semiquadratic_cost.h>
#include <ilqgames/cost/semiquadratic_polyline2_cost.h>
#include <ilqgames/cost/weighted_convex_proximity_cost.h>
#include <ilqgames/dynamics/concatenated_dynamical_system.h>
#include <ilqgames/dynamics/single_player_car_5d.h>
#include <ilqgames/dynamics/single_player_car_6d.h>
#include <ilqgames/dynamics/single_player_unicycle_4d.h>
#include <ilqgames/examples/n_player_intersection_example.h>
#include <ilqgames/geometry/polyline2.h>
#include <ilqgames/solver/ilq_solver.h>
#include <ilqgames/solver/lq_feedback_solver.h>
#include <ilqgames/solver/problem.h>
#include <ilqgames/solver/solver_params.h>
#include <ilqgames/utils/solver_log.h>
#include <ilqgames/utils/strategy.h>
#include <ilqgames/utils/types.h>

#include <math.h>
#include <memory>
#include <vector>

namespace ilqgames {

  // Time.
  static constexpr Time kTimeStep = 0.1;      // s
  static constexpr Time kTimeHorizon = 10.0;  // s
  static constexpr size_t kNumTimeSteps =
      static_cast<size_t>(kTimeHorizon / kTimeStep);
  
  // Car inter-axle distance.
  static constexpr float kInterAxleLength = 4.0;  // m
  
  // Cost weights.
  static constexpr float kStateRegularization = 1.0;
  static constexpr float kControlRegularization = 5.0;
  
  static constexpr float kOmegaCostWeight = 0.1;
  static constexpr float kJerkCostWeight = 0.1;
  static constexpr float kMaxOmega = 1.0;
  
  static constexpr float kACostWeight = 0.1;
  static constexpr float kCurvatureCostWeight = 1.0;
  static constexpr float kMaxVCostWeight = 10.0;
  static constexpr float kNominalVCostWeight = 100.0;
  
  static constexpr float kGoalCostWeight = 0.1;
  static constexpr float kLaneCostWeight = 25.0;
  static constexpr float kLaneBoundaryCostWeight = 100.0;
  
  static constexpr float kMinProximity = 6.0;
  
  static constexpr float kMinV = 1.0;     // m/s

  using P1 = SinglePlayerCar6D;
  using ProxCost = ProximityCost;

  static constexpr bool kOrientedRight = true;
  static constexpr bool kConstraintOrientedInside = false;
  
  // Lane width.
  static constexpr float kLaneHalfWidth = 2.5;  // m

  // Player consts
  static float kPProximityCostWeight [N];
  static float kPGoalX[N];  // m
  static float kPGoalY[N];   // m
  static float kPMaxV[N];   // m/s
  static float kPNominalV[N];  // m/s
  static float kPInitialX[N];  // m
  static float kPInitialY[N];   // m
  static float kPInitialHeading[N];      // rad
  static float kPInitialSpeed[N];  // m/s
  static Dimension kPXIdx[N];
  static Dimension kPYIdx[N];
  static Dimension kPHeadingIdx[N];
  static Dimension kPPhiIdx[N];
  static Dimension kPVIdx[N];
  static Dimension kPAIdx[N];
  static Dimension kPOmegaIdx[N];
  static Dimension kPJerkIdx[N];


NPlayerIntersectionExample::NPlayerIntersectionExample() {

  //static float kPProximityCostWeight [N];
  for (auto i = 0; i < N; i++)
  {
    kPProximityCostWeight[i] = 50.0;
  }
  
  
  // Goal points.
  
  //static float kPGoalX[N];  // m
  //static float kPGoalY[N];   // m
  for (auto i = 0; i < N; i++)
  {
    kPGoalX[i] = 50.0*i;
    kPGoalY[i] = 50.0*i;
  }
  
  // Nominal and max speed.
  //static float kPMaxV[N];   // m/s
  //static float kPNominalV[N];  // m/s
  
  for (auto i = 0; i < N; i++)
  {
    kPMaxV[i] = 12.0;
    kPNominalV[i] = 5.0;
  }
  
  // Initial state.
  //static float kPInitialX[N];  // m
  //static float kPInitialY[N];   // m
  //static float kPInitialHeading[N];      // rad
  //static float kPInitialSpeed[N];  // m/s
  
  for (auto i = 0; i < N; i++)
  {
    kPInitialX[i] = 12.0*i;
    kPInitialY[i] = 5.0*i;
    kPInitialHeading[i] = 5.0*i;
    kPInitialSpeed[i] = 5.0;
  }
  
  // State dimensions.
  //using P1 = SinglePlayerCar6D;
  //using P2 = SinglePlayerCar6D;
  //using P3 = SinglePlayerUnicycle4D;
  
  //static Dimension kPXIdx[N];
  //static Dimension kPYIdx[N];
  //static Dimension kPHeadingIdx[N];
  //static Dimension kPPhiIdx[N];
  //static Dimension kPVIdx[N];
  //static Dimension kPAIdx[N];
  
  for (auto i = 0; i < N; i++)
  {
    kPXIdx[i] = P1::kNumXDims;
    kPYIdx[i] = P1::kNumXDims;
    kPHeadingIdx[i] = P1::kNumXDims;
    kPPhiIdx[i] = P1::kNumXDims;
    kPVIdx[i] = P1::kNumXDims;
    kPAIdx[i] = P1::kNumXDims;
  }
  
  // Control dimensions.
  //static Dimension kPOmegaIdx[N];
  //static Dimension kPJerkIdx[N];
  for (auto i = 0; i < N; i++)
  {
    kPOmegaIdx[i] = 0;
    kPJerkIdx[i] = 1;
  }
  
}


void NPlayerIntersectionExample::ConstructDynamics() {
  SubsystemList ssl;
  for (auto i = 0; i < N; i++)
  {
    ssl.push_back(std::make_shared<P1>(kInterAxleLength));
  }
  dynamics_.reset(new ConcatenatedDynamicalSystem(ssl));
}

void NPlayerIntersectionExample::ConstructInitialState() {
  // Set up initial state.
  x0_ = VectorXf::Zero(dynamics_->XDim());

  for (auto i = 0; i < N; i++)
  {
    x0_(kPXIdx[i]) = kPInitialX[i];
    x0_(kPYIdx[i]) = kPInitialY[i];
    x0_(kPHeadingIdx[i]) = kPInitialHeading[i];
    x0_(kPVIdx[i]) = kPInitialSpeed[i];
  }
}


void NPlayerIntersectionExample::ConstructPlayerCosts() {
  // Set up costs for all players.
  for (auto i = 0; i < N; i++)
  {
    player_costs_.emplace_back("P" + std::to_string(i), kStateRegularization, kControlRegularization);
  }

  // Stay in lanes.
  const Polyline2 lane1(
      {Point2(kPInitialX[0], -1000.0), Point2(kPInitialX[0], 1000.0)});
  const Polyline2 lane2(
      {Point2(kPInitialX[1], 1000.0), Point2(kPInitialX[1], 18.0),
       Point2(kPInitialX[1] + 0.5, 15.0), Point2(kPInitialX[1] + 1.0, 14.0),
       Point2(kPInitialX[1] + 3.0, 12.5), Point2(kPInitialX[1] + 6.0, 12.0),
       Point2(1000.0, 12.0)});
  const Polyline2 lane3(
      {Point2(-1000.0, kPInitialY[2]), Point2(1000.0, kPInitialY[2])});

  for (auto i = 0; i < N; i++)
  {
    //p_lane_cost[i]
    std::pair<Dimension, Dimension> position_idxs(kPXIdx[i], kPYIdx[i]);
    player_costs_[i].AddStateCost(std::make_shared<QuadraticPolyline2Cost>(kLaneCostWeight, lane1, position_idxs, "LaneCenter"));

    //p_lane_r_constraint[i]
    //player_costs_[i].AddStateConstraint(std::make_shared<Polyline2SignedDistanceConstraint>(lane3, position_idxs,
    //                                                                                kLaneHalfWidth, !kOrientedRight,
    //                                                                                "LaneRightBoundary"));

    //p_lane_l_constraint[i]
    //player_costs_[i].AddStateConstraint(std::shared_ptr<Polyline2SignedDistanceConstraint>(lane3, position_idxs,
    //                                                                                -kLaneHalfWidth, kOrientedRight,
    //                                                                                "LaneLeftBoundary"));

    //p_min_v_constraint[i]
    player_costs_[i].AddStateConstraint(std::make_shared<SingleDimensionConstraint>(
                                 kPVIdx[i], kMinV, kOrientedRight, "MinV"));
    //p_max_v_constraint[i]
    player_costs_[i].AddStateConstraint(std::make_shared<SingleDimensionConstraint>(
                                 kPVIdx[i], kPMaxV[i], !kOrientedRight, "MaxV"));
    //p_nominal_v_cost[i]
    player_costs_[i].AddStateCost(std::make_shared<QuadraticCost>(
                           kNominalVCostWeight, kPVIdx[i], kPNominalV[i], "NominalV"));

    //p_omega_max_constraint[i]
    player_costs_[i].AddControlConstraint(i, std::make_shared<SingleDimensionConstraint>(
                                           kPOmegaIdx[i], kMaxOmega, !kOrientedRight, "SteeringMax"));
    //p_omega_min_constraint[i]
    player_costs_[i].AddControlConstraint(i, std::make_shared<SingleDimensionConstraint>(
                                           kPOmegaIdx[i], -kMaxOmega, kOrientedRight, "SteeringMin"));
    //p_omega_cost[i] 
    player_costs_[i].AddControlCost(i, std::make_shared<QuadraticCost>(
                                kOmegaCostWeight, kPOmegaIdx[i], 0.0, "Steering"));
    //p_jerk_cost[i]
    player_costs_[i].AddControlCost(i, std::make_shared<QuadraticCost>(kJerkCostWeight, kPJerkIdx[i], 0.0, "Jerk"));

  }


  for (auto i = 0; i < N; i++)
  {
    for (auto j = 0; j < N; j++)
    {
      std::pair<Dimension, Dimension> position_idxs_i(kPXIdx[i], kPYIdx[i]);
      std::pair<Dimension, Dimension> position_idxs_j(kPXIdx[j], kPYIdx[j]);
      player_costs_[i].AddStateConstraint(std::make_shared<ProximityConstraint>(position_idxs_i, position_idxs_j,
                                                                       kMinProximity, kConstraintOrientedInside,
                                                                       "ProximityConstraint" + std::to_string(i) + "x" + std::to_string(j)));
    }
  }

}

inline std::vector<float> NPlayerIntersectionExample::Xs(
    const VectorXf& x) const {
      std::vector<float> ret_val; 
      for (auto i = 0; i < N; i++)
      {
        ret_val.push_back(x(kPXIdx[i]));
      }
      return ret_val;
}

inline std::vector<float> NPlayerIntersectionExample::Ys(
    const VectorXf& x) const {
      std::vector<float> ret_val; 
      for (auto i = 0; i < N; i++)
      {
        ret_val.push_back(x(kPYIdx[i]));
      }
      return ret_val;
}

inline std::vector<float> NPlayerIntersectionExample::Thetas(
    const VectorXf& x) const {
      std::vector<float> ret_val; 
      for (auto i = 0; i < N; i++)
      {
        ret_val.push_back(x(kPHeadingIdx[i]));
      }
      return ret_val;
}


//===================================================================================================
/*
NPlayerIntersectionExample::NPlayerIntersectionExample(
    const SolverParams& params) {


// Time.
static constexpr Time kTimeStep = 0.1;      // s
static constexpr Time kTimeHorizon = 10.0;  // s
static constexpr size_t kNumTimeSteps =
    static_cast<size_t>(kTimeHorizon / kTimeStep);

// Car inter-axle distance.
static constexpr float kInterAxleLength = 4.0;  // m

// Cost weights.
static constexpr float kStateRegularization = 1.0;
static constexpr float kControlRegularization = 5.0;

static constexpr float kOmegaCostWeight = 0.1;
static constexpr float kJerkCostWeight = 0.1;
static constexpr float kMaxOmega = 1.0;

static constexpr float kACostWeight = 0.1;
static constexpr float kCurvatureCostWeight = 1.0;
static constexpr float kMaxVCostWeight = 10.0;
static constexpr float kNominalVCostWeight = 100.0;

static constexpr float kGoalCostWeight = 0.1;
static constexpr float kLaneCostWeight = 25.0;
static constexpr float kLaneBoundaryCostWeight = 100.0;

static constexpr float kMinProximity = 6.0;
static float kPProximityCostWeight [N];
for (auto i = 0; i < N; i++)
{
  kPProximityCostWeight[i] = 50.0;
}
using ProxCost = ProximityCost;

static constexpr bool kOrientedRight = true;
static constexpr bool kConstraintOrientedInside = false;

// Lane width.
static constexpr float kLaneHalfWidth = 2.5;  // m

// Goal points.

static float kPGoalX[N];  // m
static float kPGoalY[N];   // m
for (auto i = 0; i < N; i++)
{
  kPGoalX[i] = 50.0*i;
  kPGoalY[i] = 50.0*i;
}

// Nominal and max speed.
static float kPMaxV[N];   // m/s
static constexpr float kMinV = 1.0;     // m/s

static float kPNominalV[N];  // m/s

for (auto i = 0; i < N; i++)
{
  kPMaxV[i] = 12.0;
  kPNominalV[i] = 5.0;
}

// Initial state.
static float kPInitialX[N];  // m
static float kPInitialY[N];   // m
static float kPInitialHeading[N];      // rad
static float kPInitialSpeed[N];  // m/s

for (auto i = 0; i < N; i++)
{
  kPInitialX[i] = 12.0*i;
  kPInitialY[i] = 5.0*i;
  kPInitialHeading[i] = 5.0*i;
  kPInitialSpeed[i] = 5.0;
}

// State dimensions.
using P1 = SinglePlayerCar6D;
//using P2 = SinglePlayerCar6D;
//using P3 = SinglePlayerUnicycle4D;

static Dimension kPXIdx[N];
static Dimension kPYIdx[N];
static Dimension kPHeadingIdx[N];
static Dimension kPPhiIdx[N];
static Dimension kPVIdx[N];
static Dimension kPAIdx[N];

for (auto i = 0; i < N; i++)
{
  kPXIdx[i] = P1::kNumXDims;
  kPYIdx[i] = P1::kNumXDims;
  kPHeadingIdx[i] = P1::kNumXDims;
  kPPhiIdx[i] = P1::kNumXDims;
  kPVIdx[i] = P1::kNumXDims;
  kPAIdx[i] = P1::kNumXDims;
}

// Control dimensions.
static Dimension kPOmegaIdx[N];
static Dimension kPJerkIdx[N];
for (auto i = 0; i < N; i++)
{
  kPOmegaIdx[i] = 0;
  kPJerkIdx[i] = 1;
}



  // Create dynamics.
  //const std::shared_ptr<const ConcatenatedDynamicalSystem> dynamics(
  //    new ConcatenatedDynamicalSystem(
  //        {std::make_shared<P1>(kInterAxleLength),
  //         std::make_shared<P2>(kInterAxleLength), std::make_shared<P3>()},
  //        kTimeStep));

  SubsystemList ssl;
  for (auto i = 0; i < N; i++)
  {
    ssl.push_back(std::make_shared<P1>(kInterAxleLength));
  }
  const std::shared_ptr<const ConcatenatedDynamicalSystem> dynamics(
      new ConcatenatedDynamicalSystem(ssl, kTimeStep));


  // Set up initial state.
  x0_ = VectorXf::Zero(dynamics->XDim());
  //x0_(kP1XIdx) = kP1InitialX;
  //x0_(kP1YIdx) = kP1InitialY;
  //x0_(kP1HeadingIdx) = kP1InitialHeading;
  //x0_(kP1VIdx) = kP1InitialSpeed;
  //x0_(kP2XIdx) = kP2InitialX;
  //x0_(kP2YIdx) = kP2InitialY;
  //x0_(kP2HeadingIdx) = kP2InitialHeading;
  //x0_(kP2VIdx) = kP2InitialSpeed;
  //x0_(kP3XIdx) = kP3InitialX;
  //x0_(kP3YIdx) = kP3InitialY;
  //x0_(kP3HeadingIdx) = kP3InitialHeading;
  //x0_(kP3VIdx) = kP3InitialSpeed;

  for (auto i = 0; i < N; i++)
  {
    x0_(kPXIdx[i]) = kPInitialX[i];
    x0_(kPYIdx[i]) = kPInitialY[i];
    x0_(kPHeadingIdx[i]) = kPInitialHeading[i];
    x0_(kPVIdx[i]) = kPInitialSpeed[i];
  }

  // Set up initial strategies and operating point.
  strategies_.reset(new std::vector<Strategy>());
  for (PlayerIndex ii = 0; ii < dynamics->NumPlayers(); ii++)
    strategies_->emplace_back(kNumTimeSteps, dynamics->XDim(),
                              dynamics->UDim(ii));

  operating_point_.reset(
      new OperatingPoint(kNumTimeSteps, dynamics->NumPlayers(), 0.0, dynamics));

  // Set up costs for all players.
  //PlayerCost p1_cost(kStateRegularization, kControlRegularization);
  //PlayerCost p2_cost(kStateRegularization, kControlRegularization);
  //PlayerCost p3_cost(kStateRegularization, kControlRegularization);
  PlayerCost p_cost[N];
  for (auto i = 0; i < N; i++)
  {
    p_cost[i] =  PlayerCost(kStateRegularization, kControlRegularization);
  }


  // Stay in lanes.
  const Polyline2 lane1(
      {Point2(kPInitialX[0], -1000.0), Point2(kPInitialX[0], 1000.0)});
  const Polyline2 lane2(
      {Point2(kPInitialX[1], 1000.0), Point2(kPInitialX[1], 18.0),
       Point2(kPInitialX[1] + 0.5, 15.0), Point2(kPInitialX[1] + 1.0, 14.0),
       Point2(kPInitialX[1] + 3.0, 12.5), Point2(kPInitialX[1] + 6.0, 12.0),
       Point2(1000.0, 12.0)});
  const Polyline2 lane3(
      {Point2(-1000.0, kPInitialY[2]), Point2(1000.0, kPInitialY[2])});

  //const std::shared_ptr<QuadraticPolyline2Cost> p1_lane_cost(
  //    new QuadraticPolyline2Cost(kLaneCostWeight, lane1, {kP1XIdx, kP1YIdx},
  //                               "LaneCenter"));
  //const std::shared_ptr<Polyline2SignedDistanceConstraint> p1_lane_r_constraint(
  //    new Polyline2SignedDistanceConstraint(lane1, {kP1XIdx, kP1YIdx},
  //                                          kLaneHalfWidth, !kOrientedRight,
  //                                          "LaneRightBoundary"));
  //const std::shared_ptr<Polyline2SignedDistanceConstraint> p1_lane_l_constraint(
  //    new Polyline2SignedDistanceConstraint(lane1, {kP1XIdx, kP1YIdx},
  //                                          -kLaneHalfWidth, kOrientedRight,
  //                                          "LaneLeftBoundary"));
  //p1_cost.AddStateCost(p1_lane_cost);
  //p1_cost.AddStateConstraint(p1_lane_r_constraint);
  //p1_cost.AddStateConstraint(p1_lane_l_constraint);

  //const std::shared_ptr<QuadraticPolyline2Cost> p2_lane_cost(
  //    new QuadraticPolyline2Cost(kLaneCostWeight, lane2, {kP2XIdx, kP2YIdx},
  //                               "LaneCenter"));
  //const std::shared_ptr<Polyline2SignedDistanceConstraint> p2_lane_r_constraint(
  //    new Polyline2SignedDistanceConstraint(lane2, {kP2XIdx, kP2YIdx},
  //                                          kLaneHalfWidth, !kOrientedRight,
  //                                          "LaneRightBoundary"));
  //const std::shared_ptr<Polyline2SignedDistanceConstraint> p2_lane_l_constraint(
  //    new Polyline2SignedDistanceConstraint(lane2, {kP2XIdx, kP2YIdx},
  //                                          -kLaneHalfWidth, kOrientedRight,
  //                                          "LaneLeftBoundary"));
  //p2_cost.AddStateCost(p2_lane_cost);
  //p2_cost.AddStateConstraint(p2_lane_r_constraint);
  //p2_cost.AddStateConstraint(p2_lane_l_constraint);

  //const std::shared_ptr<QuadraticPolyline2Cost> p3_lane_cost(
  //    new QuadraticPolyline2Cost(kLaneCostWeight, lane3, {kP3XIdx, kP3YIdx},
  //                               "LaneCenter"));
  //const std::shared_ptr<Polyline2SignedDistanceConstraint> p3_lane_r_constraint(
  //    new Polyline2SignedDistanceConstraint(lane3, {kP3XIdx, kP3YIdx},
  //                                          kLaneHalfWidth, !kOrientedRight,
  //                                          "LaneRightBoundary"));
  //const std::shared_ptr<Polyline2SignedDistanceConstraint> p3_lane_l_constraint(
  //    new Polyline2SignedDistanceConstraint(lane3, {kP3XIdx, kP3YIdx},
  //                                          -kLaneHalfWidth, kOrientedRight,
  //                                          "LaneLeftBoundary"));
  //p3_cost.AddStateCost(p3_lane_cost);
  //p3_cost.AddStateConstraint(p3_lane_r_constraint);
  //p3_cost.AddStateConstraint(p3_lane_l_constraint);

  for (auto i = 0; i < N; i++)
  {
    //p_lane_cost[i]
    p_cost[i].AddStateCost(std::shared_ptr<QuadraticPolyline2Cost>(kLaneCostWeight, lane1, {kPXIdx[i], kPYIdx[i]}, "LaneCenter"));

    //p_lane_r_constraint[i]
    p_cost[i].AddStateConstraint(std::shared_ptr<Polyline2SignedDistanceConstraint>(lane3, {kPXIdx[i], kPYIdx[i]},
                                                                                    kLaneHalfWidth, !kOrientedRight,
                                                                                    "LaneRightBoundary"));

    //p_lane_l_constraint[i]
    p_cost[i].AddStateConstraint(std::shared_ptr<Polyline2SignedDistanceConstraint>(lane3, {kPXIdx[i], kPYIdx[i]},
                                                                                    -kLaneHalfWidth, kOrientedRight,
                                                                                    "LaneLeftBoundary"));
  }

  // Max/min/nominal speed costs.
  //const auto p1_min_v_constraint = std::make_shared<SingleDimensionConstraint>(
  //    kP1VIdx, kMinV, kOrientedRight, "MinV");
  //const auto p1_max_v_constraint = std::make_shared<SingleDimensionConstraint>(
  //    kP1VIdx, kP1MaxV, !kOrientedRight, "MaxV");
  //const auto p1_nominal_v_cost = std::make_shared<QuadraticCost>(
  //    kNominalVCostWeight, kP1VIdx, kP1NominalV, "NominalV");
  //p1_cost.AddStateConstraint(p1_min_v_constraint);
  //p1_cost.AddStateConstraint(p1_max_v_constraint);
  //p1_cost.AddStateCost(p1_nominal_v_cost);

  //const auto p2_min_v_constraint = std::make_shared<SingleDimensionConstraint>(
  //    kP2VIdx, kMinV, kOrientedRight, "MinV");
  //const auto p2_max_v_constraint = std::make_shared<SingleDimensionConstraint>(
  //    kP2VIdx, kP2MaxV, !kOrientedRight, "MaxV");
  //const auto p2_nominal_v_cost = std::make_shared<QuadraticCost>(
  //    kNominalVCostWeight, kP2VIdx, kP2NominalV, "NominalV");
  //p2_cost.AddStateConstraint(p2_min_v_constraint);
  //p2_cost.AddStateConstraint(p2_max_v_constraint);
  //p2_cost.AddStateCost(p2_nominal_v_cost);

  //const auto p3_min_v_constraint = std::make_shared<SingleDimensionConstraint>(
  //    kP3VIdx, kMinV, kOrientedRight, "MinV");
  //const auto p3_max_v_constraint = std::make_shared<SingleDimensionConstraint>(
  //    kP3VIdx, kP3MaxV, !kOrientedRight, "MaxV");
  //const auto p3_nominal_v_cost = std::make_shared<QuadraticCost>(
  //    kNominalVCostWeight, kP3VIdx, kP3NominalV, "NominalV");
  //p3_cost.AddStateConstraint(p3_min_v_constraint);
  //p3_cost.AddStateConstraint(p3_max_v_constraint);
  //p3_cost.AddStateCost(p3_nominal_v_cost);

  for (auto i = 0; i < N; i++)
  {
    //p_min_v_constraint[i]
    p_cost[i].AddStateConstraint(std::make_shared<SingleDimensionConstraint>(
                                 kPVIdx[i], kMinV, kOrientedRight, "MinV"));
    //p_max_v_constraint[i]
    p_cost[i].AddStateConstraint(std::make_shared<SingleDimensionConstraint>(
                                 kPVIdx[i], kPMaxV[i], !kOrientedRight, "MaxV"));
    //p_nominal_v_cost[i]
    p_cost[i].AddStateCost(std::make_shared<QuadraticCost>(
                           kNominalVCostWeight, kPVIdx[i], kPNominalV[i], "NominalV"));
  }


  // // Curvature costs for P1 and P2.
  // const auto p1_curvature_cost = std::make_shared<QuadraticCost>(
  //     kCurvatureCostWeight, kP1PhiIdx, 0.0, "Curvature");
  // p1_cost.AddStateCost(p1_curvature_cost);

  // const auto p2_curvature_cost = std::make_shared<QuadraticCost>(
  //     kCurvatureCostWeight, kP2PhiIdx, 0.0, "Curvature");
  // p2_cost.AddStateCost(p2_curvature_cost);

  // // Penalize acceleration for cars.
  // const auto p1_a_cost = std::make_shared<QuadraticCost>(kACostWeight,
  // kP1AIdx,
  //                                                        0.0,
  //                                                        "Acceleration");
  // p1_cost.AddStateCost(p1_a_cost);

  // const auto p2_a_cost = std::make_shared<QuadraticCost>(kACostWeight,
  // kP2AIdx,
  //                                                        0.0,
  //                                                        "Acceleration");
  // p2_cost.AddStateCost(p2_a_cost);

  // Penalize control effort.
  //const auto p1_omega_max_constraint =
  //    std::make_shared<SingleDimensionConstraint>(
  //        kP1OmegaIdx, kMaxOmega, !kOrientedRight, "SteeringMax");
  //const auto p1_omega_min_constraint =
  //    std::make_shared<SingleDimensionConstraint>(
  //        kP1OmegaIdx, -kMaxOmega, kOrientedRight, "SteeringMin");
  //const auto p1_omega_cost = std::make_shared<QuadraticCost>(
  //    kOmegaCostWeight, kP1OmegaIdx, 0.0, "Steering");
  //const auto p1_jerk_cost =
  //    std::make_shared<QuadraticCost>(kJerkCostWeight, kP1JerkIdx, 0.0, "Jerk");
  //p1_cost.AddControlConstraint(0, p1_omega_max_constraint);
  //p1_cost.AddControlConstraint(0, p1_omega_min_constraint);
  //p1_cost.AddControlCost(0, p1_omega_cost);
  //p1_cost.AddControlCost(0, p1_jerk_cost);

  //const auto p2_omega_max_constraint =
  //    std::make_shared<SingleDimensionConstraint>(
  //        kP2OmegaIdx, kMaxOmega, !kOrientedRight, "SteeringMax");
  //const auto p2_omega_min_constraint =
  //    std::make_shared<SingleDimensionConstraint>(
  //        kP2OmegaIdx, -kMaxOmega, kOrientedRight, "SteeringMin");
  //const auto p2_omega_cost = std::make_shared<QuadraticCost>(
  //    kOmegaCostWeight, kP2OmegaIdx, 0.0, "Steering");
  //const auto p2_jerk_cost =
  //    std::make_shared<QuadraticCost>(kJerkCostWeight, kP2JerkIdx, 0.0, "Jerk");
  //p2_cost.AddControlConstraint(1, p2_omega_max_constraint);
  //p2_cost.AddControlConstraint(1, p2_omega_min_constraint);
  //p2_cost.AddControlCost(1, p2_omega_cost);
  //p2_cost.AddControlCost(1, p2_jerk_cost);

  //const auto p3_omega_max_constraint =
  //    std::make_shared<SingleDimensionConstraint>(
  //        kP3OmegaIdx, kMaxOmega, !kOrientedRight, "SteeringMax");
  //const auto p3_omega_min_constraint =
  //    std::make_shared<SingleDimensionConstraint>(
  //        kP3OmegaIdx, -kMaxOmega, kOrientedRight, "SteeringMin");
  //const auto p3_omega_cost = std::make_shared<QuadraticCost>(
  //    kOmegaCostWeight, kP3OmegaIdx, 0.0, "Steering");
  //const auto p3_a_cost = std::make_shared<QuadraticCost>(kACostWeight, kP3AIdx,
  //                                                       0.0, "Acceleration");
  //p3_cost.AddControlConstraint(2, p3_omega_max_constraint);
  //p3_cost.AddControlConstraint(2, p3_omega_min_constraint);
  //p3_cost.AddControlCost(2, p3_omega_cost);
  //p3_cost.AddControlCost(2, p3_a_cost);


  for (auto i = 0; i < N; i++)
  {
    //p_omega_max_constraint[i]
    p_cost[i].AddControlConstraint(i, std::make_shared<SingleDimensionConstraint>(
                                           kPOmegaIdx[i], kMaxOmega, !kOrientedRight, "SteeringMax"));
    //p_omega_min_constraint[i]
    p_cost[i].AddControlConstraint(i, std::make_shared<SingleDimensionConstraint>(
                                           kPOmegaIdx[i], -kMaxOmega, kOrientedRight, "SteeringMin"));
    //p_omega_cost[i] 
    p_cost[i].AddControlCost(i, std::make_shared<QuadraticCost>(
                                kOmegaCostWeight, kPOmegaIdx[i], 0.0, "Steering"));
    //p_jerk_cost[i]
    p_cost[i].AddControlCost(i, std::make_shared<QuadraticCost>(kJerkCostWeight, kPJerkIdx[i], 0.0, "Jerk"));
  }



  // Goal costs.
  // constexpr float kFinalTimeWindow = 0.5;  // s
  // const auto p1_goalx_cost = std::make_shared<FinalTimeCost>(
  //     std::make_shared<QuadraticCost>(kGoalCostWeight, kP1XIdx, kP1GoalX),
  //     kTimeHorizon - kFinalTimeWindow, "GoalX");
  // const auto p1_goaly_cost = std::make_shared<FinalTimeCost>(
  //     std::make_shared<QuadraticCost>(kGoalCostWeight, kP1YIdx, kP1GoalY),
  //     kTimeHorizon - kFinalTimeWindow, "GoalY");
  // p1_cost.AddStateCost(p1_goalx_cost);
  // p1_cost.AddStateCost(p1_goaly_cost);

  // const auto p2_goalx_cost = std::make_shared<FinalTimeCost>(
  //     std::make_shared<QuadraticCost>(kGoalCostWeight, kP2XIdx, kP2GoalX),
  //     kTimeHorizon - kFinalTimeWindow, "GoalX");
  // const auto p2_goaly_cost = std::make_shared<FinalTimeCost>(
  //     std::make_shared<QuadraticCost>(kGoalCostWeight, kP2YIdx, kP2GoalY),
  //     kTimeHorizon - kFinalTimeWindow, "GoalY");
  // p2_cost.AddStateCost(p2_goalx_cost);
  // p2_cost.AddStateCost(p2_goaly_cost);

  // const auto p3_goalx_cost = std::make_shared<FinalTimeCost>(
  //     std::make_shared<QuadraticCost>(kGoalCostWeight, kP3XIdx, kP3GoalX),
  //     kTimeHorizon - kFinalTimeWindow, "GoalX");
  // const auto p3_goaly_cost = std::make_shared<FinalTimeCost>(
  //     std::make_shared<QuadraticCost>(kGoalCostWeight, kP3YIdx, kP3GoalY),
  //     kTimeHorizon - kFinalTimeWindow, "GoalY");
  // p3_cost.AddStateCost(p3_goalx_cost);
  // p3_cost.AddStateCost(p3_goaly_cost);

  // Pairwise proximity costs.
  // const std::shared_ptr<ProxCost> p1p2_proximity_cost(
  //     new ProxCost(kP1ProximityCostWeight, {kP1XIdx, kP1YIdx},
  //                  {kP2XIdx, kP2YIdx}, kMinProximity, "ProximityP2"));
  // const std::shared_ptr<ProxCost> p1p3_proximity_cost(
  //     new ProxCost(kP1ProximityCostWeight, {kP1XIdx, kP1YIdx},
  //                  {kP3XIdx, kP3YIdx}, kMinProximity, "ProximityP3"));
  // p1_cost.AddStateCost(p1p2_proximity_cost);
  // p1_cost.AddStateCost(p1p3_proximity_cost);

  // const std::shared_ptr<ProxCost> p2p1_proximity_cost(
  //     new ProxCost(kP2ProximityCostWeight, {kP2XIdx, kP2YIdx},
  //                  {kP1XIdx, kP1YIdx}, kMinProximity, "ProximityP1"));
  // const std::shared_ptr<ProxCost> p2p3_proximity_cost(
  //     new ProxCost(kP2ProximityCostWeight, {kP2XIdx, kP2YIdx},
  //                  {kP3XIdx, kP3YIdx}, kMinProximity, "ProximityP3"));
  // p2_cost.AddStateCost(p2p1_proximity_cost);
  // p2_cost.AddStateCost(p2p3_proximity_cost);

  // const std::shared_ptr<ProxCost> p3p1_proximity_cost(
  //     new ProxCost(kP3ProximityCostWeight, {kP3XIdx, kP3YIdx},
  //                  {kP1XIdx, kP1YIdx}, kMinProximity, "ProximityP1"));
  // const std::shared_ptr<ProxCost> p3p2_proximity_cost(
  //     new ProxCost(kP3ProximityCostWeight, {kP3XIdx, kP3YIdx},
  //                  {kP2XIdx, kP2YIdx}, kMinProximity, "ProximityP2"));
  // p3_cost.AddStateCost(p3p1_proximity_cost);
  // p3_cost.AddStateCost(p3p2_proximity_cost);

  // Collision-avoidance constraints.
  //const std::shared_ptr<ProximityConstraint> p1p2_proximity_constraint(
  //    new ProximityConstraint({kP1XIdx, kP1YIdx}, {kP2XIdx, kP2YIdx},
  //                            kMinProximity, kConstraintOrientedInside,
  //                            "ProximityConstraintP2"));
  //const std::shared_ptr<ProximityConstraint> p1p3_proximity_constraint(
  //    new ProximityConstraint({kP1XIdx, kP1YIdx}, {kP3XIdx, kP3YIdx},
  //                            kMinProximity, kConstraintOrientedInside,
  //                            "ProximityConstraintP3"));
  //p1_cost.AddStateConstraint(p1p2_proximity_constraint);
  //p1_cost.AddStateConstraint(p1p3_proximity_constraint);

  //const std::shared_ptr<ProximityConstraint> p2p1_proximity_constraint(
  //    new ProximityConstraint({kP2XIdx, kP2YIdx}, {kP1XIdx, kP1YIdx},
  //                            kMinProximity, kConstraintOrientedInside,
  //                            "ProximityConstraintP1"));
  //const std::shared_ptr<ProximityConstraint> p2p3_proximity_constraint(
  //    new ProximityConstraint({kP2XIdx, kP2YIdx}, {kP3XIdx, kP3YIdx},
  //                            kMinProximity, kConstraintOrientedInside,
  //                            "ProximityConstraintP3"));
  //p2_cost.AddStateConstraint(p2p1_proximity_constraint);
  //p2_cost.AddStateConstraint(p2p3_proximity_constraint);

  //const std::shared_ptr<ProximityConstraint> p3p1_proximity_constraint(
  //    new ProximityConstraint({kP3XIdx, kP3YIdx}, {kP1XIdx, kP1YIdx},
  //                            kMinProximity, kConstraintOrientedInside,
  //                            "ProximityConstraintP1"));
  //const std::shared_ptr<ProximityConstraint> p3p2_proximity_constraint(
  //    new ProximityConstraint({kP3XIdx, kP3YIdx}, {kP2XIdx, kP2YIdx},
  //                            kMinProximity, kConstraintOrientedInside,
  //                            "ProximityConstraintP2"));
  //p3_cost.AddStateConstraint(p3p1_proximity_constraint);
  //p3_cost.AddStateConstraint(p3p2_proximity_constraint);

  for (auto i = 0; i < N; i++)
  {
    for (auto j = 0; j < N; j++)
    {
      p_cost[i].AddStateConstraint(std::shared_ptr<ProximityConstraint>({kPXIdx[i], kPYIdx[i]}, {kPXIdx[j], kPYIdx[j]},
                                                                       kMinProximity, kConstraintOrientedInside,
                                                                       "ProximityConstraint"));
    }
  }

  // Set up solver.
  //solver_.reset(new ILQSolver(dynamics, {p1_cost, p2_cost, p3_cost,
  //                            kTimeHorizon, params));
  solver_.reset(new ILQSolver(dynamics, p_cost,
                              kTimeHorizon, params));
}

inline std::vector<float> NPlayerIntersectionExample::Xs(
    const VectorXf& x) const {
      std::vector<float> ret_val; 
      for (auto i = 0; i < N; i++)
      {
        ret_val.push_back(x(kPXIdx[i]))
      }
      return ret_val;
}

inline std::vector<float> NPlayerIntersectionExample::Ys(
    const VectorXf& x) const {
      std::vector<float> ret_val; 
      for (auto i = 0; i < N; i++)
      {
        ret_val.push_back(x(kPYIdx[i]))
      }
      return ret_val;
}

inline std::vector<float> NPlayerIntersectionExample::Thetas(
    const VectorXf& x) const {
      std::vector<float> ret_val; 
      for (auto i = 0; i < N; i++)
      {
        ret_val.push_back(x(kPHeadingIdx[i]))
      }
      return ret_val;
}

}  // namespace ilqgames
*/
} // namespace ilqgames
