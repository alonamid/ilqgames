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
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; O4;55;29MR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Please contact the author(s) of this libra3ry if you have any questions.
 * Authors: David Fridovich-Keil   ( dfk@eecs.berkeley.edu )
 */

///////////////////////////////////////////////////////////////////////////////
//
// Highway Merging example. Ordering is given by the following:
// P3, P2: Inner lane cars. P3 is in front of P2.
// P1, P4: Outer lane cars - P1 is in front of the merging point, P4 is behind.
// P5, P6: Cars in the merging lane. P5 is in front of P6.
//
///////////////////////////////////////////////////////////////////////////////

#include <ilqgames/constraint/polyline2_signed_distance_constraint.h>
#include <ilqgames/constraint/proximity_constraint.h>
#include <ilqgames/constraint/single_dimension_constraint.h>
#include <ilqgames/cost/curvature_cost.h>
#include <ilqgames/cost/final_time_cost.h>
#include <ilqgames/cost/locally_convex_proximity_cost.h>
#include <ilqgames/cost/nominal_path_length_cost.h>
#include <ilqgames/cost/orientation_cost.h>
#include <ilqgames/cost/proximity_cost.h>
#include <ilqgames/cost/quadratic_cost.h>
#include <ilqgames/cost/quadratic_difference_cost.h>
#include <ilqgames/cost/quadratic_polyline2_cost.h>
#include <ilqgames/cost/semiquadratic_cost.h>
#include <ilqgames/cost/semiquadratic_polyline2_cost.h>
#include <ilqgames/cost/weighted_convex_proximity_cost.h>
#include <ilqgames/dynamics/concatenated_dynamical_system.h>
#include <ilqgames/dynamics/single_player_car_6d.h>
#include <ilqgames/dynamics/single_player_unicycle_4d.h>
// #include <ilqgames/examples/three_player_overtaking_example.h>
#include <ilqgames/examples/highway_merging_example.h>
#include <ilqgames/geometry/polyline2.h>
#include <ilqgames/solver/ilq_solver.h>
#include <ilqgames/solver/problem.h>
#include <ilqgames/solver/solver_params.h>
#include <ilqgames/utils/solver_log.h>
#include <ilqgames/utils/strategy.h>
#include <ilqgames/utils/types.h>

#include <math.h>
#include <memory>
#include <vector>

namespace ilqgames {

namespace {
// Time.
static constexpr Time kTimeStep = 0.1;     // s
static constexpr Time kTimeHorizon = 15.0; // s
static constexpr size_t kNumTimeSteps =
    static_cast<size_t>(kTimeHorizon / kTimeStep);

// Car inter-axle distance.
static constexpr float kInterAxleLength = 4.0; // m

// Cost weights.

static constexpr float kOmegaCostWeight = 50.0;
static constexpr float kJerkCostWeight = 50.0;

static constexpr float kACostWeight = 5.0;
static constexpr float kP1NominalVCostWeight = 0.1;
static constexpr float kP2NominalVCostWeight = 0.1;
static constexpr float kP3NominalVCostWeight = 0.1;
static constexpr float kP4NominalVCostWeight = 0.1;
static constexpr float kP5NominalVCostWeight = 0.1;
static constexpr float kP6NominalVCostWeight = 0.1;

// Newly added, 05-23-2020 19:18 p.m.
static constexpr float kMinV = 0.0;    // m/s
static constexpr float kP1MaxV = 35.8; // m/s
static constexpr float kP2MaxV = 35.8; // m/s
static constexpr float kP4MaxV = 35.8; // m/s
static constexpr float kP3MaxV = 35.8; // m/s
static constexpr float kP5MaxV = 35.8; // m/s
static constexpr float kP6MaxV = 35.8; // m/s

// static constexpr float kLaneCostWeight = 25.0;
// static constexpr float kLaneBoundaryCostWeight = 100.0;

static constexpr float kLaneCostWeight = 2.0;
static constexpr float kLaneBoundaryCostWeight = 100.0;

// static constexpr float kLaneCostWeight = 0.0;
// static constexpr float kLaneBoundaryCostWeight = 0.0;

static constexpr float kMinProximity = 4.75;
static constexpr float kP1ProximityCostWeight = 10.0;
static constexpr float kP2ProximityCostWeight = 10.0;
static constexpr float kP3ProximityCostWeight = 10.0;
static constexpr float kP4ProximityCostWeight = 10.0;
static constexpr float kP5ProximityCostWeight = 10.0;
static constexpr float kP6ProximityCostWeight = 10.0;

// static constexpr float kP4ProximityCostWeight = 0.0;
// static constexpr float kP2ProximityCostWeight = 0.0;
// static constexpr float kP1ProximityCostWeight = 0.0;
// static constexpr float kP3ProximityCostWeight = 0.0;
// static constexpr float kP5ProximityCostWeight = 0.0;
// static constexpr float kP6ProximityCostWeight = 0.0;

using ProxCost = ProximityCost;

// Heading weight
// static constexpr float kNominalHeadingCostWeight = 150.0;
static constexpr float kNominalHeadingCostWeight = 10.0;

static constexpr bool kOrientedRight = true;
static constexpr bool kConstraintOrientedInside = false;
// Lane width.
static constexpr float kLaneHalfWidth = 2.5; // m

// Nominal speed.
static constexpr float kP1NominalV = 5.0; // m/s
static constexpr float kP2NominalV = 5.0; // m/s
static constexpr float kP3NominalV = 5.0; // m/s
static constexpr float kP4NominalV = 5.0; // m/s
static constexpr float kP5NominalV = 5.0; // m/s
static constexpr float kP6NominalV = 5.0; // m/s

// Initial state.

// static constexpr float kP1InitialX = 0.0;   // m
// static constexpr float kP1InitialY = -20.0; // m
// static constexpr float kP2InitialX = 12.0;  // m
// static constexpr float kP2InitialY = -10.0; // m
// static constexpr float kP3InitialX = 0.0;  // m
// static constexpr float kP3InitialY = 20.0; // m
// static constexpr float kP4InitialX = 6.0; // m
// static constexpr float kP4InitialY = 0.0; // m
// static constexpr float kP5InitialX = -5.0; // m
// static constexpr float kP5InitialY = 20.0; // m
// static constexpr float kP6InitialX = -5.0;  // m
// static constexpr float kP6InitialY = -20.0; // m

static constexpr float kP1InitialX = 0.0;   // m
static constexpr float kP1InitialY = -20.0; // m
static constexpr float kP2InitialX = 9.0;  // m
static constexpr float kP2InitialY = -10.0; // m
static constexpr float kP3InitialX = 0.0;  // m
static constexpr float kP3InitialY = 15.0; // m
static constexpr float kP4InitialX = 6.0; // m
static constexpr float kP4InitialY = 0.0; // m
static constexpr float kP5InitialX = -5.0; // m
static constexpr float kP5InitialY = 15.0; // m
static constexpr float kP6InitialX = -5.0;  // m
static constexpr float kP6InitialY = -25.0; // m

static constexpr float kP1InitialHeading = M_PI_2;       // rad
static constexpr float kP2InitialHeading = M_PI * 2 / 3; // rad
static constexpr float kP3InitialHeading = M_PI_2;       // rad
static constexpr float kP4InitialHeading = M_PI * 2 / 3; // rad
static constexpr float kP5InitialHeading = M_PI_2;       // rad
static constexpr float kP6InitialHeading = M_PI_2;       // rad

static constexpr float kP1InitialSpeed = 3.0; // m/s
static constexpr float kP2InitialSpeed = 0.1; // m/s
static constexpr float kP3InitialSpeed = 1.0; // m/s
static constexpr float kP4InitialSpeed = 0.1; // m/s
static constexpr float kP5InitialSpeed = 1.0; // m/s
static constexpr float kP6InitialSpeed = 3.0; // m/s

static constexpr float kP1InitialAcceleration = -0.45; // m/s
static constexpr float kP2InitialAcceleration = -0.45; // m/s
static constexpr float kP3InitialAcceleration = -0.45; // m/s
static constexpr float kP4InitialAcceleration = -0.45; // m/s
static constexpr float kP5InitialAcceleration = -0.45; // m/s
static constexpr float kP6InitialAcceleration = -0.45; // m/s

// State dimensions.
using P1 = SinglePlayerCar6D;
using P2 = SinglePlayerCar6D;
using P3 = SinglePlayerCar6D;
using P4 = SinglePlayerCar6D;
using P5 = SinglePlayerCar6D;
using P6 = SinglePlayerCar6D;

static const Dimension kP1XIdx = P1::kPxIdx;
static const Dimension kP1YIdx = P1::kPyIdx;
static const Dimension kP1HeadingIdx = P1::kThetaIdx;
static const Dimension kP1PhiIdx = P1::kPhiIdx;
static const Dimension kP1VIdx = P1::kVIdx;
static const Dimension kP1AIdx = P1::kAIdx;

static const Dimension kP2XIdx = P1::kNumXDims + P2::kPxIdx;
static const Dimension kP2YIdx = P1::kNumXDims + P2::kPyIdx;
static const Dimension kP2HeadingIdx = P1::kNumXDims + P2::kThetaIdx;
static const Dimension kP2PhiIdx = P1::kNumXDims + P2::kPhiIdx;
static const Dimension kP2VIdx = P1::kNumXDims + P2::kVIdx;
static const Dimension kP2AIdx = P1::kNumXDims + P2::kAIdx;

// static const Dimension kP2XIdx = P3::kNumXDims + P2::kNumXDims + P2::kPxIdx;
// static const Dimension kP2YIdx = P3::kNumXDims + P2::kNumXDims + P2::kPyIdx;
// static const Dimension kP2HeadingIdx = P3::kNumXDims + P2::kNumXDims +
// P2::kThetaIdx; static const Dimension kP2PhiIdx = P3::kNumXDims +
// P2::kNumXDims + P2::kPhiIdx; static const Dimension kP2VIdx = P3::kNumXDims +
// P2::kNumXDims + P2::kVIdx; static const Dimension kP2AIdx = P3::kNumXDims +
// P2::kNumXDims + P2::kAIdx;

static const Dimension kP3XIdx = P1::kNumXDims + P2::kNumXDims + P3::kPxIdx;
static const Dimension kP3YIdx = P1::kNumXDims + P2::kNumXDims + P3::kPyIdx;
static const Dimension kP3HeadingIdx =
    P1::kNumXDims + P2::kNumXDims + P3::kThetaIdx;
static const Dimension kP3PhiIdx = P1::kNumXDims + P2::kNumXDims + P3::kPhiIdx;
static const Dimension kP3VIdx = P1::kNumXDims + P2::kNumXDims + P3::kVIdx;
static const Dimension kP3AIdx = P1::kNumXDims + P2::kNumXDims + P3::kAIdx;

static const Dimension kP4XIdx =
    P1::kNumXDims + P2::kNumXDims + P3::kNumXDims + P4::kPxIdx;
static const Dimension kP4YIdx =
    P1::kNumXDims + P2::kNumXDims + P3::kNumXDims + P4::kPyIdx;
static const Dimension kP4HeadingIdx =
    P1::kNumXDims + P2::kNumXDims + P3::kNumXDims + P4::kThetaIdx;
static const Dimension kP4PhiIdx =
    P1::kNumXDims + P2::kNumXDims + P3::kNumXDims + P4::kPhiIdx;
static const Dimension kP4VIdx =
    P1::kNumXDims + P2::kNumXDims + P3::kNumXDims + P4::kVIdx;
static const Dimension kP4AIdx =
    P1::kNumXDims + P2::kNumXDims + P3::kNumXDims + P4::kAIdx;

static const Dimension kP5XIdx =
    P1::kNumXDims + P2::kNumXDims + P3::kNumXDims + P4::kNumXDims + P5::kPxIdx;
static const Dimension kP5YIdx =
    P1::kNumXDims + P2::kNumXDims + P3::kNumXDims + P1::kNumXDims + P5::kPyIdx;
static const Dimension kP5HeadingIdx = P1::kNumXDims + P2::kNumXDims +
                                       P3::kNumXDims + P4::kNumXDims +
                                       P5::kThetaIdx;
static const Dimension kP5PhiIdx =
    P1::kNumXDims + P2::kNumXDims + P3::kNumXDims + P4::kNumXDims + P5::kPhiIdx;
static const Dimension kP5VIdx =
    P1::kNumXDims + P2::kNumXDims + P3::kNumXDims + P4::kNumXDims + P5::kVIdx;
static const Dimension kP5AIdx =
    P1::kNumXDims + P2::kNumXDims + P3::kNumXDims + P4::kNumXDims + P5::kAIdx;

static const Dimension kP6XIdx = P1::kNumXDims + P2::kNumXDims + P3::kNumXDims +
                                 P4::kNumXDims + P5::kNumXDims + P6::kPxIdx;
static const Dimension kP6YIdx = P1::kNumXDims + P2::kNumXDims + P3::kNumXDims +
                                 P4::kNumXDims + P5::kNumXDims + P6::kPyIdx;
static const Dimension kP6HeadingIdx = P1::kNumXDims + P2::kNumXDims +
                                       P3::kNumXDims + P4::kNumXDims +
                                       P5::kNumXDims + P6::kThetaIdx;
static const Dimension kP6PhiIdx = P1::kNumXDims + P2::kNumXDims +
                                   P3::kNumXDims + P4::kNumXDims +
                                   P5::kNumXDims + P6::kPhiIdx;
static const Dimension kP6VIdx = P1::kNumXDims + P2::kNumXDims + P3::kNumXDims +
                                 P4::kNumXDims + P5::kNumXDims + P6::kVIdx;
static const Dimension kP6AIdx = P1::kNumXDims + P2::kNumXDims + P3::kNumXDims +
                                 P4::kNumXDims + P5::kNumXDims + P6::kAIdx;

// Control dimensions.
static const Dimension kP1OmegaIdx = 0;
static const Dimension kP1JerkIdx = 1;
static const Dimension kP2OmegaIdx = 0;
static const Dimension kP2JerkIdx = 1;
static const Dimension kP3OmegaIdx = 0;
static const Dimension kP3JerkIdx = 1;
static const Dimension kP4OmegaIdx = 0;
static const Dimension kP4JerkIdx = 1;
static const Dimension kP5OmegaIdx = 0;
static const Dimension kP5JerkIdx = 1;
static const Dimension kP6OmegaIdx = 0;
static const Dimension kP6JerkIdx = 1;

} // anonymous namespace

HighwayMergingExample::HighwayMergingExample(const SolverParams &params) {
  // Create dynamics.
  const std::shared_ptr<const ConcatenatedDynamicalSystem> dynamics(
      new ConcatenatedDynamicalSystem(
          {std::make_shared<SinglePlayerCar6D>(kInterAxleLength),
           std::make_shared<SinglePlayerCar6D>(kInterAxleLength),
           std::make_shared<SinglePlayerCar6D>(kInterAxleLength),
           std::make_shared<SinglePlayerCar6D>(kInterAxleLength),
           std::make_shared<SinglePlayerCar6D>(kInterAxleLength),
           std::make_shared<SinglePlayerCar6D>(kInterAxleLength)},
          kTimeStep));

  // Set up initial state.
  x0_ = VectorXf::Zero(dynamics->XDim());

  x0_(kP1XIdx) = kP1InitialX;
  x0_(kP1YIdx) = kP1InitialY;
  x0_(kP1HeadingIdx) = kP1InitialHeading;
  x0_(kP1VIdx) = kP1InitialSpeed;

  x0_(kP2XIdx) = kP2InitialX;
  x0_(kP2YIdx) = kP2InitialY;
  x0_(kP2HeadingIdx) = kP2InitialHeading;
  x0_(kP2VIdx) = kP2InitialSpeed;

  x0_(kP3XIdx) = kP3InitialX;
  x0_(kP3YIdx) = kP3InitialY;
  x0_(kP3HeadingIdx) = kP3InitialHeading;
  x0_(kP3VIdx) = kP3InitialSpeed;

  x0_(kP4XIdx) = kP4InitialX;
  x0_(kP4YIdx) = kP4InitialY;
  x0_(kP4HeadingIdx) = kP4InitialHeading;
  x0_(kP4VIdx) = kP4InitialSpeed;

  x0_(kP5XIdx) = kP5InitialX;
  x0_(kP5YIdx) = kP5InitialY;
  x0_(kP5HeadingIdx) = kP5InitialHeading;
  x0_(kP5VIdx) = kP5InitialSpeed;

  x0_(kP6XIdx) = kP6InitialX;
  x0_(kP6YIdx) = kP6InitialY;
  x0_(kP6HeadingIdx) = kP6InitialHeading;
  x0_(kP6VIdx) = kP6InitialSpeed;

  // Set up initial strategies and operating point.
  strategies_.reset(new std::vector<Strategy>());
  for (PlayerIndex ii = 0; ii < dynamics->NumPlayers(); ii++)
    strategies_->emplace_back(kNumTimeSteps, dynamics->XDim(),
                              dynamics->UDim(ii));

  operating_point_.reset(
      new OperatingPoint(kNumTimeSteps, dynamics->NumPlayers(), 0.0, dynamics));

  // Set up costs for all players.
  PlayerCost p1_cost, p2_cost, p3_cost, p4_cost, p5_cost, p6_cost;

  // Stay in lanes.

  const Polyline2 lane1({Point2(kP2InitialX, kP2InitialY),
                         Point2(kP2InitialX - 2.0, kP2InitialY + 5.0),
                         Point2(kP4InitialX, kP4InitialY),
                         Point2(kP4InitialX - 1.0, kP4InitialY + 5.0),
                         Point2(kP4InitialX - 2.0, kP4InitialY + 10.0),
                         Point2(kP3InitialX, kP3InitialY),
                         Point2(kP3InitialX, 1000.0)});
  const Polyline2 lane2(
      {Point2(kP3InitialX, -1000.0), Point2(kP3InitialX, 1000.0)});
  const Polyline2 lane3(
      {Point2(kP5InitialX, -1000.0), Point2(kP5InitialX, 1000.0)});

  // for (size_t kk = 0; kk < kNumTimeSteps; kk++) {
  //   const Point2 p1_route_point =
  //       lane2.PointAt(kP1InitialY + 1000.0 + kP1InitialSpeed * kk *
  //       kTimeStep);
  //   const Point2 p2_route_point =
  //       lane1.PointAt(kP2InitialSpeed * kk * kTimeStep);
  //   const Point2 p3_route_point = lane1.PointAt(
  //       std::sqrt(29) + std::sqrt(41) + kP4InitialSpeed * kk * kTimeStep);
  //   const Point2 p4_route_point =
  //       lane2.PointAt(kP3InitialY + 1000.0 + kP3InitialSpeed * kk *
  //       kTimeStep);
  //   const Point2 p5_route_point =
  //       lane3.PointAt(kP5InitialY + 1000.0 + kP5InitialSpeed * kk *
  //       kTimeStep);
  //   const Point2 p6_route_point =
  //       lane3.PointAt(kP6InitialY + 1000.0 + kP6InitialSpeed * kk *
  //       kTimeStep);

  //   operating_point_->xs[kk](kP1XIdx) = p1_route_point.x();
  //   operating_point_->xs[kk](kP1YIdx) = p1_route_point.y();
  //   operating_point_->xs[kk](kP2XIdx) = p2_route_point.x();
  //   operating_point_->xs[kk](kP2YIdx) = p2_route_point.y();
  //   operating_point_->xs[kk](kP4XIdx) = p3_route_point.x();
  //   operating_point_->xs[kk](kP4YIdx) = p3_route_point.y();
  //   operating_point_->xs[kk](kP3XIdx) = p4_route_point.x();
  //   operating_point_->xs[kk](kP3YIdx) = p4_route_point.y();
  //   operating_point_->xs[kk](kP5XIdx) = p5_route_point.x();
  //   operating_point_->xs[kk](kP5YIdx) = p5_route_point.y();
  //   operating_point_->xs[kk](kP6XIdx) = p6_route_point.x();
  //   operating_point_->xs[kk](kP6YIdx) = p6_route_point.y();

  //   operating_point_->xs[kk](kP1VIdx) = 10.0;
  //   operating_point_->xs[kk](kP2VIdx) = 10.0;
  //   operating_point_->xs[kk](kP4VIdx) = 10.0;
  //   operating_point_->xs[kk](kP3VIdx) = 10.0;
  //   operating_point_->xs[kk](kP5VIdx) = 10.0;
  //   operating_point_->xs[kk](kP6VIdx) = 10.0;
  // }

  const std::shared_ptr<QuadraticPolyline2Cost> p1_lane_cost(
      new QuadraticPolyline2Cost(kLaneCostWeight, lane2, {kP1XIdx, kP1YIdx},
                                 "LaneCenter"));
  const std::shared_ptr<Polyline2SignedDistanceConstraint> p1_lane_r_constraint(
      new Polyline2SignedDistanceConstraint(lane2, {kP1XIdx, kP1YIdx},
                                            -kLaneHalfWidth, kOrientedRight,
                                            "LaneRightBoundary"));
  const std::shared_ptr<Polyline2SignedDistanceConstraint> p1_lane_l_constraint(
      new Polyline2SignedDistanceConstraint(lane2, {kP1XIdx, kP1YIdx},
                                            kLaneHalfWidth, !kOrientedRight,
                                            "LaneLeftBoundary"));
  p1_cost.AddStateCost(p1_lane_cost);
  p1_cost.AddStateConstraint(p1_lane_r_constraint);
  p1_cost.AddStateConstraint(p1_lane_l_constraint);

  const std::shared_ptr<QuadraticPolyline2Cost> p2_lane_cost(
      new QuadraticPolyline2Cost(kLaneCostWeight, lane1, {kP2XIdx, kP2YIdx},
                                 "LaneCenter"));
  const std::shared_ptr<Polyline2SignedDistanceConstraint> p2_lane_r_constraint(
      new Polyline2SignedDistanceConstraint(lane1, {kP2XIdx, kP2YIdx},
                                            -kLaneHalfWidth, kOrientedRight,
                                            "LaneRightBoundary"));
  const std::shared_ptr<Polyline2SignedDistanceConstraint> p2_lane_l_constraint(
      new Polyline2SignedDistanceConstraint(lane1, {kP2XIdx, kP2YIdx},
                                            kLaneHalfWidth, !kOrientedRight,
                                            "LaneLeftBoundary"));
  p2_cost.AddStateCost(p2_lane_cost);
  p2_cost.AddStateConstraint(p2_lane_r_constraint);
  p2_cost.AddStateConstraint(p2_lane_l_constraint);

  const std::shared_ptr<QuadraticPolyline2Cost> p3_lane_cost(
      new QuadraticPolyline2Cost(kLaneCostWeight, lane2, {kP3XIdx, kP3YIdx},
                                 "LaneCenter"));
  const std::shared_ptr<Polyline2SignedDistanceConstraint> p3_lane_r_constraint(
      new Polyline2SignedDistanceConstraint(lane2, {kP3XIdx, kP3YIdx},
                                            -kLaneHalfWidth, kOrientedRight,
                                            "LaneRightBoundary"));
  const std::shared_ptr<Polyline2SignedDistanceConstraint> p3_lane_l_constraint(
      new Polyline2SignedDistanceConstraint(lane2, {kP3XIdx, kP3YIdx},
                                            kLaneHalfWidth, !kOrientedRight,
                                            "LaneLeftBoundary"));
  p3_cost.AddStateCost(p3_lane_cost);
  p3_cost.AddStateConstraint(p3_lane_r_constraint);
  p3_cost.AddStateConstraint(p3_lane_l_constraint);

  const std::shared_ptr<QuadraticPolyline2Cost> p4_lane_cost(
      new QuadraticPolyline2Cost(kLaneCostWeight, lane1, {kP4XIdx, kP4YIdx},
                                 "LaneCenter"));
  const std::shared_ptr<Polyline2SignedDistanceConstraint> p4_lane_r_constraint(
      new Polyline2SignedDistanceConstraint(lane1, {kP4XIdx, kP4YIdx},
                                            -kLaneHalfWidth, kOrientedRight,
                                            "LaneRightBoundary"));
  const std::shared_ptr<Polyline2SignedDistanceConstraint> p4_lane_l_constraint(
      new Polyline2SignedDistanceConstraint(lane1, {kP4XIdx, kP4YIdx},
                                            kLaneHalfWidth, !kOrientedRight,
                                            "LaneLeftBoundary"));
  p4_cost.AddStateCost(p4_lane_cost);
  p4_cost.AddStateConstraint(p4_lane_r_constraint);
  p4_cost.AddStateConstraint(p4_lane_l_constraint);

  const std::shared_ptr<QuadraticPolyline2Cost> p5_lane_cost(
      new QuadraticPolyline2Cost(kLaneCostWeight, lane3, {kP5XIdx, kP5YIdx},
                                 "LaneCenter"));
  const std::shared_ptr<Polyline2SignedDistanceConstraint> p5_lane_r_constraint(
      new Polyline2SignedDistanceConstraint(lane3, {kP5XIdx, kP5YIdx},
                                            -kLaneHalfWidth, kOrientedRight,
                                            "LaneRightBoundary"));
  const std::shared_ptr<Polyline2SignedDistanceConstraint> p5_lane_l_constraint(
      new Polyline2SignedDistanceConstraint(lane3, {kP5XIdx, kP5YIdx},
                                            kLaneHalfWidth, !kOrientedRight,
                                            "LaneLeftBoundary"));
  p5_cost.AddStateCost(p5_lane_cost);
  p5_cost.AddStateConstraint(p5_lane_r_constraint);
  p5_cost.AddStateConstraint(p5_lane_l_constraint);

  const std::shared_ptr<QuadraticPolyline2Cost> p6_lane_cost(
      new QuadraticPolyline2Cost(kLaneCostWeight, lane3, {kP6XIdx, kP6YIdx},
                                 "LaneCenter"));
  const std::shared_ptr<Polyline2SignedDistanceConstraint> p6_lane_r_constraint(
      new Polyline2SignedDistanceConstraint(lane3, {kP6XIdx, kP6YIdx},
                                            -kLaneHalfWidth, kOrientedRight,
                                            "LaneRightBoundary"));
  const std::shared_ptr<Polyline2SignedDistanceConstraint> p6_lane_l_constraint(
      new Polyline2SignedDistanceConstraint(lane3, {kP6XIdx, kP6YIdx},
                                            kLaneHalfWidth, !kOrientedRight,
                                            "LaneLeftBoundary"));
  p6_cost.AddStateCost(p6_lane_cost);
  p6_cost.AddStateConstraint(p6_lane_r_constraint);
  p6_cost.AddStateConstraint(p6_lane_l_constraint);

  // Max/min/nominal speed costs.

  const auto p1_min_v_constraint = std::make_shared<SingleDimensionConstraint>(
      kP1VIdx, kMinV, kOrientedRight, "MinV");
  const auto p1_max_v_constraint = std::make_shared<SingleDimensionConstraint>(
      kP1VIdx, kP1MaxV, !kOrientedRight, "MaxV");
  const auto p1_nominal_v_cost = std::make_shared<QuadraticCost>(
      kP1NominalVCostWeight, kP1VIdx, kP1NominalV, "NominalV");
  p1_cost.AddStateConstraint(p1_min_v_constraint);
  p1_cost.AddStateConstraint(p1_max_v_constraint);
  p1_cost.AddStateCost(p1_nominal_v_cost);

  const auto p2_min_v_constraint = std::make_shared<SingleDimensionConstraint>(
      kP2VIdx, kMinV, kOrientedRight, "MinV");
  const auto p2_max_v_constraint = std::make_shared<SingleDimensionConstraint>(
      kP2VIdx, kP2MaxV, !kOrientedRight, "MaxV");
  const auto p2_nominal_v_cost = std::make_shared<QuadraticCost>(
      kP2NominalVCostWeight, kP2VIdx, kP2NominalV, "NominalV");
  p2_cost.AddStateConstraint(p2_min_v_constraint);
  p2_cost.AddStateConstraint(p2_max_v_constraint);
  p2_cost.AddStateCost(p2_nominal_v_cost);

  const auto p3_min_v_constraint = std::make_shared<SingleDimensionConstraint>(
      kP3VIdx, kMinV, kOrientedRight, "MinV");
  const auto p3_max_v_constraint = std::make_shared<SingleDimensionConstraint>(
      kP3VIdx, kP3MaxV, !kOrientedRight, "MaxV");
  const auto p3_nominal_v_cost = std::make_shared<QuadraticCost>(
      kP3NominalVCostWeight, kP3VIdx, kP3NominalV, "NominalV");
  p3_cost.AddStateConstraint(p3_min_v_constraint);
  p3_cost.AddStateConstraint(p3_max_v_constraint);
  p3_cost.AddStateCost(p3_nominal_v_cost);

  const auto p4_min_v_constraint = std::make_shared<SingleDimensionConstraint>(
      kP4VIdx, kMinV, kOrientedRight, "MinV");
  const auto p4_max_v_constraint = std::make_shared<SingleDimensionConstraint>(
      kP4VIdx, kP4MaxV, !kOrientedRight, "MaxV");
  const auto p4_nominal_v_cost = std::make_shared<QuadraticCost>(
      kP4NominalVCostWeight, kP4VIdx, kP4NominalV, "NominalV");
  p4_cost.AddStateConstraint(p4_min_v_constraint);
  p4_cost.AddStateConstraint(p4_max_v_constraint);
  p4_cost.AddStateCost(p4_nominal_v_cost);

  const auto p5_min_v_constraint = std::make_shared<SingleDimensionConstraint>(
      kP5VIdx, kMinV, kOrientedRight, "MinV");
  const auto p5_max_v_constraint = std::make_shared<SingleDimensionConstraint>(
      kP5VIdx, kP5MaxV, !kOrientedRight, "MaxV");
  const auto p5_nominal_v_cost = std::make_shared<QuadraticCost>(
      kP5NominalVCostWeight, kP5VIdx, kP5NominalV, "NominalV");
  p5_cost.AddStateConstraint(p5_min_v_constraint);
  p5_cost.AddStateConstraint(p5_max_v_constraint);
  p5_cost.AddStateCost(p5_nominal_v_cost);

  const auto p6_min_v_constraint = std::make_shared<SingleDimensionConstraint>(
      kP6VIdx, kMinV, kOrientedRight, "MinV");
  const auto p6_max_v_constraint = std::make_shared<SingleDimensionConstraint>(
      kP6VIdx, kP6MaxV, !kOrientedRight, "MaxV");
  const auto p6_nominal_v_cost = std::make_shared<QuadraticCost>(
      kP6NominalVCostWeight, kP6VIdx, kP6NominalV, "NominalV");
  p6_cost.AddStateConstraint(p6_min_v_constraint);
  p6_cost.AddStateConstraint(p6_max_v_constraint);
  p6_cost.AddStateCost(p6_nominal_v_cost);

  // // To delete below:

  // const auto p3_nominal_v_cost = std::make_shared<QuadraticCost>(
  //     kP4NominalVCostWeight, kP4VIdx, kP4NominalV, "NominalV");
  // p3_cost.AddStateCost(p3_nominal_v_cost);

  // const auto p2_nominal_v_cost = std::make_shared<QuadraticCost>(
  //     kP2NominalVCostWeight, kP2VIdx, kP2NominalV, "NominalV");
  // p2_cost.AddStateCost(p2_nominal_v_cost);

  // const auto p1_nominal_v_cost = std::make_shared<QuadraticCost>(
  //     kP1NominalVCostWeight, kP1VIdx, kP1NominalV, "NominalV");
  // p1_cost.AddStateCost(p1_nominal_v_cost);

  // const auto p4_nominal_v_cost = std::make_shared<QuadraticCost>(
  //     kP3NominalVCostWeight, kP3VIdx, kP3NominalV, "NominalV");
  // p4_cost.AddStateCost(p4_nominal_v_cost);

  // const auto p5_nominal_v_cost = std::make_shared<QuadraticCost>(
  //     kP5NominalVCostWeight, kP5VIdx, kP5NominalV, "NominalV");
  // p5_cost.AddStateCost(p5_nominal_v_cost);

  // const auto p6_nominal_v_cost = std::make_shared<QuadraticCost>(
  //     kP6NominalVCostWeight, kP6VIdx, kP6NominalV, "NominalV");
  // p6_cost.AddStateCost(p6_nominal_v_cost);

  // // To delete above.

  // Curvature costs for P3 and P2.
  // const auto p3_curvature_cost = std::make_shared<QuadraticCost>(
  //     kCurvatureCostWeight, kP4PhiIdx, 0.0, "Curvature");
  // p3_cost.AddStateCost(p3_curvature_cost);

  // const auto p2_curvature_cost = std::make_shared<QuadraticCost>(
  //     kCurvatureCostWeight, kP2PhiIdx, 0.0, "Curvature");
  // p2_cost.AddStateCost(p2_curvature_cost);

  // // Penalize acceleration for cars.
  // const auto p3_a_cost = std::make_shared<QuadraticCost>(kACostWeight,
  // kP4AIdx,
  //                                                        0.0,
  //                                                        "Acceleration");
  // p3_cost.AddStateCost(p3_a_cost);

  // const auto p2_a_cost = std::make_shared<QuadraticCost>(kACostWeight,
  // kP2AIdx,
  //                                                        0.0,
  //                                                        "Acceleration");
  // p2_cost.AddStateCost(p2_a_cost);

  // Penalize control effort.

  const auto p1_omega_cost = std::make_shared<QuadraticCost>(
      kOmegaCostWeight, kP1OmegaIdx, 0.0, "Steering");
  const auto p1_jerk_cost =
      std::make_shared<QuadraticCost>(kJerkCostWeight, kP1JerkIdx, 0.0, "Jerk");
  p1_cost.AddControlCost(0, p1_omega_cost);
  p1_cost.AddControlCost(0, p1_jerk_cost);

  const auto p2_omega_cost = std::make_shared<QuadraticCost>(
      kOmegaCostWeight, kP2OmegaIdx, 0.0, "Steering");
  const auto p2_jerk_cost =
      std::make_shared<QuadraticCost>(kJerkCostWeight, kP2JerkIdx, 0.0, "Jerk");
  p2_cost.AddControlCost(1, p2_omega_cost);
  p2_cost.AddControlCost(1, p2_jerk_cost);

  const auto p3_omega_cost = std::make_shared<QuadraticCost>(
      kOmegaCostWeight, kP3OmegaIdx, 0.0, "Steering");
  const auto p3_jerk_cost =
      std::make_shared<QuadraticCost>(kJerkCostWeight, kP3JerkIdx, 0.0, "Jerk");
  p3_cost.AddControlCost(2, p3_omega_cost);
  p3_cost.AddControlCost(2, p3_jerk_cost);

  const auto p4_omega_cost = std::make_shared<QuadraticCost>(
      kOmegaCostWeight, kP4OmegaIdx, 0.0, "Steering");
  const auto p4_jerk_cost =
      std::make_shared<QuadraticCost>(kJerkCostWeight, kP4JerkIdx, 0.0, "Jerk");
  p4_cost.AddControlCost(3, p4_omega_cost);
  p4_cost.AddControlCost(3, p4_jerk_cost);

  const auto p5_omega_cost = std::make_shared<QuadraticCost>(
      kOmegaCostWeight, kP5OmegaIdx, 0.0, "Steering");
  const auto p5_jerk_cost =
      std::make_shared<QuadraticCost>(kJerkCostWeight, kP5JerkIdx, 0.0, "Jerk");
  p5_cost.AddControlCost(4, p5_omega_cost);
  p5_cost.AddControlCost(4, p5_jerk_cost);

  const auto p6_omega_cost = std::make_shared<QuadraticCost>(
      kOmegaCostWeight, kP6OmegaIdx, 0.0, "Steering");
  const auto p6_jerk_cost =
      std::make_shared<QuadraticCost>(kACostWeight, kP6JerkIdx, 0.0, "Jerk");
  p6_cost.AddControlCost(5, p6_omega_cost);
  p6_cost.AddControlCost(5, p6_jerk_cost);

  // Check all p3 <-> p4 swaps (lower case) above !!!

  // Pairwise proximity costs.

  // Pairwise proximity costs: Player 1.

  const std::shared_ptr<ProximityConstraint> p1p2_proximity_constraint(
      new ProximityConstraint({kP1XIdx, kP1YIdx}, {kP2XIdx, kP2YIdx},
                              kMinProximity, kConstraintOrientedInside,
                              "ProximityConstraintP2"));
  const std::shared_ptr<ProximityConstraint> p1p3_proximity_constraint(
      new ProximityConstraint({kP1XIdx, kP1YIdx}, {kP3XIdx, kP3YIdx},
                              kMinProximity, kConstraintOrientedInside,
                              "ProximityConstraintP3"));
  const std::shared_ptr<ProximityConstraint> p1p4_proximity_constraint(
      new ProximityConstraint({kP1XIdx, kP1YIdx}, {kP4XIdx, kP4YIdx},
                              kMinProximity, kConstraintOrientedInside,
                              "ProximityConstraintP4"));
  const std::shared_ptr<ProximityConstraint> p1p5_proximity_constraint(
      new ProximityConstraint({kP1XIdx, kP1YIdx}, {kP5XIdx, kP5YIdx},
                              kMinProximity, kConstraintOrientedInside,
                              "ProximityConstraintP5"));
  const std::shared_ptr<ProximityConstraint> p1p6_proximity_constraint(
      new ProximityConstraint({kP1XIdx, kP1YIdx}, {kP6XIdx, kP6YIdx},
                              kMinProximity, kConstraintOrientedInside,
                              "ProximityConstraintP6"));
  p1_cost.AddStateConstraint(p1p2_proximity_constraint);
  p1_cost.AddStateConstraint(p1p3_proximity_constraint);
  p1_cost.AddStateConstraint(p1p4_proximity_constraint);
  p1_cost.AddStateConstraint(p1p5_proximity_constraint);
  p1_cost.AddStateConstraint(p1p6_proximity_constraint);

  // Pairwise proximity costs: Player 2.

  const std::shared_ptr<InitialTimeCost> p2p1_initial_proximity_cost(
      new InitialTimeCost(
          std::shared_ptr<QuadraticDifferenceCost>(new QuadraticDifferenceCost(
              kP2ProximityCostWeight, {kP2XIdx, kP2YIdx}, {kP1XIdx, kP1YIdx})),
          params.adversarial_time, "InitialProximityCostP2P1"));
  p2_cost.AddStateCost(p2p1_initial_proximity_cost);
  initial_time_costs_.push_back(p2p1_initial_proximity_cost);

  const std::shared_ptr<FinalTimeCost> p2p1_final_proximity_cost(
      new FinalTimeCost(std::shared_ptr<ProxCost>(new ProxCost(
                            kP2ProximityCostWeight, {kP2XIdx, kP2YIdx},
                            {kP1XIdx, kP1YIdx}, kMinProximity)),
                        params.adversarial_time, "FinalProximityCostP2P1"));
  p2_cost.AddStateCost(p2p1_final_proximity_cost);
  final_time_costs_.push_back(p2p1_final_proximity_cost);

  const std::shared_ptr<ProxCost> p2p3_proximity_cost(
      new ProxCost(kP2ProximityCostWeight, {kP2XIdx, kP2YIdx},
                   {kP3XIdx, kP3YIdx}, kMinProximity, "ProximityP3"));
  const std::shared_ptr<ProxCost> p2p4_proximity_cost(
      new ProxCost(kP2ProximityCostWeight, {kP2XIdx, kP2YIdx},
                   {kP4XIdx, kP4YIdx}, kMinProximity, "ProximityP4"));
  const std::shared_ptr<ProxCost> p2p5_proximity_cost(
      new ProxCost(kP2ProximityCostWeight, {kP2XIdx, kP2YIdx},
                   {kP5XIdx, kP5YIdx}, kMinProximity, "ProximityP5"));
  const std::shared_ptr<ProxCost> p2p6_proximity_cost(
      new ProxCost(kP2ProximityCostWeight, {kP2XIdx, kP2YIdx},
                   {kP6XIdx, kP6YIdx}, kMinProximity, "ProximityP6"));

  p2_cost.AddStateCost(p2p3_proximity_cost);
  p2_cost.AddStateCost(p2p4_proximity_cost);
  p2_cost.AddStateCost(p2p5_proximity_cost);
  p2_cost.AddStateCost(p2p6_proximity_cost);

  // Pairwise proximity costs: Player 3.
  // p4p3_proximity_cost:  Modified to include adversarial phase

  const std::shared_ptr<InitialTimeCost> p3p1_initial_proximity_cost(
      new InitialTimeCost(
          std::shared_ptr<QuadraticDifferenceCost>(new QuadraticDifferenceCost(
              kP3ProximityCostWeight, {kP3XIdx, kP3YIdx}, {kP1XIdx, kP1YIdx})),
          params.adversarial_time, "InitialProximityCostP3P1"));
  p3_cost.AddStateCost(p3p1_initial_proximity_cost);
  initial_time_costs_.push_back(p3p1_initial_proximity_cost);

  const std::shared_ptr<FinalTimeCost> p3p1_final_proximity_cost(
      new FinalTimeCost(std::shared_ptr<ProxCost>(new ProxCost(
                            kP3ProximityCostWeight, {kP3XIdx, kP3YIdx},
                            {kP1XIdx, kP1YIdx}, kMinProximity)),
                        params.adversarial_time, "FinalProximityCostP3P1"));
  p3_cost.AddStateCost(p3p1_final_proximity_cost);
  final_time_costs_.push_back(p3p1_final_proximity_cost);

  const std::shared_ptr<ProxCost> p3p2_proximity_cost(
      new ProxCost(kP3ProximityCostWeight, {kP3XIdx, kP3YIdx},
                   {kP2XIdx, kP2YIdx}, kMinProximity, "ProximityP2"));
  const std::shared_ptr<ProxCost> p3p4_proximity_cost(
      new ProxCost(kP3ProximityCostWeight, {kP3XIdx, kP3YIdx},
                   {kP4XIdx, kP4YIdx}, kMinProximity, "ProximityP4"));
  const std::shared_ptr<ProxCost> p3p5_proximity_cost(
      new ProxCost(kP3ProximityCostWeight, {kP3XIdx, kP3YIdx},
                   {kP5XIdx, kP5YIdx}, kMinProximity, "ProximityP5"));
  const std::shared_ptr<ProxCost> p3p6_proximity_cost(
      new ProxCost(kP3ProximityCostWeight, {kP3XIdx, kP3YIdx},
                   {kP6XIdx, kP6YIdx}, kMinProximity, "ProximityP6"));
  p3_cost.AddStateCost(p3p2_proximity_cost);
  p3_cost.AddStateCost(p3p4_proximity_cost);
  p3_cost.AddStateCost(p3p5_proximity_cost);
  p3_cost.AddStateCost(p3p6_proximity_cost);

  // Pairwise proximity costs: Player 4.
  // p1p3_proximity_cost:  Modified to include adversarial phase

  const std::shared_ptr<InitialTimeCost> p4p1_initial_proximity_cost(
      new InitialTimeCost(
          std::shared_ptr<QuadraticDifferenceCost>(new QuadraticDifferenceCost(
              kP4ProximityCostWeight, {kP4XIdx, kP4YIdx}, {kP1XIdx, kP1YIdx})),
          params.adversarial_time, "InitialProximityCostP4P1"));
  p4_cost.AddStateCost(p4p1_initial_proximity_cost);
  initial_time_costs_.push_back(p4p1_initial_proximity_cost);

  const std::shared_ptr<FinalTimeCost> p4p1_final_proximity_cost(
      new FinalTimeCost(std::shared_ptr<ProxCost>(new ProxCost(
                            kP4ProximityCostWeight, {kP4XIdx, kP4YIdx},
                            {kP1XIdx, kP1YIdx}, kMinProximity)),
                        params.adversarial_time, "FinalProximityCostP4P1"));
  p4_cost.AddStateCost(p4p1_final_proximity_cost);
  final_time_costs_.push_back(p4p1_final_proximity_cost);

  const std::shared_ptr<ProxCost> p4p2_proximity_cost(
      new ProxCost(kP4ProximityCostWeight, {kP4XIdx, kP4YIdx},
                   {kP2XIdx, kP2YIdx}, kMinProximity, "ProximityP2"));
  const std::shared_ptr<ProxCost> p4p3_proximity_cost(
      new ProxCost(kP4ProximityCostWeight, {kP4XIdx, kP4YIdx},
                   {kP3XIdx, kP3YIdx}, kMinProximity, "ProximityP3"));
  const std::shared_ptr<ProxCost> p4p5_proximity_cost(
      new ProxCost(kP4ProximityCostWeight, {kP4XIdx, kP4YIdx},
                   {kP5XIdx, kP5YIdx}, kMinProximity, "ProximityP5"));
  const std::shared_ptr<ProxCost> p4p6_proximity_cost(
      new ProxCost(kP4ProximityCostWeight, {kP4XIdx, kP4YIdx},
                   {kP6XIdx, kP6YIdx}, kMinProximity, "ProximityP6"));
  p4_cost.AddStateCost(p4p2_proximity_cost);
  p4_cost.AddStateCost(p4p3_proximity_cost);
  p4_cost.AddStateCost(p4p5_proximity_cost);
  p4_cost.AddStateCost(p4p6_proximity_cost);

  // Pairwise proximity costs: Player 5.

  const std::shared_ptr<InitialTimeCost> p5p1_initial_proximity_cost(
      new InitialTimeCost(
          std::shared_ptr<QuadraticDifferenceCost>(new QuadraticDifferenceCost(
              kP5ProximityCostWeight, {kP5XIdx, kP5YIdx}, {kP1XIdx, kP1YIdx})),
          params.adversarial_time, "InitialProximityCostP5P1"));
  p5_cost.AddStateCost(p5p1_initial_proximity_cost);
  initial_time_costs_.push_back(p5p1_initial_proximity_cost);

  const std::shared_ptr<FinalTimeCost> p5p1_final_proximity_cost(
      new FinalTimeCost(std::shared_ptr<ProxCost>(new ProxCost(
                            kP5ProximityCostWeight, {kP5XIdx, kP5YIdx},
                            {kP1XIdx, kP1YIdx}, kMinProximity)),
                        params.adversarial_time, "FinalProximityCostP5P1"));
  p5_cost.AddStateCost(p5p1_final_proximity_cost);
  final_time_costs_.push_back(p5p1_final_proximity_cost);

  const std::shared_ptr<ProxCost> p5p2_proximity_cost(
      new ProxCost(kP5ProximityCostWeight, {kP5XIdx, kP5YIdx},
                   {kP2XIdx, kP2YIdx}, kMinProximity, "ProximityP2"));
  const std::shared_ptr<ProxCost> p5p4_proximity_cost(
      new ProxCost(kP5ProximityCostWeight, {kP5XIdx, kP5YIdx},
                   {kP4XIdx, kP4YIdx}, kMinProximity, "ProximityP4"));
  const std::shared_ptr<ProxCost> p5p3_proximity_cost(
      new ProxCost(kP5ProximityCostWeight, {kP5XIdx, kP5YIdx},
                   {kP3XIdx, kP3YIdx}, kMinProximity, "ProximityP3"));
  const std::shared_ptr<ProxCost> p5p6_proximity_cost(
      new ProxCost(kP5ProximityCostWeight, {kP5XIdx, kP5YIdx},
                   {kP6XIdx, kP6YIdx}, kMinProximity, "ProximityP6"));

  p5_cost.AddStateCost(p5p2_proximity_cost);
  p5_cost.AddStateCost(p5p4_proximity_cost);
  p5_cost.AddStateCost(p5p3_proximity_cost);
  p5_cost.AddStateCost(p5p6_proximity_cost);

  // Pairwise proximity costs: Player 6.

  const std::shared_ptr<InitialTimeCost> p6p1_initial_proximity_cost(
      new InitialTimeCost(
          std::shared_ptr<QuadraticDifferenceCost>(new QuadraticDifferenceCost(
              kP6ProximityCostWeight, {kP6XIdx, kP6YIdx}, {kP1XIdx, kP1YIdx})),
          params.adversarial_time, "InitialProximityCostP6P1"));
  p6_cost.AddStateCost(p6p1_initial_proximity_cost);
  initial_time_costs_.push_back(p6p1_initial_proximity_cost);

  const std::shared_ptr<FinalTimeCost> p6p1_final_proximity_cost(
      new FinalTimeCost(std::shared_ptr<ProxCost>(new ProxCost(
                            kP6ProximityCostWeight, {kP6XIdx, kP6YIdx},
                            {kP1XIdx, kP1YIdx}, kMinProximity)),
                        params.adversarial_time, "FinalProximityCostP6P1"));
  p6_cost.AddStateCost(p6p1_final_proximity_cost);
  final_time_costs_.push_back(p6p1_final_proximity_cost);

  const std::shared_ptr<ProxCost> p6p2_proximity_cost(
      new ProxCost(kP6ProximityCostWeight, {kP6XIdx, kP6YIdx},
                   {kP2XIdx, kP2YIdx}, kMinProximity, "ProximityP2"));
  const std::shared_ptr<ProxCost> p6p4_proximity_cost(
      new ProxCost(kP6ProximityCostWeight, {kP6XIdx, kP6YIdx},
                   {kP4XIdx, kP4YIdx}, kMinProximity, "ProximityP4"));
  const std::shared_ptr<ProxCost> p6p3_proximity_cost(
      new ProxCost(kP6ProximityCostWeight, {kP6XIdx, kP6YIdx},
                   {kP3XIdx, kP3YIdx}, kMinProximity, "ProximityP3"));
  const std::shared_ptr<ProxCost> p6p5_proximity_cost(
      new ProxCost(kP6ProximityCostWeight, {kP6XIdx, kP6YIdx},
                   {kP5XIdx, kP5YIdx}, kMinProximity, "ProximityP5"));

  p6_cost.AddStateCost(p6p2_proximity_cost);
  p6_cost.AddStateCost(p6p4_proximity_cost);
  p6_cost.AddStateCost(p6p3_proximity_cost);
  p6_cost.AddStateCost(p6p5_proximity_cost);

  // Set up solver.
  solver_.reset(new ILQSolver(
      dynamics, {p1_cost, p2_cost, p3_cost, p4_cost, p5_cost, p6_cost},
      kTimeHorizon, params));

  // std::cout << x0_.transpose() << std::endl;
}

inline std::vector<float> HighwayMergingExample::Xs(const VectorXf &x) const {
  return {x(kP1XIdx), x(kP2XIdx), x(kP3XIdx),
          x(kP4XIdx), x(kP5XIdx), x(kP6XIdx)};
}

inline std::vector<float> HighwayMergingExample::Ys(const VectorXf &x) const {
  return {x(kP1YIdx), x(kP2YIdx), x(kP3YIdx),
          x(kP4YIdx), x(kP5YIdx), x(kP6YIdx)};
}

inline std::vector<float>
HighwayMergingExample::Thetas(const VectorXf &x) const {
  return {x(kP1HeadingIdx), x(kP2HeadingIdx), x(kP3HeadingIdx),
          x(kP4HeadingIdx), x(kP5HeadingIdx), x(kP6HeadingIdx)};
}

} // namespace ilqgames
