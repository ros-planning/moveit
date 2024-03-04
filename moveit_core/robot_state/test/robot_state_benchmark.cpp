/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2023, PickNik Robotics.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Robert Haschke, Mario Prats */

// This file contains various benchmarks related to RobotState and matrix multiplication and inverse with Eigen types.
// To run this benchmark, 'cd' to the build/moveit_core/robot_state directory and directly run the binary.

#include <benchmark/benchmark.h>
#include <random>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/treejnttojacsolver.hpp>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/utils/robot_model_test_utils.h>

// Robot and planning group for benchmarks.
constexpr char PANDA_TEST_ROBOT[] = "panda";
constexpr char PANDA_TEST_GROUP[] = "panda_arm";

namespace
{
Eigen::Isometry3d createTestIsometry()
{
  // An arbitrary Eigen::Isometry3d object.
  return Eigen::Translation3d(1, 2, 3) * Eigen::AngleAxisd(0.13 * M_PI, Eigen::Vector3d::UnitX()) *
         Eigen::AngleAxisd(0.29 * M_PI, Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(0.42 * M_PI, Eigen::Vector3d::UnitZ());
}
}  // namespace

// Benchmark time to multiply an Eigen::Affine3d with an Eigen::Matrix4d.
static void multiplyAffineTimesMatrixNoAlias(benchmark::State& st)
{
  Eigen::Isometry3d isometry = createTestIsometry();
  Eigen::Affine3d result;
  for (auto _ : st)
  {
    benchmark::DoNotOptimize(result.affine().noalias() = isometry.affine() * isometry.matrix());
    benchmark::ClobberMemory();
  }
}

// Benchmark time to multiply an Eigen::Matrix4d with an Eigen::Matrix4d.
static void multiplyMatrixTimesMatrixNoAlias(benchmark::State& st)
{
  Eigen::Isometry3d isometry = createTestIsometry();
  Eigen::Isometry3d result;
  for (auto _ : st)
  {
    benchmark::DoNotOptimize(result.matrix().noalias() = isometry.matrix() * isometry.matrix());
    benchmark::ClobberMemory();
  }
}

// Benchmark time to multiply an Eigen::Isometry3d with an Eigen::Isometry3d.
static void multiplyIsometryTimesIsometryNoAlias(benchmark::State& st)
{
  Eigen::Isometry3d isometry = createTestIsometry();
  Eigen::Isometry3d result;
  for (auto _ : st)
  {
    benchmark::DoNotOptimize(result.affine().noalias() = isometry.affine() * isometry.matrix());
    benchmark::ClobberMemory();
  }
}

// Benchmark time to multiply an Eigen::Matrix4d with an Eigen::Matrix4d.
static void multiplyMatrixTimesMatrix(benchmark::State& st)
{
  Eigen::Isometry3d isometry = createTestIsometry();
  Eigen::Isometry3d result;
  for (auto _ : st)
  {
    benchmark::DoNotOptimize(result.matrix() = isometry.matrix() * isometry.matrix());
    benchmark::ClobberMemory();
  }
}
// Benchmark time to multiply an Eigen::Isometry3d with an Eigen::Isometry3d.
static void multiplyIsometryTimesIsometry(benchmark::State& st)
{
  Eigen::Isometry3d isometry = createTestIsometry();
  Eigen::Isometry3d result;
  for (auto _ : st)
  {
    benchmark::DoNotOptimize(result = isometry * isometry);
    benchmark::ClobberMemory();
  }
}

// Benchmark time to invert an Eigen::Isometry3d.
static void inverseIsometry3d(benchmark::State& st)
{
  Eigen::Isometry3d isometry = createTestIsometry();
  Eigen::Isometry3d result;
  for (auto _ : st)
  {
    benchmark::DoNotOptimize(result = isometry.inverse());
    benchmark::ClobberMemory();
  }
}

// Benchmark time to invert an Eigen::Affine3d(Eigen::Isometry).
static void inverseAffineIsometry(benchmark::State& st)
{
  Eigen::Affine3d affine = createTestIsometry();
  Eigen::Affine3d result;
  for (auto _ : st)
  {
    benchmark::DoNotOptimize(result = affine.inverse(Eigen::Isometry));
    benchmark::ClobberMemory();
  }
}

// Benchmark time to invert an Eigen::Affine3d.
static void inverseAffine(benchmark::State& st)
{
  Eigen::Affine3d affine = createTestIsometry();
  Eigen::Affine3d result;
  for (auto _ : st)
  {
    benchmark::DoNotOptimize(result = affine.inverse());
    benchmark::ClobberMemory();
  }
}

// Benchmark time to invert an Eigen::Matrix4d.
static void inverseMatrix4d(benchmark::State& st)
{
  Eigen::Affine3d affine = createTestIsometry();
  Eigen::Affine3d result;
  for (auto _ : st)
  {
    benchmark::DoNotOptimize(result = affine.matrix().inverse());
    benchmark::ClobberMemory();
  }
}

struct RobotStateBenchmark : ::benchmark::Fixture
{
  void SetUp(const ::benchmark::State& /*state*/) override
  {
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Warn))
      ros::console::notifyLoggerLevelsChanged();

    robot_model = moveit::core::loadTestingRobotModel(PANDA_TEST_ROBOT);
  }

  std::vector<moveit::core::RobotState> constructStates(size_t num)
  {
    std::vector<moveit::core::RobotState> states;
    states.reserve(num);
    for (size_t i = 0; i < num; i++)
      states.emplace_back(robot_model);
    return states;
  }
  std::vector<moveit::core::RobotState> constructStates(size_t num, const moveit::core::RobotState& state)
  {
    std::vector<moveit::core::RobotState> states;
    states.reserve(num);
    for (size_t i = 0; i < num; i++)
      states.emplace_back(state);
    return states;
  }

  std::vector<size_t> randomPermudation(size_t num)
  {
    std::vector<size_t> result;
    result.reserve(num);
    for (size_t i = 0; i < num; i++)
      result.push_back(i);

    std::random_device random_device;
    std::mt19937 generator(random_device());

    std::shuffle(result.begin(), result.end(), generator);
    return result;
  }

  moveit::core::RobotModelPtr robot_model;
};

// Benchmark time to construct RobotStates
BENCHMARK_DEFINE_F(RobotStateBenchmark, construct)(benchmark::State& st)
{
  for (auto _ : st)
  {
    auto states = constructStates(st.range(0));
    benchmark::DoNotOptimize(states);
    benchmark::ClobberMemory();
  }
}

// Benchmark time to copy-construct a RobotState.
BENCHMARK_DEFINE_F(RobotStateBenchmark, copyConstruct)(benchmark::State& st)
{
  moveit::core::RobotState state(robot_model);
  state.setToDefaultValues();
  state.update();

  for (auto _ : st)
  {
    auto states = constructStates(st.range(0), state);
    benchmark::DoNotOptimize(states);
    benchmark::ClobberMemory();
  }
}

// Benchmark time to call `setToRandomPositions` and `update` on RobotState.
BENCHMARK_DEFINE_F(RobotStateBenchmark, update)(benchmark::State& st)
{
  auto states = constructStates(st.range(0));
  auto permutation = randomPermudation(states.size());
  for (auto _ : st)
  {
    for (auto i : permutation)  // process states in random order to challenge the cache
    {
      states[i].setToRandomPositions();
      states[i].update();
    }
  }
}

// Benchmark time to compute the Jacobian, using MoveIt's `getJacobian` function.
BENCHMARK_DEFINE_F(RobotStateBenchmark, jacobianMoveIt)(benchmark::State& st)
{
  moveit::core::RobotState state(robot_model);
  const moveit::core::JointModelGroup* jmg = state.getJointModelGroup(PANDA_TEST_GROUP);
  if (!jmg)
  {
    st.SkipWithError("The planning group doesn't exist.");
    return;
  }

  // Manually seeded RandomNumberGenerator for deterministic results
  random_numbers::RandomNumberGenerator rng(0);

  for (auto _ : st)
  {
    // Time only the jacobian computation, not the forward kinematics.
    st.PauseTiming();
    state.setToRandomPositions(jmg, rng);
    state.updateLinkTransforms();
    st.ResumeTiming();
    state.getJacobian(jmg);
  }
}

// Benchmark time to compute the Jacobian using KDL.
BENCHMARK_DEFINE_F(RobotStateBenchmark, jacobianKDL)(benchmark::State& st)
{
  moveit::core::RobotState state(robot_model);
  const moveit::core::JointModelGroup* jmg = state.getJointModelGroup(PANDA_TEST_GROUP);
  if (!jmg)
  {
    st.SkipWithError("The planning group doesn't exist.");
    return;
  }
  KDL::Tree kdl_tree;
  if (!kdl_parser::treeFromUrdfModel(*robot_model->getURDF(), kdl_tree))
  {
    st.SkipWithError("Can't create KDL tree.");
    return;
  }

  KDL::TreeJntToJacSolver jacobian_solver(kdl_tree);
  KDL::Jacobian jacobian(kdl_tree.getNrOfJoints());
  KDL::JntArray kdl_q(kdl_tree.getNrOfJoints());
  const std::string tip_link = jmg->getLinkModelNames().back();

  // Manually seeded RandomNumberGenerator for deterministic results
  random_numbers::RandomNumberGenerator rng(0);

  for (auto _ : st)
  {
    // Time only the jacobian computation, not the forward kinematics.
    st.PauseTiming();
    state.setToRandomPositions(jmg, rng);
    state.copyJointGroupPositions(jmg, &kdl_q.data[0]);
    st.ResumeTiming();
    jacobian_solver.JntToJac(kdl_q, jacobian, tip_link);
  }
}

BENCHMARK(multiplyAffineTimesMatrixNoAlias);
BENCHMARK(multiplyMatrixTimesMatrixNoAlias);
BENCHMARK(multiplyIsometryTimesIsometryNoAlias);
BENCHMARK(multiplyMatrixTimesMatrix);
BENCHMARK(multiplyIsometryTimesIsometry);

BENCHMARK(inverseIsometry3d);
BENCHMARK(inverseAffineIsometry);
BENCHMARK(inverseAffine);
BENCHMARK(inverseMatrix4d);

BENCHMARK_REGISTER_F(RobotStateBenchmark, construct)
    ->RangeMultiplier(10)
    ->Range(100, 10000)
    ->Unit(benchmark::kMillisecond);
BENCHMARK_REGISTER_F(RobotStateBenchmark, copyConstruct)
    ->RangeMultiplier(10)
    ->Range(100, 10000)
    ->Unit(benchmark::kMillisecond);
BENCHMARK_REGISTER_F(RobotStateBenchmark, update)->RangeMultiplier(10)->Range(10, 10000)->Unit(benchmark::kMillisecond);

BENCHMARK_REGISTER_F(RobotStateBenchmark, jacobianMoveIt);
BENCHMARK_REGISTER_F(RobotStateBenchmark, jacobianKDL);

BENCHMARK_MAIN();
