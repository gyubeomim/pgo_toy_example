/*

  The MIT License

  Copyright (c) 2020 Edward Im

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.

*/
#include "alias_types.h"

#include <Eigen/Dense>

#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/optimizable_graph.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/solver.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/types/slam3d/se3quat.h>
#include <g2o/types/slam3d/types_slam3d.h>

#include <iostream>
#include <vector>

/// \brief Get data from the random uniform distribution.
/// \param[in] lowerBndr Lower bound.
/// \param[in] lowerBndr Upper bound.
static double UniformRand(double lowerBndr, double upperBndr){
  return lowerBndr + ((double) std::rand() / (RAND_MAX + 1.0)) * (upperBndr - lowerBndr);
}

/// \brief Get data from the random gaussian distribution.
/// \param[in] mean Mean of gaussian distribution.
/// \param[in] sigma Sigma of gaussian distribution.
static double GaussRand(double mean, double sigma){
  double x, y, r2;
  do {
    x = -1.0 + 2.0 * UniformRand(0.0, 1.0);
    y = -1.0 + 2.0 * UniformRand(0.0, 1.0);
    r2 = x * x + y * y;
  } while (r2 > 1.0 || r2 == 0.0);
  return mean + sigma * y * std::sqrt(-2.0 * log(r2) / r2);
}

/// \class Sampling
/// \brief Sample data from the specific distribution. Currently available distrubitions are gaussian and uniform distribution.
class Sampling {
 public:
  /// \brief Sample data from the uniform distribution.
  static int Uniform(int from, int to);

  /// \brief Sample data from the uniform distribution.
  static double Uniform();

  /// \brief Sample data from the gaussian distribution.
  static double Gaussian(double sigma);
};

/// \class PGOToyExample
/// \brief toy example of pose graph optimization (PGO) class.
class PGOToyExample {
 public:
  /// The constructor.
  /// \param[in] verbose Optimization verbose setting.
  PGOToyExample(bool verbose=true);

  /// \brief Set original pose data.
  void SetOriginalPoses();

  /// \brief Make current pose and add those data into the optimizer.
  void MakeCurrentPoseAndAddVertex();

  /// \brief Add edges into the optimizer.
  void AddEdge();

  /// \brief Reset all optimization process.
  void Reset();

  /// \brief Get optimizer instance.
  g2o::SparseOptimizer* GetOptimizer() { return optimizer_; }

  /// \brief Get original poses.
  std::vector<g2o::SE3Quat, Eigen::aligned_allocator<g2o::SE3Quat>> GetOriginalPoses() { return original_poses_; }

  /// \brief Get the size of poses.
  int GetOriginalPosesSize() { return original_poses_.size(); }

 private:
  /// \brief g2o optimizer instance.
  g2o::SparseOptimizer* optimizer_;

  /// \brief Original poses vector.
  std::vector<g2o::SE3Quat, Eigen::aligned_allocator<g2o::SE3Quat>> original_poses_;

  /// \brief Id of each vertex.
  int vertex_id_;

  /// \brief Boolean of optimization verbose setting.
  bool verbose_;

  /// \brief Number of poses.
  int num_poses_;
};

