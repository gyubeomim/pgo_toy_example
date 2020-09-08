#include "pgo_toy_example.h"

int Sampling::Uniform(int from, int to) {
  return static_cast<int>(UniformRand(from, to));
}
double Sampling::Uniform() {
  return UniformRand(0., 1.);
}
double Sampling::Gaussian(double sigma) {
  return GaussRand(0., sigma);
}

PGOToyExample::PGOToyExample(bool verbose)
    : vertex_id_(0), verbose_(verbose)
{
  num_poses_ = 15; // Set number of poses.

  // Set g2o solver.
  std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linear_solver;
  linear_solver = g2o::make_unique<g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>>();

  // use LM method to minimize nonlinear least square.
  g2o::OptimizationAlgorithmLevenberg* solver =
      new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<g2o::BlockSolver_6_3>(std::move(linear_solver)));
  solver->setUserLambdaInit(1);

  optimizer_ = new g2o::SparseOptimizer();
  optimizer_->setAlgorithm(solver);
}

void PGOToyExample::SetOriginalPoses() {
  // Set the original poses.
  for(int i=0; i<num_poses_; i++){
    Vec3 trans;
    if(i==0) {
      trans = Vec3(0,0,0);
    }
    else if(i>=1 && i <=7) {
      trans = Vec3(i + Sampling::Gaussian(.2),
                   Sampling::Gaussian(.2),
                   Sampling::Gaussian(.2));
    }
    else if(i==8) {
      trans = Vec3(8 + Sampling::Gaussian(.2),
                   7 - i + Sampling::Gaussian(.2),
                   Sampling::Gaussian(.2));
    }
    else if(i>=9) {
      trans = Vec3(17 - i + Sampling::Gaussian(.2),
                   -1 + Sampling::Gaussian(.2),
                   Sampling::Gaussian(.2));
    }

    Quaternion q;
    q.setIdentity(); // Set the initial rotation to identity.

    g2o::SE3Quat pose;
    pose.setRotation(q);
    pose.setTranslation(trans);

    original_poses_.push_back(pose);
  }
}

void PGOToyExample::MakeCurrentPoseAndAddVertex() {
  // Add noise in original poses and add vertex in g2o.
  for(int i=0;i<original_poses_.size();i++){
    g2o::VertexSE3Expmap* vtx = new g2o::VertexSE3Expmap();
    vtx->setId(vertex_id_);

    if(i==0){
      vtx->setFixed(true);
    }

    Vec3 trans(Sampling::Gaussian(.2),
               Sampling::Gaussian(.2),
               Sampling::Gaussian(.2));
    Quaternion q;
    q.UnitRandom();

    g2o::SE3Quat origin = original_poses_.at(i);
    g2o::SE3Quat noise;
    noise.setTranslation(origin.translation() + trans); // Add noise to original poses.

    // Add noise pose into the g2o optimizer.
    vtx->setEstimate(noise);
    optimizer_->addVertex(vtx);
    vertex_id_ += 1;
  }
}

void PGOToyExample::AddEdge() {
  // Add temporal edges.
  for(int i=1;i<original_poses_.size();i++){
    g2o::EdgeSE3Expmap* e(new g2o::EdgeSE3Expmap());
    g2o::SE3Quat relative_pose = original_poses_.at(i-1).inverse() * original_poses_.at(i);

    e->setMeasurement(relative_pose);

#if 0
    // if not using the information matrix.
    MatXX information = Eigen::MatrixXd::Identity(6,6);
#else
    // Use the random information matrix. Information matrix should be symmetric.
    MatXX A = Eigen::MatrixXd::Random(6,6).cwiseAbs();
    MatXX information = A.transpose() * A;
#endif

    e->setInformation(information);
    e->vertices()[0] = optimizer_->vertex(i-1);
    e->vertices()[1] = optimizer_->vertex(i);

    // Add the edge into the g2o optimizer.
    optimizer_->addEdge(e);
  }

  // Add non-temporal edges. (5 & 11)
  g2o::EdgeSE3Expmap* e511(new g2o::EdgeSE3Expmap());
  g2o::SE3Quat relative_pose = original_poses_.at(5).inverse() * original_poses_.at(11);
  e511->setMeasurement(relative_pose);
  MatXX A = Eigen::MatrixXd::Random(6,6).cwiseAbs();
  MatXX information = A.transpose() * A;
  e511->setInformation(information);
  e511->vertices()[0] = optimizer_->vertex(5);
  e511->vertices()[1] = optimizer_->vertex(11);
  optimizer_->addEdge(e511);

  // Add non-temporal edges. (3 & 14)
  g2o::EdgeSE3Expmap* e314(new g2o::EdgeSE3Expmap());
  relative_pose = original_poses_.at(3).inverse() * original_poses_.at(14);
  e314->setMeasurement(relative_pose);
  A = Eigen::MatrixXd::Random(6,6).cwiseAbs();
  information = A.transpose() * A;
  e314->setInformation(information);
  e314->vertices()[0] = optimizer_->vertex(3);
  e314->vertices()[1] = optimizer_->vertex(14);
  optimizer_->addEdge(e314);
}

void PGOToyExample::Reset() {
  // reset all vertices and edges.
  optimizer_->clear();
  original_poses_.clear();
  vertex_id_ = 0;

  // set original vertices.
  SetOriginalPoses();

  // make current vertices and add vertices to optimizer.
  MakeCurrentPoseAndAddVertex();

  // add edges to optimizer.
  AddEdge();

  // optimization intialization
  optimizer_->initializeOptimization();
  optimizer_->setVerbose(verbose_);
}
