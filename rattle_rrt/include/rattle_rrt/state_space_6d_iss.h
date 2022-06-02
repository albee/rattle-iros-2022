/* 
# state_space_6d

A 6D state space for x-y-z double integrator kinodynamic planning.

Keenan Albee, 2021
MIT Space Systems Lab
*/

#pragma once
#include <eigen3/Eigen/Dense>
#include <limits>
#include <cfloat>

#include <rattle_rrt/state_space.tpp>
#include <rattle_rrt/types.h>
#include <rbd/rigidBodyDynamics.h>
#include <bullet_collision_checker/bullet_collision_checker.h>


namespace rrt {
typedef Eigen::Matrix<double, 6, 1> STATE_6D;

class StateSpace6DISS: public StateSpace<STATE_6D> {
  /* The state space for a granite table free flyer, extends the abstract StateSpace.

  state vector: [rx ry th vx vy w]
  */
public:
  Eigen::Matrix<double, 6, 1> min_bounds_;  // min_bounds = min_x, min_y, min_th, min_xd, min_yd, min_thd
  Eigen::Matrix<double, 6, 1> max_bounds_;  // max_bounds = max_x, max_y, max_th, max_xd, max_yd, max_thd
  int dimension_;  // state space dimension
  static const int use_combos_ = 0;  // use combinations of the F T inputs?

  static const int num_F = 7;  // num_F : number of possible force inputs (motion primitives) to sample from

  RigidBodyDynamics model_;  // the dynamics model to evaluate
  collision_checker::BulletCollisionChecker bullet_ = collision_checker::BulletCollisionChecker();  // collision-checking module

  typedef Eigen::Array<STATE_6D, num_F, 1> X_NEXT_ARRAY;
  typedef Eigen::Matrix<double, 3, num_F> F_ARRAY;

  StateSpace6DISS(Eigen::Matrix<double, 6, 1> min_bounds, 
                  Eigen::Matrix<double, 6, 1> max_bounds, double mass, double ixx, double iyy, double izz);

  void initialize_bullet(std::vector<Obstacle> obs_vec);

  // template funcs
  STATE_6D find_x_rand(unsigned int* seed) const override;
  std::tuple<STATE_6D, Vec3, Vec3, int> find_x_new_data(const STATE_6D& source, const STATE_6D& target, double TIMESTEP) override;

  std::tuple<STATE_6D, Vec3, Vec3, int> find_x_new_data(const STATE_6D& source, const STATE_6D& target,
                                                        double TIMESTEP, double MAX_TIMESTEP) override;

  STATE_6D find_x_new(const STATE_6D& source, const STATE_6D& target, double TIMESTEP) override;
  STATE_6D find_x_new(const STATE_6D& source, const STATE_6D& target, double stepSize, double maxStepSize) override;

  double get_dist(const STATE_6D& source, const STATE_6D& target) const override;
  bool is_state_valid(const STATE_6D& pt, const std::vector<Obstacle>& obstacles = std::vector<Obstacle>()) override;
  bool is_segment_valid(const STATE_6D& from, const STATE_6D& to, const std::vector<Obstacle>& obstacles = std::vector<Obstacle>()) override;

  int get_d() const override;

  // derived class funcs
  void get_min_result(STATE_6D& x_new, Vec3& prev_F, int& extend_success,
                      X_NEXT_ARRAY& results, const STATE_6D& target, F_ARRAY& F);

  double get_R3_dist(double x, double y = 0, double z = 0, double w = 0) const;
};
}; // namespace rrt

// #endif  // STATE_6D_SPACE_6D_H_
