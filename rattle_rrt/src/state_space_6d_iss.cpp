/* 
# state_space_6d_iss

A 6D state space for x-y-z double integrator kinodynamic planning.

state vector: [rx ry ry vx vy vz]

Keenan Albee, 2021
MIT Space Systems Lab
*/

#include <rattle_rrt/state_space_6d_iss.h>


namespace rrt {

/* ************************************************************************** */
StateSpace6DISS::StateSpace6DISS(Eigen::Matrix<double, 6, 1> min_bounds, 
                                 Eigen::Matrix<double, 6, 1> max_bounds, double mass, double ixx, double iyy, double izz) {
    /* Create a 6D state space, with min_bounds and max_bounds.
    */
    min_bounds_ = min_bounds;  // [rx ry th vx vy w]
    max_bounds_ = max_bounds;  // [rx ry th vx vy w]
    dimension_ = 6;                 // dimension (+1 for quaternion)
    model_ = RigidBodyDynamics{mass, ixx, iyy, izz};
    model_.SetExternalForce(0, 0, 0);  // microgravity (or static on granite table)
}


void StateSpace6DISS::initialize_bullet(std::vector<Obstacle> obs_vec) {
  /* Initialize the world used by bullet for collision checks
  */
  for (Obstacle obs : obs_vec){
    bullet_.AddObstacle(rrt::Vec3{obs.a, obs.b, obs.c}, rrt::Vec3{obs.xc, obs.yc, obs.zc}, Eigen::Quaternion<decimal_t>{0.0, 0.0, 0.0, 1.0});
    std::cout << "Obstacle: " << obs << std::endl;
  }
}


/* ************************************************************************** */
STATE_6D StateSpace6DISS::find_x_rand(unsigned int* seed) const {
  /* Generate a random 6D state
  */
  STATE_6D random;

  // generate random Euclidean states for within provided bounds
  for (int i = 0; i < 6; ++i) {
      random(i) = (rand_r(seed)/(static_cast<double>(RAND_MAX))*(max_bounds_(i) - min_bounds_(i)))
                  + min_bounds_(i);
  }
  return random;
}


/* ************************************************************************** */
std::tuple<STATE_6D, Vec3, Vec3, int> StateSpace6DISS::find_x_new_data(const STATE_6D& source, const STATE_6D& target, double timestep) {
  /* Finds a state in the direction of target from source. Additionally, return the inputs used to reach x_new.

  @param:
  F: a [3 x num_F] array of force inputs to try.
  @return: Returns the best inputs to obtain state vector that most nearly reaches target.
  {reached_state, force vector, torque vector=0}
  */

  // Propagate for a single timestep
  double DT = .05;  // delta-t for dynamics forward propagation, TODO: can replace with CasADi
  int num_timesteps = static_cast<int>(timestep/DT);

  // possible moves, note that these are 3xN matrices!
  F_ARRAY F;

  F << 0.1, -0.1,  0,  0,      0, 0,          0,
       0,  0,        0.1, -0.1, 0, 0,         0,
       0,  0,        0,  0,        0.1, -0.1, 0;  // N

  X_NEXT_ARRAY results;  // this is a 3D array (i, 1)(state) where i:=F_idx, state:=state_idx

  for (int i = 0; i < num_F; i++) {  // simulate different inputs to the system
    // Set initial conditions based on source
    model_.SetPosition(source(0), source(1), source(2));  // r, COM
    model_.SetVelocity(source(3), source(4), source(5));  // v, COM
    double R[9] = {1.0, 0.0, 0.0,
                   0.0, 1.0, 0.0,
                   0.0, 0.0, 1.0
                  };
    model_.SetRotation(R);                    //  q, this must have 9 elements in rotation matrix format
    model_.SetAngularVelocity(0.0, 0.0, 0.0);  // w, around COM

    for (int tidx = 0; tidx < num_timesteps; tidx++) {  // simulate num_timesteps forward
      // Apply force at the COM
      model_.SetExternalForce(F(0, i), F(1, i), F(2, i));  // x, y, z

      // Apply torque about the COM
      model_.SetExternalTorque(0.0, 0.0, 0.0);  // about x, y, z

      // Forward propagate, and see where system ends up
      model_.EulerStep(DT);

      // Store the result
      model_.GetPosition(&results(i, 1)(0), &results(i, 1)(1), &results(i, 1)(2));
      model_.GetVelocity(&results(i, 1)(3), &results(i, 1)(4), &results(i, 1)(5));
    }
  }

  // Return the input with the best result
  int extend_success = 1;
  STATE_6D x_new;
  x_new.setZero();
  Vec3 prev_F;
  prev_F.setZero();

  get_min_result(x_new, prev_F, extend_success, results, target, F);
  Vec3 input_F;
  input_F << prev_F;

  rrt::Vec3 dummy_T;
  dummy_T.setZero();
  return std::make_tuple(x_new, input_F, dummy_T, extend_success);  // the new point, and inputs used to get there
}


/* ************************************************************************** */
std::tuple<STATE_6D, Vec3, Vec3, int> StateSpace6DISS::find_x_new_data(
  /* Allows variable time stepping, TO DO
  */
  const STATE_6D& source, const STATE_6D& target,
  double timestep, double max_timestep) {
  return(find_x_new_data(source, target, timestep));
}


/* ************************************************************************** */
STATE_6D StateSpace6DISS::find_x_new(const STATE_6D& source, const STATE_6D& target, double timestep) {
  /* Only grabs x_new
  */
  STATE_6D x_new;
  Vec3 prev_F;
  Vec3 prev_T;
  prev_F.setZero();
  prev_T.setZero();
  int extend_success;
  std::tie(x_new, prev_F, prev_T, extend_success) = find_x_new_data(source, target, timestep);
  return x_new;
}


/* ************************************************************************** */
STATE_6D StateSpace6DISS::find_x_new(const STATE_6D& source,
                  const STATE_6D& target,
                  double stepSize, double maxStepSize) {
    /* Adaptive step size version, which scales the possible inputs based on goal proximity
    */
  STATE_6D x_new;
  x_new = find_x_new(source, target, stepSize);
  return x_new;
}


/* ************************************************************************** */
void StateSpace6DISS::get_min_result(STATE_6D& x_new, Vec3& prev_F, int& extend_success,
  X_NEXT_ARRAY& results, const STATE_6D& target, F_ARRAY& F) {
  /* Take the result with min(distance)
  @param:
  results: is an (i, 1) array of states of size Vec6
  F: [3 x num_F]

  @return:
  results: updated with values
  prev_F: update with best force input
  */
  STATE_6D cur_state;  // should probably initialize this
  double cur_dist = DBL_MAX;
  double lowest_dist = DBL_MAX;  // lowest distance
  int lowest_idx = -1;

  for (int i = 0; i < num_F; i++) {
    cur_state = results(i, 0);
    // Skip if not valid
    if (!is_state_valid(cur_state)) {
      continue;
    }
    cur_dist = get_dist(cur_state, target);  // distance from x_rand state to x_new state
    if (cur_dist < lowest_dist) {
      lowest_dist = cur_dist;
      lowest_idx = i;
    }
  }

  if (lowest_idx == -1) {
    extend_success = 0;
    return;  // failed to branch out
  }

  x_new = results(lowest_idx, 0);
  prev_F = F.col(lowest_idx).transpose();

  return;
}


/* ************************************************************************** */
double StateSpace6DISS::get_dist(const STATE_6D& source, const STATE_6D& target) const {
  /* Calculate the distance between states by combining their metrics. This
  is essentially the cost-to-go and is one of the most important functions in
  determining RRT performance.
  */
  double dist, wp, wv, p_dist, v_dist;  // distance, and weights on metrics
  const STATE_6D& diff = target - source;

  wp = 1.0; wv = 0.01;

  p_dist = get_R3_dist(diff(0), diff(1), diff(2));  // position
  v_dist = get_R3_dist(diff(3), diff(4), diff(5));  // linear velocity

  dist = wp*p_dist + wv*v_dist;
  return dist;
}


/* ************************************************************************** */
double StateSpace6DISS::get_R3_dist(double x, double y, double z, double w) const {
  /* Calculate Euclidean distance (up to 4 dims)
  */
  double dist;
  dist = sqrt(powf(x, 2) + powf(y, 2) + powf(z, 2) + powf(w, 2));
  return dist;
}


/* ************************************************************************** */
bool StateSpace6DISS::is_state_valid(const STATE_6D& pt, const std::vector<Obstacle>& obstacles) {
  /* Returns a boolean indicating whether the given point is within bounds.
  Checks: 
  [ ] in bounds
  [ ] satisfies obstalces constraints (ellipsoid-ellipsoid), using bullet
  */
  // Bounds checking
  if (pt(0) < min_bounds_(0) || pt(0) > max_bounds_(0) ||    // x y z
      pt(1) < min_bounds_(1) || pt(1) > max_bounds_(1) ||
      pt(2) < min_bounds_(2) || pt(2) > max_bounds_(2) ||
      pt(3) < min_bounds_(3) || pt(3) > max_bounds_(3) ||    // xd yd zd
      pt(4) < min_bounds_(4) || pt(4) > max_bounds_(4) ||
      pt(5) < min_bounds_(5) || pt(5) > max_bounds_(5)) {
    return false;
  }

  // Obstacle checking (3D) ellipsoid-on-ellipsoid
  if (!bullet_.IsFreeState(rrt::Vec3{pt(0), pt(1), pt(2)}, Eigen::Quaternion<decimal_t>{0.0, 0.0, 0.0, 1.0}) ) {
    // std::cout << "collision!: " << pt(0) << " " << pt(1) << " " << pt(2) << std::endl;
    return false;
  }

  return true;  // valid!
}


/* ************************************************************************** */
int StateSpace6DISS::get_d() const {
  /* Return the state space dimensionality
  */
  return dimension_;
}


/* ************************************************************************** */
bool StateSpace6DISS::is_segment_valid(const STATE_6D& from, const STATE_6D& to, const std::vector<Obstacle>& obstacles) {
  /* Check motion validity from one state to another.
  TODO: this needs to do checking in between states!!
  */
  bool valid = true;

  // assume from is already checked...
  // valid = is_state_valid(from, obstacles);
  valid = is_state_valid(to, obstacles);

  return valid;
}

}; // namespace rrt
