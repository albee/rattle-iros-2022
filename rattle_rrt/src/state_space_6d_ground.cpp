/* 
# state_space_6d_ground

A 6D state space for x-y-th kinodynamic planning.

state vector: [rx ry th vx vy w]

Keenan Albee, 2021
MIT Space Systems Lab
*/

#include <rattle_rrt/state_space_6d_ground.h>


namespace rrt {

/* ************************************************************************** */
StateSpace6DGround::StateSpace6DGround(Eigen::Matrix<double, 6, 1> min_bounds, Eigen::Matrix<double, 6, 1> max_bounds, double mass, double ixx, double iyy, double izz) {
    /* Create a 6D state space, with min_bounds and max_bounds.
    */
    min_bounds_ = min_bounds;  // [rx ry th vx vy w]
    max_bounds_ = max_bounds;  // [rx ry th vx vy w]
    dimension_ = 6;                 // dimension (+1 for quaternion)
    model_ = RigidBodyDynamics{mass, ixx, iyy, izz};
    model_.SetExternalForce(0, 0, 0);  // microgravity (or static on granite table)
}


/* ************************************************************************** */
STATE_6D StateSpace6DGround::find_x_rand(unsigned int* seed) const {
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
std::tuple<STATE_6D, Vec3, Vec3, int> StateSpace6DGround::find_x_new_data(const STATE_6D& source, const STATE_6D& target, double timestep) {
  /* Finds a state in the direction of target from source. Currently, this method uses motion primitive evaluation!
  */

  // Propagate for a single timestep
  double DT = .05;  // delta-t for dynamics forward propagation, TODO: can replace with CasADi
  int num_timesteps = static_cast<int>(timestep/DT);

  // possible moves, note that these are 3xN matrices!
  F_ARRAY F;
  T_ARRAY T;
  F.setZero();
  T.setZero();
  F << 0.15, -0.15,  0,  0, 0,
        0,  0,  0.15, -0.15,   0;  // N
  T << 0,  0,  0,  0, 0;  // N-m

  X_NEXT_ARRAY results;  // this is a 3D array (i, j) where i:=move_idx, j:=state_idx (defaults to just 0)

  int j = 0;
  double dummy_var;

  for (int i = 0; i < num_F; i++) {  // simulate different inputs to the system
    // Set initial conditions based on source
    model_.SetPosition(source(0), source(1), 0.0);  // p, COM
    model_.SetVelocity(source(3), source(4), 0.0);  // v, COM
    double R[9] = {cos(source(2)), -sin(source(2)), 0.0,
                    sin(source(2)), -cos(source(2)), 0.0,
                    0.0, 0.0, 0.0
                  };
    model_.SetRotation(R);                    //  q, this must have 9 elements in rotation matrix format
    model_.SetAngularVelocity(0.0, 0.0, source(5));  // w, around COM

    for (int tidx = 0; tidx < num_timesteps; tidx++) {  // simulate num_timesteps forward
      // Apply force at the COM
      model_.SetExternalForce(F(0, i), F(1, i), 0.0);  // x, y, z

      // Apply torque about the COM
      model_.SetExternalTorque(0.0, 0.0, T(0, i));  // about x, y, z

      // Forward propagate, and see where system ends up
      model_.EulerStep(DT);

      // Store the result
      model_.GetPosition(&results(i, j)(0), &results(i, j)(1), &dummy_var);
      model_.GetVelocity(&results(i, j)(3), &results(i, j)(4), &dummy_var);
      double R_result[9];
      model_.GetRotation(R_result);
      results(i, j)(2) = atan(-R_result[1]/R_result[0]);  // atan(-(-sin/cos)), always radians [-pi/2, pi/2]
      model_.GetAngularVelocity(&dummy_var, &dummy_var, &results(i, j)(5));
    }
  }

  // Return the input with the best result
  int extend_success = 1;
  STATE_6D x_new;
  x_new.setZero();
  Vec3 prev_F;
  Vec3 prev_T;
  prev_F.setZero();
  prev_T.setZero();

  get_min_result(x_new, prev_F, prev_T, extend_success, results, target, F, T);
  return std::make_tuple(x_new, prev_F, prev_T, extend_success);  // failed to branch out
}


/* ************************************************************************** */
std::tuple<STATE_6D, Vec3, Vec3, int> StateSpace6DGround::find_x_new_data(
  /* Allows variable time stepping, TO DO
  */
  const STATE_6D& source, const STATE_6D& target,
  double timestep, double max_timestep) {
  return(find_x_new_data(source, target, timestep));
}


/* ************************************************************************** */
STATE_6D StateSpace6DGround::find_x_new(const STATE_6D& source, const STATE_6D& target, double timestep) {
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
STATE_6D StateSpace6DGround::find_x_new(const STATE_6D& source,
                  const STATE_6D& target,
                  double stepSize, double maxStepSize) {
    /* Adaptive step size version, which scales the possible inputs based on goal proximity
    */
  STATE_6D x_new;
  x_new = find_x_new(source, target, stepSize);
  return x_new;
}


/* ************************************************************************** */
void StateSpace6DGround::get_min_result(STATE_6D& x_new, Vec3& prev_F, Vec3& prev_T, int& extend_success,
  X_NEXT_ARRAY& results, const STATE_6D& target, F_ARRAY& F, T_ARRAY& T) {
  /* Take the result with min(distance)
  */
  STATE_6D cur_state;  // should probably initialize this
  double cur_dist = DBL_MAX;
  double lowest_dist = DBL_MAX;  // lowest distance
  Eigen::Vector2i lowest_idx(-1, -1);

  int j = 0;
  for (int i = 0; i < num_F; i++) {
    cur_state = results(i, j);
    // Skip if not valid
    if (!is_state_valid(cur_state)) {
      continue;
    }
    cur_dist = get_dist(cur_state, target);  // distance from x_rand state to x_new state
    if (cur_dist < lowest_dist) {
      lowest_dist = cur_dist;
      lowest_idx << i, j;
    }
  }

  if (lowest_idx.x() == -1) {
    extend_success = 0;
    return;  // failed to branch out
  }

  x_new = results(lowest_idx.x(), lowest_idx.y());
  prev_F = F.col(lowest_idx.x()).transpose();

  prev_T = T.col(lowest_idx.x()).transpose();

  return;
}


/* ************************************************************************** */
double StateSpace6DGround::get_dist(const STATE_6D& source, const STATE_6D& target) const {
  /* Calculate the distance between states by combining their metrics. This
  is essentially the cost-to-go.
  */
  double dist, wp, wq, wv, ww, p_dist, v_dist, q_dist, w_dist;  // distance, and weights on metrics
  const STATE_6D& diff = target - source;

  wp = 20.0; wv = 1.0; wq = 0.0; ww = 0.0;  //no angular weighting!

  p_dist = get_R3_dist(diff(0), diff(1));  // position
  v_dist = get_R3_dist(diff(3), diff(4));  // linear velocity
  q_dist = get_theta_dist(source(2), target(2));     // angle
  w_dist = get_R3_dist(diff(5));  // angular velocity

  dist = wp*p_dist + wv*v_dist + wq*q_dist + ww*w_dist;
  return dist;
}


/* ************************************************************************** */
double StateSpace6DGround::get_R3_dist(double x, double y, double z, double w) const {
  /* Calculate Euclidean distance (up to 4 dims)
  */
  double dist;
  dist = sqrt(powf(x, 2) + powf(y, 2) + powf(z, 2) + powf(w, 2));
  return dist;
}


/* ************************************************************************** */
double StateSpace6DGround::get_theta_dist(double th1, double th2) const {
  /* Calculate angular distance. Wraps around -PI and PI
  */
  double PI = 3.1415926535;
  double dist = th1 - th2;
  if (dist > PI) {
    dist -= 2*PI;
  }
  else if (dist < PI) {
    dist += 2*PI;
  }

  return abs(dist);
}


/* ************************************************************************** */
bool StateSpace6DGround::is_state_valid(const STATE_6D& pt,  const std::vector<Obstacle>& obstacles) {
  /* Returns a boolean indicating whether the given point is within bounds.
  */
  // Bounds checking
  if (pt(0) < min_bounds_(0) || pt(0) > max_bounds_(0) ||    // x y th
      pt(1) < min_bounds_(1) || pt(1) > max_bounds_(1) ||
      pt(2) < min_bounds_(2) || pt(2) > max_bounds_(2) ||
      pt(3) < min_bounds_(3) || pt(3) > max_bounds_(3) ||    // xd yd thd
      pt(4) < min_bounds_(4) || pt(4) > max_bounds_(4) ||
      pt(5) < min_bounds_(5) || pt(5) > max_bounds_(5)) {
    return false;
  }

  return true;  // valid!
}


/* ************************************************************************** */
bool StateSpace6DGround::is_segment_valid(const STATE_6D& from, const STATE_6D& to, const std::vector<Obstacle>& obstacles) {
  /* Check motion validity from one state to another: currently, only checks obstacle collision
  */
  bool valid = true;
  Eigen::Vector2d pos = to.head(2);

  for(uint i = 0; i < obstacles.size(); i++) {
    if (obstacles[i].is_in_collision(pos)) {
      valid = false;
    }
  }
  return valid;
}


/* ************************************************************************** */
int StateSpace6DGround::get_d() const {
  /* Return the state space dimensionality
  */
  return dimension_;
}

}; // namespace rrt
