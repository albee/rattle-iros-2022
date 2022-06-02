/* 
# state_space_4d

A 4D state space for x-y double integrator kinodynamic planning.

state vector: [rx ry vx vy]

Keenan Albee, 2021
MIT Space Systems Lab
*/

#include <rattle_rrt/state_space_4d.h>


namespace rrt {

/* ************************************************************************** */
StateSpace4D::StateSpace4D(Eigen::Matrix<double, 4, 1> min_bounds, Eigen::Matrix<double, 4, 1> max_bounds, double mass, double ixx, double iyy, double izz) {
    /* Create a 4D state space, with min_bounds and max_bounds.
    */
    min_bounds_ = min_bounds;  // [rx ry vx vy]
    max_bounds_ = max_bounds;  // [rx ry vx vy]
    dimension_ = 4;                 // dimension (+1 for quaternion)
    model_ = RigidBodyDynamics{mass, ixx, iyy, izz};
    model_.SetExternalForce(0, 0, 0);  // microgravity (or static on granite table)
}


void StateSpace4D::initialize_bullet(std::vector<Obstacle> obs_vec) {
  /* Initialize the world used by bullet for collision checks
  */
  for (Obstacle obs : obs_vec){
    // make z huge
    bullet_.AddObstacle(rrt::Vec3{obs.a, obs.b, obs.c}, rrt::Vec3{obs.xc, obs.yc, obs.zc}, Eigen::Quaternion<decimal_t>{0.0, 0.0, 0.0, 1.0});
    std::cout << "Obstacle: " << obs << std::endl;
  }
}


/* ************************************************************************** */
STATE_4D StateSpace4D::find_x_rand(unsigned int* seed) const {
  /* Generate a random 4D state
  */
  STATE_4D random;

  // generate random Euclidean states for within provided bounds
  for (int i = 0; i < 4; ++i) {
      random(i) = (rand_r(seed)/(static_cast<double>(RAND_MAX))*(max_bounds_(i) - min_bounds_(i)))
                  + min_bounds_(i);
  }
  return random;
}


/* ************************************************************************** */
std::tuple<STATE_4D, Vec3, Vec3, int> StateSpace4D::find_x_new_data(const STATE_4D& source, const STATE_4D& target, double timestep) {
  /* Finds a state in the direction of target from source. Currently, this method uses motion primitive evaluation!

  @return: Returns the best inputs to obtain state vector that most nearly reaches target.
  */

  // Propagate for a single timestep
  double DT = .05;  // delta-t for dynamics forward propagation, TODO: can replace with CasADi
  int num_timesteps = static_cast<int>(timestep/DT);

  // possible moves, note that these are 2xN matrices!
  F_ARRAY F;
  F.setZero();
  F << 0.1, -0.1,  0,  0, 0,
       0,  0,  0.1, -0.1,   0;  // N

  X_NEXT_ARRAY results;  // this is a 3D array (i, 1)(state) where i:=F_idx, state:=state_idx

  double dummy_var;

  for (int i = 0; i < num_F; i++) {  // simulate different inputs to the system
    // Set initial conditions based on source
    model_.SetPosition(source(0), source(1), 0.0);  // r, COM
    model_.SetVelocity(source(2), source(3), 0.0);  // v, COM
    double R[9] = {1.0, 0.0, 0.0,
                   0.0, 1.0, 0.0,
                   0.0, 0.0, 1.0
                  };
    model_.SetRotation(R);                    //  q, this must have 9 elements in rotation matrix format
    model_.SetAngularVelocity(0.0, 0.0, 0.0);  // w, around COM

    for (int tidx = 0; tidx < num_timesteps; tidx++) {  // simulate num_timesteps forward
      // Apply force at the COM
      model_.SetExternalForce(F(0, i), F(1, i), 0.0);  // x, y, z

      // Apply torque about the COM
      model_.SetExternalTorque(0.0, 0.0, 0.0);  // about x, y, z

      // Forward propagate, and see where system ends up
      model_.EulerStep(DT);

      // Store the result
      model_.GetPosition(&results(i, 1)(0), &results(i, 1)(1), &dummy_var);
      model_.GetVelocity(&results(i, 1)(2), &results(i, 1)(3), &dummy_var);
    }
  }

  // Return the input with the best result
  int extend_success = 1;
  STATE_4D x_new;
  x_new.setZero();
  Vec2 prev_F;
  prev_F.setZero();

  get_min_result(x_new, prev_F, extend_success, results, target, F);
  Vec3 input_F;
  input_F << prev_F, 0.0;

  rrt::Vec3 dummy_T;
  dummy_T.setZero();
  return std::make_tuple(x_new, input_F, dummy_T, extend_success);  // the new point, and inputs used to get there
}


/* ************************************************************************** */
std::tuple<STATE_4D, Vec3, Vec3, int> StateSpace4D::find_x_new_data(
  /* Allows variable time stepping,
  TODO: not implemented!
  */
  const STATE_4D& source, const STATE_4D& target,
  double timestep, double max_timestep) {
  return(find_x_new_data(source, target, timestep));
}


/* ************************************************************************** */
STATE_4D StateSpace4D::find_x_new(const STATE_4D& source, const STATE_4D& target, double timestep) {
  /* Only grabs x_new
  TODO: currently just a passthrough
  */
  STATE_4D x_new;
  Vec3 prev_F;
  Vec3 prev_T;
  prev_F.setZero();
  prev_T.setZero();
  int extend_success;
  std::tie(x_new, prev_F, prev_T, extend_success) = find_x_new_data(source, target, timestep);
  return x_new;
}


/* ************************************************************************** */
STATE_4D StateSpace4D::find_x_new(const STATE_4D& source,
                  const STATE_4D& target,
                  double stepSize, double maxStepSize) {
  /* Adaptive step size version, which scales the possible inputs based on goal proximity
  */
  STATE_4D x_new;
  x_new = find_x_new(source, target, stepSize);
  return x_new;
}


/* ************************************************************************** */
void StateSpace4D::get_min_result(STATE_4D& x_new, Vec2& prev_F, int& extend_success,
  X_NEXT_ARRAY& results, const STATE_4D& target, F_ARRAY& F) {
  /* Take the result with min(distance)
  @param:
  results: is an (i, 1) array of states of size Vec4
  F: [2 x num_F]

  @return:
  results: updated with values
  prev_F: update with best force input
  */
  STATE_4D cur_state;  // should probably initialize this
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
double StateSpace4D::get_dist(const STATE_4D& source, const STATE_4D& target) const {
  /* Calculate the distance between states by combining their metrics. This
  is essentially the cost-to-go and is one of the most important functions in
  determining RRT performance.
  */
  double dist, wp, wv, p_dist, v_dist;  // distance, and weights on metrics
  const STATE_4D& diff = target - source;

  wp = 30.0; wv = 1.0;

  p_dist = get_R3_dist(diff(0), diff(1));  // position
  v_dist = get_R3_dist(diff(2), diff(3));  // linear velocity

  dist = wp*p_dist + wv*v_dist;
  return dist;
}


/* ************************************************************************** */
double StateSpace4D::get_R3_dist(double x, double y, double z, double w) const {
  /* Calculate Euclidean distance (up to 4 dims, default args are 0)
  */
  double dist;
  dist = sqrt(powf(x, 2) + powf(y, 2) + powf(z, 2) + powf(w, 2));
  return dist;
}


/* ************************************************************************** */
bool StateSpace4D::is_state_valid(const STATE_4D& pt, const std::vector<Obstacle>& obstacles) {
  /* Returns a boolean indicating whether the given point is within bounds.
  */
  // Bounds checking
  if (pt(0) < min_bounds_(0) || pt(0) > max_bounds_(0) ||     // x y
      pt(1) < min_bounds_(1) || pt(1) > max_bounds_(1) ||
      pt(2) < min_bounds_(2) || pt(2) > max_bounds_(2) ||
      pt(3) < min_bounds_(3) || pt(3) > max_bounds_(3)) {    // xd yd 
    return false;
  }

  // Obstacle checking (3D) ellipsoid-on-ellipsoid
  if (!bullet_.IsFreeState(rrt::Vec3{pt(0), pt(1), 0.0}, Eigen::Quaternion<decimal_t>{0.0, 0.0, 0.0, 1.0}) ) {
    // std::cout << "collision!: " << pt(0) << " " << pt(1) << " " << pt(2) << std::endl;
    return false;
  }

  return true;  // valid!
}


/* ************************************************************************** */
bool StateSpace4D::is_segment_valid(const STATE_4D& from, const STATE_4D& to, const std::vector<Obstacle>& obstacles) {
  /* Check motion validity from one state to another: currently, only checks obstacle collision
  TODO: do more accurate collision detection!
  */
  bool valid = true;
  Eigen::Vector2d pos = to.head(2);
  valid = is_state_valid(to, obstacles);

  // for(uint i = 0; i < obstacles.size(); i++) {
  //   if (obstacles[i].is_in_collision(pos)) {
  //     valid = false;
  //   }
  // }
  return valid;
}


/* ************************************************************************** */
int StateSpace4D::get_d() const {
  /* Return the state space dimensionality
  */
  return dimension_;
}
}; // namespace rrt
