/* 
# state_space_13d

A 13D state space for Newton-Euler kinodynamic planning.

state vector: [rx ry rz vx vy vz qx qy qz qw wx wy wz]

Keenan Albee, 2021
MIT Space Systems Lab
*/

#include <rattle_rrt/state_space_13d.h>


namespace rrt {

/* ************************************************************************** */
StateSpace13D::StateSpace13D(Eigen::Matrix<double, 9, 1> min_bounds, Eigen::Matrix<double, 9, 1> max_bounds, double mass,
  double ixx, double iyy, double izz) {
  /*
  * Create a 13D state space, with min_bounds and max_bounds.
  */
  min_bounds_ = min_bounds;
  max_bounds_ = max_bounds;
  model_ = RigidBodyDynamics{mass, ixx, iyy, izz};
  dimension_ = 13;                 // dimension (+1 for quaternion)
  model_.SetExternalForce(0, 0, 0);  // microgravity (or static on granite table)
}


/* ************************************************************************** */
STATE_13D StateSpace13D::find_x_rand(unsigned int* seed) const {
  /*
  * Generate a random 13D state
  */
  int VEC_SIZE = 13;
  STATE_13D random;

  // generate random Euclidean states for pos, vel
  for (int i = 0; i < 6; ++i) {
      random(i) = (rand_r(seed)/(static_cast<double>(RAND_MAX))*(max_bounds_(i) - min_bounds_(i)))
        + min_bounds_(i);
  }
  // generate uniform random SO(3) states for quat
  random.segment(6, 4) = find_quat_rand(seed);
  // generate random Euclidean states for ang vel
  for (int i = 10; i < VEC_SIZE; ++i) {
    random(i) = (rand_r(seed)/(static_cast<double>(RAND_MAX))*(max_bounds_(i-4) - min_bounds_(i-4)))
      + min_bounds_(i-4);
  }
  return random;
}


/* ************************************************************************** */
std::tuple<STATE_13D, Vec3, Vec3, int> StateSpace13D::find_x_new_data(const STATE_13D& source, const STATE_13D& target, double TIMESTEP) {
  /*
  * Finds a state in the direction of target from source
  * some steering options:
  * 1. random steering
  * 2. heuristic steering (motion primitives part of this)
  * 3. exact steering (requires solution to BVP or using a lattice and knonw solutions)
  * 4. optimal exact steering
  */
  // Propagate for a single TIMESTEP
  double DT = .01;  // delta-t for dynamics forward propagation
  int NUM_TIMESTEPS = static_cast<int>(TIMESTEP/DT);

  // Create a quat
  Quaternion<double> q(source(9), source(6), source(7), source(8));

  F_ARRAY F;
  T_ARRAY T;

  // F << 1,-1, 0, 0,
  //      0, 0, 1,-1,
  //      0, 0, 0, 0;

  // T << 0, 0, 0, 0,
  //      0, 0, 0, 0,
  //      0, 0, 0, 0;

  // F << 1,-1, 0, 0, 0, 0, 0,
  //      0, 0, 1,-1, 0, 0, 0,
  //      0, 0, 0, 0, 1,-1, 0;

  // T << 0, 0, 0, 0, 0, 0, 0,
  //      0, 0, 0, 0, 0, 0, 0,
  //      0, 0, 0, 0, 0, 0, 0;

  // F << 0, 0,
  //      0, 0,
  //      0, 0;

  // T << 0, 0,
  //      0, 0,
  //     -1, 1;


  F << 1, -1,  0,  0,  0,  0,  0,  .0,  0,  0,  0,  0,
        0,  0,  1, -1,  0,  0,  0,  .0,  0,  0,  0,  0,
        0,  0,  0,  0,  1, -1,  0,  .0,  0,  0,  0,  0;

  T << 0,  0,  0,  0,  0,  0,  1, -1,  0,  0,  0,  0,
        0,  0,  0, .0,  0,  0,  0,  0,  1, -1,  0,  0,
        0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1, -1;

  X_NEXT_ARRAY results;  // this is a hack to get a 3D array

  if (use_combos_) {
    for (int i = 0; i < num_F; i++) {                 // simulate different inputs to the system
      for (int j = 0; j < num_T; j++) {
        // Set initial conditions based on source
        model_.SetPosition(source(0), source(1), source(2));  // p, COM
        model_.SetVelocity(source(3), source(4), source(5));  // v, COM
        model_.SetRotation(q);                                // q, this must have 9 elements in R format
        model_.SetAngularVelocity(source(10), source(11), source(12));  // w, around COM

        if (i == 0 && j == 0) {  // source state
          // logf("==== source");
          // PrintSimulationState(&model_);
        }

        for (int tidx = 0; tidx < NUM_TIMESTEPS; tidx++) {  // simulate NUM_TIMESTEPS forward
          // Apply force at the COM
          model_.SetExternalForce(F(0, i), F(1, i), F(2, i));  // x, y, z

          // Apply torque about the COM
          model_.SetExternalTorque(T(0, j), T(1, j), T(2, j));  // about x, y, z

          // Forward propagate, and see where system ends up
          model_.EulerStep(DT);

          // Store the result
          model_.GetPosition(&results(i, j)(0), &results(i, j)(1), &results(i, j)(2));
          model_.GetVelocity(&results(i, j)(3), &results(i, j)(4), &results(i, j)(5));
          model_.get_quaternion(&results(i, j)(6), &results(i, j)(7), &results(i, j)(8), &results(i, j)(9));
          model_.GetAngularVelocity(&results(i, j)(10), &results(i, j)(11), &results(i, j)(12));

          // logf("====" <<tidx*DT);
          // PrintSimulationState(&model_);
        }
        // logf("====" << i << " " << j);
        // PrintSimulationState(&model_);
      }
    }
  } else {
    int j = 0;
    for (int i = 0; i < num_F; i++) {                 // simulate different inputs to the system
        // Set initial conditions based on source
        model_.SetPosition(source(0), source(1), source(2));  // p, COM
        model_.SetVelocity(source(3), source(4), source(5));  // v, COM
        model_.SetRotation(q);                    //  q, this must have 9 elements in R format
        model_.SetAngularVelocity(source(10), source(11), source(12));  // w, around COM

        if (i == 0) {  // source state
          // logf("---- source");
          // PrintSimulationState(&model_);
        }

        for (int tidx = 0; tidx < NUM_TIMESTEPS; tidx++) {  // simulate NUM_TIMESTEPS forward
          // Apply force at the COM
          model_.SetExternalForce(F(0, i), F(1, i), F(2, i));  // x, y, z

          // Apply torque about the COM
          model_.SetExternalTorque(T(0, i), T(1, i), T(2, i));  // about x, y, z

          // Forward propagate, and see where system ends up
          model_.EulerStep(DT);

          // Store the result
          model_.GetPosition(&results(i, j)(0), &results(i, j)(1), &results(i, j)(2));
          model_.GetVelocity(&results(i, j)(3), &results(i, j)(4), &results(i, j)(5));
          model_.get_quaternion(&results(i, j)(6), &results(i, j)(7), &results(i, j)(8), &results(i, j)(9));
          model_.GetAngularVelocity(&results(i, j)(10), &results(i, j)(11), &results(i, j)(12));

          // logf("----" <<tidx*DT);
          // PrintSimulationState(&model_);
        }
        // logf("----" << i << " " << j);
        // PrintSimulationState(&model);
      }
  }

  // Return the input with the best result
  int extend_success = 1;
  STATE_13D x_new;
  x_new.setZero();
  Vec3 prev_F, prev_T;
  prev_F.setZero(); prev_T.setZero();

  get_min_result(x_new, prev_F, prev_T, extend_success, results, target, F, T);
  return std::make_tuple(x_new, prev_F, prev_T, extend_success);  // failed to branch out
}


/* ************************************************************************** */
void StateSpace13D::get_min_result(STATE_13D& x_new, Vec3& prev_F, Vec3& prev_T, int& extend_success,
  X_NEXT_ARRAY& results, const STATE_13D& target, F_ARRAY& F, T_ARRAY& T) {
  // Take the result with min(distance)
  STATE_13D cur_state;  // should probably initialize this
  double cur_dist = DBL_MAX;
  double lowest_dist = DBL_MAX;  // lowest distance
  Eigen::Vector2i lowest_idx(-1, -1);

  if (use_combos_) {
    for (int i = 0; i < num_F; i++) {
      for (int j = 0; j < num_T; j++) {
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
    }
  } else {
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
  }
  if (lowest_idx.x() == -1) {
    // logf("No valid states!");
    extend_success = 0;
    return;  // failed to branch out
  }
  x_new = results(lowest_idx.x(), lowest_idx.y());
  prev_F = F.col(lowest_idx.x()).transpose();
  if (use_combos_)
    prev_T = T.col(lowest_idx.y()).transpose();
  else
    prev_T = T.col(lowest_idx.x()).transpose();
  return;
}


/* ************************************************************************** */
std::tuple<STATE_13D, Vec3, Vec3, int> StateSpace13D::find_x_new_data(
  /* Allows variable time stepping, TO DO
  */
  const STATE_13D& source, const STATE_13D& target, double TIMESTEP, double MAX_TIMESTEP) {
  return(find_x_new_data(source, target, TIMESTEP));
}


/* ************************************************************************** */
STATE_13D StateSpace13D::find_x_new(const STATE_13D& source, const STATE_13D& target, double TIMESTEP) {
  /* Only grabs x_new
  */
  STATE_13D x_new;
  Vec3 prev_F, prev_T;
  int extend_success;
  std::tie(x_new, prev_F, prev_T, extend_success) = find_x_new_data(source, target, TIMESTEP);
  return x_new;
}


/* ************************************************************************** */
STATE_13D StateSpace13D::find_x_new(const STATE_13D& source,
                              const STATE_13D& target,
                              double stepSize, double maxStepSize) {
  /* Adaptive step size version, which scales the possible inputs based on goal proximity
  */
  STATE_13D x_new;
  x_new = find_x_new(source, target, stepSize);
  return x_new;
}


/* ************************************************************************** */
double StateSpace13D::get_dist(const STATE_13D& source, const STATE_13D& target) const {
  /*
  * Calculate the distance between states by combining their metrics for RBD planning
  */
  double dist, wp, wq, wv, ww, p_dist, v_dist, q_dist, w_dist;  // distance, and weights on metrics
  const STATE_13D& diff = target - source;

  wp = 20; wv = 1; wq = 1; ww = 1;  // should normalize these so components are [0,1]

  p_dist = get_R3_dist(diff(0), diff(1), diff(2));  // position
  v_dist = get_R3_dist(diff(3), diff(4), diff(5));  // linear velocity
  q_dist = get_SO3_dist(source(6), source(7), source(8), source(9),
    target(6), target(7), target(8), target(9));     // quaternion
  w_dist = get_R3_dist(diff(10), diff(11), diff(12));  // angular velocity

  dist = wp*p_dist + wv*v_dist + wq*q_dist + ww*w_dist;
  return dist;
}


/* ************************************************************************** */
double StateSpace13D::get_R3_dist(double x, double y, double z, double w) const {
  /*
  * Calculate Euclidean distance (up to 4 dims)
  */
  double dist;
  dist = sqrt(powf(x, 2) + powf(y, 2) + powf(z, 2) + powf(w, 2));
  return dist;
}


/* ************************************************************************** */
double StateSpace13D::get_SO3_dist(double q1a, double q1b, double q1c, double q1d,
  double q2a, double q2b, double q2c, double q2d) const {
  /*
  * Calculate SO(3) distance
  */
  double dist1, dist2;
  dist1 = acos(q1a*q2a + q1b*q2b + q1c*q2c + q1d*q2d);
  dist2 = acos(q1a*-q2a + q1b*-q2b + q1c*-q2c + q1d*-q2d);
  dist1 = !(dist1 < dist2) ? dist2:dist1;  // min
  return dist1;
}


/* ************************************************************************** */
Vec4 StateSpace13D::find_quat_rand(unsigned int* seed) const {
  /*
  * Generate a random, uniform unit quaternion, from LaValle
  */
  Vec4 quat;
  double u1, u2, u3;
  u1 = rand_r(seed)/(static_cast<double>(RAND_MAX));
  u2 = rand_r(seed)/(static_cast<double>(RAND_MAX));
  u3 = rand_r(seed)/(static_cast<double>(RAND_MAX));

  quat(0) = sqrt(1-u1)*sin(2*M_PI*u2);
  quat(1) = sqrt(1-u1)*cos(2*M_PI*u2);
  quat(2) = sqrt(u1)*sin(2*M_PI*u3);
  quat(3) = sqrt(u1)*cos(2*M_PI*u3);
  return quat;
}


/* ************************************************************************** */
int StateSpace13D::get_d() const {
  /*
  * Return the state space dimensionality
  */
    return dimension_;
}


/* ************************************************************************** */
bool StateSpace13D::is_state_valid(const STATE_13D& pt, const std::vector<Obstacle>& obstacles) {
  /*
  * Returns a boolean indicating whether the given point is within bounds.
  */
  // Bounds checking
  if (pt(0) < min_bounds_(0) || pt(0) > max_bounds_(0) ||    // p
    pt(1) < min_bounds_(1) || pt(1) > max_bounds_(1) ||
    pt(2) < min_bounds_(2) || pt(2) > max_bounds_(2) ||
    pt(3) < min_bounds_(3) || pt(3) > max_bounds_(3) ||    // v
    pt(4) < min_bounds_(4) || pt(4) > max_bounds_(4) ||
    pt(5) < min_bounds_(5) || pt(5) > max_bounds_(5) ||
    pt(10) < min_bounds_(6) || pt(10) > max_bounds_(6) ||  // w
    pt(11) < min_bounds_(7) || pt(11) > max_bounds_(7) ||
    pt(12) < min_bounds_(8) || pt(12) > max_bounds_(8) ) {
    return false;
  }

  return true;  // valid!
}


/* ************************************************************************** */
bool StateSpace13D::is_segment_valid(const STATE_13D& from, const STATE_13D& to, const std::vector<Obstacle>& obstacles) {
  /*
  * Check motion validity from one state to another: currently, only checks obstacle collision
  */
  bool valid = true;
  Eigen::Vector3d pos = to.head(3);

  for(uint i = 0; i < obstacles.size(); i++) { 
    if (obstacles[i].is_in_collision(pos)) {
      valid = false;
    }
  } 

  return valid;
}


/* ************************************************************************** */
void StateSpace13D::print_simulation_state(RigidBodyDynamics* rigidBodyDynamics) {
  /*
  * Prints out the current simulation parameters of the rigid body
  */
  // query current rigid body parameters
  double x, y, z;
  rigidBodyDynamics->GetPosition(&x, &y, &z);
  double velx, vely, velz;
  rigidBodyDynamics->GetVelocity(&velx, &vely, &velz);
  double qx, qy, qz, qw;
  rigidBodyDynamics->get_quaternion(&qx, &qy, &qz, &qw);
  double angularVelocity[3];
  rigidBodyDynamics->GetAngularVelocity(&angularVelocity[0], &angularVelocity[1], &angularVelocity[2]);

  // printf("p: %G %G %G\n", x, y, z);
  // printf("v: %G %G %G\n", velx, vely, velz);
  // printf("q: %G %G %G %G\n", qx, qy, qz, qw);
  // printf("w: %G %G %G\n", angularVelocity[0], angularVelocity[1], angularVelocity[2]);
  // printf("Current inertia tensor:\n");
  // printf("%G %G %G\n", inertiaTensor[0], inertiaTensor[1], inertiaTensor[2]);
  // printf("%G %G %G\n", inertiaTensor[3], inertiaTensor[4], inertiaTensor[5]);
  // printf("%G %G %G\n", inertiaTensor[6], inertiaTensor[7], inertiaTensor[8]);
}

}  // namespace rrt
