/* 
# state_space_2d

A 2D state space for x-y kinematic planning.

Keenan Albee, 2021
MIT Space Systems Lab
*/

#include <rattle_rrt/state_space_2d.h>


namespace rrt {

/* ************************************************************************** */
StateSpace2D::StateSpace2D(Eigen::Vector2d min_bounds, Eigen::Vector2d max_bounds)
    : min_bounds_(min_bounds), max_bounds_(max_bounds) {
}

/* ************************************************************************** */
STATE_2D StateSpace2D::find_x_rand(unsigned int* seed) const {
    /*
    * Generate a random 2D state
    */
    return STATE_2D(rand_r(seed)/(double)RAND_MAX * width() + min_bounds_(0),
                    rand_r(seed)/(double)RAND_MAX * height() + min_bounds_(1));
}


/* ************************************************************************** */
double StateSpace2D::width() const {  // D1
    return max_bounds_(0) - min_bounds_(0);
}


/* ************************************************************************** */

double StateSpace2D::height() const {  // D2
    return max_bounds_(1) - min_bounds_(1);
}


/* ************************************************************************** */
std::tuple<STATE_2D, Vec3, Vec3, int> StateSpace2D::find_x_new_data(const STATE_2D& source,
                                const STATE_2D& target,
                                double TIMESTEP) {
    /*
    * Return a new state, along with the info for arriving there. TO DO
    */
    throw "Not implemented!";
}

/* ************************************************************************** */
std::tuple<STATE_2D, Vec3, Vec3, int> StateSpace2D::find_x_new_data(const STATE_2D& source,
                                const STATE_2D& target,
                                double TIMESTEP, double MAX_TIMESTEP) {
    /*
    * Return a new state, along with the info for arriving there, adaptive. TO DO
    */
    throw "Not implemented!";
}

/* ************************************************************************** */
STATE_2D StateSpace2D::find_x_new(const STATE_2D& source,
                                const STATE_2D& target,
                                double stepSize) {
    /*
    * Finds a state in the direction of target from source
    * This is the steering function, which relies on finding viable inputs
    * to produce a reasonable output.
    */
    STATE_2D delta = target - source;
    delta = delta / delta.norm();  //  unit vector

    STATE_2D val = source + delta * stepSize;
    return val;
}

/* ************************************************************************** */
STATE_2D StateSpace2D::find_x_new(const STATE_2D& source,
                                  const STATE_2D& target,
                                  double stepSize, double maxStepSize) {
    /*
    * Adaptive step size is enabled, which scales the expansion size based on the distance to the goal
    */
    STATE_2D delta = target - source;
    delta = delta / delta.norm();  //  unit vector

    STATE_2D val = source + delta * stepSize;
    return val;
}


/* ************************************************************************** */
double StateSpace2D::get_dist(const STATE_2D& from, const STATE_2D& to) const {
    /*
    * Calculate the distance between two states from Start state
    * to End state. @return The distance between the states
    */
    STATE_2D delta = from - to;
    return sqrtf(powf(delta.x(), 2) + powf(delta.y(), 2));
}


/* ************************************************************************** */
bool StateSpace2D::is_state_valid(const STATE_2D& pt, const std::vector<Obstacle>& obstacles) {
    /*
    * Returns a boolean indicating whether the given point is within bounds.
    */
    return pt.x() >= 0 && pt.y() >= 0 && pt.x() < width() &&
        pt.y() < height();
}


/* ************************************************************************** */
bool StateSpace2D::is_segment_valid(const STATE_2D& from, const STATE_2D& to, const std::vector<Obstacle>& obstacles) {
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
int StateSpace2D::get_d() const {
    /*
    * Return the state space dimensionality.
    */
    return dimension_;
}

}  // namespace rrt
