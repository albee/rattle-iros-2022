/* 
# state_space_2d

A 2D state space for x-y kinematic planning.

Keenan Albee, 2021
MIT Space Systems Lab
*/

# pragma once
#include <eigen3/Eigen/Dense>
#include <limits>
#include <cfloat>

#include <rattle_rrt/state_space.tpp>
#include <rattle_rrt/types.h>
#include <rbd/rigidBodyDynamics.h>


namespace rrt {
typedef Eigen::Matrix<double, 2, 1> STATE_2D;

class StateSpace2D : public StateSpace<STATE_2D> {
    /* A 2d plane with continuous states and no obstacles. No discretization, extends the abstract StateSpace.
    */
 public:
    Eigen::Vector2d min_bounds_, max_bounds_;
    int dimension_ = 2;

    StateSpace2D(Eigen::Vector2d min_bounds, Eigen::Vector2d max_bounds);
 
    // template funcs
    STATE_2D find_x_rand(unsigned int* seed) const override;

    std::tuple<STATE_2D, Vec3, Vec3, int> find_x_new_data(const STATE_2D& source, const STATE_2D& target, double TIMESTEP) override;
    std::tuple<STATE_2D, Vec3, Vec3, int> find_x_new_data(const STATE_2D& source, const STATE_2D& target, double TIMESTEP, double MAX_TIMESTEP) override;

    STATE_2D find_x_new(const STATE_2D& source, const STATE_2D& target, double stepSize) override;
    STATE_2D find_x_new(const STATE_2D& source, const STATE_2D& target, double stepSize, double maxStepSize) override;

    double get_dist(const STATE_2D& from, const STATE_2D& to) const override;

    bool is_state_valid(const STATE_2D& pt, const std::vector<Obstacle>& obstacles = std::vector<Obstacle>()) override;
    bool is_segment_valid(const STATE_2D& from, const STATE_2D& to, const std::vector<Obstacle>& obstacles = std::vector<Obstacle>()) override;

    int get_d() const override;

    // derived funcs
    double width() const;
    double height() const;
};

}  // namespace rrt
