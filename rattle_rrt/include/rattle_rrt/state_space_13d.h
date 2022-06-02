/* 
# state_space_13d

A 13D state space for Newton-Euler kinodynamic planning.

Keenan Albee, 2021
MIT Space Systems Lab
*/

#ifndef STATE_SPACE_13D_H_
#define STATE_SPACE_13D_H_

#include <eigen3/Eigen/Dense>
#include <limits>
#include <cfloat>

#include <rattle_rrt/state_space.tpp>
#include <rattle_rrt/types.h>
#include <rbd/rigidBodyDynamics.h>


namespace rrt {
typedef Eigen::Matrix<double, 13, 1> STATE_13D;
  /* The state space for a microgravity free flyer, extends the abstract StateSpace.
  (Newton-Euler dynamics)

  state vector: [rx ry rz vx vy vz qx qy qz qw wx wy wz]

  num_F : number of possible force inputs (motion primitives) to sample from
  num_T : number of possible torque inputs (motion primitives) to sample from
  */

class StateSpace13D : public StateSpace<STATE_13D> {
  /* The state space for a free flyer, extends the abstract StateSpace.
  */
public:
    // NOTE: bounds do NOT allow for restrictions on orientation, SO(3), as-is
    Eigen::Matrix<double, 9, 1> min_bounds_;  // min_bounds = min_x, min_y, min_z,min_xd, min_yd, min_zd, min_phid, min_thetad, min_psid
    Eigen::Matrix<double, 9, 1> max_bounds_;  // max_bounds = max_x, max_y, max_z,max_xd, max_yd, max_zd, max_phid, max_thetad, max_psid
    int dimension_;
    static const int use_combos_ = 0;  // use combinations of the F T inputs?

    static const int num_F = 12;
    static const int num_T = 12;

    RigidBodyDynamics model_;  // the dynamics model to evaluate

    typedef Eigen::Array<STATE_13D, num_F, num_T> X_NEXT_ARRAY;
    typedef Eigen::Matrix<double, 3, num_F> F_ARRAY;
    typedef Eigen::Matrix<double, 3, num_T> T_ARRAY;

    StateSpace13D(Eigen::Matrix<double, 9, 1> min_bounds, Eigen::Matrix<double, 9, 1> max_bounds, double mass, double ixx, double iyy, double izz);

    // template funcs
    STATE_13D find_x_rand(unsigned int* seed) const override;
    std::tuple<STATE_13D, Vec3, Vec3, int> find_x_new_data(const STATE_13D& source, const STATE_13D& target, double TIMESTEP) override;
    std::tuple<STATE_13D, Vec3, Vec3, int> find_x_new_data(const STATE_13D& source, const STATE_13D& target, double TIMESTEP, double MAX_TIMESTEP) override;

    STATE_13D find_x_new(const STATE_13D& source, const STATE_13D& target, double TIMESTEP) override;
    STATE_13D find_x_new(const STATE_13D& source, const STATE_13D& target, double stepSize, double maxStepSize) override;

    double get_dist(const STATE_13D& source, const STATE_13D& target) const override;

    bool is_state_valid(const STATE_13D& pt, const std::vector<Obstacle>& obstacles = std::vector<Obstacle>()) override;
    bool is_segment_valid(const STATE_13D& start, const STATE_13D& goal, const std::vector<Obstacle>& obstacles = std::vector<Obstacle>()) override;

    int get_d() const override;

    // derived funcs
    double get_R3_dist(double x, double y, double z = 0, double w = 0) const;
    double get_SO3_dist(double q1a, double q1b, double q1c, double q1d, double q2a, double q2b, double q2c, double q2d) const;

    Vec4 find_quat_rand(unsigned int* seed) const;

    void get_min_result(STATE_13D& x_new, Vec3& prev_F, Vec3& prev_T, int& extend_success,
                    X_NEXT_ARRAY& results, const STATE_13D& target, F_ARRAY& F, T_ARRAY& T);

    void print_simulation_state(RigidBodyDynamics* rigidBodyDynamics);
};

}  // namespace rrt
#endif  // STATE_13D_SPACE_13D_H_
