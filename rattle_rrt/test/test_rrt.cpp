/* 
# main.cpp

Initializes an RRT tree of state space dimension {2, 6, 13}.
2:= 2D kinematic
4:= 2D double integrator
6_iss:= 3D double integrator
6_ground:= 2D double integrator w/ rotation
13:= 3D Newton-Euler

Keenan Albee, 2021
MIT Space Systems Lab
*/

#include <eigen3/Eigen/Dense>
#include <rattle_rrt/state_space.tpp>
#include <rattle_rrt/state_space_2d.h>
#include <rattle_rrt/state_space_4d.h>
#include <rattle_rrt/state_space_6d_ground.h>
#include <rattle_rrt/state_space_6d_iss.h>
#include <rattle_rrt/state_space_13d.h>
#include <rattle_rrt/types.h>
#include <rattle_rrt/tree.tpp>

#include <rbd/rigidBodyDynamics.h>

#include <memory>
#include <vector>
#include <chrono>

using std::cout;
using std::endl;
using std::shared_ptr;
using std::vector;
using std::make_shared;


void setup_2d();
void setup_4d();
void setup_6d_ground();

std::tuple<std::shared_ptr<rrt::Tree<rrt::Vec6>>, shared_ptr<rrt::StateSpace6DISS>> setup_6d_iss();
void run_6d(std::shared_ptr<rrt::Tree<rrt::Vec6>> rrt_6d);

void setup_13d();
void verify_bullet(shared_ptr<rrt::StateSpace6DISS> state_space_iss);

int main(int argc, char** argv) {
    /* RRT test routine
    */
    // setup_2d();

    // cout << "Setting up a sample tree (6D)..." << endl;
    // setup_6d_ground();
    // cout << "...tree computed and sent to output file." << endl;

    // cout << "Setting up a sample tree (4D)..." << endl;
    // setup_4d();
    // cout << "...tree computed and sent to output file." << endl;

    cout << "Setting up a sample tree (6D)..." << endl;
    std::shared_ptr<rrt::Tree<rrt::Vec6>> rrt_iss;
    shared_ptr<rrt::StateSpace6DISS> state_space_iss;

    std::tie(rrt_iss, state_space_iss) = setup_6d_iss();
    // verify_bullet(state_space_iss);

    std::vector<double> times;
    for (int i=0; i < 30; i++){
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        std::tie(rrt_iss, state_space_iss) = setup_6d_iss();
        run_6d(rrt_iss);
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        std::chrono::duration<double> dt = end - begin; 
        double dt_d = dt.count();
        std::cout << "dt " << dt_d << std::endl;
        times.push_back(dt_d);
    }

    for (auto itr = times.begin(); itr < times.end(); itr++){
        cout << *itr << " ";
    }
    cout << endl;

    cout << "...tree computed and sent to output file." << endl;

    // cout << "Setting up a sample tree (13D)..." << endl;
    // setup_13d();
    // cout << "...tree computed and sent to output file." << endl;
    return 0;
}

void run_6d(std::shared_ptr<rrt::Tree<rrt::Vec6>> rrt_6d){
    // run
    rrt_6d->build_RRT();
    rrt_6d->get_path_nodes();

    // write for debugging
    rrt_6d->write_path("rrt_path_output.csv");

    std::vector<std::tuple<rrt::Vec6, rrt::Vec6>> edges_6d = rrt_6d->get_rrt_edges();  // get all the RRT edges
}


void setup_2d() {
     /*
     * 2D, no dynamics
     */

    // Vec2 min_bounds, max_bounds;
    // double min_x = -1.0;
    // double min_y = -1.0;
    // min_bounds << min_x, min_y;

    // double max_x = 1.0;
    // double max_y = 1.0;
    // max_bounds << max_x, max_y;

    // //start state, goal state
    // Vec2 start, goal;
    // start << 0.0,0.0;
    // goal << 0.1,0.7;

    // //params
    // int max_iterations;
    // bool is_ASC_enabled, remember_actions;
    // double timestep, max_timestep, goal_bias, goal_max_dist;
    // max_iterations = 10000;
    // is_ASC_enabled = false;
    // timestep = .03;
    // max_timestep = .1;
    // goal_bias = 0;
    // goal_max_dist = .015;
    // remember_actions = false;

    // // set params, create tree
    // shared_ptr<StateSpace2D<Vec2>> state_space_ptr =
    //     make_shared<StateSpace2D<Vec2>> (min_bounds, max_bounds);

    // shared_ptr<Tree<Vec2>> rrt = tree_for_2d(state_space_ptr,
    //     start, goal, max_iterations, is_ASC_enabled, timestep,
    //     max_timestep, goal_bias, goal_max_dist, remember_actions);

    // shared_ptr<Tree<Vec2>> rrt = std::make_shared<Tree<Vec2>>(state_space);  // this is the rrt tree data structure, as a shared_ptr
    // rrt->setStartState(start);
    // rrt->setGoalState(goal);
    // rrt->set_params(max_iterations, is_ASC_enabled, timestep,
    //                 max_timestep, goal_bias, goal_max_dist, remember_actions);

    // // run
    // result = rrt->build_RRT();
    // logf(result);
}

void setup_4d() {
    /* 2 DoF, double integrator dynamics
    */

    // RBD params (for Astrobee)
    double mass = 19.58;  // kg
    double ixx = 0.153;  // kg-m2
    double iyy = 0.143;  // kg-m2
    double izz = 0.162;  // kg-m2

    // Statespace params
    rrt::Vec4 min_bounds;
    double min_x = -0.88;
    double min_y = -0.88;
    double min_xd = -.05;
    double min_yd = -.05;
    min_bounds << min_x, min_y, min_xd, min_yd;

    rrt::Vec4 max_bounds;
    double max_x = 0.88;
    double max_y = 0.88;
    double max_xd = .05;
    double max_yd = .05;
    max_bounds << max_x, max_y, max_xd, max_yd;

    // start state, goal state
    rrt::Vec4 start;
    rrt::Vec4 goal;
    start << -0.5, 0.7,       // x, y
             0.0, 0.0;         // xd, yd
    goal << 0.5, -0.7,
            0.0, 0.0;

    // params
    int max_iterations = 10000;
    bool is_ASC_enabled = false;
    bool remember_actions = true;
    double timestep = 0.5;
    double max_timestep = 1.0;
    double goal_bias = 0.05;
    double goal_max_dist = 3.0;  /// extremely important in determining termination!
    std::vector<Obstacle> obstacles;

    obstacles.push_back(Obstacle(0.05, 0.8, 0.1, -0.2, 0.35, 0.0));
    obstacles.push_back(Obstacle(0.05, 0.8, 0.1, 0.3, -0.35, 0.0));

    // create the state space
    shared_ptr<rrt::StateSpace4D> state_space_ptr_4d =
        make_shared<rrt::StateSpace4D> (min_bounds, max_bounds, mass, ixx, iyy, izz);

    state_space_ptr_4d->initialize_bullet(obstacles);  // NB: Astrobee is assumed to be 0.3m diameter sphere

    // now, make the tree
    shared_ptr<rrt::Tree<rrt::Vec4>> rrt_4d =
        make_shared<rrt::Tree<rrt::Vec4>>(state_space_ptr_4d);  // this is the rrt tree data structure, as a shared_ptr

    // set tree parameters
    rrt_4d->start_state_ = start;
    rrt_4d->goal_state_ = goal;
    rrt_4d->set_params(max_iterations, is_ASC_enabled, timestep, max_timestep, goal_bias, goal_max_dist, remember_actions, obstacles);

    // run
    rrt_4d->build_RRT();
    rrt_4d->get_path_nodes();
    
    // write for debugging
    rrt_4d->write_path("rrt_path_output.csv");

    std::vector<std::tuple<rrt::Vec4, rrt::Vec4>> edges_4d = rrt_4d->get_rrt_edges();  // get all the RRT edges
}

std::tuple<std::shared_ptr<rrt::Tree<rrt::Vec6>>, shared_ptr<rrt::StateSpace6DISS>> setup_6d_iss() {
    /* 3 DoF, double integrator dynamics
    */

    // RBD params (for Astrobee)
    double mass = 9.58;  // kg
    double ixx = 0.153;  // kg-m2
    double iyy = 0.143;  // kg-m2
    double izz = 0.162;  // kg-m2

    // Statespace params
    rrt::Vec6 min_bounds;
    double min_x = 10.5;
    double min_y = -9.8;
    double min_z = 4.2;
    double min_xd = -.05;
    double min_yd = -.05;
    double min_zd = -.05;
    min_bounds << min_x, min_y, min_z, min_xd, min_yd, min_zd;

    rrt::Vec6 max_bounds;
    double max_x = 11.0;
    double max_y = -7.5;
    double max_z = 5.3;
    double max_xd = .05;
    double max_yd = .05;
    double max_zd = .05;
    max_bounds << max_x, max_y, max_z, max_xd, max_yd, max_zd;

    // start state, goal state
    rrt::Vec6 start;
    rrt::Vec6 goal;
    start << 10.8, -9.6, 4.8,         // x, y, z
             0.0, 0.0, 0.0;         // xd, yd, zd
    goal << 10.75, -7.8, 5.0,
            0.0, 0.0, 0.0;

    // params
    int max_iterations = 10000;
    bool is_ASC_enabled = false;
    bool remember_actions = true;
    double timestep = 2.0;
    double max_timestep = 2.0;
    double goal_bias = 0.05;
    double goal_max_dist = 0.2;
    std::vector<Obstacle> obstacles;

    // obstacles.push_back(Obstacle(1.0, 1.0, 1.0, 10.0, 0.0, 0.0));
    // obstacles.push_back(Obstacle(0.2, 0.2, 0.65, 10.8, -9.0, 4.9));
    obstacles.push_back(Obstacle(0.15, 0.15, 0.75, 10.2, -9.1, 4.8));
    obstacles.push_back(Obstacle(0.2, 0.2, 0.3, 10.5, -9.1, 4.7));
    obstacles.push_back(Obstacle(0.15, 0.15, 0.75, 10.8, -9.1, 4.8));
    obstacles.push_back(Obstacle(0.2, 0.2, 0.3, 11.1, -9.1, 4.7));
    obstacles.push_back(Obstacle(0.15, 0.15, 0.75, 11.4, -9.1, 4.8));
    obstacles.push_back(Obstacle(0.75, 0.15, 0.15, 10.8, -8.6, 4.1));
    obstacles.push_back(Obstacle(0.15, 0.15, 0.75, 10.8, -8.6, 4.7));
    obstacles.push_back(Obstacle(0.75, 0.15, 0.15, 10.8, -8.6, 5.3));
    obstacles.push_back(Obstacle(0.75, 0.15, 0.15, 10.8, -8.1, 4.7));
    obstacles.push_back(Obstacle(0.15, 0.15, 0.75, 10.8, -8.1, 5.3));

    // create the state space
    shared_ptr<rrt::StateSpace6DISS> state_space_ptr_6d =
        make_shared<rrt::StateSpace6DISS> (min_bounds, max_bounds, mass, ixx, iyy, izz);

    state_space_ptr_6d->initialize_bullet(obstacles);  // NB: Astrobee is assumed to be 0.3m diameter sphere

    // now, make the tree
    shared_ptr<rrt::Tree<rrt::Vec6>> rrt_6d =
        make_shared<rrt::Tree<rrt::Vec6>>(state_space_ptr_6d);  // this is the rrt tree data structure, as a shared_ptr

    // set tree parameters
    rrt_6d->start_state_ = start;
    rrt_6d->goal_state_ = goal;
    rrt_6d->set_params(max_iterations, is_ASC_enabled, timestep, max_timestep, goal_bias, goal_max_dist, remember_actions, obstacles);

    return std::make_tuple(rrt_6d, state_space_ptr_6d);
}


void verify_bullet(shared_ptr<rrt::StateSpace6DISS> state_space_iss) {
    /* Set up a simple collision and non-collision scenario
    */
    bool out = state_space_iss->bullet_.IsFreeState(rrt::Vec3{0.0, 0.0, 0.0}, Eigen::Quaternion<double>{0.0, 0.0, 0.0, 1.0});
    std::cout << "free?: " << out << std::endl;

    out = state_space_iss->bullet_.IsFreeState(rrt::Vec3{10.8, -9, 4.9}, Eigen::Quaternion<double>{0.0, 0.0, 0.0, 1.0});
    std::cout << "free?: " << out << std::endl;
}


void setup_6d_ground() {
    // /*
    // 3 DoF, planar dynamics
    // */

    // // RBD params (for Astrobee)
    // double mass = 19.58;  // kg
    // double ixx = 0.153;  // kg-m2
    // double iyy = 0.143;  // kg-m2
    // double izz = 0.162;  // kg-m2

    // // Statespace params
    // rrt::Vec6 min_bounds;
    // double min_x = -0.88;
    // double min_y = -0.88;
    // double min_th = -3.14159;
    // double min_xd = -.05;
    // double min_yd = -.05;
    // double min_thd = -0.1;
    // min_bounds << min_x, min_y, min_th, min_xd, min_yd, min_thd;

    // rrt::Vec6 max_bounds;
    // double max_x = 0.88;
    // double max_y = 0.88;
    // double max_th = 3.14159;
    // double max_xd = .05;
    // double max_yd = .05;
    // double max_thd = 0.1;
    // max_bounds << max_x, max_y, max_th, max_xd, max_yd, max_thd;

    // // start state, goal state
    // rrt::Vec6 start;
    // rrt::Vec6 goal;
    // start << -0.5, 0.7, 0.0,         // x, y, th
    //          0.0, 0.0, 0.0;         // xd, yd, thd
    // goal << 0.5, -0.7, 0.0,
    //         0.0, 0.0, 0.0;

    // // params
    // int max_iterations = 10000;
    // bool is_ASC_enabled = false;
    // bool remember_actions = true;
    // double timestep = 0.5;
    // double max_timestep = 1.0;
    // double goal_bias = 0.05;
    // double goal_max_dist = 5.0;
    // std::vector<Obstacle> obstacles;

    // obstacles.push_back(Obstacle(0.05, 0.8, 0.1, -0.2, 0.35, 0.0));
    // obstacles.push_back(Obstacle(0.05, 0.8, 0.1, 0.3, -0.35, 0.0));

    // // create the state space
    // shared_ptr<rrt::StateSpace6DGround> state_space_ptr_6d =
    //     make_shared<rrt::StateSpace6DGround> (min_bounds, max_bounds, mass, ixx, iyy, izz);

    // // now, make the tree
    // shared_ptr<rrt::Tree<rrt::Vec6>> rrt_6d =
    //     make_shared<rrt::Tree<rrt::Vec6>>(state_space_ptr_6d);  // this is the rrt tree data structure, as a shared_ptr

    // // set tree parameters
    // rrt_6d->start_state_ = start;
    // rrt_6d->goal_state_ = goal;
    // rrt_6d->set_params(max_iterations, is_ASC_enabled, timestep, max_timestep, goal_bias, goal_max_dist, remember_actions, obstacles);

    // // run
    // rrt_6d->build_RRT();
    // rrt_6d->get_path_nodes();
    
    // // write for debugging
    // rrt_6d->write_path("rrt_path_output.csv");

    // std::vector<std::tuple<rrt::Vec6, rrt::Vec6>> edges_6d = rrt_6d->get_rrt_edges();  // get all the RRT edges
}

void setup_13d() {
    // /*
    // * 6 DoF, RBD
    // */

    // // RBD params (for Astrobee)
    // double mass = 9.58;  // kg
    // double ixx = 0.153;  // kg-m2
    // double iyy = 0.143;  // kg-m2
    // double izz = 0.162;  // kg-m2

    // // Statespace params
    // rrt::Vec9 min_bounds;
    // double min_x = -1.0;
    // double min_y = -1.0;
    // double min_z = -1.0;
    // double min_xd = -.1;
    // double min_yd = -.1;
    // double min_zd = -.1;
    // double min_phid = -1.0;
    // double min_thetad = -1.0;
    // double min_psid = -1.0;
    // min_bounds << min_x, min_y, min_z, min_xd, min_yd, min_zd, min_phid, min_thetad, min_psid;

    // rrt::Vec9 max_bounds;
    // double max_x = 1.0;
    // double max_y = 1.0;
    // double max_z = 1.0;
    // double max_xd = .1;
    // double max_yd = .1;
    // double max_zd = .1;
    // double max_phid = 1.0;
    // double max_thetad = 1.0;
    // double max_psid = 1.0;
    // max_bounds << max_x, max_y, max_z, max_xd, max_yd, max_zd, max_phid, max_thetad, max_psid;

    // // start state, goal state
    // rrt::Vec13 start, goal;
    // start << 0.0, 0.0, 0.0,         // x, y, z
    //             0.0, 0.0, 0.0,         // xd, yd, zd
    //             1.0, 0.0, 0.0, 0.0,    // qx, qy, qz, qth
    //             0.0, 0.0, 0.0;         // wx, wy, wz
    // goal << -0.5, -0.7, -0.7,
    //         0.0, 0.0, 0.0,
    //         // 1.0, 0.0, 0.0, 0.0,
    //         // 0.577, 0.577, 0.577, 0.0,
    //         0.0, 1.0, 0.0, 0.0,
    //         0.0, 0.0, 0.0;

    // // params
    // int max_iterations = 5000;
    // bool is_ASC_enabled = false;
    // bool remember_actions = true;
    // double timestep = 0.5;
    // double max_timestep = 1.0;
    // double goal_bias = 0.05;
    // double goal_max_dist = 10.0;

    // // create the state space object
    // std::shared_ptr<rrt::StateSpace13D> state_space_ptr =
    //     std::make_shared<rrt::StateSpace13D> (min_bounds, max_bounds, mass, ixx, iyy, izz);

    // shared_ptr<rrt::Tree<rrt::Vec13>> rrt =
    //     std::make_shared<rrt::Tree<rrt::Vec13>>(state_space_ptr);  // this is the rrt tree data structure, as a shared_ptr

    // rrt->start_state_ = start;
    // rrt->goal_state_ = goal;
    // rrt->set_params(max_iterations, is_ASC_enabled, timestep, max_timestep, goal_bias, goal_max_dist, remember_actions);

    // // run
    // rrt->build_RRT();
    // rrt->get_path_nodes();

    // // write for debugging
    // rrt->write_path("rrt_path_output.csv");
}
