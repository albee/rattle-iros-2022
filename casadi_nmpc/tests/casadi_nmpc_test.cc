/**
 * @file casadi_nmpc_test.cc
 * @brief Test casadi_nmpc non-ROS components
 * @author Keenan Albee (albee@mit.edu)
 * Copyright 2021 Keenan Albee
 */

#include "casadi_nmpc/casadi_nmpc.h"
#include "casadi_nmpc/pd_attitude_controller.h"

#include <ff_msgs/EkfState.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <ff_msgs/ControlState.h>

#include <Eigen/Dense>
#include <memory>
#include <cmath>
#include <math.h>

#include "gtest/gtest.h"

/* ************************************************************************** */
TEST(CasadiNMPCTest, ZPolyCalc){  // test online tube MPC updates
    casadi_nmpc::CasadiNMPCNodelet nodelet; 

    Eigen::Matrix<double, 6, 1> w_bound;
    Eigen::Matrix<double, 3, 1> u_max;
    w_bound << 0.02, 0.02, 0.02, 0.002, 0.002, 0.002;  //
    nodelet.u_mag_ = Eigen::Matrix<double, 3, 1>{0.15, 0.15, 0.15};
    w_bound = nodelet.evaluate_w(8.0, 0.5);
    u_max << 0.15, 0.15, 0.15;
    ros::Duration(2.0).sleep();

    // these values are all set via YAML
    nodelet.prepare_mrpi(w_bound, u_max, 0.2, 10.0, 5.0, 10.0, 2.0);
   
    // std::cout << "w_bound " << w_bound << "\nu_mag " << nodelet.u_mag_ << "\nMPC_dt " << nodelet.MPC_dt_ <<
    //               "\nmass " << nodelet.params_model_.mass  << "\nQ_pos " << nodelet.Q_pos_anc_factor_ <<
    //               "\nQ_vel " << nodelet.Q_vel_anc_factor_ << "\nR_anc " << nodelet.R_anc_factor_;
   
    std::cout << "K_ " << nodelet.K_dr_ << "\nAu_ " << nodelet.Au_ << "\nbu_ " << nodelet.bu_ << "\nAZ_ " <<
              nodelet.AZ_ << "\nbZ_ " << nodelet.bZ_ << std::endl;
}

/* ************************************************************************** */
TEST(CasadiNMPCTest, Interpolate){
    casadi_nmpc::CasadiNMPCNodelet nodelet; 
    nodelet.eigen_x_des_traj_.resize(2,20);
    nodelet.eigen_x_des_traj_.row(0) << 0.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0,  0.0, 0.0, -0.707, 0.707,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    nodelet.eigen_x_des_traj_.row(1) << 0.4,  1.0, 1.0, 1.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0, 1.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    double t0 = nodelet.eigen_x_des_traj_.row(0)(0);
    double t1 = nodelet.eigen_x_des_traj_.row(1)(0);
    double th = 0.2;
    Eigen::MatrixXd state_interp = nodelet.interpolate_state(nodelet.eigen_x_des_traj_.row(0), nodelet.eigen_x_des_traj_.row(1), t0, t1, th);
    std::cout << "interp: " << state_interp << std::endl;
    std::cout << "done" << state_interp.cols() << std::endl;


    // whole trajectory
    nodelet.eigen_x_des_traj_.resize(5,20);
    nodelet.eigen_x_des_traj_.row(0) << 0.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0,  0.0, 0.0, -0.707, 0.707,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    nodelet.eigen_x_des_traj_.row(1) << 0.3,  0.2, 0.2, 0.2,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0, 1.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    nodelet.eigen_x_des_traj_.row(2) << 0.6,  0.4, 0.4, 0.4,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0, 1.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    nodelet.eigen_x_des_traj_.row(3) << 0.9,  0.6, 0.6, 0.6,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0, 1.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    nodelet.eigen_x_des_traj_.row(4) << 1.2,  0.8, 0.8, 0.8,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0, 1.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    std::cout << "eig: \n" << nodelet.eigen_x_des_traj_ << std::endl;

    int last_row = nodelet.eigen_x_des_traj_.rows() - 1;
    t0 = nodelet.eigen_x_des_traj_.row(0)(0);
    double tf = nodelet.eigen_x_des_traj_.row(last_row)(0);
    double dt = 0.2;

    Eigen::MatrixXd traj_interp = nodelet.interpolate_traj(nodelet.eigen_x_des_traj_, t0, dt, tf);
    std::cout << "interp: \n" << traj_interp << std::endl;


    // whole trajectory again
    nodelet.eigen_x_des_traj_.resize(5,20);
    nodelet.eigen_x_des_traj_.row(0) << 1.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0,  0.0, 0.0, -0.707, 0.707,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    nodelet.eigen_x_des_traj_.row(1) << 2.0,  0.2, 0.2, 0.2,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0, 1.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    nodelet.eigen_x_des_traj_.row(2) << 3.0,  0.4, 0.4, 0.4,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0, 1.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    nodelet.eigen_x_des_traj_.row(3) << 4.0,  0.6, 0.6, 0.6,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0, 1.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    nodelet.eigen_x_des_traj_.row(4) << 5.0,  0.8, 0.8, 0.8,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0, 1.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    std::cout << "eig: \n" << nodelet.eigen_x_des_traj_ << std::endl;

    last_row = nodelet.eigen_x_des_traj_.rows() - 1;
    t0 = nodelet.eigen_x_des_traj_.row(0)(0);
    tf = nodelet.eigen_x_des_traj_.row(last_row)(0);
    dt = 0.2;

    traj_interp = nodelet.interpolate_traj(nodelet.eigen_x_des_traj_, t0, dt, tf);
    std::cout << "interp: \n" << traj_interp << std::endl;
}

/* ************************************************************************** */
TEST(CasadiNMPCTest, PDControl) {
     // setup nmpc and set variables
    casadi_nmpc::CasadiNMPCNodelet nodelet; 
    nodelet.u_mag_ << 0.1, 0.1, 0.1;
    nodelet.torque_mag_ << 0.01, 0.01, 0.01;
    nodelet.traj_idx_ = 0;
    nodelet.eigen_x_des_traj_.resize(1,20);

    Vector3d u_torques;

    // x_real_complete_ = [x y z qx qy qz qw vx vy vz wx wyz wz]
    // eigen_x_des_traj_ = [[t x y z xd yd zd qx qy qz qw wx wy wz xdd ydd zdd wxd wyd wzd] ... ]

    // (1) no error
    nodelet.x_real_complete_ <<        0.0, 0.0, 0.0,  0.0, 0.0, 0.0, 1.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0;
    nodelet.eigen_x_des_traj_ << 0.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0, 1.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0;
    u_torques = nodelet.calc_torques_PD(false);

    std::cout << "torques: " << u_torques << std::endl;
    ASSERT_EQ(u_torques[0], 0);

    // // (2) just x (negative)
    nodelet.x_real_complete_ << 0.0, 0.0, 0.0,  0.0, 0.0, 0.0, 1.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0;
    nodelet.eigen_x_des_traj_ << 0.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0,  -0.9589243, 0, 0, 0.2836622,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0; // (negative)
    u_torques = nodelet.calc_torques_PD(false);
    ASSERT_LT(u_torques[0], 0.0);
 
    // // (3) just y (negative)
    nodelet.x_real_complete_ << 0.0, 0.0, 0.0,  0.0, 0.0, 0.0, 1.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0;
    nodelet.eigen_x_des_traj_ << 0.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0,  0, -0.9589243, 0, 0.2836622,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0;
    u_torques = nodelet.calc_torques_PD(false);
    ASSERT_LT(u_torques[1], 0.0);

    // // (4) just z (negative)
    nodelet.x_real_complete_ << 0.0, 0.0, 0.0,  0.0, 0.0, 0.0, 1.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0;
    nodelet.eigen_x_des_traj_ << 0.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0,  0, 0, -0.9589243, 0.2836622,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0;
    u_torques = nodelet.calc_torques_PD(false);
    ASSERT_LT(u_torques[2], 0.0);

    // // (5) just xd (positive)
    nodelet.x_real_complete_ << 0.0, 0.0, 0.0,  0.0, 0.0, 0.0, 1.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0;
    nodelet.eigen_x_des_traj_ << 0.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0, 1.0,  0.5, 0.0, 0.0,  0.0, 0.0, 0.0;
    u_torques = nodelet.calc_torques_PD(false);
    ASSERT_GT(u_torques[0], 0.0);

    // // (6) just yd (positive)
    nodelet.x_real_complete_ << 0.0, 0.0, 0.0,  0.0, 0.0, 0.0, 1.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0;
    nodelet.eigen_x_des_traj_ << 0.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0, 1.0,  0.0, 0.5, 0.0,  0.0, 0.0, 0.0;
    u_torques = nodelet.calc_torques_PD(false);
    ASSERT_GT(u_torques[1], 0.0);

    // // (7) just zd (positive)
    nodelet.x_real_complete_ << 0.0, 0.0, 0.0,  0.0, 0.0, 0.0, 1.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0;
    nodelet.eigen_x_des_traj_ << 0.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0, 1.0,  0.0, 0.0, 0.5,  0.0, 0.0, 0.0;
    u_torques = nodelet.calc_torques_PD(false);
    ASSERT_GT(u_torques[2], 0.0);

    // // (8) scaling works, different values
    nodelet.torque_mag_ << 0.001, 0.001, 0.001;
    nodelet.x_real_complete_ << 0.0, 0.0, 0.0,  0.0, 0.0, 0.0, 1.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0;
    nodelet.eigen_x_des_traj_ << 0.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0,  0.7071068, 0, 0, 0.7071068,  0.5, 0.5, 0.5,  0.0, 0.0, 0.0;
    u_torques = nodelet.calc_torques_PD(false);
    ASSERT_EQ(sqrt(pow(u_torques[0], 2) + pow(u_torques[1], 2) + pow(u_torques[2], 2)), 0.001);
    std::cout << "Final test: " << u_torques << std::endl;
}


/* ************************************************************************** */
TEST(CasadiNMPCTest, CheckInputLimits) {
    casadi_nmpc::CasadiNMPCNodelet nodelet;
    nodelet.u_mag_ << 0.1, 0.1, 0.1;
    nodelet.torque_mag_ << 0.1, 0.1, 0.1;

    // cut negatives
    {
    tf2::Vector3 F_xyz_B(-1.0, -1.0, -1.0);  // must convert to Body frame
    tf2::Vector3 T_xyz_B(-1.0, -1.0, -1.0);
    std::cout << F_xyz_B.getX() << " " << F_xyz_B.getY() << " " << F_xyz_B.getZ() << std::endl;
    std::cout << T_xyz_B.getX() << " " << T_xyz_B.getY() << " " << T_xyz_B.getZ() << std::endl;

    std::tie(F_xyz_B, T_xyz_B) = nodelet.check_input_limits(F_xyz_B, T_xyz_B);

    std::cout << F_xyz_B.getX() << " " << F_xyz_B.getY() << " " << F_xyz_B.getZ() << std::endl;
    std::cout << T_xyz_B.getX() << " " << T_xyz_B.getY() << " " << T_xyz_B.getZ() << std::endl;

    Eigen::Vector3f eig_F_xyz_B{F_xyz_B.getX(), F_xyz_B.getY(), F_xyz_B.getZ()};
    Eigen::Vector3f eig_T_xyz_B{T_xyz_B.getX(), T_xyz_B.getY(), T_xyz_B.getZ()};
    bool comp1 = eig_F_xyz_B.isApprox(Eigen::Vector3f{-0.1, -0.1, -0.1});
    bool comp2 = eig_T_xyz_B.isApprox(Eigen::Vector3f{-0.1, -0.1, -0.1});
    std::cout << comp1 << " " << comp2 << std::endl;
    EXPECT_TRUE(comp1 && comp2);
    }

    // cut positives
    {
    tf2::Vector3 F_xyz_B(1.0, 1.0, 1.0);  // must convert to Body frame
    tf2::Vector3 T_xyz_B(1.0, 1.0, 1.0);
    std::cout << F_xyz_B.getX() << " " << F_xyz_B.getY() << " " << F_xyz_B.getZ() << std::endl;
    std::cout << T_xyz_B.getX() << " " << T_xyz_B.getY() << " " << T_xyz_B.getZ() << std::endl;

    std::tie(F_xyz_B, T_xyz_B) = nodelet.check_input_limits(F_xyz_B, T_xyz_B);

    std::cout << F_xyz_B.getX() << " " << F_xyz_B.getY() << " " << F_xyz_B.getZ() << std::endl;
    std::cout << T_xyz_B.getX() << " " << T_xyz_B.getY() << " " << T_xyz_B.getZ() << std::endl;

    Eigen::Vector3f eig_F_xyz_B{F_xyz_B.getX(), F_xyz_B.getY(), F_xyz_B.getZ()};
    Eigen::Vector3f eig_T_xyz_B{T_xyz_B.getX(), T_xyz_B.getY(), T_xyz_B.getZ()};
    bool comp1 = eig_F_xyz_B.isApprox(Eigen::Vector3f{0.1, 0.1, 0.1});
    bool comp2 = eig_T_xyz_B.isApprox(Eigen::Vector3f{0.1, 0.1, 0.1});
    std::cout << comp1 << " " << comp2 << std::endl;
    EXPECT_TRUE(comp1 && comp2);
    }

    // don't touch
    {
    tf2::Vector3 F_xyz_B(0.05, 0.05, -0.05);  // must convert to Body frame
    tf2::Vector3 T_xyz_B(0.05, 0.05, -0.05);
    std::cout << F_xyz_B.getX() << " " << F_xyz_B.getY() << " " << F_xyz_B.getZ() << std::endl;
    std::cout << T_xyz_B.getX() << " " << T_xyz_B.getY() << " " << T_xyz_B.getZ() << std::endl;

    std::tie(F_xyz_B, T_xyz_B) = nodelet.check_input_limits(F_xyz_B, T_xyz_B);

    std::cout << F_xyz_B.getX() << " " << F_xyz_B.getY() << " " << F_xyz_B.getZ() << std::endl;
    std::cout << T_xyz_B.getX() << " " << T_xyz_B.getY() << " " << T_xyz_B.getZ() << std::endl;

    Eigen::Vector3f eig_F_xyz_B{F_xyz_B.getX(), F_xyz_B.getY(), F_xyz_B.getZ()};
    Eigen::Vector3f eig_T_xyz_B{T_xyz_B.getX(), T_xyz_B.getY(), T_xyz_B.getZ()};
    bool comp1 = eig_F_xyz_B.isApprox(Eigen::Vector3f{0.05, 0.05, -0.05});
    bool comp2 = eig_T_xyz_B.isApprox(Eigen::Vector3f{0.05, 0.05, -0.05});
    std::cout << comp1 << " " << comp2 << std::endl;
    EXPECT_TRUE(comp1 && comp2);
    }
}


/* ************************************************************************** */
TEST(CasadiNMPCTest, MPCControl) {
    // CasadiNMPCNodelet::read_traj_DLR("input/TEST7-GND/");  // read it in ourselves

    // NODELET_INFO_STREAM("unit test...");
    // x_real_ << 10.9, -9.65, 4.8, 0, 0, 0;  // we are here
    // w_bound_ << 0.01, 0.01, 0.01, 0.01, 0.01, 0.01;
    // double dt = MPC_dt_;
    // traj_idx_ = 0;
    // print_x_des_traj_(traj_idx_); // we want to be here

    // // check successful read-in of reference traj
    // // make sure to not include time!
    // NODELET_INFO_STREAM("Traj read-in..." << eigen_x_des_traj_(2,3));
    // if (abs(eigen_x_des_traj_(2,3) - 4.9) < 1E-6) {
    //   NODELET_INFO_STREAM("...okay!");
    // }
    // else {
    //   NODELET_INFO_STREAM("...failed!");
    // }

    // prepare_mrpi(w_bound_, u_mag_, MPC_dt_, mass_, Q_pos_anc_factor_, Q_vel_anc_factor_, R_anc_factor_);  // get K_dr and Au and bu ready
    // NODELET_INFO_STREAM("\n" << w_bound_ << "\n" << u_mag_ << "\n" << dt);

    // // check same values as MATLAB
    // Vector3d u_forces;
    // Vector3d u0_mpc;
    // Vector3d u0_dr;
    // Matrix<double, 6, 1> x_nom;
    // u_forces = call_mpc_func_casadi(false);
    // NODELET_INFO_STREAM("CasADi test..." << u_forces(1));
    // if (abs(u_forces(2) - u_mag_[2]) < 0.001) {
    //   NODELET_INFO_STREAM("...okay!");
    // }
    // else {
    //   NODELET_INFO_STREAM("...failed!");
    // }

    // x_real_ << 10.9, -9.75, 4.9, 0, 0, 0;  // we are here
    // u_forces = call_mpc_func_casadi(false);
    // NODELET_INFO_STREAM("CasADi test..." << u_forces(1));
    // if (abs(u_forces(1) - u_mag_[1]) < 0.001) {
    //   NODELET_INFO_STREAM("...okay!");
    // }
    // else {
    //   NODELET_INFO_STREAM("...failed!");
    // }

    // // Test Tube MPC
    // /// (1) Compute the nominal MPC input
    // std::tie(u_forces, u0_mpc, u0_dr, x_nom) = call_tube_mpc_func_casadi();

    // /// (2) Compute the ancillary controller input and sum
    // // u_forces = add_ancillary_input(x_nom, u_forces);
    // ros::param::set("/td/tube_mpc/unit_test_complete", true);
}


// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tester");
  ros::NodeHandle nh;
  testing::GTEST_FLAG(filter) = "CasadiNMPCTest.ZPolyCalc";
  return RUN_ALL_TESTS();
}