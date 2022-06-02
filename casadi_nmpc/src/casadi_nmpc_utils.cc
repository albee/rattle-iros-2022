/*
# casadi_nmpc_utils.cc

Utility functions for casadi_nmpc.

Keenan Albee
MIT Space Systems Lab, 06.23.21
*/
#include "casadi_nmpc/casadi_nmpc.h"
#include "dlr_utils/dlr_utils.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Scalar.h"
#include "math.h"


namespace casadi_nmpc{

/* ************************************************************************** */
std::string CasadiNMPCNodelet::quat2str(tf2::Quaternion q) {
    std::stringstream ss;
    ss << q[0] << "," << q[1] << "," << q[2] << "," << q[3];
    return ss.str();
}


/* ************************************************************************** */
void CasadiNMPCNodelet::get_all_YAML_parameters() {
    /* Get all parameters needed for MPC and tube MPC. Most are specified by YAML.
    */
    // TVR frame
    ros::param::getCached("/reswarm/r_RI_ISS_x", r_RI_(0));
    ros::param::getCached("/reswarm/r_RI_ISS_y", r_RI_(1));
    ros::param::getCached("/reswarm/r_RI_ISS_z", r_RI_(2));

    double u_mag;
    double torque_mag;

    ros::param::getCached("/reswarm/casadi_nmpc/tube_update_period", tube_update_dt_);
    ros::param::getCached("/reswarm/casadi_nmpc/control_period", control_dt_);
    ros::param::getCached("/reswarm/casadi_nmpc/T", T);
    ros::param::getCached("/reswarm/casadi_nmpc/N", N);
    ros::param::getCached("/reswarm/casadi_nmpc/u_mag", u_mag);
    ros::param::getCached("/reswarm/casadi_nmpc/torque_mag", torque_mag);
    MPC_rate_ = (double)N/T;
    MPC_dt_ = 1.0/MPC_rate_;
    u_mag_ << u_mag, u_mag, u_mag;
    torque_mag_ << torque_mag, torque_mag, torque_mag;

    // set gains and mass
    gains gains;
    if (ground_.compare("true") == 0) {
        ros::param::getCached("/reswarm/casadi_nmpc/Q_pos_factor_ground", gains.Q_pos_factor);
        ros::param::getCached("/reswarm/casadi_nmpc/Q_vel_factor_ground", gains.Q_vel_factor);
        ros::param::getCached("/reswarm/casadi_nmpc/R_factor_ground", gains.R_factor);
        ros::param::getCached("/reswarm/casadi_nmpc/QN_pos_factor_ground", gains.QN_pos_factor);
        ros::param::getCached("/reswarm/casadi_nmpc/QN_vel_factor_ground", gains.QN_vel_factor);

        ros::param::getCached("/reswarm/casadi_nmpc/Q_pos_tube_factor_ground", gains.Q_pos_tube_factor);
        ros::param::getCached("/reswarm/casadi_nmpc/Q_vel_tube_factor_ground", gains.Q_vel_tube_factor);
        ros::param::getCached("/reswarm/casadi_nmpc/R_tube_factor_ground", gains.R_tube_factor);
        ros::param::getCached("/reswarm/casadi_nmpc/QN_pos_tube_factor_ground", gains.QN_pos_tube_factor);
        ros::param::getCached("/reswarm/casadi_nmpc/QN_vel_tube_factor_ground", gains.QN_vel_tube_factor);

        ros::param::getCached("/reswarm/casadi_nmpc/mass_ground", params_model_.mass);
    }
    else {
        ros::param::getCached("/reswarm/casadi_nmpc/Q_pos_factor_iss", gains.Q_pos_factor);
        ros::param::getCached("/reswarm/casadi_nmpc/Q_vel_factor_iss", gains.Q_vel_factor);
        ros::param::getCached("/reswarm/casadi_nmpc/R_factor_iss", gains.R_factor);
        ros::param::getCached("/reswarm/casadi_nmpc/QN_pos_factor_iss", gains.QN_pos_factor);
        ros::param::getCached("/reswarm/casadi_nmpc/QN_vel_factor_iss", gains.QN_vel_factor);

        ros::param::getCached("/reswarm/casadi_nmpc/Q_pos_tube_factor_iss", gains.Q_pos_tube_factor);
        ros::param::getCached("/reswarm/casadi_nmpc/Q_vel_tube_factor_iss", gains.Q_vel_tube_factor);
        ros::param::getCached("/reswarm/casadi_nmpc/R_tube_factor_iss", gains.R_tube_factor);
        ros::param::getCached("/reswarm/casadi_nmpc/QN_pos_tube_factor_iss", gains.QN_pos_tube_factor);
        ros::param::getCached("/reswarm/casadi_nmpc/QN_vel_tube_factor_iss", gains.QN_vel_tube_factor);

        ros::param::getCached("/reswarm/casadi_nmpc/mass_iss", params_model_.mass);
    }

    ros::param::getCached("/reswarm/casadi_nmpc/Q_pos_anc", gains.Q_pos_anc_factor);
    ros::param::getCached("/reswarm/casadi_nmpc/Q_vel_anc", gains.Q_vel_anc_factor);
    ros::param::getCached("/reswarm/casadi_nmpc/R_anc", gains.R_anc_factor);

    make_gains(gains);  // create all gain values
    switch_gains(0);
    dm_m_ = casadi::DM{params_model_.mass};  // use casadi uniform constructor
}


/* ************************************************************************** */
void CasadiNMPCNodelet::update_regulation_setpoint(Vector3f x0, Vector4f a0) {
    /* Update eigen_x_des_traj_reg_ using x0 [x y z] and a0 [qx qy qz qw]
    */
    for (int i = 0; i < eigen_x_des_traj_reg_.rows(); i++) {
        eigen_x_des_traj_reg_(i,1) = (double)x0(0);
        eigen_x_des_traj_reg_(i,2) = (double)x0(1);
        eigen_x_des_traj_reg_(i,3) = (double)x0(2);
        eigen_x_des_traj_reg_(i,4) = 0;
        eigen_x_des_traj_reg_(i,5) = 0;
        eigen_x_des_traj_reg_(i,6) = 0;
        eigen_x_des_traj_reg_(i,7) = (double)a0(0);
        eigen_x_des_traj_reg_(i,8) = (double)a0(1);
        eigen_x_des_traj_reg_(i,9) = (double)a0(2);
        eigen_x_des_traj_reg_(i,10) = (double)a0(3);
        eigen_x_des_traj_reg_(i,11) = 0;
        eigen_x_des_traj_reg_(i,12) = 0;
        eigen_x_des_traj_reg_(i,13) = 0;
    }
}


/* ************************************************************************** */
void CasadiNMPCNodelet::get_initial_regulation()  {
    /* Get parameters specifying where to start the regulation (from execute_asap)
    */
    ros::param::get("/reswarm/primary/x_start", x0_(0));
    ros::param::get("/reswarm/primary/y_start", x0_(1));
    ros::param::get("/reswarm/primary/z_start", x0_(2));
    ros::param::get("/reswarm/primary/qx_start", a0_(0));
    ros::param::get("/reswarm/primary/qy_start", a0_(1));
    ros::param::get("/reswarm/primary/qz_start", a0_(2));
    ros::param::get("/reswarm/primary/qw_start", a0_(3));

    update_regulation_setpoint(x0_, a0_);
}


/* ************************************************************************** */
void CasadiNMPCNodelet::read_traj_from_file(std::string traj_filename){
    /*
    Read in the trajectory directly (Caroline format), no passing through ROS
    */
    std::string DATA_PATH = ros::package::getPath("data")+"/";
    std::string traj_file = DATA_PATH + traj_filename;

    // Set very important variables
    eigen_x_des_traj_ = dlr::get_dlr_output_x<MatrixXd>(traj_file);  // read in .dat trajectory
    dt_traj_ = eigen_x_des_traj_(1, 0) - eigen_x_des_traj_(0, 0);
    traj_rate_ = 1.0/dt_traj_;
    HAVE_TRAJ = 1;
    N_traj_ = eigen_x_des_traj_.rows();
}


/* ************************************************************************** */
Eigen::Matrix3f q2dcm(const Vector4f &q) {
    /* Quaternion (x, y, z, w) to direction cosine matrix

    Gehring, C., Bellicoso, C. D., Bloesch, M., Sommer, H., Fankhauser, P., Hutter, M., & Siegwart, R. (n.d.). Kindr Attitude Representation. 2018.
    */
    Matrix3f dcm;
    dcm(0,0) = pow(q(3),2) + pow(q(0),2) - pow(q(1),2) - pow(q(2),2);
    dcm(0,1) = 2*(q(0)*q(1) + q(3)*q(2));
    dcm(0,2) = 2*(q(0)*q(2) - q(3)*q(1));
    dcm(1,0) = 2*(q(0)*q(1) - q(3)*q(2));
    dcm(1,1) = pow(q(3),2) - pow(q(0),2) + pow(q(1),2) - pow(q(2),2);
    dcm(1,2) = 2*(q(1)*q(2) + q(3)*q(0));
    dcm(2,0) = 2*(q(0)*q(2) + q(3)*q(1));
    dcm(2,1) = 2*(q(1)*q(2) - q(3)*q(0));
    dcm(2,2) = pow(q(3),2) - pow(q(0),2) - pow(q(1),2) + pow(q(2),2);
    return dcm.transpose();
}


/* ************************************************************************** */
casadi::DM CasadiNMPCNodelet::eigen2dm(MatrixXd mat) {
    /*
    Convert from eigen to CasADi's DM input
    */
    casadi::DM dm_mat{std::vector<double>(mat.data(), mat.size() + mat.data())};
    dm_mat = reshape(dm_mat, mat.rows(), mat.cols());

    return dm_mat;
}

/* ************************************************************************** */
void CasadiNMPCNodelet::print_x_des_traj_(int idx) {
    /*
    Validate x_des_traj_
    */
    // std::string my_str;
    // for (int idx0 = 0; idx0 < MATLAB_MAX_ROWS_; idx0++) {
    //   for (int idx1 = 0; idx1 < MPC_NUM_COLS_; idx1++) {
    //     if (idx0 == idx-1) {
    //       my_str.append(std::to_string(x_des_traj_[idx1*MATLAB_MAX_ROWS_ + idx0]));
    //       my_str.append(" ");
    //     }
    //   }
    // }
    // NODELET_INFO_STREAM(my_str);

    IOFormat HeavyFmt(FullPrecision, 0, ", ", ";\n", "[", "]", "[", "]");
    std::string sep = "\n----------------------------------------\n";
    NODELET_INFO_STREAM(sep << eigen_x_des_traj_.block(idx,0,1,7).format(HeavyFmt) << sep);
}

/* ************************************************************************** */
void CasadiNMPCNodelet::make_gains(gains gains_YAML){
    /* Make MPC gain options.
    */
   // [1]
    gains gains_permissive_u = {
        100, 5, 0.1, 1000, 50,
        100, 5, 0.1, 1000, 50,
        5, 2, 0.5
    };

    // [2]
    gains gains_strict_u = {
        100, 5, 10, 100, 10,
        100, 5, 10, 100, 10,
        5, 2, 10
    };

    // [3]
    gains gains_cautious_tube_mpc = {
        50, 2, 0.5, 100, 10,
        50, 2, 0.5, 100, 10,
        10, 5, 4
    };

    gains_list_ground_.insert(gains_list_ground_.end(), { gains_YAML, gains_permissive_u, gains_strict_u, gains_cautious_tube_mpc });
    gains_list_iss_.insert(gains_list_iss_.end(), { gains_YAML, gains_permissive_u, gains_strict_u, gains_cautious_tube_mpc });
}


/* ************************************************************************** */
void CasadiNMPCNodelet::switch_gains(int gain_mode) {
    /* Switch MPC gains using gains struct.
    */
    gains gains;
    if (ground_.compare("true") == 0) {
        gains = gains_list_ground_[gain_mode];
    }
    else {
        gains = gains_list_iss_[gain_mode];
    }

    Q1 = gains.Q_pos_factor;
    Q2 = gains.Q_pos_factor;
    Q3 = gains.Q_pos_factor;
    Q4 = gains.Q_vel_factor;
    Q5 = gains.Q_vel_factor;
    Q6 = gains.Q_vel_factor;
    R1 = gains.R_factor;
    R2 = gains.R_factor;
    R3 = gains.R_factor;
    QN1 = gains.QN_pos_factor;
    QN2 = gains.QN_pos_factor;
    QN3 = gains.QN_pos_factor;
    QN4 = gains.QN_vel_factor;
    QN5 = gains.QN_vel_factor;
    QN6 = gains.QN_vel_factor;

    Q1_T_ = gains.Q_pos_tube_factor;
    Q2_T_ = gains.Q_pos_tube_factor;
    Q3_T_ = gains.Q_pos_tube_factor;
    Q4_T_ = gains.Q_vel_tube_factor;
    Q5_T_ = gains.Q_vel_tube_factor;
    Q6_T_ = gains.Q_vel_tube_factor;
    R1_T_ = gains.R_tube_factor;
    R2_T_ = gains.R_tube_factor;
    R3_T_ = gains.R_tube_factor;
    QN1_T_ = gains.QN_pos_tube_factor;
    QN2_T_ = gains.QN_pos_tube_factor;
    QN3_T_ = gains.QN_pos_tube_factor;
    QN4_T_ = gains.QN_vel_tube_factor;
    QN5_T_ = gains.QN_vel_tube_factor;
    QN6_T_ = gains.QN_vel_tube_factor;

    Q_pos_anc_factor_ = gains.Q_pos_anc_factor;
    Q_vel_anc_factor_ = gains.Q_vel_anc_factor;
    R_anc_factor_ = gains.R_anc_factor;

    dm_Q1_ = casadi::DM{Q1};
    dm_Q2_ = casadi::DM{Q2};
    dm_Q3_ = casadi::DM{Q3};
    dm_Q4_ = casadi::DM{Q4};
    dm_Q5_ = casadi::DM{Q5};
    dm_Q6_ = casadi::DM{Q6};
    dm_R1_ = casadi::DM{R1};
    dm_R2_ = casadi::DM{R2};
    dm_R3_ = casadi::DM{R3};
    dm_QN1_ = casadi::DM{QN1};
    dm_QN2_ = casadi::DM{QN2};
    dm_QN3_ = casadi::DM{QN3};
    dm_QN4_ = casadi::DM{QN4};
    dm_QN5_ = casadi::DM{QN5};
    dm_QN6_ = casadi::DM{QN6};

    dm_Q1_T_= casadi::DM{Q1_T_};
    dm_Q2_T_= casadi::DM{Q2_T_};
    dm_Q3_T_= casadi::DM{Q3_T_};
    dm_Q4_T_= casadi::DM{Q4_T_};
    dm_Q5_T_= casadi::DM{Q5_T_};
    dm_Q6_T_= casadi::DM{Q6_T_};
    dm_R1_T_= casadi::DM{R1_T_};
    dm_R2_T_= casadi::DM{R2_T_};
    dm_R3_T_= casadi::DM{R3_T_};
    dm_QN1_T_ = casadi::DM{QN1_T_};
    dm_QN2_T_ = casadi::DM{QN2_T_};
    dm_QN3_T_ = casadi::DM{QN3_T_};
    dm_QN4_T_ = casadi::DM{QN4_T_};
    dm_QN5_T_ = casadi::DM{QN5_T_};
    dm_QN6_T_ = casadi::DM{QN6_T_}; // use casadi uniform constructor
}


Eigen::MatrixXd CasadiNMPCNodelet::interpolate_traj(Eigen::MatrixXd x_traj, double t0, double dt, double tf) {
  /* interpolate the state trajectory, x_traj to match desired dt. Creates a new x_traj_new with dt from t0
  to tf, interpolating from the closest 2 x_traj timestamps.
  @params:
  This should work for variable dt of x_traj, but is untested.
  x_traj: eig traj in form [[t  x y z  xd yd zd  qx qy qz qw  wx wy wz  xdd ydd zdd  wxd wyd wzd] ... ]
  t0: interp start time
  dt: interp increment
  tf: interp final time

  @return:
  x_traj_new: interpolated trajectory, obeying t_step from t0 to tf
  */
  int num_steps = (int) ceil((double(tf) - double(t0))/double(dt) + 1.0); // be conservative and extend beyond the alotted time
  Eigen::MatrixXd x_traj_new(num_steps, 20);

  Eigen::VectorXd t_traj = x_traj.col(0);
  std::vector<double> t_traj_vec;
  t_traj_vec.resize(t_traj.size());
  Eigen::VectorXd::Map(&t_traj_vec[0], t_traj.size()) = t_traj;  // eigen --> std::vector

  double dt0 = t_traj(1) - t_traj(0);
  if (dt0 == dt) {
    return x_traj;  // no interpolation needed
  }

  // std::cout << "num_steps: " << num_steps << " t_traj_vec: " << t_traj_vec << " dt0: " << dt0 << 
  //               " dt: " << dt << " t0: " << t0 << " tf: " << tf << std::endl;

  double ti;
  int idx;
  for (int i=0; (t0 + i*dt) < (tf + dt); i++) { 
    ti = t0 + i*dt;  // current ti we care about
    auto itr = std::lower_bound(t_traj_vec.begin(), t_traj_vec.end(), ti);  // find the first element >= ti
    idx = std::distance(t_traj_vec.begin(), itr);
    // std::cout << "idx: " << idx << std::endl;

    if (itr == t_traj_vec.end()) {  // nothing left!
      x_traj_new.row(i) = x_traj.row(idx - 1);
      if (ti > x_traj_new.row(i)(0)) {  // one step beyond
        x_traj_new.row(i)(0) = ti;
      }
    } 
    else if (itr == t_traj_vec.begin()) {  // we are at the start!
      x_traj_new.row(i) = x_traj.row(idx);
    }
    else if (*itr == ti) {  // matches exactly: lucky!
      x_traj_new.row(i) = x_traj.row(idx);
    }
    else{  // okay now we actually have to interpolate...
      x_traj_new.row(i) = interpolate_state(x_traj.row(idx - 1), x_traj.row(idx), t_traj_vec[idx - 1], t_traj_vec[idx], ti);
    }
    // std::cout << x_traj_new.row(i) << std::endl;
    // std::cout << x_traj.row(idx) << std::endl;
  }
  
  return x_traj_new;
}


Eigen::MatrixXd CasadiNMPCNodelet::interpolate_state(Eigen::MatrixXd x0, Eigen::MatrixXd x1, double t0, double t1, double t_h) {
  /* interpolate between two state vectors, x0, x1: [t  x y z  xd yd zd  qx qy qz qw  wx wy wz  xdd ydd zdd  wxd wyd wzd] 
  x0[7] = qx
  x0[11] = wx
  @params:
  t0: interp start time
  t1: interp final time
  t_h: interp desired time
  x0: interp start state
  x1: interp final state

  @return:
  x_h: interpolated state at intermediate time h \in [0, 1]
  */
  Eigen::MatrixXd x_h(1, 20);
  x_h(0, 0) = t_h;

  double h = (t_h - t0)/(t1 - t0);
  double r0;
  double r1;
  double r_h;

  // lerp everything except quat
  for (int i = 1; i < x0.cols(); i++){
    if (i != 7 && i != 8 && i != 9 && i != 10){  // if not the quat
      r0 = x0(i);
      r1 = x1(i);
      r_h = r0 + (r1 - r0)*h;  // linear interp
      x_h(0, i) = r_h;
    }
  }

  // slerp the quat
  tf2::Quaternion q0{x0(0, 7), x0(0, 8), x0(0, 9), x0(0, 10)};
  tf2::Quaternion q1{x1(0, 7), x1(0, 8), x1(0, 9), x1(0, 10)};

  tf2::Quaternion q_h = q0.slerp(q1, h);

  x_h(0, 7) = (double) q_h.getX();
  x_h(0, 8) = (double) q_h.getY();
  x_h(0, 9) = (double) q_h.getZ();
  x_h(0, 10) = (double) q_h.getW();

  return x_h;  // interpolated result
}


/* ************************************************************************** */
void CasadiNMPCNodelet::set_mRPI_fallback_values() {
    /* Uses fallback gain, input, and mRPI values for predetermined gain values.
    Only used if mRPI call fails entirely (e.g., bad uc_bound).
    These values are determined using:
    u = [0.15 0.15 0.15]
    */
    NODELET_INFO_STREAM("...using fallback mRPI.");
    using_fallback_mrpi_ = true;

    K_dr_.resize(3, 6);
    K_dr_ <<
    -1.1594,         0,         0,   -4.9716,         0,         0,
            0,   -1.1594,         0,         0,   -4.9716,         0,
            0,         0,   -1.1594,         0,         0,   -4.9716;

    Au_.resize(6, 3);
    Au_ <<
    1,     0,     0,
    0,     1,     0,
    0,     0,     1,
    -1,     0,     0,
    0,    -1,     0,
    0,     0,    -1;

    bu_.resize(6, 1);
    // assumes u_max is 0.3
    bu_ << 0.0704,    0.0704,    0.0704,    0.0704,    0.0704,    0.0704;  // tightened!

    AZ_.resize(100, 6);
    AZ_ <<
    1.0000,         0,         0,         0,         0,         0,
    -1.0000,         0,         0,         0,         0,         0,
    -0.1289,         0,         0,   -0.9917,         0,         0,
    0.5424,         0,         0,   -0.8401,         0,         0,
            0,         0,         0,   -1.0000,         0,         0,
    -0.5424,         0,         0,    0.8401,         0,         0,
    0.1289,         0,         0,    0.9917,         0,         0,
            0,         0,         0,    1.0000,         0,         0,
            0,    1.0000,         0,         0,         0,         0,
            0,   -1.0000,         0,         0,         0,         0,
            0,   -0.1289,         0,         0,   -0.9917,         0,
            0,    0.5424,         0,         0,   -0.8401,         0,
            0,         0,         0,         0,   -1.0000,         0,
            0,   -0.5424,         0,         0,    0.8401,         0,
            0,    0.1289,         0,         0,    0.9917,         0,
            0,         0,         0,         0,    1.0000,         0,
            0,         0,    1.0000,         0,         0,         0,
            0,         0,   -1.0000,         0,         0,         0,
            0,         0,   -0.1289,         0,         0,   -0.9917,
            0,         0,    0.5424,         0,         0,   -0.8401,
            0,         0,         0,         0,         0,   -1.0000,
            0,         0,   -0.5424,         0,         0,    0.8401,
            0,         0,    0.1289,         0,         0,    0.9917,
            0,         0,         0,         0,         0,    1.0000,
            0,         0,         0,         0,         0,         0,
            0,         0,         0,         0,         0,         0,
            0,         0,         0,         0,         0,         0,
            0,         0,         0,         0,         0,         0,
            0,         0,         0,         0,         0,         0,
            0,         0,         0,         0,         0,         0,
            0,         0,         0,         0,         0,         0,
            0,         0,         0,         0,         0,         0,
            0,         0,         0,         0,         0,         0,
            0,         0,         0,         0,         0,         0,
            0,         0,         0,         0,         0,         0,
            0,         0,         0,         0,         0,         0,
            0,         0,         0,         0,         0,         0,
            0,         0,         0,         0,         0,         0,
            0,         0,         0,         0,         0,         0,
            0,         0,         0,         0,         0,         0,
            0,         0,         0,         0,         0,         0,
            0,         0,         0,         0,         0,         0,
            0,         0,         0,         0,         0,         0,
            0,         0,         0,         0,         0,         0,
            0,         0,         0,         0,         0,         0,
            0,         0,         0,         0,         0,         0,
            0,         0,         0,         0,         0,         0,
            0,         0,         0,         0,         0,         0,
            0,         0,         0,         0,         0,         0,
            0,         0,         0,         0,         0,         0,
            0,         0,         0,         0,         0,         0,
            0,         0,         0,         0,         0,         0,
            0,         0,         0,         0,         0,         0,
            0,         0,         0,         0,         0,         0,
            0,         0,         0,         0,         0,         0,
            0,         0,         0,         0,         0,         0,
            0,         0,         0,         0,         0,         0,
            0,         0,         0,         0,         0,         0,
            0,         0,         0,         0,         0,         0,
            0,         0,         0,         0,         0,         0,
            0,         0,         0,         0,         0,         0,
            0,         0,         0,         0,         0,         0,
            0,         0,         0,         0,         0,         0,
            0,         0,         0,         0,         0,         0,
            0,         0,         0,         0,         0,         0,
            0,         0,         0,         0,         0,         0,
            0,         0,         0,         0,         0,         0,
            0,         0,         0,         0,         0,         0,
            0,         0,         0,         0,         0,         0,
            0,         0,         0,         0,         0,         0,
            0,         0,         0,         0,         0,         0,
            0,         0,         0,         0,         0,         0,
            0,         0,         0,         0,         0,         0,
            0,         0,         0,         0,         0,         0,
            0,         0,         0,         0,         0,         0,
            0,         0,         0,         0,         0,         0,
            0,         0,         0,         0,         0,         0,
            0,         0,         0,         0,         0,         0,
            0,         0,         0,         0,         0,         0,
            0,         0,         0,         0,         0,         0,
            0,         0,         0,         0,         0,         0,
            0,         0,         0,         0,         0,         0,
            0,         0,         0,         0,         0,         0,
            0,         0,         0,         0,         0,         0,
            0,         0,         0,         0,         0,         0,
            0,         0,         0,         0,         0,         0,
            0,         0,         0,         0,         0,         0,
            0,         0,         0,         0,         0,         0,
            0,         0,         0,         0,         0,         0,
            0,         0,         0,         0,         0,         0,
            0,         0,         0,         0,         0,         0,
            0,         0,         0,         0,         0,         0,
            0,         0,         0,         0,         0,         0,
            0,         0,         0,         0,         0,         0,
            0,         0,         0,         0,         0,         0,
            0,         0,         0,         0,         0,         0,
            0,         0,         0,         0,         0,         0,
            0,         0,         0,         0,         0,         0,
            0,         0,         0,         0,         0,         0,
            0,         0,         0,         0,         0,         0;

    bZ_.resize(100, 1);
    bZ_ <<     0.0185, 0.0185, 0.0140, 0.0144, 0.0132, 0.0144, 0.0140, 0.0132, 0.0185, 0.0185, 0.0140, 0.0144, 0.0132, 0.0144, 0.0140, 0.0132, 0.0185, 0.0185, 0.0140, 0.0144, 0.0132, 0.0144, 0.0140, 0.0132,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0;
    dm_K_dr_ = eigen2dm(K_dr_);
    dm_Au_ = eigen2dm(Au_);
    dm_bu_ = eigen2dm(bu_);
    dm_AZ_ = eigen2dm(AZ_);
    dm_bZ_ = eigen2dm(bZ_);
}
} // end namespace casadi_nmpc