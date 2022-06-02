/*
call_dlr.cc

Script to call DLR executable
*/

#ifndef TRAJ_UTILS_H_
#define TRAJ_UTILS_H_

#include <iostream>
#include <stdlib.h>
#include <sstream>
#include <fstream>
#include <string>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>

/* ************************************************************************** */
template<typename M>
M get_traj_from_file_u(const std::string& path) {
  /* Get DLR's motion planning input history, with the following format:
  results_input_0.data

  Input:
  path of .dat directory, must end in "/"

  Output:
  Matrix of inputs

  [row, 20]

  output_u = Eigen::MatrixXd(
   t ux uy uz
   ...
   )

  [row, 4]
  */
  std::string FILE_INPUT  = path + "results_input_0.dat";
  std::vector<std::string> filepaths{FILE_INPUT};
  int NUM_FILES = int(filepaths.size());
  uint rows = 0;
  std::vector <double> values;  // store data from file as parsed
  std::vector <Eigen::MatrixXd> raw_data;  // store matrix data from each file

  // for each file, grab all the data and put it in an output Eigen::MatrixXd
  for (int i = 0; i < NUM_FILES; i++) {
    std::string file = filepaths[i];
    rows = 0;
    std::ifstream indata;
    indata.open(file);
    if(indata.fail()){
      std::cout << "Traj file read failed!" << std::endl;
    }
    std::string line;

  // Get raw data from each file; parse until no more lines
  while (std::getline(indata, line)) {
    std::stringstream lineStream(line);
    std::string cell;
    while (std::getline(lineStream, cell, ' ')) {
      if (!cell.empty()) {  // big ol bugs without this -- delimiter repeats an unspecified number of times before each value
        values.push_back(std::stod(cell));
      }
    }
    ++rows;
  }
   indata.close();
   Eigen::MatrixXd data_matrix = Eigen::Map<const Eigen::Matrix<typename M::Scalar, M::RowsAtCompileTime, M::ColsAtCompileTime, Eigen::RowMajor>>(values.data(), rows, values.size()/rows);
   raw_data.push_back(data_matrix);
   values.clear();  // prepare for new set of data
  }
  Eigen::MatrixXd formatted_matrix(rows, 3); // [rows, 3] matrix.

  // Transfer data from file matrix to output matrix
  formatted_matrix.col(0) = raw_data[0].col(0);  // extract time from first column, they're all the same
  for (int j = 0; j < 3; j++) {  // position data (start at 1, since first column is always time)
    formatted_matrix.col(j).setZero();  // translational inputs, hardcoded
  }
  return formatted_matrix;
}

/* ************************************************************************** */
template<typename M>
M get_traj_from_file_x(const std::string& path){
  /*
  Gets DLR's motion planning executable output, with the following format:

  results_pos_0.dat, results_vel_0.dat, results_ori_0.dat, results_ome_0.dat

  The contents of
  these files appears in 4 columns: Time, x-dim, y-dim, z-dim. There are no headers.
  There are 301 rows of entries, corresponding to a 60 s maneuver sampled at 0.2 s.

  Input: absolute path of .dat files

  Output:

  output_x = Eigen::MatrixXd(
    t x y z xd yd zd qw qx qy qz wx wy wz xdd ydd zdd wxd wyd wzd
    ...
    )
  */
  std::string FILE_POS  = path + "results_pos_0.dat";
  std::string FILE_VEL  = path + "results_vel_0.dat";
  std::string FILE_QUAT = path + "results_ori_0.dat";
  std::string FILE_ANG_VEL = path + "results_ome_0.dat";

  //std::string FILE_ACC  = path + "Results_acc.dat";
  //std::string FILE_JERK = path + "Results_jerk.dat";

  std::vector<std::string> filepaths{FILE_POS, FILE_VEL, FILE_QUAT, FILE_ANG_VEL};
  int NUM_FILES = int(filepaths.size());
  uint rows = 0;
  std::vector <double> values;  // store data from file as parsed
  std::vector <Eigen::MatrixXd> raw_data;  // store matrix data from each file

  // for each file, grab all the data and put it in an output Eigen::MatrixXd
  for (int i = 0; i < NUM_FILES; i++) {
    std::string file = filepaths[i];
    rows = 0;
    std::ifstream indata;
    indata.open(file);
    if(indata.fail()){
      std::cout << "Traj file read failed!" << std::endl;
    }
    std::string line;

   // Get raw data from each file; parse until no more lines
   while (std::getline(indata, line)) {
     std::stringstream lineStream(line);
     std::string cell;
     // std::cout << lineStream.str() << '\n';
     while (std::getline(lineStream, cell, ' ')) {
       if (!cell.empty()) {  // big ol bugs without this -- delimiter repeats an unspecified number of times before each value
         values.push_back(std::stod(cell));
       }
     }
     ++rows;
  }
   indata.close();
   Eigen::MatrixXd data_matrix = Eigen::Map<const Eigen::Matrix<typename M::Scalar, M::RowsAtCompileTime, M::ColsAtCompileTime, Eigen::RowMajor>>(values.data(), rows, values.size()/rows);
   raw_data.push_back(data_matrix);
   values.clear();  // prepare for new set of data
  }
  Eigen::MatrixXd formatted_matrix(rows, 20); // [rows, 20] matrix.

  // // Transfer data from file read matrices to output matrix
  formatted_matrix.col(0) = raw_data[0].col(0);  // extract time from first column, they're all the same
  for (int j = 0; j < 4; j++) {
    if (j < 3) {
      // formatted_matrix.col(j)    = raw_data[0].col(j+1);  // position data
      formatted_matrix.col(1+j)    = raw_data[0].col(j+1);  // position data
      formatted_matrix.col(4+j)  = raw_data[1].col(j+1);  // linear velocity data
      formatted_matrix.col(7+j)  = raw_data[2].col(j+1);  // Eigen::MatrixXd::Ones(rows, 1)*start_pose_chaser_ground[3+j]; orientation (quat)
      formatted_matrix.col(11+j) = raw_data[3].col(j+1);  // angular velocity
      formatted_matrix.col(14+j).setZero();  // raw_data[2].col(i);  // linear accelaration data, hardcoded
      formatted_matrix.col(17+j).setZero();  // angular accelaration, hardcoded
   }
   else {  // only for quat
     formatted_matrix.col(7+j) = raw_data[2].col(j+1); // orientation (quat), hardcoded
   }
  }

  return formatted_matrix;
}  // end load_dat_chaser

#endif // TRAJ_UTILS_H_
