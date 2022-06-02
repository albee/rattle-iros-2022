/* 
Copyright (c) 2018, Keenan Albee
Adapted from Mike Watterson.
*/

#ifndef TYPES_H_
#define TYPES_H_

#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <Eigen/Core>
#include <Eigen/Dense>

#include <vector>

/*
 * This file is to define typdefs for all the eigen types as either float or
 * double. Add flag "-DRRT_AD_USE_SINGLE_PRECISION" to compile the entire package with
 * float, otherwise, will default to double.
 */
namespace rrt {

#ifndef RRT_AD_USE_SINGLE_PRECISION
typedef double decimal_t;
#else
typedef float decimal_t;
#endif

typedef Eigen::Matrix<decimal_t, 2, 1> Vec2;
typedef Eigen::Matrix<decimal_t, 3, 1> Vec3;
typedef Eigen::Matrix<decimal_t, 4, 1> Vec4;
typedef Eigen::Matrix<decimal_t, 6, 1> Vec6;
typedef Eigen::Matrix<decimal_t, 9, 1> Vec9;
typedef Eigen::Matrix<decimal_t, 13, 1> Vec13;
typedef Eigen::Matrix<decimal_t, 17, 1> Vec17;

typedef Eigen::Matrix<decimal_t, 3, 3> Mat3;
typedef Eigen::Matrix<decimal_t, 4, 4> Mat4;

typedef Eigen::Matrix<decimal_t, Eigen::Dynamic, 1> VecD;
typedef Eigen::Matrix<decimal_t, Eigen::Dynamic, 3> MatD3;
typedef Eigen::Matrix<decimal_t, Eigen::Dynamic, Eigen::Dynamic> MatD;

typedef std::vector<VecD> VecDVec;
typedef std::vector<MatD> MatDVec;

typedef Eigen::Quaternion<decimal_t> Quat;

}  // namespace rrt

#endif  // TYPES_H_
