#include <eigen3/Eigen/Dense>
#include <rattle_rrt/state_space.tpp>
#include <rattle_rrt/state_space_6d_iss.h>
#include <rattle_rrt/types.h>
#include <rattle_rrt/tree.tpp>

#include <rbd/rigidBodyDynamics.h>

#include <memory>
#include <vector>

int main() {
  // (1) Bullet collision checking unit test
  collision_checker::BulletCollisionChecker bullet_ = collision_checker::BulletCollisionChecker();  // collision-checking module

  std::vector<Obstacle> obstacles;
  obstacles.push_back(Obstacle(1.0, 1.0, 1.0, 10.0, 10.0, 10.0));
  // obstacles.push_back(Obstacle(0.05, 0.8, 0.1, 0.3, -0.35, 0.0));

  for (Obstacle obs : obstacles){
    bullet_.AddObstacle(rrt::Vec3{obs.a, obs.b, obs.c}, rrt::Vec3{obs.xc, obs.yc, obs.zc}, Eigen::Quaternion<double>{0.0, 0.0, 0.0, 1.0});
    std::cout << "Obstacle: " << obs << std::endl;
  }

  // Check the robot at various points
  rrt::Vec3 pt{10.0, 10.0, 10.0};
  std::cout << "setup: " << pt(0) << " " << pt(1) << " " << pt(2) << std::endl;
  // Obstacle checking (3D) ellipsoid-on-ellipsoid
  if (!bullet_.IsFreeState(rrt::Vec3{pt(0), pt(1), pt(2)}, Eigen::Quaternion<double>{0.0, 0.0, 0.0, 1.0}) ) {
    std::cout << "collision!" << std::endl;
  }

  // z
  pt = rrt::Vec3{9.0, 0.1, 0.0};
  std::cout << "setup: " << pt(0) << " " << pt(1) << " " << pt(2) << std::endl;
  // Obstacle checking (3D) ellipsoid-on-ellipsoid
  if (!bullet_.IsFreeState(rrt::Vec3{pt(0), pt(1), pt(2)}, Eigen::Quaternion<double>{0.0, 0.0, 0.0, 1.0}) ) {
    std::cout << "collision!" << std::endl;
  }

  pt = rrt::Vec3{0.0, 0.0, 0.14};
  std::cout << "setup: " << pt(0) << " " << pt(1) << " " << pt(2) << std::endl;
  // Obstacle checking (3D) ellipsoid-on-ellipsoid
  if (!bullet_.IsFreeState(rrt::Vec3{pt(0), pt(1), pt(2)}, Eigen::Quaternion<double>{0.0, 0.0, 0.0, 1.0}) ) {
    std::cout << "collision!" << std::endl;
  }

  pt = rrt::Vec3{0.0, 0.0, 1.16};
  std::cout << "setup: " << pt(0) << " " << pt(1) << " " << pt(2) << std::endl;
  // Obstacle checking (3D) ellipsoid-on-ellipsoid
  if (!bullet_.IsFreeState(rrt::Vec3{pt(0), pt(1), pt(2)}, Eigen::Quaternion<double>{0.0, 0.0, 0.0, 1.0}) ) {
    std::cout << "collision!" << std::endl;
  }

  pt = rrt::Vec3{0.0, 0.0, 1.14};
  std::cout << "setup: " << pt(0) << " " << pt(1) << " " << pt(2) << std::endl;
  // Obstacle checking (3D) ellipsoid-on-ellipsoid
  if (!bullet_.IsFreeState(rrt::Vec3{pt(0), pt(1), pt(2)}, Eigen::Quaternion<double>{0.0, 0.0, 0.0, 1.0}) ) {
    std::cout << "collision!" << std::endl;
  }

  // y
  pt = rrt::Vec3{0.0, 0.66, 0.0};
  std::cout << "setup: " << pt(0) << " " << pt(1) << " " << pt(2) << std::endl;
  // Obstacle checking (3D) ellipsoid-on-ellipsoid
  if (!bullet_.IsFreeState(rrt::Vec3{pt(0), pt(1), pt(2)}, Eigen::Quaternion<double>{0.0, 0.0, 0.0, 1.0}) ) {
    std::cout << "collision!" << std::endl;
  }

  pt = rrt::Vec3{0.0, 0.64, 0.0};
  std::cout << "setup: " << pt(0) << " " << pt(1) << " " << pt(2) << std::endl;
  // Obstacle checking (3D) ellipsoid-on-ellipsoid
  if (!bullet_.IsFreeState(rrt::Vec3{pt(0), pt(1), pt(2)}, Eigen::Quaternion<double>{0.0, 0.0, 0.0, 1.0}) ) {
    std::cout << "collision!" << std::endl;
  }

  return 0;
}