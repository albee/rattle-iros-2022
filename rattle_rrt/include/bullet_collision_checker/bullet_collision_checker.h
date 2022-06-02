/* A collision-checking module, using bullet.

Credit to Abhi Cauligi of Stanford ASL.
*/

#ifndef SCP_BULLET_COLLISION_CHECKER_H_
#define SCP_BULLET_COLLISION_CHECKER_H_

#include <btBulletDynamicsCommon.h>
#include <boost/math/constants/constants.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <vector>
#include <string>

#include "btBulletCollisionCommon.h"
// #include "BulletCollision/CollisionShapes/btConvex2dShape.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkPairDetector.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.h"
#include "BulletCollision/NarrowPhaseCollision/btPointCollector.h"
// #include "BulletDynamics/Featherstone/btMultiBody.h"
// #include "LinearMath/btConvexHull.h"
#include "bullet_collision_checker/types.h"

namespace collision_checker {

struct SignedDistanceResult {
  scp::decimal_t sd;
  scp::Vec3 co1_pt;
  scp::Vec3 co2_pt;
};

class BulletCollisionChecker {
  btCollisionWorld* m_world;
  btBroadphaseInterface* m_broadphase;
  btCollisionDispatcher* m_dispatcher;
  btCollisionConfiguration* m_coll_config;
  btCollisionObject* robot;  // the robot, against which collision checks occur

  std::vector<btCollisionObject*> convex_robot_components;
  std::vector<btCollisionObject*> convex_env_components;

  double m_contactDistance;

 public:
    BulletCollisionChecker();
    ~BulletCollisionChecker();

    unsigned int max_objects = 100;

    void SetContactDistance(float distance);
    double GetContactDistance() {return m_contactDistance;}

    void ComputeDistance(size_t env_idx, SignedDistanceResult& out);

    void SetTransformation(btCollisionObject* o, scp::Vec3 v, scp::Quat q);
    void SetTransformation(btCollisionObject* o, scp::Vec3 v);

    std::vector<btCollisionObject*> GetConvexComponents(btCollisionShape* cs);
    std::vector<btCollisionObject*> GetConvexComponents(btCollisionObject* co);
    std::vector<btCollisionObject*> GetConvexComponents(btCollisionShape* cs, btTransform tr);

    void AddRobot(scp::Vec3 a=scp::Vec3{0.3, 0.3, 0.3},
                  scp::Vec3 r=scp::Vec3{0.0, 0.0, 0.0},
                  scp::Quat q=scp::Quat{0.0, 0.0, 0.0, 1.0});
    void AddObstacle(scp::Vec3 a, scp::Vec3 r, scp::Quat q);

    bool IsFreeState(scp::Vec3 v, scp::Quat q);
    bool IsFreeMotion(scp::Vec3 v, scp::Vec3 w);
    bool IsFreeMotion(scp::Vec3 v, scp::Quat vrot, scp::Vec3 w, scp::Quat wrot);

    void SetMargin(btCollisionObject* obj, scp::decimal_t dx);
    void SetMargin(btCollisionShape* cs, scp::decimal_t dx);

 private:
    void ComputeDistance(const btCollisionShape* cs1, const btTransform& tr1,
        const btCollisionShape* cs2, const btTransform& tr2,
        btScalar* result, btScalar max_d2);
};
}  // namespace collision_checker
#endif  // SCP_BULLET_COLLISION_CHECKER_H_