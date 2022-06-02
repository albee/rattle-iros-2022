/* 
# obstacle.h

Ellipsoidal obstacle representation, used by RRT for collision avoidance.

Keenan Albee, 2021
MIT Space Systems Lab
*/
#include <iostream>
#include <btBulletCollisionCommon.h>

struct BinaryCollisionCallback : public btCollisionWorld::ContactResultCallback {
  /* Credit to Abhi Cauligi, Stanford ASL for this interface
  */
  bool is_collision;

  BinaryCollisionCallback() : btCollisionWorld::ContactResultCallback(), is_collision(false) {}

  virtual btScalar addSingleResult(btManifoldPoint& cp,
    const btCollisionObjectWrapper* colObj0Wrap, int partId0, int index0,
    const btCollisionObjectWrapper* colObj1Wrap, int partId1, int index1) {
    is_collision = true;
    return 0;
  }
};

struct Obstacle  
/* An ellipsoidal obstacle
*/
{  
  float a;  // "radius", i.e., semi-major axis length
  float b;
  float c;
  float xc;  // centroid location
  float yc;
  float zc;
  Eigen::Matrix3d P;
  double astrobee_radius_ = 0.2;  // Astrobee radius in meters (for collision checking)

  Obstacle(float a_in, float b_in, float c_in, float xc_in, float yc_in, float zc_in){
    a = a_in; b = b_in; c = c_in;
    xc = xc_in; yc = yc_in; zc = zc_in;
    P << 1/pow(a, 2.0), 0.0,           0.0,
          0.0,           1/pow(b, 2.0), 0.0,
          0.0,           0.0,           1/pow(c, 2.0);
  }   

  bool is_in_collision(Eigen::Vector3d pos) const {
    /* Return true if the state is in collision for the given obstacles. 3D.

    Uses the bullet3 library's collision detection features. Feels like attaching
    a rocket to a slug...
    */
    bool in_collision = false;
    // Eigen::Vector3d x_err = pos - Eigen::Vector3d(xc, yc, zc);
    // if ( (x_err.transpose() )*P*x_err < 1.0) {
    //   in_collision = true;
    // }

    // All the bullet machinery we need
    btCollisionConfiguration* bt_collision_configuration;
    btCollisionDispatcher* bt_dispatcher;
    btBroadphaseInterface* bt_broadphase;
    btCollisionWorld* bt_collision_world;
    double scene_size = 2;
    unsigned int max_objects = 2;

    bt_collision_configuration = new btDefaultCollisionConfiguration();
    bt_dispatcher = new btCollisionDispatcher(bt_collision_configuration);

    btScalar sscene_size = (btScalar) scene_size;
    btVector3 worldAabbMin(-sscene_size, -sscene_size, -sscene_size);
    btVector3 worldAabbMax(sscene_size, sscene_size, sscene_size);

    bt_broadphase = new bt32BitAxisSweep3(worldAabbMin, worldAabbMax, max_objects, 0, true);

    bt_collision_world = new btCollisionWorld(bt_dispatcher, bt_broadphase, bt_collision_configuration);

    // Create two collision objects
    btCollisionObject* obstacle = new btCollisionObject();
    btCollisionObject* robot = new btCollisionObject();

    // Move each to a specific location
    obstacle->getWorldTransform().setOrigin(btVector3((btScalar) xc, (btScalar) yc, (btScalar) zc));
    robot->getWorldTransform().setOrigin(btVector3((btScalar) pos(0), (btScalar) pos(1), (btScalar) pos(2)));
    // std::cout << xc << " " << yc << " " << zc << " " << pos << std::endl;

    //Create an ellipsoid of the proper size
    btSphereShape* sphere_shape = new btSphereShape(astrobee_radius_);  // 0.2
    //Set the shape of each collision object
    obstacle->setCollisionShape(sphere_shape);
    robot->setCollisionShape(sphere_shape);

    //Add the collision objects to our collision world
    bt_collision_world->addCollisionObject(obstacle);
    bt_collision_world->addCollisionObject(robot);

    // Check for collision between the two objects
    BinaryCollisionCallback result; // wrapper to store the collision check result
    bt_collision_world->contactPairTest(obstacle, robot, result);
    
    in_collision = result.is_collision;
    std::cout << "Collision?: " << in_collision << std::endl;

    return in_collision;

    // bt_collision_world->performDiscreteCollisionDetection();

    // int numManifolds = bt_collision_world->getDispatcher()->getNumManifolds();
    // std::cout << "3d obs check: " << numManifolds << std::endl;

    // for (int i = 0; i < numManifolds; i++) {
    //   btPersistentManifold* contactManifold = bt_collision_world->getDispatcher()->getManifoldByIndexInternal(i);
    //   const btCollisionObject* obA = contactManifold->getBody0();
    //   const btCollisionObject* obB = contactManifold->getBody1();
    //   contactManifold->refreshContactPoints(obA->getWorldTransform(), obB->getWorldTransform());
    //   int numContacts = contactManifold->getNumContacts();
    //   if (numContacts > 0){
    //     in_collision = true;
    //   }
    //   std::cout << in_collision << std::endl;
    // }

    // return in_collision;
  }

  /*
  Return true if the state is in collision for the given obstacles. 2D
  */
  bool is_in_collision(Eigen::Vector2d pos) const {
    bool in_collision = false;
    Eigen::Vector2d pos2;
    pos2 << pos;
    Eigen::Vector2d x_err = pos2 - Eigen::Vector2d(xc, yc);

    if ( (x_err.transpose() )*P.block<2, 2>(0, 0)*x_err < 1.0) {  // only check x and y components
      in_collision = true;
    }
          // std::cout << in_collision << std::endl;
    return in_collision;
  }

  friend std::ostream& operator<<(std::ostream& os, const Obstacle& obs) {
    /* Print obstacle.
    */
    return os << "[" << obs.a << " " << obs.b << " " << obs.c << " " << obs.xc << " " << obs.yc << " " << obs.zc << "]" << std::endl;
  }
}; 