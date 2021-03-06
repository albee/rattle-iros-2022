#ifndef RIGIDBODYDYNAMICS_H_
#define RIGIDBODYDYNAMICS_H_

/*
Copyright (c) 2007 Jernej Barbic

Permission is hereby granted, free of charge, to any person
obtaining a copy of this software and associated documentation
files (the "Software"), to deal in the Software without
restriction, including without limitation the rights to use,
copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the
Software is furnished to do so, subject to the following
conditions:

The above copyright notice and this permission notice shall be
included in all copies or substantial portions of the Software.

Except as contained in this notice, the name(s) of the above 
copyright holders shall not be used in advertising or otherwise 
to promote the sale, use or other dealings in this Software 
without prior written authorization.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
OTHER DEALINGS IN THE SOFTWARE.

============================================

Rigid body dynamics
Jernej Barbic, CMU, 2003-2007
Version 1.0

My contact information (email):
my_first_name.my_last_name@gmail.com
where you need to replace my_first_name with jernej
and my_last_name with barbic
(these are anti-spam measures)
Don't forget the . in between my first name and last name.

Classes 'RigidBodyDynamics' and 'RigidBodyDynamics_GeneralTensor' 
implement 6-DOF rigid dynamics of a single rigid body, as explained,
for example, in:
   
David Baraff:
"An Introduction to Physically Based Modeling:
 Rigid Body Simulation I: Unconstrained Rigid Body Dynamics"
(SIGGRAPH 97 Course Notes)

In other words, these two classes allow you to simulate the 
motion of a single rigid body, under any specified (potentially
time-varying) external forces and torques. Arbitrary tensors 
of inertia are supported.  The solution is computed by numerically 
timestepping the ordinary differential equations of rigid body 
motion, derived from the Newton's 2nd law, and conservation of 
linear momentum and angular momentum.

For example, ballistic motion can be simulated
if gravity is used as the external force.
Objects bouncing off the ground/impacting other objects
can be simulated if you combine
'RigidBodyDynamics' (or 'RigidBodyDynamics_GeneralTensor')
with a collision detection algorithm that provides
the contact external forces. This is most easily done
when the contact force are penalty-based contact forces.
Impulse simulations are not directly supported; however
the API does allow you to instantenously change 
linear velocity/angular velocity, provided you compute
the velocity changes yourself.

Why are there two classes 
('RigidBodyDynamics' and 'RigidBodyDynamics_GeneralTensor')? 
============================================================

Use 'RigidBodyDynamics' class if the inertia tensor around the 
center of mass, **in the rest** configuration, with respect to the 
world coordinate axes, is diagonal. This doesn't mean that the inertia 
tensor around the center of mass will be diagonal for arbitrary object 
rotations, the assumption is only that the tensor **at rest** be diagonal. 
  
Use 'RigidBodyDynamics_GeneralTensor' class for a general 3x3 matrix 
inertia tensor in the rest configuration.

Both 'RigidBodyDynamics' and 'RigidBodyDynamics_GeneralTensor' correctly treat 
the inertia tensor as non-diagonal for non-identity rotations. 

The reason for making 'RigidBodyDynamics' a separate class is that
time-stepping is slightly faster for 'RigidBodyDynamics' since the 
internal update of the current inertia tensor can be made simpler.


How is the position/orientation of a rigid body defined in this class?
======================================================================

The current configuration of the rigid body is specified by the
current world coordinate position of the center of mass 
(this can be retrieved/set via the 'Position' vector),
and by specifying the current rotation of the object around the center of mass
(this can be retrieved/set via the 'Rotation' matrix).
One way to visualize this is to first position the object's center of mass to
position 'Position', then rotate the object around the center of mass
by the rotation matrix 'Rotation'.

Here is an alternative explanation:
Suppose the body is initially positioned such that the center of mass
coincides with the origin of the world coordinate system and suppose
the location of some material point on the object is given by a
3-dim vector X, in the world coordinate system. 
Then, the current location of this material point, again in the world
coordinate system, equals: 
Position + Rotation * X,
where Position is the current location of the center of mass, and
Rotation is the current rotation of the object around its center of mass.

The routines in this class timestep Position and Rotation according
to your provided external forces, using the explicit Euler integrator.
They do not provide any code to
actually display the rigid body on the screen. For that, you need
to use a 3D graphics library, such as OpenGL.

How to use the classes?
=======================

You must:
1. Specify object rigid body parameters (in the constructor): 
  1.1. mass, and
  1.2. inertia tensor in the rest configuration. This tensor is a 3x3 matrix and must be given with respect to the world-coordinate axes, and around the center of mass.
2. Provide external forces and torques at every timestep (these forces
and torques are integrated under the assumption that they are constant during each timestep)
3. Call the explicit Euler integrator routine to time-step the dynamics to the next time step

At any timestep, you can query object position/rotation, velocity/angular velocity, linear/angular momentum, current inertia tensor (with respect to world-coordinate axes), etc.

See also example.cpp.

Misc
====

All matrices are stored in row-major format:

    [ A[0] A[1] A[2] ]
A = [ A[3] A[4] A[5] ]
    [ A[6] A[7] A[8] ]


Dependencies: the Quaternion C++ class (another class by Jernej Barbic)

Things to do: 
Implement a higher order integrator, such as Runge-Kutta 2nd order or 4th order.
Add support for impulses.

*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "./quaternion.h"

class RigidBodyDynamics {
 public:
  // inertia tensor parameter refers to the following coordinate system (and with body's rotation = identity)
  //   origin: center of mass
  //   axes: aligned with world-coordinate-system axes
  // constructor will set position and rotation to zero/identity, use SetPosition/Rotation to place the object
  //  wherever you want
  // assumes diagonal inertia tensor at rest (and given by diag(inertiaTensorAtRestX,inertiaTensorAtRestY,
  // inertiaTensorAtRestZ))
  RigidBodyDynamics() = default;
  RigidBodyDynamics(double mass, double inertiaTensorAtRestX, double inertiaTensorAtRestY, double inertiaTensorAtRestZ);

  // === query current state ===
  inline void GetPosition(double * x, double * y, double * z);
  // retrieve current world-coordinate position of a point which has world-coordinates (X,Y,Z) when center of mass
  // is at the world-coordinate origin, and rotation is identity
  inline void GetPointPosition(double X, double Y, double Z, double * x, double * y, double * z);
  inline void GetRotation(double * R);
  inline Quaternion<double> GetQuaternion();  // returns the quaternion corresponding to the current rotation
  inline void GetVelocity(double * vx, double * vy, double * vz);
  inline void GetAngularVelocity(double * wx, double * wy, double * wz);  // magnitude is in radians / sec
  inline void GetAngularMomentum(double * amx, double * amy, double * amz);
  inline void GetInertiaTensor(double * IT);  // returns the current inertia tensor (row-major 3x3 matrix)
  inline void get_quaternion(double* x_, double* y_, double* z_, double* w_);

  // === set current state ===

  // NOTE: all of the following routines of course also update the internal simulation state as necessary,
  //  e.g. if you change angular velocity, angular momentum will also be updated
  inline void SetPosition(double x, double y, double z);  // updates the position of the center of mass
  // updates the position of the center of mass, such that point with material coordinates (X,Y,Z)
  // (in object's coordinate system) ends up at world-coordinate position (posx,posy,posz)
  inline void SetPosition(double X, double Y, double Z, double posx, double posy, double posz);
  inline void SetRotation(double R[9]);  // updates the rotation
  inline void SetRotation(Quaternion<double> q);  // updates the rotation
  inline void SetVelocity(double vx, double vy, double vz);
  inline void SetAngularVelocity(double wx, double wy, double wz);  // magnitude is in radians / sec
  inline void ResetBodyToRest();  // resets position to the origin, rotation to identity, and linear and angular
  // velocities to zero

  // === prescribe external forces/torques ===

  // all the forces and torques remain active (for all subsequent timesteps) until explicitly changed; use
  // ResetWrenches() to reset to zero or Set* to set to a new value
  // the torques refer to the coordinate system where the origin is at the center of mass, and where the axes are
  // aligned with the world coordinate axes
  // (and not with the body's axes!!)
  // you need to set the forces and torques separately; setting the forces doesn't set any torques
  inline void ResetExternalForce();  // does not reset torques
  inline void SetExternalForce(double fx, double fy, double fz);  // sets the current total external force
  inline void AddExternalForce(double fx, double fy, double fz);  // force is added to whatever force already
  // previously specified
  inline void ResetExternalTorque();  // does not reset forces
  inline void SetExternalTorque(double tx, double ty, double tz);  // sets the current total external torque
  inline void AddExternalTorque(double tx, double ty, double tz);  // torque is added to whatever torques already
  // previously specified
  inline void ResetWrenches() { ResetExternalForce(); ResetExternalTorque(); }

  // this helper function computes torque arising from a single force f acting at world-coordinate position forcePos
  // NOTE: you must still call one of the above functions to actually set the torque,  i.e. call
  // (Set/Add)ExternalTorque(tx,ty,tz) after computing the torque via ComputeTorque
  inline void ComputeTorque(double forcePosx, double forcePosy, double forcePosz,
    double fx, double fy, double fz,
    double * tx, double * ty, double * tz);

  // upVector must be normalized
  inline void AddGravitationalForce(
    double g = 9.81, double upVectorx = 0.0, double upVectory = 1.0, double upVectorz = 0.0);

  // === timestepping functions ===

  // performs one step of Euler integration, i.e. advances solution
  // from t to t + timestep
  void EulerStep(double timestep);

 protected:
  // computes the inertia tensor (given the current rotation matrix)
  virtual void ComputeInertiaTensor(double intertiaTensor[9]);

  // computes the angular velocity from the angular momentum
  // via equation w = (R * I_body^{-1} * R^T) L
  // using the current rotation matrix
  virtual void ComputeAngularVelocity();

  // computes angular momentum from the angular velocity, using the equation L = I * w
  // assumes that I and w have been pre-set to the correct values
  void ComputeAngularMomentum();

  double positionX, positionY, positionZ;  // world-coordinate position of center
  double velocityX, velocityY, velocityZ;  // world-coordinate position of center

  double R[9];  // orientation matrix

  double mass;

  double externalForceX, externalForceY, externalForceZ;
  double externalTorqueX, externalTorqueY, externalTorqueZ;

  double angularVelocityX, angularVelocityY, angularVelocityZ;
  double angularMomentumX, angularMomentumY, angularMomentumZ;

  Quaternion<double> q;

  // inertia tensor are with respect to the world coordinate system
  // the diagonal components of inertiaTensorAtRest
  double inertiaTensorAtRestX, inertiaTensorAtRestY, inertiaTensorAtRestZ;
  // the diagonal components of inverseInertiaTensorAtRest
  double inverseInertiaTensorAtRestX, inverseInertiaTensorAtRestY, inverseInertiaTensorAtRestZ;
  void Invert3x3Matrix(double * A, double * AInv);
};

/************* INLINED FUNCTIONS **********************************/
inline void RigidBodyDynamics::get_quaternion(double* x_, double* y_, double* z_, double* w_) {
  q.get_quaternion(x_, y_, z_, w_);
}

inline void RigidBodyDynamics::ResetExternalForce() {
  externalForceX = externalForceY = externalForceZ = 0;
}

inline void RigidBodyDynamics::ResetExternalTorque() {
  externalTorqueX = externalTorqueY = externalTorqueZ = 0;
}


inline void RigidBodyDynamics::SetPosition(double x, double y, double z) {
  positionX = x;
  positionY = y;
  positionZ = z;
}


inline void RigidBodyDynamics::SetRotation(double R[9]) {
  memcpy(this->R, R, sizeof(double)*9);
  // update the quaternion
  q = Quaternion<double>::Matrix2Quaternion(R);
}

inline void RigidBodyDynamics::SetRotation(Quaternion<double> q) {
  this->q = q;
  q.Quaternion2Matrix(R);
}


inline void RigidBodyDynamics::SetExternalForce(double fx, double fy, double fz) {
  externalForceX = fx;
  externalForceY = fy;
  externalForceZ = fz;
}

inline void RigidBodyDynamics::AddExternalForce(double fx, double fy, double fz) {
  externalForceX += fx;
  externalForceY += fy;
  externalForceZ += fz;
}

inline void RigidBodyDynamics::SetExternalTorque(double tx, double ty, double tz) {
  externalTorqueX = tx;
  externalTorqueY = ty;
  externalTorqueZ = tz;
}

inline void RigidBodyDynamics::AddExternalTorque(double tx, double ty, double tz) {
  externalTorqueX += tx;
  externalTorqueY += ty;
  externalTorqueZ += tz;
}


inline void RigidBodyDynamics::ComputeTorque
  (double forcePosx, double forcePosy, double forcePosz,
  double fx, double fy, double fz,
  double * tx, double * ty, double * tz) {
  double rx, ry, rz;

  // compute the torque handle
  rx = forcePosx - positionX;
  ry = forcePosy - positionY;
  rz = forcePosz - positionZ;

  // t = r x f
  *tx = ry * fz - fy * rz;
  *ty = fx * rz - rx * fz;
  *tz = rx * fy - fx * ry;
}

inline void RigidBodyDynamics::AddGravitationalForce(double g, double upVectorx, double upVectory, double upVectorz) {
  double force = mass * g;

  externalForceX -= upVectorx * force;
  externalForceY -= upVectory * force;
  externalForceZ -= upVectorz * force;
}

// computes angular momentum, using the equation L = I * w
// assumes that I and w have been pre-set to the correct values
inline void RigidBodyDynamics::ComputeAngularMomentum() {
  double inertiaTensor[9];
  ComputeInertiaTensor(inertiaTensor);
  angularMomentumX = inertiaTensor[0] * angularVelocityX + inertiaTensor[1] * angularVelocityY
  + inertiaTensor[2] * angularVelocityZ;
  angularMomentumY = inertiaTensor[3] * angularVelocityX + inertiaTensor[4] * angularVelocityY
  + inertiaTensor[5] * angularVelocityZ;
  angularMomentumZ = inertiaTensor[6] * angularVelocityX + inertiaTensor[7] * angularVelocityY
  + inertiaTensor[8] * angularVelocityZ;
}

inline void RigidBodyDynamics::SetVelocity(double vx, double vy, double vz) {
  velocityX = vx;
  velocityY = vy;
  velocityZ = vz;
}

inline void RigidBodyDynamics::SetAngularVelocity(double wx, double wy, double wz) {
  angularVelocityX = wx;
  angularVelocityY = wy;
  angularVelocityZ = wz;
  ComputeAngularMomentum();
}

inline void RigidBodyDynamics::GetAngularVelocity(double * wx, double * wy, double * wz) {
  *wx = angularVelocityX;
  *wy = angularVelocityY;
  *wz = angularVelocityZ;
}

inline void RigidBodyDynamics::GetAngularMomentum(double * amx, double * amy, double * amz) {
  *amx = angularMomentumX;
  *amy = angularMomentumY;
  *amz = angularMomentumZ;
}


inline void RigidBodyDynamics::GetPosition(double * x, double * y, double * z) {
  *x = positionX;
  *y = positionY;
  *z = positionZ;
}

inline void RigidBodyDynamics::GetPointPosition(double X, double Y, double Z, double * x, double * y, double * z) {
  *x = positionX + R[0] * X + R[1] * Y + R[2] * Z;
  *y = positionY + R[3] * X + R[4] * Y + R[5] * Z;
  *z = positionZ + R[6] * X + R[7] * Y + R[8] * Z;
}

inline void RigidBodyDynamics::GetRotation(double * R_copy) {
  R_copy[0] = R[0];
  R_copy[1] = R[1];
  R_copy[2] = R[2];
  R_copy[3] = R[3];
  R_copy[4] = R[4];
  R_copy[5] = R[5];
  R_copy[6] = R[6];
  R_copy[7] = R[7];
  R_copy[8] = R[8];
}

inline void RigidBodyDynamics::GetVelocity(double * vx, double * vy, double * vz) {
  *vx = velocityX;
  *vy = velocityY;
  *vz = velocityZ;
}

inline Quaternion<double> RigidBodyDynamics::GetQuaternion() {  // returns the current rotation quaternion
  return q;
}


inline void RigidBodyDynamics::SetPosition(double X, double Y, double Z, double posx, double posy, double posz) {
  // (posx, posy, posz) = (x, y, z) + R (X, Y, Z)
  // (x, y, z) = (posx, posy, posz) - R (X, Y, Z)
  double x, y, z;
  x = posx - R[0] * X - R[1] * Y - R[2] * Z;
  y = posy - R[3] * X - R[4] * Y - R[5] * Z;
  z = posz - R[6] * X - R[7] * Y - R[8] * Z;

  SetPosition(x, y, z);
}

inline void RigidBodyDynamics::GetInertiaTensor(double* IT) {
  ComputeInertiaTensor(IT);
}

inline void RigidBodyDynamics::ResetBodyToRest() {
  positionX = positionY = positionZ = 0;
  velocityX = velocityY = velocityZ = 0;

  memset(R, 0, sizeof(double)*9);
  R[0] = 1; R[4] = 1; R[8] = 1;

  q = Quaternion<double>(1);

  angularVelocityX = angularVelocityY = angularVelocityZ = 0;
  angularMomentumX = angularMomentumY = angularMomentumZ = 0;
}

#endif  // RIGIDBODYDYNAMICS_H_
