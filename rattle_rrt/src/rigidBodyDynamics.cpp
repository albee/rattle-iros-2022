/*Copyright (c) 2007 Jernej Barbic

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
*/

#include "rbd/rigidBodyDynamics.h"

RigidBodyDynamics::RigidBodyDynamics(double mass, double inertiaTensorAtRestX,
        double inertiaTensorAtRestY, double inertiaTensorAtRestZ) {
  this->mass = mass;
  this->inertiaTensorAtRestX = inertiaTensorAtRestX;
  this->inertiaTensorAtRestY = inertiaTensorAtRestY;
  this->inertiaTensorAtRestZ = inertiaTensorAtRestZ;

  inverseInertiaTensorAtRestX = 1.0 / inertiaTensorAtRestX;
  inverseInertiaTensorAtRestY = 1.0 / inertiaTensorAtRestY;
  inverseInertiaTensorAtRestZ = 1.0 / inertiaTensorAtRestZ;

  ResetBodyToRest();
  ResetExternalForce();
  ResetExternalTorque();
}


// performs one step of Euler integration
// Note: this contains the translation dynamics and calls the dynamics for angular velocity and attitude
void RigidBodyDynamics::EulerStep(double timestep) {
  // 3dof part
  positionX += timestep * velocityX;
  positionY += timestep * velocityY;
  positionZ += timestep * velocityZ;

  velocityX += timestep * externalForceX / mass;
  velocityY += timestep * externalForceY / mass;
  velocityZ += timestep * externalForceZ / mass;

  // 6dof part
  // update the quaternion rotation
  q = q + (0.5 * timestep) * Quaternion<double>(0, angularVelocityX, angularVelocityY, angularVelocityZ) * q;
  // re-normalize the quaternion
  q.Normalize();

  // update angularMomentum
  angularMomentumX += timestep * externalTorqueX;
  angularMomentumY += timestep * externalTorqueY;
  angularMomentumZ += timestep * externalTorqueZ;

  // reconstruct the rotation matrix
  q.Quaternion2Matrix(R);

  // reconstruct angular velocity
  ComputeAngularVelocity();
}

// computes the inertia tensor, using the current rotation matrix
// assumes diagonal inertia tensor at rest
// 48 flops
void RigidBodyDynamics::ComputeInertiaTensor(double inertiaTensor[9]) {
  // inertiaTensor = R * diag(inertiaTensorAtRestX, inertiaTensorAtRestY, inertiaTensorAtRestZ) * R^T

  inertiaTensor[0] = R[0] * inertiaTensorAtRestX * R[0] + R[1] * inertiaTensorAtRestY * R[1]
    + R[2] * inertiaTensorAtRestZ * R[2];
  inertiaTensor[1] = R[3] * inertiaTensorAtRestX * R[0] + R[4] * inertiaTensorAtRestY * R[1]
    + R[5] * inertiaTensorAtRestZ * R[2];
  inertiaTensor[2] = R[6] * inertiaTensorAtRestX * R[0] + R[7] * inertiaTensorAtRestY * R[1]
    + R[8] * inertiaTensorAtRestZ * R[2];

  inertiaTensor[4] = R[3] * inertiaTensorAtRestX * R[3] + R[4] * inertiaTensorAtRestY * R[4]
    + R[5] * inertiaTensorAtRestZ * R[5];
  inertiaTensor[5] = R[6] * inertiaTensorAtRestX * R[3] + R[7] * inertiaTensorAtRestY * R[4]
    + R[8] * inertiaTensorAtRestZ * R[5];

  inertiaTensor[8] = R[6] * inertiaTensorAtRestX * R[6] + R[7] * inertiaTensorAtRestY * R[7]
    + R[8] * inertiaTensorAtRestZ * R[8];

  // symmetric
  inertiaTensor[3] = inertiaTensor[1];
  inertiaTensor[6] = inertiaTensor[2];
  inertiaTensor[7] = inertiaTensor[5];
}

// computes the angular velocity from the angular momentum
// using the current rotation matrix
// inertia tensor at rest is diagonal
// 33 flops
// w = R * I^{body}^{-1} * R^T * L
void RigidBodyDynamics::ComputeAngularVelocity() {
  double temp0, temp1, temp2;

  // temp = R^T * L
  temp0 = R[0] * angularMomentumX + R[3] * angularMomentumY + R[6] * angularMomentumZ;
  temp1 = R[1] * angularMomentumX + R[4] * angularMomentumY + R[7] * angularMomentumZ;
  temp2 = R[2] * angularMomentumX + R[5] * angularMomentumY + R[8] * angularMomentumZ;

  // temp = I^{body}^{-1} * temp = diag(invIBodyX, invIBodyY, invIBodyZ) * temp;
  temp0 = inverseInertiaTensorAtRestX * temp0;
  temp1 = inverseInertiaTensorAtRestY * temp1;
  temp2 = inverseInertiaTensorAtRestZ * temp2;

  // angularVelocity = R * temp
  angularVelocityX = R[0] * temp0 + R[1] * temp1 + R[2] * temp2;
  angularVelocityY = R[3] * temp0 + R[4] * temp1 + R[5] * temp2;
  angularVelocityZ = R[6] * temp0 + R[7] * temp1 + R[8] * temp2;
}

// inverts a 3x3 matrix
void RigidBodyDynamics::Invert3x3Matrix(double * A, double * AInv) {
  // det(A)
  double invDeterminant = 1.0 /
           (-A[2] * A[4] * A[6] +
             A[1] * A[5] * A[6] +
             A[2] * A[3] * A[7] -
       A[0] * A[5] * A[7] -
       A[1] * A[3] * A[8] +
       A[0] * A[4] * A[8]);

  AInv[0] = invDeterminant * (-A[5] * A[7] + A[4] * A[8]);
  AInv[1] = invDeterminant * (A[2] * A[7] - A[1] * A[8]);
  AInv[2] = invDeterminant * (-A[2] * A[4] + A[1] * A[5]);
  AInv[3] = invDeterminant * (A[5] * A[6] - A[3] * A[8]);
  AInv[4] = invDeterminant * (-A[2] * A[6] + A[0] * A[8]);
  AInv[5] = invDeterminant * (A[2] * A[3] - A[0] * A[5]);
  AInv[6] = invDeterminant * (-A[4] * A[6] + A[3] * A[7]);
  AInv[7] = invDeterminant * (A[1] * A[6] - A[0] * A[7]);
  AInv[8] = invDeterminant * (-A[1] * A[3] + A[0] * A[4]);
}

