#!/usr/bin/env python3
from __future__ import division
from autograd import make_jvp
from autograd import jacobian
import autograd.numpy as np
import timeit
from autograd.differential_operators import make_jvp_reversemode

def f(x):
   return np.dot(A, x)


dt = 0.1
# # params = np.array([18.9715, 0.0, 0.0, 0.2517])
u = np.array([1.0, 1.0, 1.0, 0.1, 0.1, 0.1])
x = np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
#
params = np.array([18.9715, 0.0, 0.0, 0.0, 0.2517, 0.2517, 0.2517, 0.0, 0.0, 0.0])

# types of dynamics: 3DoF, simplified 3DoF, simplified 6DoF,


# u = np.array([1.0, 1.0, 0.1])
# x = np.array([1.0, 1.0, 1.0])
def forward_dyn_3DoF(params):  # Planar Astrobee dynamics
   # inertial params
   # print(simplified)
   mass = params[0]
   cx = 0.0
   cy = 0.0
   izz = params[-1]
   # states
   omega_z = x[2]
   M = np.array([[mass, 0, -mass * cy],
                 [0, mass, mass * cx],
                 [-mass * cy, mass * cx, izz + mass * (cx * cx + cy * cy)]])

   inv_M = np.linalg.inv(M)

   # this was computed symbolically as autograd does not support it
   # inv_M = np.array([[(mass*cy**2 + izz)/(izz*mass), -(cx*cy)/izz, cy/izz],
   #                          [-(cx*cy)/izz, (mass*cx**2 + izz)/(izz*mass), -cx/izz],
   #                          [cy/izz, -cx/izz, 1/izz]])

   # inv_M = np.array([[1/mass, 0, 0],
   #                  [0, 1/mass, 0],
   #                  [0, 0, 1/izz]])
   C = np.array([-omega_z * omega_z * cx * mass, -omega_z * omega_z * cy * mass, 0])
   ans = np.dot(inv_M, u - C)
   #return_ans = np.hstack([dt*ans + x, ans[0:2]])
   # print(u.shape)
   # print(C.shape)
   return ans
simplified =0
omega_z = x[2]

def forward_dyn_3DoF_super_simplified(params):  # Planar Astrobee dynamics

   mass = params[0]
   cx = params[1]
   cy = params[2]
   izz = params[-1]
   # states

   M = np.array([[mass, 0, -mass * cy],
                 [0, mass, mass * cx],
                 [-mass * cy, mass * cx, izz + mass * (cx * cx + cy * cy)]])

   inv_M = np.linalg.inv(M)

   # this was computed symbolically as autograd does not support it
   # inv_M = np.array([[(mass*cy**2 + izz)/(izz*mass), -(cx*cy)/izz, cy/izz],
   #                          [-(cx*cy)/izz, (mass*cx**2 + izz)/(izz*mass), -cx/izz],
   #                          [cy/izz, -cx/izz, 1/izz]])

   # inv_M = np.array([[1/mass, 0, 0],
   #                  [0, 1/mass, 0],
   #                  [0, 0, 1/izz]])
   C = np.array([-omega_z * omega_z * cx * mass, -omega_z * omega_z * cy * mass, 0])

   return np.dot(inv_M, u - C)








def forward_dyn_6DoF(params):
   if simplified:
      mass = params[0]
      cx = 0.0
      cy = 0.0
      cz = 0.0
      ixx = params[1]
      iyy = params[2]
      izz = params[-1]
      ixz = 0.0
      iyz = 0.0
      ixy = 0.0
   else:
      mass = params[0]
      cx = params[1]
      cy = params[2]
      cz = params[3]
      ixx = params[4]
      iyy = params[5]
      izz = params[6]
      ixy = params[7]
      ixz = params[8]
      iyz = params[9]

   omega = np.array(x[3:6])
   I_cm = np.array([[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]])
   c = np.array([cx, cy, cz])

   I_intermediate = I_cm - np.dot(mass, np.matmul(skew(c), skew(c)))
   M = np.vstack([np.hstack([np.identity(3) * mass, -mass * skew(c)]),
                  np.hstack([mass * skew(c), I_cm - mass * np.matmul(skew(c), skew(c))])])
   inv_M = np.linalg.inv(M)
   #inv_M =get_inverse_M([mass, cx, cy, cz, ixx, iyy, izz, ixy, ixz, iyz])

   C = np.hstack([np.dot(mass, np.matmul(np.matmul(skew(omega), skew(omega)), c)),
               np.matmul(skew(omega), np.matmul(I_intermediate, omega))])

   ans = np.matmul(inv_M, u - C)
   return ans

def skew(vec):
   skew_mat = np.array([[0, -vec[2], vec[1]],
                        [vec[2], 0, -vec[0]],
                        [-vec[1], vec[0], 0]])
   return skew_mat

func_name = forward_dyn_3DoF
accel = 1

def euler_forward_state_propagation(params):
   # print(x.shape)
   f1 = dt * func_name(params)
   x_kplus1 = x + f1  # velocities
   if accel:
      x_kplus1 = np.hstack([x_kplus1, func_name(params)[0:2]])
      # print(x_kplus1.shape)
   return x_kplus1





# params = np.array([18.9715, 0.01, 0.01, 0.2517])
# A = np.random.randn(2, 2)
# x = np.array([1.0, 1.0])
print(forward_dyn_6DoF(params))


#
# start = timeit.default_timer()
# jvp_f_x = make_jvp(forward_dyn_6DoF)(params)
# print(jvp_f_x(params))
# stop = timeit.default_timer()
# print('jvp Time: ', stop - start)
t = 0
for i in range(600):
   get_jac = jacobian(forward_dyn_6DoF)
   start = timeit.default_timer()
   ans = get_jac(params)
   #print(np.vstack([ans*dt, ans]))
   stop = timeit.default_timer()
   t_curr = stop - start
   t= t +( t_curr)
print(t/100)






# A = np.array([[3, 1], [1, 5]])
# params = np.array([10.0, 0.01, 0.0, 0.250])
# u = np.array([0.1, 0.1, 0.01])
# def f(params):
#    mass = params[0]
#    cx = params[1]
#    cy = params[2]
#    izz = params[3]
#    M = np.array([[mass, 0.0, -mass * cy],
#                  [0.0, mass, mass * cx],
#                  [-mass * cy, mass * cx, mass * (cx * cx + cy * cy)]])
#                  #[-mass * cy, mass * cx, izz + mass * (cx * cx + cy * cy)]]
#    print(np.dot(np.linalg.inv(M), u))
#    return np.dot(np.linalg.inv(M), u)
#
# f_jvp_fast = make_jvp_reversemode(f)(params) # This precomputes things
#
#
#
# start = timeit.default_timer()
# for i in range(60):
#    f_jvp_fast(np.array([0, 0, 0, 1]))
# stop = timeit.default_timer()
# print(stop-start)
# # f_jvp_slow = make_jvp(f)(params)
# # start = timeit.default_timer()
# # print(f_jvp_slow(np.array([1,0])))
# # stop = timeit.default_timer()
# # print(stop-start)
# grad_jax = grad(f)
# print(grad_jax(params))