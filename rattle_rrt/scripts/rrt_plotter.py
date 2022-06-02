#!/usr/bin/env python

# RRT Plotter
# Keenan Albee, 10/07/20

import csv
import matplotlib.pyplot as plt
from cycler import cycler
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.collections import LineCollection
from mpl_toolkits.mplot3d.art3d import Line3DCollection
import numpy as np
import matplotlib.animation as animation
import rospkg

# GLOBALS
rospack = rospkg.RosPack()
DATA_PATH = rospack.get_path("data")+"/output/"  # output directory of CSV

RRT_OUTPUT_FILE = DATA_PATH + 'rrt_output.csv'
RRT_PATH_OUTPUT_FILE = DATA_PATH + 'rrt_path_output.csv'
# RRT_OUTPUT_FILE = './saved_runs/long_6_dof/rrt_output.csv'
# RRT_PATH_OUTPUT_FILE = './saved_runs/long_6_dof/rrt_path_output.csv'


def main():
	USE_PATH = 1

	# Plot info
	x1 = []
	y1 = []
	z1 = []
	x2 = []
	y2 = []
	z2 = []
	vx = []
	vy = []
	vz = []
	qx = []
	qy = []
	qz = []
	qth = []
	wx = []
	wy = []
	wz = []
	path_x = []
	path_y = []
	path_z = []
	path_vx = []
	path_vy = []
	path_vz = []
	path_qx = []
	path_qy = []
	path_qz = []
	path_qth = []
	path_wx = []
	path_wy = []
	path_wz = []

	# retrieve state information from the CSV
	if (USE_PATH == 1):
		get_var_path("p", path_x, path_y, path_z)
		get_var_path("v", path_vx, path_vy, path_vz)
		get_var_path("q", path_qx, path_qy, path_qz, path_qth)
		get_var_path("w", path_wx, path_wy, path_wz)
	else:
		get_var2("p",x1, y1, z1, x2, y2, z2)
		get_var("v",vx, vy, vz)
		get_var("q",qx, qy, qz, qth)
		get_var("w",wx, wy, wz)

	# Plot 2D pos
	# fig = plt.figure()
	# ax1 = fig.add_subplot(111)
	# ax1.axis('equal')
	# ax1.set_aspect('equal','box')
	# ax1.set_xlabel('x')
	# ax1.set_ylabel('y')
	# plot_p_2d(fig, x1, y1, x2, y2, path_x, path_y)
	# plot_v_2d(fig, x1, y1, x2, y2, path_x, path_y)
	# plt.show()

	# Plot 3D pos
	fig = plt.figure()
	ax1 = fig.add_subplot(111, projection='3d')
	ax1.axis('equal')
	ax1.set_aspect('equal','box')
	ax1.set_xlim3d([-1.0, 1.0])
	ax1.set_ylim3d([-1.0, 1.0])
	ax1.set_zlim3d([-1.0, 1.0])

	# Animation parameters
	# center = ax1.plot(path_x[0:1], path_y[0:1], path_z[0:1], 'bo', ms=10,  zorder=2)[0] # grab a Line3D object

	# v_quiver = ax1.quiver(path_x[0], path_y[0], path_z[0], path_vx[0], path_vy[0], path_vz[0], length=5, normalize=True, pivot='tail')

	if (USE_PATH == 1):
		plot_p_3d_path(fig, path_x, path_y, path_z)	
	else:
		plot_p_3d(fig, x1, y1, z1, x2, y2, z2)

	# animate_3d(fig,"q")

	# plot_v_3d(fig, x1, y1, z1, vx, vy, vz)
	# plot_v_3d_path(fig, path_x, path_y, path_z, path_vx, path_vy, path_vz)

	# plot_q_3d(fig, x1, y1, z1, qx, qy, qz)
	# plot_p_3d_path(fig, path_x, path_y, path_z)
	# plot_q_3d_path(fig, path_x, path_y, path_z, path_qx, path_qy, path_qz)

	# plot_w_3d(fig, x1, y1, z1, wx, wy, wz, path_x, path_y, path_z, path_wx, path_wy, path_wz)
	plt.show()




'''
Parse 
'''
def get_var(var, a1, b1, c1, *args):
	offset=0
	if(var=="v"):
		offset=3
	elif(var=="q"):
		offset=6
	elif(var=="w"):
		offset=10

	with open(RRT_OUTPUT_FILE,'rb') as csvfile:
	    plots = csv.reader(csvfile, delimiter=',')
	    i = 0
	    for row in plots:
			i+=1
			if i%2 != 0:
				a1.append(float(row[0+offset]))
				b1.append(float(row[1+offset]))
				if (len(row) > 2):
					c1.append(float(row[2+offset]))
				for d1 in args:
					d1.append(float(row[3+offset]))

'''
Parse 
'''
def get_var2(var, a1, b1, c1, a2, b2, c2):
	offset=0
	if(var=="v"):
		offset=3
	elif(var=="q"):
		offset=6
	elif(var=="w"):
		offset=10

	with open(RRT_OUTPUT_FILE,'rb') as csvfile:
	    plots = csv.reader(csvfile, delimiter=',')
	    i = 0
	    for row in plots:
			i+=1
			if i%2 == 0:
				a1.append(float(row[0+offset]))
				b1.append(float(row[1+offset]))
				if (len(row) > 2):
					c1.append(float(row[2+offset]))

			else:
				a2.append(float(row[0+offset]))
				b2.append(float(row[1+offset]))
				if (len(row) > 2):
					c2.append(float(row[2+offset]))

'''
Get the ordered solution path
'''
def get_var_path(var, path_a, path_b, path_c, *args):
	offset=0
	if(var=="v"):
		offset=3
	elif(var=="q"):
		offset=6
	elif(var=="w"):
		offset=10

	with open(RRT_PATH_OUTPUT_FILE,'rb') as csvfile:
	    plots = csv.reader(csvfile, delimiter=',')
	    for row in plots:
	        path_a.append(float(row[0+offset]))
	        path_b.append(float(row[1+offset]))
	        if (len(row) > 2):
	        	path_c.append(float(row[2+offset]))
	    for path_d in args:
	    	path_d.append(float(row[3+offset])) # quaternion

def plot_p_2d(fig, x1, y1, x2, y2, path_x, path_y):
	# Plot nodes
	ax1 = fig.axes[0]
	ax1.scatter(x2,y2)

	# Quickly generate a collection of line segments for edges
	xpts = np.stack((np.array(x1),np.array(x2)),axis=1)
	ypts = np.stack((np.array(y1),np.array(y2)),axis=1)
	segs = np.zeros((len(x1), 2, 2), float)
	segs[:, :, 0] = xpts # nx2 of x points. Each n is a different line.
	segs[:, :, 1] = ypts # nx2 of y points. Each n is a different line.
	line_segments = LineCollection(segs, linestyle='solid')
	ax1.add_collection(line_segments)

	# Plot the solution path
	ax1.plot(path_x, path_y, 'r', linewidth=3.0)

def plot_v_2d(fig, x1, y1, x2, y2, path_x, path_y):
	# Plot nodes
	ax1 = fig.axes[0]
	ax1.scatter(x2,y2)

	# Quickly generate a collection of line segments for edges
	xpts = np.stack((np.array(x1),np.array(x2)),axis=1)
	ypts = np.stack((np.array(y1),np.array(y2)),axis=1)
	segs = np.zeros((len(x1), 2, 2), float)
	segs[:, :, 0] = xpts # nx2 of x points. Each n is a different line.
	segs[:, :, 1] = ypts # nx2 of y points. Each n is a different line.
	line_segments = LineCollection(segs, linestyle='solid')
	ax1.add_collection(line_segments)

	# Plot the solution path
	ax1.plot(path_x, path_y, 'r', linewidth=3.0)

def plot_p_3d(fig, x1, y1, z1, x2, y2, z2):
	ax1 = fig.axes[0]
	ax1.set_prop_cycle(cycler('color', ['b']))

	# Quickly plot edges as a Line3DCollection
	xpts = np.stack((np.array(x1),np.array(x2)),axis=1)
	ypts = np.stack((np.array(y1),np.array(y2)),axis=1)
	zpts = np.stack((np.array(z1),np.array(z2)),axis=1)

	segs = np.zeros((len(x1), 2, 3), float)
	segs[:, :, 0] = xpts # nx2 of x points
	segs[:, :, 1] = ypts # nx2 of y points
	segs[:, :, 2] = zpts # nx2 of z points

	line_segments = Line3DCollection(segs, linestyle='solid', zorder=3)
	ax1.add_collection3d(line_segments, zdir='z')

	# Plot nodes
	ax1.scatter(x2,y2,z2, zorder=4, s=1)

def plot_p_3d_path(fig, path_x, path_y, path_z):
	ax1 = fig.axes[0]
	# Plot the solution path
	ax1.plot(path_x, path_y, path_z, 'r', linewidth=3.0, zorder=1)

def plot_v_3d(fig, x, y, z, vx, vy, vz):
	ax1 = fig.axes[0]
	ax1.set_prop_cycle(cycler('color', ['b']))

	# Plot nodes
	ax1.quiver(x, y, z, vx, vy, vz, length=.05)

def plot_v_3d_path(fig, path_x, path_y, path_z, path_vx, path_vy, path_vz):
	ax1 = fig.axes[0]
	ax1.quiver(path_x, path_y, path_z, path_vx, path_vy, path_vz, length=.05)

def plot_q_3d(fig, x, y, z, qx, qy, qz):
	ax1 = fig.axes[0]
	ax1.set_prop_cycle(cycler('color', ['b']))

	# Plot nodes
	ax1.quiver(x, y, z, qx, qy, qz, length=.05)

def plot_q_3d_path(fig, path_x, path_y, path_z, path_qx, path_qy, path_qz):
	ax1 = fig.axes[0]
	ax1.quiver(path_x, path_y, path_z, path_qx, path_qy, path_qz, length=.05)

def plot_w_3d(fig, x, y, z, wx, wy, wz, path_x, path_y, path_z, path_wx, path_wy, path_wz):
	ax1 = fig.axes[0]
	ax1.set_prop_cycle(cycler('color', ['b']))

	# Plot nodes
	# ax1.quiver(x, y, z, vx, vy, vz, length=.05)
	ax1.quiver(path_x, path_y, path_z, path_wx, path_wy, path_wz, length=.05)

	# Plot the solution path
	ax1.plot(path_x, path_y, path_z, 'r', linewidth=3.0)

def update_anim_p(frame):
	center.set_data(path_x[frame], path_y[frame])
	center.set_3d_properties(path_z[frame])
	MULT=5.0
	update = [ [[path_x[frame], path_y[frame], path_z[frame]], 
			 [path_x[frame]+MULT*path_vx[frame], path_y[frame]+MULT*path_vy[frame], path_z[frame]+MULT*path_vz[frame]]] ]
	# v_quiver.set_segments(update)

	return center, v_quiver

def update_anim_q(frame):
	center.set_data(path_x[frame], path_y[frame])
	center.set_3d_properties(path_z[frame])
	MULT=0.5
	update = [ [[path_x[frame], path_y[frame], path_z[frame]], 
			 [path_x[frame]+MULT*path_qx[frame], path_y[frame]+MULT*path_qy[frame], path_z[frame]+MULT*path_qz[frame]]] ]
	v_quiver.set_segments(update)

	return center, v_quiver

def animate_3d(fig,type):
	if type=="p":
		plot_p_3d_path(fig, path_x, path_y, path_z)
		anim = animation.FuncAnimation(fig,
									   update_anim_p,
									   frames=len(path_x),
	                          		   interval=100,
	                          		   blit=False)
		# anim.save('tester.mp4', fps=10, extra_args=['-vcodec', 'libx264'])
		anim.save('tester.gif', fps=10, writer='imagemagick')
	elif type=="q":
		plot_p_3d_path(fig, path_x, path_y, path_z)
		anim = animation.FuncAnimation(fig,
									   update_anim_q,
									   frames=len(path_x),
	                          		   interval=100,
	                          		   blit=False)
		# anim.save('tester.mp4', fps=10, extra_args=['-vcodec', 'libx264'])
		anim.save('tester.gif', fps=10, writer='imagemagick')
	plt.show()



if __name__ == "__main__":
	main()