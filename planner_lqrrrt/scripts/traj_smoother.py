#!/usr/bin/env python3
import numpy as np
import copy
# import opengen as og
# import casadi.casadi as cs
# import scipy.io as sio
import scipy.linalg
# import matplotlib.pyplot as plt
# import time as ti
# import sys
import rospy
from std_msgs.msg import ColorRGBA, String, Float64MultiArray, MultiArrayDimension
from ff_msgs.msg import ControlState, EkfState, FlightMode
# from ff_msgs.srv import SetInertia
from geometry_msgs.msg import InertiaStamped, Point
from rospy.numpy_msg import numpy_msg
from visualization_msgs.msg import Marker, MarkerArray

from reswarm_msgs.msg import ReswarmStatusPrimary
from reswarm_msgs.msg import ReswarmPlannerStatus

# from scipy.interpolate import interp1d
class smoother:
    def qinv(self,q1): # Quaternion Inverse
        qout=scipy.linalg.block_diag(-np.identity(3),1)@q1;
        return qout
    def qprod(self,q2,q1):#q1 goes to q2 (ref), Quaternion Multiplication
        Psi=np.array([[q1[3],q1[2],-q1[1]],[-q1[2],q1[3],q1[0]],[q1[1],-q1[0],q1[3]],[-q1[0],-q1[1],-q1[2]]]);
        qdot=np.bmat([Psi, q1.reshape((4,1))]).A;
        qout=qdot@q2;
        return qout
    def psiinv(self,q): # Quaternion to 3 variable representation
        out=q[0:3]/q[3];
        return out
    def slerp(self,q1,q2,t): # Quaternion Interpolation
        lamb=np.dot(q1,q2);
        if lamb<0:
            q2=-q2;
            lamb=-lamb;
        if np.sqrt((1-lamb)**2)<.000001:
            r=1-t;
            s=t;
        else:
            alpha=np.arccos(lamb);
            gamma=1/np.sin(alpha);
            r=np.sin((1-t)*alpha)*gamma;
            s=np.sin(t*alpha)*gamma;
        qi=r*q1+s*q2;
        Q=qi/np.linalg.norm(qi);
        return Q;
    def check_collision(self,x1,x2,param): # Check for collisions with Ellipsoids using a Straight Line Trajectory
        t=np.linspace(0, 1, num=50);
        dist=x2[0:3,[0]]-x1[0:3,[0]];
        path=x1[0:3,[0]]+dist*t;
        k=0;
        for xobs in param.obstacle_list.T:
            distxyz = (path[0:3,:].T-xobs.T).T;
            d_list=np.einsum('ij,ij->i',(distxyz.T@param.P[:,:,k]),distxyz.T);
            k=k+1;
            if -min(d_list)+param.ell_con >= 0.0:
                return False;
        return True;
    def check_collision_lqr(self,traj_lqr,x2,param): # Check for collisions with Ellipsoids using LQR Trajectory
        path=np.bmat([[traj_lqr,x2]]).A;
        k=0;
        for xobs in param.obstacle_list.T:
            distxyz = (path[0:3,:].T-xobs.T).T;
            d_list=np.einsum('ij,ij->i',(distxyz.T@param.P[:,:,k]),distxyz.T);
            k=k+1;
            if -min(d_list)+param.ell_con >= 0.0:
                return False;
        return True;
    def check_vel(self, traj):
        for k in range(traj.shape[1]):
            vel_bool=np.abs(traj[3:6,[k]])<self.vel_max_array;
            #angvel_bool=np.abs(node.path[10:13,[k]])<self.angvel_max_array;
            for i in range(3):
                if vel_bool[i,0]==False:#or angvel_bool[i,0]==False
                    #print(vel_bool)
                    #rospy.loginfo(i)
                    #print(angvel_bool)
                    #print('---------')
                    return False;
        return True;
    def __init__(self,MaxIter,param,flag_ground): # Initialize Trajectory Smoother
        self.MaxIter=MaxIter; # Max Iteration of Trajectory Smoother
        self.DT=param.DT*param.step_size; # Time Step
        self.N=1000; # Max Length of LQR Trjaectory
        self.uzeros=np.zeros((6,1));
        self.m=param.m;
        self.I=param.I;
        self.eps=param.eps;
        self.fn_dyn=param.fn_dyn;
        self.flag_ground=flag_ground;
        self.param=param;

    def run(self,param,traj,time,u_traj): # Run Trjactory Smoothing by Shortcutting
        end_traj=traj[:,[-1]];
        newtrajlqr=traj;
        newulqr=u_traj;
        newxreflqr=traj;
        nsample=2; # 2 Samples
        force_max=.1;
        torque_max=.005*np.eye(3);
        self.max=np.array([[force_max],[force_max],[force_max],[torque_max[0,0]],[torque_max[1,1]],[torque_max[2,2]]]);
        self.vel_max=.1;
        self.angvel_max=.122;
        self.vel_max_array=np.array([[self.vel_max],[self.vel_max],[self.vel_max]]);
        self.angvel_max_array=np.array([[self.angvel_max],[self.angvel_max],[self.angvel_max]]);
        pub = rospy.Publisher('~chatter_topic', String, queue_size=10);
        #data_red_delete=[];
        # LQR Cost Matrices
        if (self.flag_ground=='false'):
            #param.Q[0,0]=1000;
            #param.Q[1,1]=1000;
            #param.Q[2,2]=1000;
            #param.Q[6,6]=1000;
            #param.Q[7,7]=1000;
            #param.Q[8,8]=1000;
            #param.R = 1000*np.identity(6);
            param.Q[0,0]=.5;
            param.Q[1,1]=.5;
            param.Q[3,3]=10;
            param.Q[4,4]=10;
            param.Q[5,5]=10;

            Iter_str = "flag_ground: "+self.flag_ground;
            #print(Iter_str);#Undo Print
            rospy.loginfo(Iter_str)
            pub.publish(Iter_str)
        else:
            #param.Q = 0 * np.identity(12); # State Cost Matrix
            param.Q[0,0]=.5;
            param.Q[1,1]=.5;
            param.Q[3,3]=10;
            param.Q[4,4]=10;
            param.Q[5,5]=10;
            #param.Q[3,3]=1;
            #param.Q[4,4]=1;
            #param.Q[8,8]=1000;
            #param.Q[11,11]=1;
            #param.R = 1*np.identity(6);
            Iter_str = "flag_ground: "+self.flag_ground;
            rospy.loginfo(Iter_str)
            pub.publish(Iter_str)
            pass;
        for i in range(self.MaxIter):
            if rospy.is_shutdown():
                break
            #data_red_delete.append(traj.shape[1]);
            if np.random.randint(0,100)>-1:
                PickPoints=np.random.choice(traj.shape[1],nsample,replace=False).tolist();
                notend=True;
            else:
                PickPoints=np.random.choice(traj.shape[1]-1,1,replace=False).tolist();
                PickPoints.append(traj.shape[1]-1);
                notend=False;
            PickPoints.sort();
            ti1=PickPoints[0];
            ti2=PickPoints[1];
            path1=traj[:,[ti1]];
            path2=traj[:,[ti2]];

            if ti1 <= 1 or ti2 <= 1:
                continue

            if (ti2 + 2) > traj.shape[1] and notend:
                continue

            if ti2 == ti1:
                continue
            traj_lqr,u_lqr,xref_lqr=self.LQR_planning(param,np.bmat([[path1,path2]]).A);
            if len(traj_lqr)==0:
                continue

            if (not self.check_collision_lqr(traj_lqr, path2,param)) and (not self.check_vel(traj_lqr)):
                continue
            newtraj = np.bmat([[traj[:,:ti1],traj_lqr[:,[0]],traj_lqr[:,[-1]],traj[:,ti2+1:]]]).A;
            #newtime = np.bmat([[time[:ti1],time[ti1],time[ti2],time[ti2+1:]]]).A;
            newtime = np.append(time[:ti1],np.append(time[ti1],np.append(time[ti2],time[ti2+1:]))).reshape((1,-1))
            tii1=np.argwhere(newtrajlqr==traj[:,[ti1]]);
            tii2=np.argwhere(newtrajlqr==traj[:,[ti2]]);
            if len(tii1)==0 or len(tii2)==0:
                continue;
            else:
                tii1=tii1[0,1];
                tii2=tii2[0,1];

            newtrajlqr=np.bmat([[newtrajlqr[:,:tii1],traj_lqr,newtrajlqr[:,tii2+1:]]]).A;
            newxreflqr=np.bmat([[newxreflqr[:,:tii1],xref_lqr,newxreflqr[:,tii2+1:]]]).A;
            newulqr=np.bmat([[newulqr[:,:tii1],u_lqr,self.uzeros,newulqr[:,tii2+1:]]]).A;
            #self.draw_plot(traj,newtrajlqr,traj_lqr)
            #self.draw_plottime(traj_lqr,xref_lqr)
            #ti.sleep(.01);
            #newu    = np.block([[u_traj[:,:ti1 + 1],u_traj[:,[ti1+1]],u_traj[:,[ti2]],u_traj[:,ti2+1:]]]);
            #newtraj.extend(traj[:ti1 + 1])
            #newtraj.append([first[0], first[1]])
            #newtraj.append([second[0], second[1]])
            #newtraj.extend(path[ti2 + 1:])
            traj = newtraj;
            time = newtime[0,:];
            Iter_str = "Iter: "+str(i)+", Max Iteration: "+str(self.MaxIter)
            #print(Iter_str);#Undo Print
            rospy.loginfo(Iter_str)
            pub.publish(Iter_str)
            #u_traj = newu;
        newtrajlqr[:,[-1]]=end_traj;
        #traj[3:6,:]=np.zeros((3,traj.shape[1]));
        #traj[10:13,:]=np.zeros((3,traj.shape[1]));
        #self.draw_plottime(data_red_delete,data_red_delete)
        return traj,time,newtrajlqr,newxreflqr,newulqr

    #def get_target_points(self,param,traj,targetd):
    #    d=0;
    #    ti=0;
    #    lastPairLen = 0;
    #    for k in range(traj.shape[1]-1):
    #        pairLen=(np.sqrt(param.dx(traj[:, k+1],traj[:, k])[param.pos_orient].T@param.dx(traj[:, k+1],traj[:, k])[param.pos_orient]))[0,0];
    #        d+=pairLen;
    #        if d >= targetd:
    #            ti = k - 1
    #            lastPairLen = pairLen;
    #            break
    #    partRatio = (d - targetd) / lastPairLen;
    #    x_trans=traj[0:6,[ti]]+(traj[0:6,[ti+1]]-traj[0:6,[ti]])*partRatio;
    #    x_rot_vel=traj[10:13,[ti]]+(traj[10:13,[ti+1]]-traj[10:13,[ti]])*partRatio;
    #    q1=traj[6:10,ti];
    #    q2=traj[6:10,ti+1];
    #    x_rot=self.slerp(q1,q2,partRatio).reshape((4,1));
    #    state_new=np.concatenate((x_trans,x_rot,x_rot_vel),axis=0);
    #    return state_new, ti


    #def draw_plot(self,traj0,traj,traj2): # Draw 3D Trajectory
    #    plt.figure(1)
    #    plt.clf()
    #    ax = plt.axes(projection="3d");
    #    #if rnd is not None:
    #    #    ax.scatter3D(rnd.x,rnd.y,rnd.z,c="black");
    #    ax.plot3D(traj0[0,:],traj0[1,:],traj0[2,:],"-b");
    #    ax.plot3D(traj[0,:],traj[1,:],traj[2,:],"-g");
    #    ax.plot3D(traj2[0,:],traj2[1,:],traj2[2,:],"-r");
    #    ax.set_xlim3d(0, 7);
    #    ax.set_ylim3d(-2,8);
    #    ax.set_zlim3d(-2,2);
    #    ax.grid(True);
    #    plt.pause(0.01);
    #def draw_plottime(self,traj0,reftraj): # Draw Time History
    #    plt.figure(2)
    #    plt.clf()
    #    #ax = plt.axes(projection="3d");
    #    #if rnd is not None:
    #    #    ax.scatter3D(rnd.x,rnd.y,rnd.z,c="black");
    #    plt.subplot(131)
    #    plt.plot(traj0[0,:],'r',reftraj[0,:],'k')
    #    plt.subplot(132)
    #    plt.plot(traj0[1,:],'r',reftraj[1,:],'k')
    #    plt.subplot(133)
    #    plt.plot(traj0[2,:],'r',reftraj[2,:],'k')
    #    plt.suptitle('Position')
    #    plt.grid(True);
    #    plt.pause(0.01);

    #def get_path_length(self,param,traj):
    #    d=0;
    #    for k in range(traj.shape[1]-1):
    #        d+=np.sqrt(param.dx(traj[:, k+1],traj[:, k])[param.pos_orient].T@param.dx(traj[:, k+1],traj[:, k])[param.pos_orient]);
    #    return d[0,0]
    def vel_sat(self,xsim):
        xsimvel_sat=np.minimum(np.maximum(xsim[3:6, [0]], -self.vel_max), self.vel_max);
        xsimangvel_sat=np.minimum(np.maximum(xsim[10:13, [0]], -self.angvel_max), self.angvel_max);
        xsim_sat=np.bmat([[xsim[0:3, [0]]],[xsimvel_sat],[xsim[6:10, [0]]],[xsimangvel_sat]]).A;
        return xsim_sat

    def LQR_planning(self,param,xref): # LQR Trajectory Planner
        u_path=np.empty((6,0));
        x_path=xref[:,[0]];
        xref_path=xref[:,[0]];
        for i in range(xref.shape[1]):
            found_path=False;
            A,B=self.get_system_model(xref[:,i],param);
            self.S=self.solve_dare(A,B,param);
            ubar=param.ubar(xref[:,i]);
            xsim=np.zeros((xref.shape[0],self.N));
            xsim[:,[0]]=self.vel_sat(x_path[:,[-1]]);
            usim=np.zeros((param.R.shape[1],self.N));
            dold=100000000000;
            K=np.linalg.pinv(param.R+B.T@self.S@B)@B.T@self.S@A;
            for k in range(self.N-1):
                dx=param.dx(xsim[:, k],xref[:,i]);
                #usim[:,[k]]=-K@dx+ubar;
                usim[:,[k]]=np.minimum(np.maximum(-K@dx+ubar, -self.max), self.max);
                f1=self.DT*self.fn_dyn(xsim[:, [k]],usim[:, [k]],param.m,param.I);
                f2=self.DT*self.fn_dyn(xsim[:, [k]]+.5*f1,usim[:, [k]],param.m,param.I);
                f3=self.DT*self.fn_dyn(xsim[:, [k]]+.5*f2,usim[:, [k]],param.m,param.I);
                f4=self.DT*self.fn_dyn(xsim[:, [k]]+f3,usim[:, [k]],param.m,param.I);
                xsim[:, [k + 1]] =self.vel_sat(xsim[:, [k]]+1/6*(f1+2*f2+2*f3+f4));
                xsim[6:10,[k+1]] =xsim[6:10,[k+1]]/np.linalg.norm(xsim[6:10,[k+1]]);
                d=np.sqrt(param.dx(xsim[:, k+1],xref[:,i])[param.pos_orient].T@param.dx(xsim[:, k+1],xref[:,i])[param.pos_orient]);
                if np.abs(d-dold)<.001:
                    break;
                if np.abs(d)<.1:
                    found_path = True;
                    u_path=np.append(u_path,usim[:,0:k+1],axis=1);
                    x_path=np.append(x_path,xsim[:,1:k+2],axis=1);
                    xref_path=np.append(xref_path,xref[:,[i]]@np.ones((1,xsim[:,1:k+2].shape[1])),axis=1);
                    break;
                dold=d;
            if not found_path:
                #print("Cannot find path");#Undo Print
                #u_path=np.append(u_path,usim[:,0:k+1],axis=1);
                #x_path=np.append(x_path,xsim[:,1:k+2],axis=1);
                #xref_path=np.append(xref_path,xref[:,[i]]@np.ones((1,xsim[:,1:k+2].shape[1])),axis=1);
                #self.draw_plottime(x_path,xref_path)
                #print(d)
                return np.array([]),np.array([]),np.array([]);
        return x_path,u_path,xref_path

    def solve_dare(self,A,B,param): # Solve discrete Ricatti Equation for LQR
        X=param.Q;
        Xn=param.Q;
        for i in range(param.MAX_ITER_LQR_Cost+120):
            Xn=np.matmul(np.matmul(A.T,X),A)-np.matmul(np.matmul(np.matmul(np.matmul(np.matmul(np.matmul(A.T,X),B),np.linalg.pinv(param.R+np.matmul(np.matmul(B.T,X),B))),B.T),X),A)+param.Q;
            if (abs(Xn-X)).max()<param.EPS:
                break;
            X=Xn;
        return Xn;

    def get_system_model(self,xdes,param): # Get Discrete model of LTV system
        Ad=np.identity(param.nx-1)+self.DT*(param.E(xdes).T@param.A(xdes)@param.E(xdes));
        Bd=self.DT*(param.E(xdes).T@param.B);
        return Ad, Bd
    def cfinitediff(self,xtraj,utraj):
        xdotdot=np.zeros((xtraj.shape[0],xtraj.shape[1]));
        for k in range(0,xtraj.shape[1]):
            if k+1!=xtraj.shape[1]:
                xdotplus=xtraj[:,[k]]+self.eps*self.fn_dyn(xtraj[:,[k]],utraj[:,[k]],self.m,self.I);
                xdotminus=xtraj[:,[k]]-self.eps*self.fn_dyn(xtraj[:,[k]],utraj[:,[k]],self.m,self.I);
                xdotdot[:,[k]]=(xdotplus-xdotminus)/(2*self.eps);
            else:
                uzeros=np.zeros((6,1));
                xdotplus=xtraj[:,[k]]+self.eps*self.fn_dyn(xtraj[:,[k]],uzeros,self.m,self.I);
                xdotminus=xtraj[:,[k]]-self.eps*self.fn_dyn(xtraj[:,[k]],uzeros,self.m,self.I);
                xdotdot[:,[k]]=(xdotplus-xdotminus)/(2*self.eps);
        return xdotdot
    def SetControlState(self,trajectory,trajectorydot):
        trajCS=ControlState();  # gnc/ctl/setpoint msg
        traj_len=trajectory.shape[1];
        pub_ControlState=rospy.Publisher('~setpoint_topic',ControlState,queue_size=traj_len,latch=True);
        r = rospy.Rate(5)
        r.sleep()
        for k in range(0,traj_len):
            trajCS.when=rospy.Time.now();
            trajCS.pose.position.x=trajectory[0,k];
            trajCS.pose.position.y=trajectory[1,k];
            trajCS.pose.position.z=trajectory[2,k];

            trajCS.pose.orientation.x=trajectory[6,k];
            trajCS.pose.orientation.y=trajectory[7,k];
            trajCS.pose.orientation.z=trajectory[8,k];
            trajCS.pose.orientation.w=trajectory[9,k];

            trajCS.twist.linear.x=trajectory[3,k];
            trajCS.twist.linear.y=trajectory[4,k];
            trajCS.twist.linear.z=trajectory[5,k];

            trajCS.twist.angular.x=trajectory[10,k];
            trajCS.twist.angular.y=trajectory[11,k];
            trajCS.twist.angular.z=trajectory[12,k];

            trajCS.accel.linear.x=trajectorydot[3,k];
            trajCS.accel.linear.y=trajectorydot[4,k];
            trajCS.accel.linear.z=trajectorydot[5,k];

            trajCS.accel.angular.x=trajectorydot[10,k];
            trajCS.accel.angular.y=trajectorydot[11,k];
            trajCS.accel.angular.z=trajectorydot[12,k];
            #print(trajCS)
            pub_ControlState.publish(trajCS)
            #print('ftrololol')
            r.sleep()

    def SetControlStateRobustTubeMPC(self,trajectory,trajectorydot):
        traj_msg=Float64MultiArray();
        traj_len=trajectory.shape[1];
        traj_msg.layout.dim = [MultiArrayDimension(), MultiArrayDimension()];
        traj_msg.layout.dim[0].size = traj_len;
        traj_msg.layout.dim[1].size = 20;
        dt=self.param.step_size*self.param.DT;
        t=0;
        pub_ControlState=rospy.Publisher('~tube_mpc_topic',Float64MultiArray,queue_size=1,latch=True);
        traj_msg.data=[];
        for k in range(0,traj_len):
            t=dt*k;#np.arange(0,traj_len*dt,dt);
            x=trajectory[0,k];
            y=trajectory[1,k];
            z=trajectory[2,k];

            qx=trajectory[6,k];
            qy=trajectory[7,k];
            qz=trajectory[8,k];
            qw=trajectory[9,k];

            xd=trajectory[3,k];
            yd=trajectory[4,k];
            zd=trajectory[5,k];

            wx=trajectory[10,k];
            wy=trajectory[11,k];
            wz=trajectory[12,k];

            xdd=trajectorydot[3,k];
            ydd=trajectorydot[4,k];
            zdd=trajectorydot[5,k];

            wxd=trajectorydot[10,k];
            wyd=trajectorydot[11,k];
            wzd=trajectorydot[12,k];

            traj_msg.data.extend([t, x, y, z, xd, yd, zd, qx, qy, qz, qw, wx, wy, wz, xdd, ydd, zdd, wxd, wyd, wzd]);
        pub_ControlState.publish(traj_msg)
        rospy.sleep(3/5)

    def PublishLQRRRT(self,trajectory,trajectorydot):
        traj_msg=Float64MultiArray();
        traj_len=trajectory.shape[1];
        traj_msg.layout.dim = [MultiArrayDimension(), MultiArrayDimension()];
        traj_msg.layout.dim[0].size = traj_len;
        traj_msg.layout.dim[1].size = 20;
        pub_ControlState=rospy.Publisher('~lqrrrt_topic',Float64MultiArray,queue_size=1,latch=True);
        traj_msg.data=[];
        for k in range(0,traj_len):
            t=rospy.get_time();
            x=trajectory[0,k];
            y=trajectory[1,k];
            z=trajectory[2,k];

            qx=trajectory[6,k];
            qy=trajectory[7,k];
            qz=trajectory[8,k];
            qw=trajectory[9,k];

            xd=trajectory[3,k];
            yd=trajectory[4,k];
            zd=trajectory[5,k];

            wx=trajectory[10,k];
            wy=trajectory[11,k];
            wz=trajectory[12,k];

            xdd=trajectorydot[3,k];
            ydd=trajectorydot[4,k];
            zdd=trajectorydot[5,k];

            wxd=trajectorydot[10,k];
            wyd=trajectorydot[11,k];
            wzd=trajectorydot[12,k];

            traj_msg.data.extend([t, x, y, z, xd, yd, zd, qx, qy, qz, qw, wx, wy, wz, xdd, ydd, zdd, wxd, wyd, wzd]);
        pub_ControlState.publish(traj_msg)
        rospy.sleep(3/5)

class InertiaInput:
    def __init__(self):
        self.m=None;
        self.I=None;
        #self._event = threading.Event()
    def __call__(self,msg):
        self.m=msg.inertia.m;
        #print(msg.inertia.m)
        #print(self.m)
        self.I=np.array([[msg.inertia.ixx, msg.inertia.ixy, msg.inertia.ixz],[msg.inertia.ixy, msg.inertia.iyy, msg.inertia.iyz], [msg.inertia.ixz, msg.inertia.iyz, msg.inertia.izz]]);
        #self._event.set()
    def get_msg(self):
        #self._event.wait(timeout)
        return self.m, self.I

class StateInput:
    def __init__(self):
        self.state=EkfState();
        #self._event = threading.Event()
    def __call__(self,msg):
        self.state=msg;
        #self._event.set()
    def get_msg(self):
        #self._event.wait(timeout)
        return self.state

class FlightModeInput:
    def __init__(self):
        self.Mode=FlightMode();
    def __call__(self,msg):
        self.Mode=msg;
        self.Mode.name='nominal';
        self.Mode.header.seq=1;
        self.Mode.control_enabled=True;
        self.Mode.collision_radius = 0.25;
        self.Mode.hard_limit_vel = 0.20000000298;
        self.Mode.hard_limit_accel = 0.0175000000745;
        self.Mode.hard_limit_omega = 0.174500003457;
        self.Mode.hard_limit_alpha = 0.174500003457;
        self.Mode.speed = 3;
    def get_msg(self):
        return self.Mode

class StatusInput:
    def __init__(self):
        self.status=ReswarmStatusPrimary();
        #self._event = threading.Event()
    def __call__(self,msg):
        self.status=msg;
    def get_msg(self):
        #self._event.wait(timeout)
        return self.status

def plot_trajp(markerArray,pub_mark,traj):
    count = 0
    MARKERS_MAX = 100
    traj_len=traj.shape[1];
    triplePoints = []
    colorsGroup = []
    for k in range(0,traj_len):

        marker = Marker()
        marker.lifetime = rospy.Duration(0)
        marker.ns = "LQRRRT";
        marker.header.frame_id = "/world"
        marker.type = marker.LINE_STRIP
        marker.action = marker.ADD
        marker.scale.x = 0.005
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.pose.orientation.w = 1.0
        p = Point()
        p.x=traj[0,k];
        p.y=traj[1,k];
        p.z=traj[2,k];
        triplePoints.append(p)
        colorsGroup.append(ColorRGBA(1.,0.,1.,1.))
        marker.pose.position.x = 0
        marker.pose.position.y = 0
        marker.pose.position.z = 0

        # We add the new marker to the MarkerArray, removing the oldest marker from it when necessary
        #if(count > MARKERS_MAX):
        #    markerArray.markers.pop(0)

        marker.points = triplePoints
        marker.colors = colorsGroup
        markerArray.markers.append(marker)

        # Renumber the marker IDs
        id = 0
        for m in markerArray.markers:
            m.id = id
            id += 1
        count += 1

        rospy.sleep(0.01)
    # Publish the MarkerArray
    pub_mark.publish(markerArray)
#***********************************************************************************************************************
class Node: # Each node on the tree has these properties
    def __init__(self, state):
            self.x = state[0,0]; # x state
            self.y = state[1,0]; # y state
            self.z = state[2,0]; # z state
            self.sv = state[0:13,0]; # x y z xdot ydot zdot qx qy qz qs wx wy wz
            self.u=[]; # Control Vector 6x1
            self.path_x = []; # Path of x states
            self.path_y = []; # Path of y states
            self.path_z = []; # Path of z states
            self.path = []; # Path of entire state
            self.parent = []; # Parent node this node corresponds to
            self.cost = 0; # Node cost
class rrtStar:
    def __init__(self,x0,xdes,zobs,P,Nobs,ell_con,randArea,sys_mass,sys_MOI):
        self.start=Node(x0); # Start Node
        self.end=Node(xdes); # End Node
        self.min_rand=randArea[0:3]; # Min State space to sample RRT* Paths
        self.max_rand=randArea[3:6]; # Max State space to sample RRT* Paths
        self.goal_sample_rate=10; # Samples goal 10% of time
        self.max_iter=300; # Max RRT* Iteration
        self.connect_circle_dist=2; # Circular Radius of to Calculate Near Nodes
        self.step_size=.125; # Step size of Interpolation between k and k+1
        self.expand_dis=1; # To find CLosest Node to the end Node
        self.update_plot=20; # Update plot Iteration
        self.d=len(x0)-1; # Dimension of State minus one

        # Obstacles
        self.obstacle_list=zobs; #States of each Obstacle
        self.P=P; # Shape of Ellipsoidal Obstacles
        self.Nobs=Nobs; # Number of Obstacles
        self.ell_con=ell_con; # Ellipsoid Obstacle Model
        # Limits
        force_max=.1;
        torque_max=.005*np.eye(3);
        self.max=np.array([[force_max],[force_max],[force_max],[torque_max[0,0]],[torque_max[1,1]],[torque_max[2,2]]]);
        self.vel_max=.1;
        self.angvel_max=.122;
        self.vel_max_array=np.array([[self.vel_max],[self.vel_max],[self.vel_max]]);
        self.angvel_max_array=np.array([[self.angvel_max],[self.angvel_max],[self.angvel_max]]);

        #LQR IC
        self.MAX_ITER_LQR_Cost=30; # Max Iteration for Computing Ricatti Equation
        self.EPS=.1; # Tolerance for computing Ricatti Equation
        self.MAX_TIME=100;# Number of steps LQR trajectory is computed
        self.N=self.MAX_TIME; # Number of steps LQR trajectory is computed
        self.DT=1.6; # Time-Step
        self.GOAL_DIST=.1; # Max Distance from xdes for LQR Convergence
        self.eps=1e-8; #eps for finite differences
        #self.A = np.identity(6) + self.DT * np.array(
        #[[0, 0, 0, 1, 0, 0], [0, 0, 0, 0, 1, 0], [0, 0, 0, 0, 0, 1], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0],
        # [0, 0, 0, 0, 0, 0]]);
        #self.B= self.DT * np.array([[0, 0, 0], [0, 0, 0], [0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]]);
        self.pos_orient=[0,1,2,6,7,8];
        self.nx = 13; # Number of states
        self.nu = 6; # Number of Control Inputs
        self.Q = 1 * np.identity(self.nx-1); # State Cost Matrix
        self.Q[3,3]=75;
        self.Q[4,4]=75;
        self.Q[5,5]=75;
        #self.Q[9,9]=10;
        #self.Q[10,10]=10;
        #self.Q[11,11]=10;
        #self.Q[6:10,6:10]=1000*self.Q[6:10,6:10];
        self.R = np.identity(self.nu); # Control Cost Matrix
        self.S = []; # Ricatti Solution
        # Mass properties
        self.m=sys_mass; #kg
        self.I=sys_MOI;
        self.invI=np.linalg.inv(self.I);
        self.eye3=np.identity(3);
        # B Matrix of LTV System for Rigid Body Dynamics
        self.B=np.array([[0, 0, 0,0,0,0], [0, 0, 0,0,0,0], [0, 0, 0,0,0,0], [1/self.m, 0, 0,0,0,0], [0, 1/self.m, 0,0,0,0], [0, 0, 1/self.m,0,0,0],[0, 0, 0,0,0,0],[0, 0, 0,0,0,0], [0, 0, 0,0,0,0], [0, 0, 0,0,0,0],[0,0,0,1/self.I[0,0],0,0],[0,0,0,0,1/self.I[1,1],0],[0,0,0,0,0,1/self.I[2,2]]]);
    def A(self,x): # A Matrix of LTV System for Rigid Body Dynamics
        q11=np.array([[0, 0, 0, 1, 0, 0], [0, 0, 0, 0, 1, 0], [0, 0, 0, 0, 0, 1], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0],[0, 0, 0, 0, 0, 0]]);
        q12=np.zeros((6, 7));
        q21=np.zeros((7, 6));
        q22=np.array([[0, .5*x[12], -.5*x[11], .5*x[10], .5*x[9], -.5*x[8],.5*x[7]], [-.5*x[12], 0, .5*x[10], .5*x[11], .5*x[8], .5*x[9],-.5*x[6]], [.5*x[11], -.5*x[10], 0, .5*x[12], -.5*x[7], .5*x[6],.5*x[9]], [-.5*x[10], -.5*x[11], -.5*x[12], 0, -.5*x[6], -.5*x[7],-.5*x[8]], [0, 0, 0, 0,0, (self.I[1,1]-self.I[2,2])/self.I[0,0]*x[12], (self.I[1,1]-self.I[2,2])/self.I[0,0]*x[11]],[0, 0, 0, 0, (self.I[2,2]-self.I[0,0])/self.I[1,1]*x[12], 0,(self.I[2,2]-self.I[0,0])/self.I[1,1]*x[10]],[0, 0, 0, 0, (self.I[0,0]-self.I[1,1])/self.I[2,2]*x[11],(self.I[0,0]-self.I[1,1])/self.I[2,2]*x[10],0]])
        Aout=np.bmat([[q11,q12],[q21,q22]]).A;
        return Aout;
    def ubar(self,xbar): # Equilibrium ubar for linearized LTV System
        uout=np.array([[0,0,0,(self.I[2,2]-self.I[1,1])*xbar[11]*xbar[12],(self.I[0,0]-self.I[2,2])*xbar[10]*xbar[12],(self.I[1,1]-self.I[0,0])*xbar[10]*xbar[11]]]).T;
        return uout
    def E(self,x):
        G=np.array([[x[9],-x[8],x[7]],[x[8],x[9],-x[6]],[-x[7],x[6],x[9]],[-x[6],-x[7],-x[8]]]);
        Eout=scipy.linalg.block_diag(self.eye3,self.eye3,G,self.eye3);
        return Eout
    def fn_dyn(self,x,u,m,I): # Nonlinear Rigid Body Dynamic System
        xyzdot=x[3:6];
        xyzdotdot=u[0:3]/m;
        x[6:10,:] =x[6:10,:]/np.linalg.norm(x[6:10,:]);
        qdot=0.5*np.array([[0., x[12,0], -x[11,0], x[10,0]],[-x[12,0], 0., x[10,0], x[11,0]],[x[11,0],-x[10,0], 0., x[12,0]],[-x[10,0], -x[11,0],-x[12,0], 0.]])@x[6:10];
        wdot=self.invI@(np.cross(-x[10:13],I@x[10:13],axisa=0, axisb=0).T)+self.invI@u[3:6];
        xdot=np.concatenate([xyzdot, xyzdotdot, qdot, wdot]);
        return xdot;
    def slerp(self,q1,q2,t): # Quaternion Interpolation
        lamb=np.dot(q1,q2);
        if lamb<0:
            q2=-q2;
            lamb=-lamb;
        if np.sqrt((1-lamb)**2)<.000001:
            r=1-t;
            s=t;
        else:
            alpha=np.arccos(lamb);
            gamma=1/np.sin(alpha);
            r=np.sin((1-t)*alpha)*gamma;
            s=np.sin(t*alpha)*gamma;
        qi=r*q1+s*q2;
        Q=qi/np.linalg.norm(qi);
        return Q;
    def qinv(self,q1): # Quaternion inverse
        qout=scipy.linalg.block_diag(-np.identity(3),1)@q1;
        return qout
    def qprod(self,q2,q1):#q1 goes to q2 (ref), Quaternion Multiplication
        Psi=np.array([[q1[3],q1[2],-q1[1]],[-q1[2],q1[3],q1[0]],[q1[1],-q1[0],q1[3]],[-q1[0],-q1[1],-q1[2]]]);
        qdot=np.bmat([Psi, q1.reshape((4,1))]).A;
        qout=qdot@q2;
        return qout
    def psiinv(self,q): # Quaternion to 3 variable representation
        q =q/np.linalg.norm(q);
        out=q[0:3]/q[3];
        return out
    def dx(self,x1,x2):#x1 goes to x2. Difference between current state to Reference State
        phi=self.psiinv(self.qprod(self.qinv(x2[6:10]),x1[6:10]));
        dxout=np.bmat([[x1[0:6]-x2[0:6],phi,x1[10:13]-x2[10:13]]]).A.T;
        return dxout
    def planning(self): # LQR-RRT* Planner
        search_until_max_iter=0;
        self.node_list=[self.start];
        pub = rospy.Publisher('~chatter_topic', String, queue_size=10);
        for i in range(self.max_iter):
            if rospy.is_shutdown():
                break
            #hello_str = "hello world %s" % rospy.get_time()
            rnd=self.get_random_node();
            if (i)%self.update_plot==0:
                Iter_str = "Iter: "+str(i)+", number of nodes: "+str(len(self.node_list))
                #print(Iter_str);#Undo Print
                rospy.loginfo(Iter_str)
                pub.publish(Iter_str)
                last_index=self.search_best_goal_node();
                traj=None;
                if last_index:
                    traj, u_traj=self.generate_final_course(last_index);
                #self.draw_plot(rnd,traj);
            nearest_ind=self.get_nearest_node_index(rnd)
            new_node=self.steer(self.node_list[nearest_ind],rnd);
            if new_node is None:
                continue;
            if self.check_collision(new_node) and self.check_vel(new_node):
                near_indices=self.find_near_nodes(new_node,nearest_ind);
                new_node= self.choose_parent(new_node,near_indices)
                if new_node:
                    self.node_list.append(new_node);
                    self.rewire(new_node,near_indices);
            #if (i)%self.update_plot==0:
                #last_index=self.search_best_goal_node();
                #traj=None;
                #if last_index:
                #    traj, u_traj=self.generate_final_course(last_index);
                #self.draw_plot(rnd,traj);


            if (not search_until_max_iter) and new_node:
                last_index=self.search_best_goal_node();
                if last_index:
                    traj, u_traj=self.generate_final_course(last_index)
                    return traj, u_traj, self;
        MaxIter_str = "Reached Max Iteration"
        print(MaxIter_str)
        rospy.loginfo(MaxIter_str)
        pub.publish(MaxIter_str)
        last_index=self.search_best_goal_node();
        if last_index:
            traj, u_traj=self.generate_final_course(last_index)
            return traj, u_traj,self
        else:
            NoPath_str = "Cannot find path"
            #print(NoPath_str) #Undo Print
            rospy.loginfo(NoPath_str)
            pub.publish(NoPath_str)
        return None, None

    def generate_final_course(self,goal_index): # Generate Final Course
        path=self.end.sv[0:13].reshape(-1,1);
        u_path=np.empty((6,0));
        node=self.node_list[goal_index];
        while node.parent:
            i=np.flip(node.path,1);
            path=np.append(path,i,axis=1);

            j=np.flip(node.u,1);
            u_path=np.append(u_path,j,axis=1);

            node=node.parent;
        path=np.flip(path,1);
        u_path=np.flip(u_path,1);
        #path.append([self.start.sv]);
        return path, u_path
    def search_best_goal_node(self): # Finds Node closest to Goal Node
        dist_to_goal_list=[self.calc_dist_to_goal(node.sv) for node in self.node_list];
        goal_inds=[dist_to_goal_list.index(i) for i in dist_to_goal_list if i<=self.expand_dis];
        if not goal_inds:
            return None
        min_cost=min([self.node_list[i].cost for i in goal_inds]);
        for i in goal_inds:
            if self.node_list[i].cost==min_cost:
                return i
        return None


    def calc_dist_to_goal(self,sv): # Calculate distance between Node and the Goal
        dist=np.sqrt(self.dx(sv,self.end.sv)[self.pos_orient].T@self.dx(sv,self.end.sv)[self.pos_orient]);
        return dist

    #def draw_plot(self,rnd=None,traj=None): # Draws 3D Trajectory using LQR-RRT*
    #    plt.clf()
    #    ax = plt.axes(projection="3d");
    #    #if rnd is not None:
    #    #    ax.scatter3D(rnd.x,rnd.y,rnd.z,c="black");
    #    for node in self.node_list:
    #        if node.parent:
    #            ax.plot3D(node.path_x,node.path_y,node.path_z,"-g");
    #    ax.scatter3D(self.start.x, self.start.y, c="red")
    #    ax.scatter3D(self.end.x, self.end.y, c="limegreen")
    #    if traj is not None:
    #        ax.plot3D(traj[0,:],traj[1,:],traj[2,:],"-r");
    #    ax.set_xlim3d(-2, 8);
    #    ax.set_ylim3d(-2,8);
    #    ax.set_zlim3d(-2,2);
    #    ax.grid(True);
    #    plt.pause(0.01);
    def rewire(self, new_node, near_inds): # Rewires the Nodes
        for i in near_inds:
            near_node=self.node_list[i];
            edge_node=self.steer(new_node,near_node);
            if edge_node is None:
                continue;
            edge_node.cost=self.calc_new_cost(new_node,near_node);
            no_collision=self.check_collision(edge_node);
            improved_cost=near_node.cost>edge_node.cost;

            if no_collision and improved_cost:
                near_node=edge_node;
                near_node.parent=new_node;
                self.propagate_cost_to_leaves(new_node);

    def propagate_cost_to_leaves(self,parent_node): # Re-computes cost from rewired Nodes
        for node in self.node_list:
            if node.parent==parent_node:
                node.cost=self.calc_new_cost(parent_node,node);
                self.propagate_cost_to_leaves(node);

    def choose_parent(self, new_node, near_inds): # Chooses a parent node with lowest cost
        pub = rospy.Publisher('~chatter_topic', String, queue_size=10);
        if not near_inds:
            return None
        costs=[];
        for i in near_inds:
            near_node=self.node_list[i];
            t_node=self.steer(near_node,new_node);
            if t_node and self.check_collision(t_node):
                costs.append(self.calc_new_cost(near_node,new_node));
            else:
                costs.append(float("inf"));
        min_cost=min(costs);

        if min_cost == float("inf"):
            NoPathInf_str = "No Path - Infinite Cost"
            #print(NoPathInf_str);#Undo Print
            rospy.loginfo(NoPathInf_str)
            pub.publish(NoPathInf_str)
            return None
        min_ind= near_inds[costs.index(min_cost)];
        new_node=self.steer(self.node_list[min_ind],new_node);
        new_node.parent=self.node_list[min_ind];
        new_node.cost=min_cost;
        return new_node

    def calc_new_cost(self,from_node,to_node): # Calculates cost of node
        x_sim, u_sim=self.LQR_planning(from_node.sv,to_node.sv);
        x_sim_sample, u_sim_sample, course_lens=self.sample_path(x_sim,u_sim);
        if len(x_sim_sample)==0:
            return float("inf");
        return from_node.cost+sum(course_lens)

    def find_near_nodes(self,new_node,nearest_ind): # Finds near nodes close to new_node
        nnode=len(self.node_list)+1;
        dist_list=[self.dx(node.sv,new_node.sv).T@self.S@self.dx(node.sv,new_node.sv) for node in self.node_list];
        r=self.connect_circle_dist*np.amin(dist_list)*(np.log(nnode)/nnode)**(1/(self.d));
        ind = [dist_list.index(i) for i in dist_list if i<=r];
        if not ind:
            ind=[nearest_ind];
        return ind

    def check_collision(self, node): # Check for collisions with Ellipsoids
            k=0;
            for xobs in self.obstacle_list.T:
                distxyz = (node.path[0:3,:].T-xobs.T).T;
                d_list=np.einsum('ij,ij->i',(distxyz.T@self.P[:,:,k]),distxyz.T);
                k=k+1;
                if -min(d_list)+self.ell_con >= 0.0:
                    return False;
            return True;
    def check_vel(self, node):
            for k in range(node.path.shape[1]):
                vel_bool=np.abs(node.path[3:6,[k]])<self.vel_max_array;
                #angvel_bool=np.abs(node.path[10:13,[k]])<self.angvel_max_array;
                for i in range(3):
                    if vel_bool[i,0]==False:#or angvel_bool[i,0]==False
                        #print(vel_bool)
                        #rospy.loginfo(i)
                        #print(angvel_bool)
                        #print('---------')
                        return False;
            return True;


    def get_random_node(self): # Find a random node from the state space
        if np.random.randint(0,100)>self.goal_sample_rate:
            rand_trans=np.array([[np.random.uniform(self.min_rand[0], self.max_rand[0]), np.random.uniform(self.min_rand[1], self.max_rand[1]), np.random.uniform(self.min_rand[2], self.max_rand[2]), 0.,0., 0.]]).T;
            s=np.random.uniform();
            sigma1=np.sqrt(1-s);
            sigma2=np.sqrt(s);
            theta1=2*3.1415*np.random.uniform();
            theta2=2*3.1415*np.random.uniform();
            rand_rot=np.array([[np.sin(theta1)*sigma1,np.cos(theta1)*sigma1,np.sin(theta2)*sigma2,np.cos(theta2)*sigma2,0.,0.,0.]]).T;
            rnd=Node(np.bmat([[rand_trans],[rand_rot]]).A);
        else: # goal point sampling
            rnd= Node(self.end.sv.reshape((self.nx,1)));
        return rnd

    def get_nearest_node_index(self,rnd_node): # Get nearest node index in tree
        Ad, Bd = self.get_system_model(rnd_node.sv);
        self.S=self.solve_dare(Ad,Bd);
        #dlist=float('inf')*np.ones((len(self.node_list),1));
        dlist=[ np.matmul(np.matmul(self.dx(node.sv,rnd_node.sv).T,self.S),self.dx(node.sv,rnd_node.sv)) for node in self.node_list];
        minind=dlist.index(min(dlist));
        return minind

    def solve_dare(self,A,B): # Solve discrete Ricatti Equation for LQR
        X=self.Q;
        Xn=self.Q;
        for i in range(self.MAX_ITER_LQR_Cost):
            Xn=np.matmul(np.matmul(A.T,X),A)-np.matmul(np.matmul(np.matmul(np.matmul(np.matmul(np.matmul(A.T,X),B),np.linalg.pinv(self.R+np.matmul(np.matmul(B.T,X),B))),B.T),X),A)+self.Q;
            if (abs(Xn-X)).max()<self.EPS:
                break;
            X=Xn;
        return Xn;

    def steer(self,from_node,to_node): # Obtain trajectory between from_node to to_node using LQR and save trajectory
        x_sim, u_sim=self.LQR_planning(from_node.sv,to_node.sv);
        x_sim_sample, u_sim_sample, course_lens=self.sample_path(x_sim,u_sim);
        if len(x_sim_sample)==0:
            return None
        newNode=copy.deepcopy(from_node);
        newNode.x=x_sim_sample[0,-1];
        newNode.y=x_sim_sample[1,-1];
        newNode.z=x_sim_sample[2,-1];
        newNode.sv=x_sim_sample[:,-1];
        newNode.path_x=x_sim_sample[0,:];
        newNode.path_y=x_sim_sample[1,:];
        newNode.path_z=x_sim_sample[2,:];
        newNode.path=x_sim_sample[0:13,:];
        newNode.u=u_sim_sample[:,:];
        newNode.cost+=sum([abs(c) for c in course_lens]);
        newNode.parent=from_node;
        return newNode;

    def sample_path(self,x_sim,u_sim): # Interpolate path obtained by LQR
        x_sim_sample=[];
        u_sim_sample=[];
        if x_sim.size==0:
            clen=[];
            return x_sim_sample, u_sim_sample, clen
        for i in range(x_sim.shape[1]-1):
            for t in np.arange(0.0, 1.0, self.step_size):
                u_sim_sample.append(u_sim[:,i]);
                x_trans=(t*x_sim[0:6,i+1]+(1.0-t)*x_sim[0:6,i]).reshape((-1,1))[:,0];
                q1=x_sim[6:10,i];
                q2=x_sim[6:10,i+1];
                x_rot=self.slerp(q1,q2,t);
                x_rot_vel=(t*x_sim[10:13,i+1]+(1.0-t)*x_sim[10:13,i]).reshape((-1,1))[:,0];
                x_sim_sample.append(np.concatenate((x_trans,x_rot,x_rot_vel),axis=0));
        x_sim_sample=np.array(x_sim_sample).T;
        u_sim_sample=np.array(u_sim_sample).T
        #diff_x_sim=np.diff(x_sim_sample);
        diff_x_sim2=[self.dx(x_sim_sample[:,k+1],x_sim_sample[:,k])[:,0] for k in range(x_sim_sample.shape[1]-1)]
        diff_x_sim=np.array(diff_x_sim2).T;
        if diff_x_sim.size==0:
            return [], [], []
        clen=np.einsum('ij,ij->i',(diff_x_sim.T@self.S),diff_x_sim.T);
        return x_sim_sample, u_sim_sample, clen

    def vel_sat(self,xsim):
        xsimvel_sat=np.minimum(np.maximum(xsim[3:6, [0]], -self.vel_max), self.vel_max);
        xsimangvel_sat=np.minimum(np.maximum(xsim[10:13, [0]], -self.angvel_max), self.angvel_max);
        xsim_sat=np.bmat([[xsim[0:3, [0]]],[xsimvel_sat],[xsim[6:10, [0]]],[xsimangvel_sat]]).A;
        return xsim_sat

    def LQR_planning(self,x0,xdes): # LQR Trajectory planner
        pub = rospy.Publisher('~chatter_topic', String, queue_size=10);
        found_path=False;
        Ad, Bd = self.get_system_model(xdes);
        self.S=self.solve_dare(Ad,Bd);
        ubar=self.ubar(xdes);
        xsim = np.zeros((self.nx, self.N));
        xsim[:, [0]] = self.vel_sat(x0.reshape((-1,1)));
        usim = np.zeros((self.nu, self.N));
        K=np.linalg.pinv(Bd.T@self.S@Bd+self.R)@(Bd.T@self.S@Ad);
        dold=100000000000;
        for k in range(0, self.N - 1):
            dx=self.dx(xsim[:, k],xdes);
            #usim[:,[k]]=-K@dx+ubar;
            usim[:,[k]]=np.minimum(np.maximum(-K@dx+ubar, -self.max), self.max)
            f1=self.DT*self.fn_dyn(xsim[:, [k]],usim[:, [k]],self.m,self.I);
            f2=self.DT*self.fn_dyn(xsim[:, [k]]+.5*f1,usim[:, [k]],self.m,self.I);
            f3=self.DT*self.fn_dyn(xsim[:, [k]]+.5*f2,usim[:, [k]],self.m,self.I);
            f4=self.DT*self.fn_dyn(xsim[:, [k]]+f3,usim[:, [k]],self.m,self.I);
            xsim[:, [k + 1]]=self.vel_sat(xsim[:, [k]]+1/6*(f1+2*f2+2*f3+f4));
            xsim[6:10,[k+1]] =xsim[6:10,[k+1]]/np.linalg.norm(xsim[6:10,[k+1]]);
            d=np.sqrt(self.dx(xsim[:, k+1],xdes)[0:3].T@self.dx(xsim[:, k+1],xdes)[0:3]);
            #if np.abs(d-dold)<.001:
            #    break;
            if np.abs(d)<self.GOAL_DIST:
                found_path = True;
                xsim=xsim[:,0:k+2];
                usim=usim[:,0:k+1];
                break;
            dold=d;
        if not found_path:
                NoPath_str = "Cannot find path"
                #print(NoPath_str)#Undo Print
                rospy.loginfo(NoPath_str)
                pub.publish(NoPath_str)
                return np.array([]),np.array([]);
        return xsim, usim;




    def get_system_model(self,xdes): # Get Discrete model of LTV system
        Ad=np.identity(self.nx-1)+self.DT*(self.E(xdes).T@self.A(xdes)@self.E(xdes));
        Bd=self.DT*(self.E(xdes).T@self.B);
        return Ad, Bd
    def cfinitediff(self,xtraj,utraj):
        xdotdot=np.zeros((xtraj.shape[0],xtraj.shape[1]));
        for k in range(0,xtraj.shape[1]):
            if k+1!=xtraj.shape[1]:
                xdotplus=xtraj[:,[k]]+self.eps*self.fn_dyn(xtraj[:,[k]],utraj[:,[k]],self.m,self.I);
                xdotminus=xtraj[:,[k]]-self.eps*self.fn_dyn(xtraj[:,[k]],utraj[:,[k]],self.m,self.I);
                xdotdot[:,[k]]=(xdotplus-xdotminus)/(2*self.eps);
            else:
                uzeros=np.zeros((6,1));
                xdotplus=xtraj[:,[k]]+self.eps*self.fn_dyn(xtraj[:,[k]],uzeros,self.m,self.I);
                xdotminus=xtraj[:,[k]]-self.eps*self.fn_dyn(xtraj[:,[k]],uzeros,self.m,self.I);
                xdotdot[:,[k]]=(xdotplus-xdotminus)/(2*self.eps);
        return xdotdot

def plot_trajy(markerArray,pub_mark,traj):
    count = 0
    MARKERS_MAX = 100
    traj_len=traj.shape[1];
    triplePoints = []
    colorsGroup = []
    for k in range(0,traj_len):

        marker = Marker()
        marker.lifetime = rospy.Duration(0)
        marker.ns = "LQRRRT";
        marker.header.frame_id = "/world"
        marker.type = marker.LINE_STRIP
        marker.action = marker.ADD
        marker.scale.x = 0.005
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.pose.orientation.w = 1.0
        p = Point()
        p.x=traj[0,k];
        p.y=traj[1,k];
        p.z=traj[2,k];
        triplePoints.append(p)
        colorsGroup.append(ColorRGBA(1.,1.,0,1.))
        marker.pose.position.x = 0
        marker.pose.position.y = 0
        marker.pose.position.z = 0

        # We add the new marker to the MarkerArray, removing the oldest marker from it when necessary
        #if(count > MARKERS_MAX):
        #    markerArray.markers.pop(0)

        marker.points = triplePoints
        marker.colors = colorsGroup
        markerArray.markers.append(marker)

        # Renumber the marker IDs
        id = 0
        for m in markerArray.markers:
            m.id = id
            id += 1
        count += 1

        rospy.sleep(0.01)
    # Publish the MarkerArray
    pub_mark.publish(markerArray)
def plot_obs(markerArray,pub_mark_obs,zobs,P):
    count = 0
    #Nobs=zobs.shape[1];
    Nobs=1;
    triplePoints = []
    colorsGroup = []
    s=1/np.sqrt(np.linalg.eigvals(P[:,:,0]));
    for k in range(0,Nobs):

        marker = Marker()
        marker.lifetime = rospy.Duration(0)
        marker.ns = "Obs";
        marker.header.frame_id = "/world"
        marker.type = marker.SPHERE_LIST
        marker.action = marker.ADD
        marker.scale.x = s[0]
        marker.scale.y = s[1]
        marker.scale.z = s[2]
        marker.color.a = 1.0
        marker.color.r = 0.5
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.pose.orientation.w = 1.0
        p = Point()
        p.x=zobs[0,k];
        p.y=zobs[1,k];
        p.z=zobs[2,k];
        triplePoints.append(p)
        colorsGroup.append(ColorRGBA(.5,1.,0,1.))
        marker.pose.position.x = 0
        marker.pose.position.y = 0
        marker.pose.position.z = 0

        # We add the new marker to the MarkerArray, removing the oldest marker from it when necessary
        #if(count > MARKERS_MAX):
        #    markerArray.markers.pop(0)

        marker.points = triplePoints
        marker.colors = colorsGroup
        markerArray.markers.append(marker)

        # Renumber the marker IDs
        id = 0
        for m in markerArray.markers:
            m.id = id
            id += 1
        count += 1

        rospy.sleep(0.01)
    # Publish the MarkerArray
    pub_mark_obs.publish(markerArray)

def rrtlqrfulldyn(xdes,zobs,Nobs,x0,randArea,test_num,P,flag_ground):
    #Initialize Input msg Callbacks
    InertiaInputObj=InertiaInput();
    markerArray_traj = MarkerArray()
    markerArray_obs = MarkerArray()
    #Initialize Subscribers
    subsub=rospy.Subscriber('~inertia_topic',numpy_msg(InertiaStamped),InertiaInputObj);
    #rospy.loginfo('HELLLLLLLLLLLLLLLLO')
    #rospy.logwarn(subsub.resolved_name+"  -  "+subsub.name)
    #rospy.logwarn(asdf.resolved_name+"  -  "+asdf.name)
    #Initialize Publishers
    pub_mark_traj = rospy.Publisher('~marker_array_topic', MarkerArray,queue_size=1)
    pub_mark_obs = rospy.Publisher('~marker_array_obs_topic', MarkerArray,queue_size=1)
    pub = rospy.Publisher('~chatter_topic', String, queue_size=10);
    rospy.sleep(3/5)

    #Get messages from Subscribers
    if test_num==4:
        sys_mass=18.9715;
        sys_MOI=np.array([[.2517, 0., 0.],[0., .2517, 0.], [0., 0., .2517]]);
    elif flag_ground=='false':
        #sys_mass, sys_MOI=InertiaInputObj.get_msg();
        sys_mass=9.584;
        sys_MOI=np.array([[0.153427995, 0., 0.],[0., 0.14271405, 0.], [0., 0., 0.162302759]]);
        Iter_str = "flag_ground: "+flag_ground;
        rospy.loginfo(Iter_str)
        pub.publish(Iter_str)
    elif flag_ground == 'true':
        sys_mass=18.9715;
        sys_MOI=np.array([[.2517, 0., 0.],[0., .2517, 0.], [0., 0., .2517]]);
        Iter_str = "flag_ground: "+flag_ground;
        rospy.loginfo(Iter_str)
        pub.publish(Iter_str)
    #pub = rospy.Publisher('~chatter_topic', String, queue_size=10)
    #pub.publish(str(sys_mass));
    #pub.publish(str(sys_MOI));
    #print(sys_mass)
    #print(sys_MOI)

    # obstacle
    ell_con = 1; # For specifiying Ellipsoid Constraint

    param=rrtStar(x0,xdes,zobs,P,Nobs,ell_con,randArea,sys_mass,sys_MOI); # Initialize LQR-RRT* Planner
    trajectory,u_traj, solution=param.planning(); # Run Planner
    trajectorydot = param.cfinitediff(trajectory,u_traj);
    plot_trajy(markerArray_traj,pub_mark_traj,trajectory)
    plot_obs(markerArray_obs,pub_mark_obs,zobs,P)
    #pub_flight_mode.publish(FlightModeInputObj.get_msg());
    #trajLQRRRT=param.SetLQRRRT(trajectory,trajectorydot)
    #rospy.signal_shutdown("Shutdown Node")
    return trajectory, trajectorydot, u_traj, param, solution

#***********************************************************************************************************************
def main():
    # ROS Init
    rospy.init_node('traj_smoother', anonymous=True)
    np.random.seed(741601)
    # start publisher here
    #reswarm_planner_pub("/reswarm/planner_lqrrrt/status", ReswarmPlanner)
    pub_lqrrrt_status=rospy.Publisher('/reswarm/planner_lqrrrt/status',ReswarmPlannerStatus,queue_size=1,latch=True)
    pub = rospy.Publisher('~chatter_topic', String, queue_size=10);
    #planner_node = PlannerNode()
    StatusInputObj = StatusInput()  # use either of these

    asdfasdf = rospy.Subscriber('~status_msg_name', numpy_msg(ReswarmStatusPrimary), StatusInputObj);
    rospy.sleep(4/5)
    #asdfasdf = rospy.Subscriber('~status_msg_name', ReswarmStatusPrimary, planner_node.status_callback)
    #rospy.logwarn(asdfasdf.resolved_name+"  -  "+asdfasdf.name)
    StatusUpdate = StatusInputObj.get_msg();
    test_num=StatusUpdate.test_number;
    if test_num==4:
        flag_ground='true';
    else:
        flag_ground = rospy.get_param('/reswarm/ground');

    standardortube='standard';

    while not rospy.is_shutdown():
        planner_finished = False;
        StatusUpdate = StatusInputObj.get_msg();
        test_num=StatusUpdate.test_number;
        start_planner = StatusUpdate.lqrrrt_activated;
        #print(start_planner)
        rospy.sleep(3/5)
        #print(test_num);
        #rospy.logwarn("Is the planner finished: " + str(planner_finished))
        if (5<=test_num<=8) and (not planner_finished) and start_planner:
            StateInputObj=StateInput();
            asdf=rospy.Subscriber('~ekf_topic',numpy_msg(EkfState),StateInputObj);
            rospy.sleep(3/5)
            state=StateInputObj.get_msg();
            pos=np.array([[state.pose.position.x,state.pose.position.y, state.pose.position.z]]);
            orient=np.array([[state.pose.orientation.x,state.pose.orientation.y, state.pose.orientation.z,state.pose.orientation.w]]);
            linvel=np.array([[0.,0., 0.]]);
            angvel=np.array([[0.,0., 0.]]);
            linacc=np.array([[state.accel.x,state.accel.y, state.accel.z]]);
            angacc=np.array([[0,0, 0]]);
            rospy.sleep(3/5)
            #x0=np.bmat([[pos,linvel,orient,angvel]]).A.T;
            #print(x0)
            if (flag_ground=='false'):
                #11.25 -5.25 4.49 Pos
                #xrandmin=np.array([10.25, -9.85, 4.1]).T;
                #xrandmax=np.array([11.55, -3.45, 5.7]).T;
                POINT_A_ISS = rospy.get_param("/reswarm/primary/point_a_iss");
                x0=np.hstack(([POINT_A_ISS[0:3]], linvel, [POINT_A_ISS[3:7]], angvel)).T;
                #print(x0)
                if test_num == 5:
                    xdes = np.array([[10.8, -7.75, 5.2, 0., 0., 0., 0., 0., -0.7071068, 0.7071068, 0., 0., 0.]]).T; # Final State
                    zobs = np.array([[11.0,-8.65,4.75],[10.5,-8.65,4.75],[10.75,-7.5,4.75],[10.75,-9.8,4.75],[10.75,-8.65,5.3],[10.75,-8.65,4.2]]).T; # Ellipsoid obstacle State
                    Nobs=6; # Number of Obstacles
                    ypos=np.array([[pos[0,1],xdes[1,0]]]);
                    randArea = np.array([10.5, np.amin(ypos), 4.2, 11.0, np.amax(ypos), 5.3]);
                    P1 = np.array([[.15 ** (-2), 0, 0], [0, 1.15 ** (-2), 0], [0, 0, .55 ** (-2)]]);
                    P2 = np.array([[.25 ** (-2), 0, 0], [0, .03 ** (-2), 0], [0, 0, .55 ** (-2)]]);
                    P3 = np.array([[.25 ** (-2), 0, 0], [0, 1.15 ** (-2), 0], [0, 0, .05 ** (-2)]]);
                    P=np.dstack((P1,P1,P2,P2,P3,P3));
                elif test_num == 6:
                    xdes = np.array([[10.8, -7.75, 5.2, 0., 0., 0., 0., 0., -0.7071068, 0.7071068, 0., 0., 0.]]).T; # Final State
                    zobs = np.array([[10.6, -8.75, 5],[11.0,-8.65,4.75],[10.5,-8.65,4.75],[10.75,-7.5,4.75],[10.75,-9.8,4.75],[10.75,-8.65,5.3],[10.75,-8.65,4.2]]).T; # Ellipsoid obstacle State
                    Nobs=zobs.shape[1]; # Number of Obstacles
                    ypos=np.array([[pos[0,1],xdes[1,0]]]);
                    randArea = np.array([10.5, np.amin(ypos), 4.2, 11.0, np.amax(ypos), 5.3]);
                    P0 = .55 ** (-2) * np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]]); # Shape of Obstacle
                    P1 = np.array([[.15 ** (-2), 0, 0], [0, 1.15 ** (-2), 0], [0, 0, .55 ** (-2)]]);
                    P2 = np.array([[.25 ** (-2), 0, 0], [0, .03 ** (-2), 0], [0, 0, .55 ** (-2)]]);
                    P3 = np.array([[.25 ** (-2), 0, 0], [0, 1.15 ** (-2), 0], [0, 0, .05 ** (-2)]]);
                    P=np.dstack((P0,P1,P1,P2,P2,P3,P3));
                elif test_num == 7:
                    xdes = np.array([[10.8, -7.75, 5.2, 0., 0., 0., 0., 0., -0.7071068, 0.7071068, 0., 0., 0.]]).T; # Final State
                    zobs = np.array([[10.6, -8.75, 5],[11.0,-8.65,4.75],[10.5,-8.65,4.75],[10.75,-7.5,4.75],[10.75,-9.8,4.75],[10.75,-8.65,5.3],[10.75,-8.65,4.2]]).T; # Ellipsoid obstacle State
                    Nobs=zobs.shape[1]; # Number of Obstacles
                    ypos=np.array([[pos[0,1],xdes[1,0]]]);
                    randArea = np.array([10.5, np.amin(ypos), 4.2, 11.0, np.amax(ypos), 5.3]);
                    P0 = .55 ** (-2) * np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]]); # Shape of Obstacle
                    P1 = np.array([[.15 ** (-2), 0, 0], [0, 1.15 ** (-2), 0], [0, 0, .55 ** (-2)]]);
                    P2 = np.array([[.25 ** (-2), 0, 0], [0, .03 ** (-2), 0], [0, 0, .55 ** (-2)]]);
                    P3 = np.array([[.25 ** (-2), 0, 0], [0, 1.15 ** (-2), 0], [0, 0, .05 ** (-2)]]);
                    P=np.dstack((P0,P1,P1,P2,P2,P3,P3));
                elif test_num == 8:
                    xdes = np.array([[10.8, -7.75, 5.2, 0., 0., 0., 0., 0., -0.7071068, 0.7071068, 0., 0., 0.]]).T; # Final State
                    zobs = np.array([[10.6, -8.75, 5],[11.0,-8.65,4.75],[10.5,-8.65,4.75],[10.75,-7.5,4.75],[10.75,-9.8,4.75],[10.75,-8.65,5.3],[10.75,-8.65,4.2]]).T; # Ellipsoid obstacle State
                    Nobs=zobs.shape[1]; # Number of Obstacles
                    ypos=np.array([[pos[0,1],xdes[1,0]]]);
                    randArea = np.array([10.5, np.amin(ypos), 4.2, 11.0, np.amax(ypos), 5.3]);
                    P0 = .55 ** (-2) * np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]]); # Shape of Obstacle
                    P1 = np.array([[.15 ** (-2), 0, 0], [0, 1.15 ** (-2), 0], [0, 0, .55 ** (-2)]]);
                    P2 = np.array([[.25 ** (-2), 0, 0], [0, .03 ** (-2), 0], [0, 0, .55 ** (-2)]]);
                    P3 = np.array([[.25 ** (-2), 0, 0], [0, 1.15 ** (-2), 0], [0, 0, .05 ** (-2)]]);
                    P=np.dstack((P0,P1,P1,P2,P2,P3,P3));
            else:
                POINT_A_GRANITE = rospy.get_param("/reswarm/primary/point_a_granite")
                #print(POINT_A_GRANITE[0:3], linvel)
                x0=np.hstack(([POINT_A_GRANITE[0:3]], linvel, [POINT_A_GRANITE[3:7]], angvel)).T;
                if test_num == 5:
                    xdes = np.array([[-0.4, -0.4, -0.76, 0., 0., 0., 0., 0., 0., 1, 0., 0., 0.]]).T; # Final State
                    zobs = np.array([[.85, 0., 0.],[-.85, 0., 0.],[0.,.85,0.],[0.,-.85,0.]]).T; # Ellipsoid obstacle State
                    Nobs=4; # Number of Obstacles
                    xpos=np.array([[pos[0,0],xdes[0,0]]]);
                    ypos=np.array([[pos[0,1],xdes[1,0]]]);
                    randArea = np.array([np.amin(xpos), np.amin(ypos), -.76, np.amax(xpos), np.amax(ypos), -.76]);
                    P1 = np.array([[.15 ** (-2), 0, 0], [0, 1. ** (-2), 0], [0, 0, 1. ** (-2)]]); # Shape of Obstacle
                    P2 = np.array([[1. ** (-2), 0, 0], [0, .15 ** (-2), 0], [0, 0, 1. ** (-2)]]);
                    P=np.dstack((P1,P1,P2,P2));
                elif test_num == 6:
                    xdes = np.array([[-0.4, -0.4, -0.76, 0., 0., 0., 0., 0., 0., 1, 0., 0., 0.]]).T; # Final State
                    zobs = np.array([[-0.6, .2, -.76],[.85, 0., 0.],[-.85, 0., 0.],[0.,.85,0.],[0.,-.85,0.]]).T; # Ellipsoid obstacle State
                    Nobs=zobs.shape[1]; # Number of Obstacles
                    xpos=np.array([[pos[0,0],xdes[0,0]]]);
                    ypos=np.array([[pos[0,1],xdes[1,0]]]);
                    randArea = np.array([np.amin(xpos), np.amin(ypos), -.76, np.amax(xpos), np.amax(ypos), -.76]);
                    P0 = .55 ** (-2) * np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]]); # Shape of Obstacle
                    P1 = np.array([[.15 ** (-2), 0, 0], [0, 1. ** (-2), 0], [0, 0, 1. ** (-2)]]); # Shape of Obstacle
                    P2 = np.array([[1. ** (-2), 0, 0], [0, .15 ** (-2), 0], [0, 0, 1. ** (-2)]]);
                    P=np.dstack((P0,P1,P1,P2,P2));
                elif test_num == 7:
                    xdes = np.array([[-0.4, -0.4, -0.76, 0., 0., 0., 0., 0., 0., 1, 0., 0., 0.]]).T; # Final State
                    zobs = np.array([[-0.6, .2, -.76],[.85, 0., 0.],[-.85, 0., 0.],[0.,-.85,0.]]).T; # Ellipsoid obstacle State
                    Nobs=zobs.shape[1]; # Number of Obstacles
                    xpos=np.array([[pos[0,0],xdes[0,0]]]);
                    ypos=np.array([[pos[0,1],xdes[1,0]]]);
                    randArea = np.array([np.amin(xpos), np.amin(ypos), -.76, np.amax(xpos), np.amax(ypos), -.76]);
                    P0 = .55 ** (-2) * np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]]); # Shape of Obstacle
                    P1 = np.array([[.15 ** (-2), 0, 0], [0, 1. ** (-2), 0], [0, 0, 1. ** (-2)]]); # Shape of Obstacle
                    P2 = np.array([[1. ** (-2), 0, 0], [0, .15 ** (-2), 0], [0, 0, 1. ** (-2)]]);
                    P=np.dstack((P0,P1,P1,P2));
                elif test_num == 8:
                    xdes = np.array([[-0.4, -0.4, -0.76, 0., 0., 0., 0., 0., 0., 1, 0., 0., 0.]]).T; # Final State
                    zobs = np.array([[-0.6, .2, -.76],[.85, 0., 0.],[-.85, 0., 0.],[0.,-.85,0.]]).T; # Ellipsoid obstacle State
                    Nobs=zobs.shape[1]; # Number of Obstacles
                    xpos=np.array([[pos[0,0],xdes[0,0]]]);
                    ypos=np.array([[pos[0,1],xdes[1,0]]]);
                    randArea = np.array([np.amin(xpos), np.amin(ypos), -.76, np.amax(xpos), np.amax(ypos), -.76]);
                    P0 = .55 ** (-2) * np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]]); # Shape of Obstacle
                    P1 = np.array([[.15 ** (-2), 0, 0], [0, 1. ** (-2), 0], [0, 0, 1. ** (-2)]]); # Shape of Obstacle
                    P2 = np.array([[1. ** (-2), 0, 0], [0, .15 ** (-2), 0], [0, 0, 1. ** (-2)]]);
                    P=np.dstack((P0,P1,P1,P2));
            np.random.seed(741601)
            traj, trajdot, u_traj, param, solution = rrtlqrfulldyn(xdes,zobs,Nobs,x0,randArea,test_num,P,flag_ground);

            lqr_done_str = 'LQRRRT* Done'
            rospy.loginfo(lqr_done_str)
            pub.publish(lqr_done_str)

            #Initialize Input msg Callbacks
            InertiaInputObj=InertiaInput();
            StateInputObj=StateInput();
            FlightModeInputObj=FlightModeInput();
            markerArray = MarkerArray()

            #Initialize Subscribers
            rospy.Subscriber('~flight_mode_topic',numpy_msg(FlightMode),FlightModeInputObj);

            #Initialize Publishers
            pub_flight_mode=rospy.Publisher('~flight_mode_topic',FlightMode,queue_size=1,latch=True);
            pub_mark = rospy.Publisher('~marker_array2_topic', MarkerArray,queue_size=1)
            rospy.sleep(1/5)

            dt=param.DT*param.step_size; # Time-step
            time=np.linspace(0,traj.shape[1]*dt,traj.shape[1]); # Time of Simulation
            #t0 = ti.time(); #tic
            np.random.seed(741601)
            paramsmooth=smoother(50,param,flag_ground); # Initialize Trajectory Smoother
            traj_smooth,time_smooth,traj_lqr,xref_lqr,u_lqr=paramsmooth.run(param,traj,time,u_traj); # Run Trajectory Smoother
            traj_lqrdot = param.cfinitediff(traj_lqr,u_lqr);
            #t1 = ti.time(); #toc
            #print(t1-t0)

            plot_trajp(markerArray,pub_mark,traj_lqr)
            paramsmooth.PublishLQRRRT(traj,trajdot);

            smoother_done_str = 'Traj Smoother Done'
            rospy.loginfo(smoother_done_str)
            pub.publish(smoother_done_str)

            planner_finished=True;
            # publish somewhere down here!
            traj_msg=ReswarmPlannerStatus();
            traj_msg.stamp=rospy.Time.now();
            traj_msg.planner_finished=planner_finished;
            traj_msg.sent_robustMPC=False;
            traj_msg.sent_PID=False;
            pub_lqrrrt_status.publish(traj_msg);
            planner_done_str = 'Meow Planner is Done'
            rospy.loginfo(planner_done_str)
            pub.publish(planner_done_str)
            if test_num==5:
                pub_flight_mode.publish(FlightModeInputObj.get_msg());
                paramsmooth.SetControlState(traj_lqr,traj_lqrdot);
                traj_msg.sent_PID=True;
                ctl_done_str = 'Bark Control is Done'
            elif test_num==6:
                pub_flight_mode.publish(FlightModeInputObj.get_msg());
                paramsmooth.SetControlState(traj_lqr,traj_lqrdot);
                traj_msg.sent_PID=True;
                ctl_done_str = 'Bark Control is Done'
            elif test_num==7:
                paramsmooth.SetControlStateRobustTubeMPC(traj_lqr,traj_lqrdot);
                traj_msg.sent_robustMPC=True;
                ctl_done_str = 'Bark Control is sent'
            elif test_num==8:
                paramsmooth.SetControlStateRobustTubeMPC(traj_lqr,traj_lqrdot);
                traj_msg.sent_robustMPC=True;
                ctl_done_str = 'Bark Control is sent'
            pub_lqrrrt_status.publish(traj_msg);
            rospy.loginfo(ctl_done_str)
            pub.publish(ctl_done_str)
            rospy.sleep(1)
        elif (test_num==13) and (not planner_finished) and start_planner:
            #rospy.logwarn(str(start_planner));
            #rospy.logwarn(str(planner_finished));
            StateInputObj=StateInput();
            asdf=rospy.Subscriber('~ekf_topic',numpy_msg(EkfState),StateInputObj);
            rospy.sleep(3/5)
            state=StateInputObj.get_msg();
            pos=np.array([[state.pose.position.x,state.pose.position.y, state.pose.position.z]]);
            orient=np.array([[state.pose.orientation.x,state.pose.orientation.y, state.pose.orientation.z,state.pose.orientation.w]]);
            linvel=np.array([[0.,0., 0.]]);
            angvel=np.array([[0.,0., 0.]]);
            linacc=np.array([[state.accel.x,state.accel.y, state.accel.z]]);
            angacc=np.array([[0,0, 0]]);
            rospy.sleep(3/5)
            #x0=np.bmat([[pos,linvel,orient,angvel]]).A.T;
            #print(x0)

            if (flag_ground=='false'):
                if standardortube == 'standard':
                    POINT_A_ISS = rospy.get_param("/reswarm/primary/point_a_iss");
                    x0=np.hstack(([POINT_A_ISS[0:3]], linvel, [POINT_A_ISS[3:7]], angvel)).T;
                    print(x0)
                    xdes = np.array([[10.8, -7.75, 5.2, 0., 0., 0., 0., 0., -0.7071068, 0.7071068, 0., 0., 0.]]).T; # Final State
                    zobs = np.array([[10.6, -8.75, 5],[11.0,-8.65,4.75],[10.5,-8.65,4.75],[10.75,-7.5,4.75],[10.75,-9.8,4.75],[10.75,-8.65,5.3],[10.75,-8.65,4.2]]).T; # Ellipsoid obstacle State
                    Nobs=zobs.shape[1]; # Number of Obstacles
                    ypos=np.array([[pos[0,1],xdes[1,0]]]);
                    randArea = np.array([10.5, np.amin(ypos), 4.2, 11.0, np.amax(ypos), 5.3]);
                    P0 = .55 ** (-2) * np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]]); # Shape of Obstacle
                    P1 = np.array([[.15 ** (-2), 0, 0], [0, 1.15 ** (-2), 0], [0, 0, .55 ** (-2)]]);
                    P2 = np.array([[.25 ** (-2), 0, 0], [0, .03 ** (-2), 0], [0, 0, .55 ** (-2)]]);
                    P3 = np.array([[.25 ** (-2), 0, 0], [0, 1.15 ** (-2), 0], [0, 0, .05 ** (-2)]]);
                    P=np.dstack((P0,P1,P1,P2,P2,P3,P3));
                elif standardortube == 'tube':
                    POINT_C_ISS = rospy.get_param("/reswarm/primary/point_c_iss");
                    x0=np.hstack(([[POINT_C_ISS[0:3]], linvel, [POINT_C_ISS[3:7]] ,angvel])).T;
                    print(x0)
                    xdes = np.array([[10.8, -9.6, 4.8, 0., 0., 0., 0., 0., -0.7071068, 0.7071068, 0., 0., 0.]]).T; # Final State
                    zobs = np.array([[10.6, -8.75, 5],[11.0,-8.65,4.75],[10.5,-8.65,4.75],[10.75,-7.5,4.75],[10.75,-9.8,4.75],[10.75,-8.65,5.3],[10.75,-8.65,4.2]]).T; # Ellipsoid obstacle State
                    Nobs=zobs.shape[1]; # Number of Obstacles
                    ypos=np.array([[pos[0,1],xdes[1,0]]]);
                    randArea = np.array([10.5, np.amin(ypos), 4.2, 11.0, np.amax(ypos), 5.3]);
                    P0 = .55 ** (-2) * np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]]); # Shape of Obstacle
                    P1 = np.array([[.15 ** (-2), 0, 0], [0, 1.15 ** (-2), 0], [0, 0, .55 ** (-2)]]);
                    P2 = np.array([[.25 ** (-2), 0, 0], [0, .03 ** (-2), 0], [0, 0, .55 ** (-2)]]);
                    P3 = np.array([[.25 ** (-2), 0, 0], [0, 1.15 ** (-2), 0], [0, 0, .05 ** (-2)]]);
                    P=np.dstack((P0,P1,P1,P2,P2,P3,P3));
            else:
                if standardortube == 'standard':
                    POINT_A_GRANITE = rospy.get_param("/reswarm/primary/point_a_granite")
                    x0=np.hstack(([POINT_A_GRANITE[0:3]], linvel, [POINT_A_GRANITE[3:7]], angvel)).T;
                    print(x0)
                    xdes = np.array([[-0.4, -0.4, -0.76, 0., 0., 0., 0., 0., 0., 1, 0., 0., 0.]]).T; # Final State
                    zobs = np.array([[-0.6, .2, -.76],[.85, 0., 0.],[-.85, 0., 0.],[0.,.85,0.],[0.,-.85,0.]]).T; # Ellipsoid obstacle State
                    Nobs=zobs.shape[1]; # Number of Obstacles
                    xpos=np.array([[pos[0,0],xdes[0,0]]]);
                    ypos=np.array([[pos[0,1],xdes[1,0]]]);
                    randArea = np.array([np.amin(xpos), np.amin(ypos), -.76, np.amax(xpos), np.amax(ypos), -.76]);
                    P0 = .55 ** (-2) * np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]]); # Shape of Obstacle
                    P1 = np.array([[.15 ** (-2), 0, 0], [0, 1. ** (-2), 0], [0, 0, 1. ** (-2)]]); # Shape of Obstacle
                    P2 = np.array([[1. ** (-2), 0, 0], [0, .15 ** (-2), 0], [0, 0, 1. ** (-2)]]);
                    P=np.dstack((P0,P1,P1,P2,P2));
                elif standardortube == 'tube':
                    POINT_C_GRANITE = rospy.get_param("/reswarm/primary/point_c_granite")
                    x0=np.hstack(([POINT_C_GRANITE[0:3]], linvel, [POINT_C_GRANITE[3:7]], angvel)).T;
                    print(x0)
                    xdes = np.array([[0.0, 0.6, -0.76, 0., 0., 0., 0., 0., 1., 0., 0., 0., 0.]]).T; # Final State
                    zobs = np.array([[-0.6, .2, -.76],[.85, 0., 0.],[-.85, 0., 0.],[0.,.85,0.],[0.,-.85,0.]]).T; # Ellipsoid obstacle State
                    Nobs=zobs.shape[1]; # Number of Obstacles
                    xpos=np.array([[pos[0,0],xdes[0,0]]]);
                    ypos=np.array([[pos[0,1],xdes[1,0]]]);
                    randArea = np.array([np.amin(xpos), np.amin(ypos), -.76, np.amax(xpos), np.amax(ypos), -.76]);
                    P0 = .55 ** (-2) * np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]]); # Shape of Obstacle
                    P1 = np.array([[.15 ** (-2), 0, 0], [0, 1. ** (-2), 0], [0, 0, 1. ** (-2)]]); # Shape of Obstacle
                    P2 = np.array([[1. ** (-2), 0, 0], [0, .15 ** (-2), 0], [0, 0, 1. ** (-2)]]);
                    P=np.dstack((P0,P1,P1,P2,P2));
            np.random.seed(8301994)
            traj, trajdot, u_traj, param, solution = rrtlqrfulldyn(xdes,zobs,Nobs,x0,randArea,test_num,P,flag_ground);

            lqr_done_str = 'LQRRRT* Done'
            rospy.loginfo(lqr_done_str)
            pub.publish(lqr_done_str)

            #Initialize Input msg Callbacks
            InertiaInputObj=InertiaInput();
            StateInputObj=StateInput();
            FlightModeInputObj=FlightModeInput();
            markerArray = MarkerArray()

            #Initialize Subscribers
            rospy.Subscriber('~flight_mode_topic',numpy_msg(FlightMode),FlightModeInputObj);

            #Initialize Publishers
            pub_flight_mode=rospy.Publisher('~flight_mode_topic',FlightMode,queue_size=1,latch=True);
            pub_mark = rospy.Publisher('~marker_array2_topic', MarkerArray,queue_size=1)
            rospy.sleep(1/5)

            dt=param.DT*param.step_size; # Time-step
            time=np.linspace(0,traj.shape[1]*dt,traj.shape[1]); # Time of Simulation
            #t0 = ti.time(); #tic
            np.random.seed(741601)
            paramsmooth=smoother(50,param,flag_ground); # Initialize Trajectory Smoother
            traj_smooth,time_smooth,traj_lqr,xref_lqr,u_lqr=paramsmooth.run(param,traj,time,u_traj); # Run Trajectory Smoother
            traj_lqrdot = param.cfinitediff(traj_lqr,u_lqr);
            #t1 = ti.time(); #toc
            #print(t1-t0)

            plot_trajp(markerArray,pub_mark,traj_lqr)
            paramsmooth.PublishLQRRRT(traj,trajdot);

            smoother_done_str = 'Traj Smoother Done'
            rospy.loginfo(smoother_done_str)
            pub.publish(smoother_done_str)

            planner_finished=True;
            # publish somewhere down here!
            traj_msg=ReswarmPlannerStatus();
            traj_msg.stamp=rospy.Time.now();
            traj_msg.planner_finished=planner_finished;
            traj_msg.sent_robustMPC=False;
            traj_msg.sent_PID=False;
            pub_lqrrrt_status.publish(traj_msg);
            planner_done_str = 'Meow Planner is Done'
            rospy.loginfo(planner_done_str)
            pub.publish(planner_done_str)
            if standardortube == 'standard':
                paramsmooth.SetControlStateRobustTubeMPC(traj_lqr,traj_lqrdot);
                traj_msg.sent_robustMPC=True;
                ctl_done_str = 'Bark Standard Control is sent'
                standardortube= 'tube';
            elif standardortube == 'tube':
                paramsmooth.SetControlStateRobustTubeMPC(traj_lqr,traj_lqrdot);
                traj_msg.sent_robustMPC=True;
                ctl_done_str = 'Bark Tube Control is sent'
                standardortube= 'standard';
            pub_lqrrrt_status.publish(traj_msg);
            rospy.loginfo(ctl_done_str)
            pub.publish(ctl_done_str)
            rospy.sleep(1)
        elif (test_num==4) and (not planner_finished) and start_planner:
            #print('TESTING44444444444')
            pos=np.array([[1,1, 1]]);
            orient=np.array([[0,0, 0,-1]]);
            linvel=np.array([[0,0, 0]]);
            angvel=np.array([[0,0, 0]]);
            linacc=np.array([[0,0, 0]]);
            angacc=np.array([[0,0, 0]]);
            x0=np.bmat([[pos,linvel,orient,angvel]]).A.T;
            xdes = np.array([[2, 2, 2, 0., 0., 0., 0., 0., 0., 1, 0., 0., 0.]]).T; # Final State
            zobs = np.array([[1125., -895., 449.]]).T; # Ellipsoid obstacle State
            P = .55 ** (-2) * np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]]);
            P=np.repeat(P[:, :, np.newaxis], zobs.shape[1], axis=2);
            Nobs=0; # Number of Obstacles
            xpos=np.array([[pos[0,0],xdes[0,0]]]);
            ypos=np.array([[pos[0,1],xdes[1,0]]]);
            zpos=np.array([[pos[0,2],xdes[2,0]]]);
            randArea = np.array([np.amin(xpos), np.amin(ypos), np.amin(zpos), np.amax(xpos), np.amax(ypos), np.amax(zpos)]);
            traj, trajdot, u_traj, param, solution = rrtlqrfulldyn(xdes,zobs,Nobs,x0,randArea,test_num,P,flag_ground);
            lqr_done_str = 'LQRRRT* Done'
            rospy.loginfo(lqr_done_str)
            pub.publish(lqr_done_str)

            #Initialize Input msg Callbacks
            markerArray = MarkerArray()

            #Initialize Publishers
            pub_mark = rospy.Publisher('~marker_array2_topic', MarkerArray,queue_size=1)
            rospy.sleep(1/5)

            dt=param.DT*param.step_size; # Time-step
            time=np.linspace(0,traj.shape[1]*dt,traj.shape[1]); # Time of Simulation
            paramsmooth=smoother(50,param,flag_ground); # Initialize Trajectory Smoother
            traj_smooth,time_smooth,traj_lqr,xref_lqr,u_lqr=paramsmooth.run(param,traj,time,u_traj); # Run Trajectory Smoother
            traj_lqrdot = param.cfinitediff(traj_lqr,u_lqr);

            plot_trajp(markerArray,pub_mark,traj_lqr)
            paramsmooth.PublishLQRRRT(traj,trajdot);

            smoother_done_str = 'Traj Smoother Done'
            rospy.loginfo(smoother_done_str)
            pub.publish(smoother_done_str)

            planner_finished=True;
            # publish somewhere down here!
            traj_msg=ReswarmPlannerStatus();
            traj_msg.stamp=rospy.Time.now();
            traj_msg.planner_finished=planner_finished;
            traj_msg.sent_robustMPC=False;
            traj_msg.sent_PID=False;
            pub_lqrrrt_status.publish(traj_msg);
            planner_done_str = 'meow planner is done'
            rospy.loginfo(planner_done_str)
            pub.publish(planner_done_str)
            ctl_done_str = 'Send to Control (Not implemented in Unit Test)'
            rospy.loginfo(ctl_done_str)
            pub.publish(ctl_done_str)
            rospy.sleep(1)
        else:
            pass;

if __name__ == '__main__':
    main()

