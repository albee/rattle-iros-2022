/*
*    This file is part of ACADO Toolkit.
*
*    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
*    Copyright (C) 2008-2009 by Boris Houska and Hans Joachim Ferreau, K.U.Leuven.
*    Developed within the Optimization in Engineering Center (OPTEC) under
*    supervision of Moritz Diehl. All rights reserved.
*
*    ACADO Toolkit is free software; you can redistribute it and/or
*    modify it under the terms of the GNU Lesser General Public
*    License as published by the Free Software Foundation; either
*    version 3 of the License, or (at your option) any later version.
*
*    ACADO Toolkit is distributed in the hope that it will be useful,
*    but WITHOUT ANY WARRANTY; without even the implied warranty of
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
*    Lesser General Public License for more details.
*
*    You should have received a copy of the GNU Lesser General Public
*    License along with ACADO Toolkit; if not, write to the Free Software
*    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*
*/


/**
*    Author David Ariens, Rien Quirynen
*    Date 2009-2013
*    http://www.acadotoolkit.org/matlab 
*/

#include <acado_optimal_control.hpp>
#include <acado_toolkit.hpp>
#include <acado/utils/matlab_acado_utils.hpp>

USING_NAMESPACE_ACADO

#include <mex.h>


void mexFunction( int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[] ) 
 { 
 
    MatlabConsoleStreamBuf mybuf;
    RedirectStream redirect(std::cout, mybuf);
    clearAllStaticCounters( ); 
 
    mexPrintf("\nACADO Toolkit for Matlab - Developed by David Ariens and Rien Quirynen, 2009-2013 \n"); 
    mexPrintf("Support available at http://www.acadotoolkit.org/matlab \n \n"); 

    if (nrhs != 0){ 
      mexErrMsgTxt("This problem expects 0 right hand side argument(s) since you have defined 0 MexInput(s)");
    } 
 
    TIME autotime;
    DifferentialState r1;
    DifferentialState r2;
    DifferentialState r3;
    DifferentialState v1;
    DifferentialState v2;
    DifferentialState v3;
    DifferentialState q1;
    DifferentialState q2;
    DifferentialState q3;
    DifferentialState q4;
    DifferentialState w1;
    DifferentialState w2;
    DifferentialState w3;
    Control u1;
    Control u2;
    Control u3;
    Control u4;
    Control u5;
    Control u6;
    OnlineData mass; 
    OnlineData I_xx; 
    OnlineData I_yy; 
    OnlineData I_zz; 
    OnlineData I_xy; 
    OnlineData I_yz; 
    OnlineData I_xz; 
    OnlineData roff1; 
    OnlineData roff2; 
    OnlineData roff3; 
    OnlineData r_des1; 
    OnlineData r_des2; 
    OnlineData r_des3; 
    OnlineData v_des1; 
    OnlineData v_des2; 
    OnlineData v_des3; 
    OnlineData q_des1; 
    OnlineData q_des2; 
    OnlineData q_des3; 
    OnlineData q_des4; 
    OnlineData w_des1; 
    OnlineData w_des2; 
    OnlineData w_des3; 
    OnlineData u_des1; 
    OnlineData u_des2; 
    OnlineData u_des3; 
    OnlineData u_des4; 
    OnlineData u_des5; 
    OnlineData u_des6; 
    BMatrix acadodata_M1;
    acadodata_M1.read( "test_data_acadodata_M1.txt" );
    BMatrix acadodata_M2;
    acadodata_M2.read( "test_data_acadodata_M2.txt" );
    Function acadodata_f1;
    acadodata_f1 << (r1-r_des1);
    acadodata_f1 << (r2-r_des2);
    acadodata_f1 << (r3-r_des3);
    acadodata_f1 << (v1-v_des1);
    acadodata_f1 << (v2-v_des2);
    acadodata_f1 << (v3-v_des3);
    acadodata_f1 << ((-pow(q1,2.00000000000000000000e+00)+pow(q2,2.00000000000000000000e+00)-pow(q3,2.00000000000000000000e+00)+pow(q4,2.00000000000000000000e+00))*(-q_des1*q_des4+q_des2*q_des3)*2.00000000000000000000e+00-(-pow(q1,2.00000000000000000000e+00)-pow(q2,2.00000000000000000000e+00)+pow(q3,2.00000000000000000000e+00)+pow(q4,2.00000000000000000000e+00))*(q_des1*q_des4+q_des2*q_des3)*2.00000000000000000000e+00-(-pow(q_des1,2.00000000000000000000e+00)+pow(q_des2,2.00000000000000000000e+00)-pow(q_des3,2.00000000000000000000e+00)+pow(q_des4,2.00000000000000000000e+00))*(-q1*q4+q2*q3)*2.00000000000000000000e+00+(-pow(q_des1,2.00000000000000000000e+00)-pow(q_des2,2.00000000000000000000e+00)+pow(q_des3,2.00000000000000000000e+00)+pow(q_des4,2.00000000000000000000e+00))*(q1*q4+q2*q3)*2.00000000000000000000e+00+(q1*q2-q3*q4)*(q_des1*q_des3+q_des2*q_des4)*2.00000000000000000000e+00*2.00000000000000000000e+00-(q1*q3+q2*q4)*(q_des1*q_des2-q_des3*q_des4)*2.00000000000000000000e+00*2.00000000000000000000e+00)/2.00000000000000000000e+00/sqrt(((-pow(q1,2.00000000000000000000e+00)+pow(q2,2.00000000000000000000e+00)-pow(q3,2.00000000000000000000e+00)+pow(q4,2.00000000000000000000e+00))*(-pow(q_des1,2.00000000000000000000e+00)+pow(q_des2,2.00000000000000000000e+00)-pow(q_des3,2.00000000000000000000e+00)+pow(q_des4,2.00000000000000000000e+00))+(-pow(q1,2.00000000000000000000e+00)-pow(q2,2.00000000000000000000e+00)+pow(q3,2.00000000000000000000e+00)+pow(q4,2.00000000000000000000e+00))*(-pow(q_des1,2.00000000000000000000e+00)-pow(q_des2,2.00000000000000000000e+00)+pow(q_des3,2.00000000000000000000e+00)+pow(q_des4,2.00000000000000000000e+00))+(-q1*q4+q2*q3)*(-q_des1*q_des4+q_des2*q_des3)*2.00000000000000000000e+00*2.00000000000000000000e+00+(pow(q1,2.00000000000000000000e+00)-pow(q2,2.00000000000000000000e+00)-pow(q3,2.00000000000000000000e+00)+pow(q4,2.00000000000000000000e+00))*(pow(q_des1,2.00000000000000000000e+00)-pow(q_des2,2.00000000000000000000e+00)-pow(q_des3,2.00000000000000000000e+00)+pow(q_des4,2.00000000000000000000e+00))+(q1*q2+q3*q4)*(q_des1*q_des2+q_des3*q_des4)*2.00000000000000000000e+00*2.00000000000000000000e+00+(q1*q2-q3*q4)*(q_des1*q_des2-q_des3*q_des4)*2.00000000000000000000e+00*2.00000000000000000000e+00+(q1*q3+q2*q4)*(q_des1*q_des3+q_des2*q_des4)*2.00000000000000000000e+00*2.00000000000000000000e+00+(q1*q3-q2*q4)*(q_des1*q_des3-q_des2*q_des4)*2.00000000000000000000e+00*2.00000000000000000000e+00+(q1*q4+q2*q3)*(q_des1*q_des4+q_des2*q_des3)*2.00000000000000000000e+00*2.00000000000000000000e+00+1.00000000000000000000e+00));
    acadodata_f1 << ((-pow(q1,2.00000000000000000000e+00)-pow(q2,2.00000000000000000000e+00)+pow(q3,2.00000000000000000000e+00)+pow(q4,2.00000000000000000000e+00))*(q_des1*q_des3-q_des2*q_des4)*2.00000000000000000000e+00-(-pow(q_des1,2.00000000000000000000e+00)-pow(q_des2,2.00000000000000000000e+00)+pow(q_des3,2.00000000000000000000e+00)+pow(q_des4,2.00000000000000000000e+00))*(q1*q3-q2*q4)*2.00000000000000000000e+00+(-q1*q4+q2*q3)*(q_des1*q_des2+q_des3*q_des4)*2.00000000000000000000e+00*2.00000000000000000000e+00-(-q_des1*q_des4+q_des2*q_des3)*(q1*q2+q3*q4)*2.00000000000000000000e+00*2.00000000000000000000e+00-(pow(q1,2.00000000000000000000e+00)-pow(q2,2.00000000000000000000e+00)-pow(q3,2.00000000000000000000e+00)+pow(q4,2.00000000000000000000e+00))*(q_des1*q_des3+q_des2*q_des4)*2.00000000000000000000e+00+(pow(q_des1,2.00000000000000000000e+00)-pow(q_des2,2.00000000000000000000e+00)-pow(q_des3,2.00000000000000000000e+00)+pow(q_des4,2.00000000000000000000e+00))*(q1*q3+q2*q4)*2.00000000000000000000e+00)/2.00000000000000000000e+00/sqrt(((-pow(q1,2.00000000000000000000e+00)+pow(q2,2.00000000000000000000e+00)-pow(q3,2.00000000000000000000e+00)+pow(q4,2.00000000000000000000e+00))*(-pow(q_des1,2.00000000000000000000e+00)+pow(q_des2,2.00000000000000000000e+00)-pow(q_des3,2.00000000000000000000e+00)+pow(q_des4,2.00000000000000000000e+00))+(-pow(q1,2.00000000000000000000e+00)-pow(q2,2.00000000000000000000e+00)+pow(q3,2.00000000000000000000e+00)+pow(q4,2.00000000000000000000e+00))*(-pow(q_des1,2.00000000000000000000e+00)-pow(q_des2,2.00000000000000000000e+00)+pow(q_des3,2.00000000000000000000e+00)+pow(q_des4,2.00000000000000000000e+00))+(-q1*q4+q2*q3)*(-q_des1*q_des4+q_des2*q_des3)*2.00000000000000000000e+00*2.00000000000000000000e+00+(pow(q1,2.00000000000000000000e+00)-pow(q2,2.00000000000000000000e+00)-pow(q3,2.00000000000000000000e+00)+pow(q4,2.00000000000000000000e+00))*(pow(q_des1,2.00000000000000000000e+00)-pow(q_des2,2.00000000000000000000e+00)-pow(q_des3,2.00000000000000000000e+00)+pow(q_des4,2.00000000000000000000e+00))+(q1*q2+q3*q4)*(q_des1*q_des2+q_des3*q_des4)*2.00000000000000000000e+00*2.00000000000000000000e+00+(q1*q2-q3*q4)*(q_des1*q_des2-q_des3*q_des4)*2.00000000000000000000e+00*2.00000000000000000000e+00+(q1*q3+q2*q4)*(q_des1*q_des3+q_des2*q_des4)*2.00000000000000000000e+00*2.00000000000000000000e+00+(q1*q3-q2*q4)*(q_des1*q_des3-q_des2*q_des4)*2.00000000000000000000e+00*2.00000000000000000000e+00+(q1*q4+q2*q3)*(q_des1*q_des4+q_des2*q_des3)*2.00000000000000000000e+00*2.00000000000000000000e+00+1.00000000000000000000e+00));
    acadodata_f1 << (-(-pow(q1,2.00000000000000000000e+00)+pow(q2,2.00000000000000000000e+00)-pow(q3,2.00000000000000000000e+00)+pow(q4,2.00000000000000000000e+00))*(q_des1*q_des2+q_des3*q_des4)*2.00000000000000000000e+00+(-pow(q_des1,2.00000000000000000000e+00)+pow(q_des2,2.00000000000000000000e+00)-pow(q_des3,2.00000000000000000000e+00)+pow(q_des4,2.00000000000000000000e+00))*(q1*q2+q3*q4)*2.00000000000000000000e+00+(pow(q1,2.00000000000000000000e+00)-pow(q2,2.00000000000000000000e+00)-pow(q3,2.00000000000000000000e+00)+pow(q4,2.00000000000000000000e+00))*(q_des1*q_des2-q_des3*q_des4)*2.00000000000000000000e+00-(pow(q_des1,2.00000000000000000000e+00)-pow(q_des2,2.00000000000000000000e+00)-pow(q_des3,2.00000000000000000000e+00)+pow(q_des4,2.00000000000000000000e+00))*(q1*q2-q3*q4)*2.00000000000000000000e+00+(q1*q3-q2*q4)*(q_des1*q_des4+q_des2*q_des3)*2.00000000000000000000e+00*2.00000000000000000000e+00-(q1*q4+q2*q3)*(q_des1*q_des3-q_des2*q_des4)*2.00000000000000000000e+00*2.00000000000000000000e+00)/2.00000000000000000000e+00/sqrt(((-pow(q1,2.00000000000000000000e+00)+pow(q2,2.00000000000000000000e+00)-pow(q3,2.00000000000000000000e+00)+pow(q4,2.00000000000000000000e+00))*(-pow(q_des1,2.00000000000000000000e+00)+pow(q_des2,2.00000000000000000000e+00)-pow(q_des3,2.00000000000000000000e+00)+pow(q_des4,2.00000000000000000000e+00))+(-pow(q1,2.00000000000000000000e+00)-pow(q2,2.00000000000000000000e+00)+pow(q3,2.00000000000000000000e+00)+pow(q4,2.00000000000000000000e+00))*(-pow(q_des1,2.00000000000000000000e+00)-pow(q_des2,2.00000000000000000000e+00)+pow(q_des3,2.00000000000000000000e+00)+pow(q_des4,2.00000000000000000000e+00))+(-q1*q4+q2*q3)*(-q_des1*q_des4+q_des2*q_des3)*2.00000000000000000000e+00*2.00000000000000000000e+00+(pow(q1,2.00000000000000000000e+00)-pow(q2,2.00000000000000000000e+00)-pow(q3,2.00000000000000000000e+00)+pow(q4,2.00000000000000000000e+00))*(pow(q_des1,2.00000000000000000000e+00)-pow(q_des2,2.00000000000000000000e+00)-pow(q_des3,2.00000000000000000000e+00)+pow(q_des4,2.00000000000000000000e+00))+(q1*q2+q3*q4)*(q_des1*q_des2+q_des3*q_des4)*2.00000000000000000000e+00*2.00000000000000000000e+00+(q1*q2-q3*q4)*(q_des1*q_des2-q_des3*q_des4)*2.00000000000000000000e+00*2.00000000000000000000e+00+(q1*q3+q2*q4)*(q_des1*q_des3+q_des2*q_des4)*2.00000000000000000000e+00*2.00000000000000000000e+00+(q1*q3-q2*q4)*(q_des1*q_des3-q_des2*q_des4)*2.00000000000000000000e+00*2.00000000000000000000e+00+(q1*q4+q2*q3)*(q_des1*q_des4+q_des2*q_des3)*2.00000000000000000000e+00*2.00000000000000000000e+00+1.00000000000000000000e+00));
    acadodata_f1 << (w1-w_des1);
    acadodata_f1 << (w2-w_des2);
    acadodata_f1 << (w3-w_des3);
    acadodata_f1 << (u1-u_des1);
    acadodata_f1 << (u2-u_des2);
    acadodata_f1 << (u3-u_des3);
    acadodata_f1 << (u4-u_des4);
    acadodata_f1 << (u5-u_des5);
    acadodata_f1 << (u6-u_des6);
    Function acadodata_f2;
    acadodata_f2 << (r1-r_des1);
    acadodata_f2 << (r2-r_des2);
    acadodata_f2 << (r3-r_des3);
    acadodata_f2 << (v1-v_des1);
    acadodata_f2 << (v2-v_des2);
    acadodata_f2 << (v3-v_des3);
    acadodata_f2 << ((-pow(q1,2.00000000000000000000e+00)+pow(q2,2.00000000000000000000e+00)-pow(q3,2.00000000000000000000e+00)+pow(q4,2.00000000000000000000e+00))*(-q_des1*q_des4+q_des2*q_des3)*2.00000000000000000000e+00-(-pow(q1,2.00000000000000000000e+00)-pow(q2,2.00000000000000000000e+00)+pow(q3,2.00000000000000000000e+00)+pow(q4,2.00000000000000000000e+00))*(q_des1*q_des4+q_des2*q_des3)*2.00000000000000000000e+00-(-pow(q_des1,2.00000000000000000000e+00)+pow(q_des2,2.00000000000000000000e+00)-pow(q_des3,2.00000000000000000000e+00)+pow(q_des4,2.00000000000000000000e+00))*(-q1*q4+q2*q3)*2.00000000000000000000e+00+(-pow(q_des1,2.00000000000000000000e+00)-pow(q_des2,2.00000000000000000000e+00)+pow(q_des3,2.00000000000000000000e+00)+pow(q_des4,2.00000000000000000000e+00))*(q1*q4+q2*q3)*2.00000000000000000000e+00+(q1*q2-q3*q4)*(q_des1*q_des3+q_des2*q_des4)*2.00000000000000000000e+00*2.00000000000000000000e+00-(q1*q3+q2*q4)*(q_des1*q_des2-q_des3*q_des4)*2.00000000000000000000e+00*2.00000000000000000000e+00)/2.00000000000000000000e+00/sqrt(((-pow(q1,2.00000000000000000000e+00)+pow(q2,2.00000000000000000000e+00)-pow(q3,2.00000000000000000000e+00)+pow(q4,2.00000000000000000000e+00))*(-pow(q_des1,2.00000000000000000000e+00)+pow(q_des2,2.00000000000000000000e+00)-pow(q_des3,2.00000000000000000000e+00)+pow(q_des4,2.00000000000000000000e+00))+(-pow(q1,2.00000000000000000000e+00)-pow(q2,2.00000000000000000000e+00)+pow(q3,2.00000000000000000000e+00)+pow(q4,2.00000000000000000000e+00))*(-pow(q_des1,2.00000000000000000000e+00)-pow(q_des2,2.00000000000000000000e+00)+pow(q_des3,2.00000000000000000000e+00)+pow(q_des4,2.00000000000000000000e+00))+(-q1*q4+q2*q3)*(-q_des1*q_des4+q_des2*q_des3)*2.00000000000000000000e+00*2.00000000000000000000e+00+(pow(q1,2.00000000000000000000e+00)-pow(q2,2.00000000000000000000e+00)-pow(q3,2.00000000000000000000e+00)+pow(q4,2.00000000000000000000e+00))*(pow(q_des1,2.00000000000000000000e+00)-pow(q_des2,2.00000000000000000000e+00)-pow(q_des3,2.00000000000000000000e+00)+pow(q_des4,2.00000000000000000000e+00))+(q1*q2+q3*q4)*(q_des1*q_des2+q_des3*q_des4)*2.00000000000000000000e+00*2.00000000000000000000e+00+(q1*q2-q3*q4)*(q_des1*q_des2-q_des3*q_des4)*2.00000000000000000000e+00*2.00000000000000000000e+00+(q1*q3+q2*q4)*(q_des1*q_des3+q_des2*q_des4)*2.00000000000000000000e+00*2.00000000000000000000e+00+(q1*q3-q2*q4)*(q_des1*q_des3-q_des2*q_des4)*2.00000000000000000000e+00*2.00000000000000000000e+00+(q1*q4+q2*q3)*(q_des1*q_des4+q_des2*q_des3)*2.00000000000000000000e+00*2.00000000000000000000e+00+1.00000000000000000000e+00));
    acadodata_f2 << ((-pow(q1,2.00000000000000000000e+00)-pow(q2,2.00000000000000000000e+00)+pow(q3,2.00000000000000000000e+00)+pow(q4,2.00000000000000000000e+00))*(q_des1*q_des3-q_des2*q_des4)*2.00000000000000000000e+00-(-pow(q_des1,2.00000000000000000000e+00)-pow(q_des2,2.00000000000000000000e+00)+pow(q_des3,2.00000000000000000000e+00)+pow(q_des4,2.00000000000000000000e+00))*(q1*q3-q2*q4)*2.00000000000000000000e+00+(-q1*q4+q2*q3)*(q_des1*q_des2+q_des3*q_des4)*2.00000000000000000000e+00*2.00000000000000000000e+00-(-q_des1*q_des4+q_des2*q_des3)*(q1*q2+q3*q4)*2.00000000000000000000e+00*2.00000000000000000000e+00-(pow(q1,2.00000000000000000000e+00)-pow(q2,2.00000000000000000000e+00)-pow(q3,2.00000000000000000000e+00)+pow(q4,2.00000000000000000000e+00))*(q_des1*q_des3+q_des2*q_des4)*2.00000000000000000000e+00+(pow(q_des1,2.00000000000000000000e+00)-pow(q_des2,2.00000000000000000000e+00)-pow(q_des3,2.00000000000000000000e+00)+pow(q_des4,2.00000000000000000000e+00))*(q1*q3+q2*q4)*2.00000000000000000000e+00)/2.00000000000000000000e+00/sqrt(((-pow(q1,2.00000000000000000000e+00)+pow(q2,2.00000000000000000000e+00)-pow(q3,2.00000000000000000000e+00)+pow(q4,2.00000000000000000000e+00))*(-pow(q_des1,2.00000000000000000000e+00)+pow(q_des2,2.00000000000000000000e+00)-pow(q_des3,2.00000000000000000000e+00)+pow(q_des4,2.00000000000000000000e+00))+(-pow(q1,2.00000000000000000000e+00)-pow(q2,2.00000000000000000000e+00)+pow(q3,2.00000000000000000000e+00)+pow(q4,2.00000000000000000000e+00))*(-pow(q_des1,2.00000000000000000000e+00)-pow(q_des2,2.00000000000000000000e+00)+pow(q_des3,2.00000000000000000000e+00)+pow(q_des4,2.00000000000000000000e+00))+(-q1*q4+q2*q3)*(-q_des1*q_des4+q_des2*q_des3)*2.00000000000000000000e+00*2.00000000000000000000e+00+(pow(q1,2.00000000000000000000e+00)-pow(q2,2.00000000000000000000e+00)-pow(q3,2.00000000000000000000e+00)+pow(q4,2.00000000000000000000e+00))*(pow(q_des1,2.00000000000000000000e+00)-pow(q_des2,2.00000000000000000000e+00)-pow(q_des3,2.00000000000000000000e+00)+pow(q_des4,2.00000000000000000000e+00))+(q1*q2+q3*q4)*(q_des1*q_des2+q_des3*q_des4)*2.00000000000000000000e+00*2.00000000000000000000e+00+(q1*q2-q3*q4)*(q_des1*q_des2-q_des3*q_des4)*2.00000000000000000000e+00*2.00000000000000000000e+00+(q1*q3+q2*q4)*(q_des1*q_des3+q_des2*q_des4)*2.00000000000000000000e+00*2.00000000000000000000e+00+(q1*q3-q2*q4)*(q_des1*q_des3-q_des2*q_des4)*2.00000000000000000000e+00*2.00000000000000000000e+00+(q1*q4+q2*q3)*(q_des1*q_des4+q_des2*q_des3)*2.00000000000000000000e+00*2.00000000000000000000e+00+1.00000000000000000000e+00));
    acadodata_f2 << (-(-pow(q1,2.00000000000000000000e+00)+pow(q2,2.00000000000000000000e+00)-pow(q3,2.00000000000000000000e+00)+pow(q4,2.00000000000000000000e+00))*(q_des1*q_des2+q_des3*q_des4)*2.00000000000000000000e+00+(-pow(q_des1,2.00000000000000000000e+00)+pow(q_des2,2.00000000000000000000e+00)-pow(q_des3,2.00000000000000000000e+00)+pow(q_des4,2.00000000000000000000e+00))*(q1*q2+q3*q4)*2.00000000000000000000e+00+(pow(q1,2.00000000000000000000e+00)-pow(q2,2.00000000000000000000e+00)-pow(q3,2.00000000000000000000e+00)+pow(q4,2.00000000000000000000e+00))*(q_des1*q_des2-q_des3*q_des4)*2.00000000000000000000e+00-(pow(q_des1,2.00000000000000000000e+00)-pow(q_des2,2.00000000000000000000e+00)-pow(q_des3,2.00000000000000000000e+00)+pow(q_des4,2.00000000000000000000e+00))*(q1*q2-q3*q4)*2.00000000000000000000e+00+(q1*q3-q2*q4)*(q_des1*q_des4+q_des2*q_des3)*2.00000000000000000000e+00*2.00000000000000000000e+00-(q1*q4+q2*q3)*(q_des1*q_des3-q_des2*q_des4)*2.00000000000000000000e+00*2.00000000000000000000e+00)/2.00000000000000000000e+00/sqrt(((-pow(q1,2.00000000000000000000e+00)+pow(q2,2.00000000000000000000e+00)-pow(q3,2.00000000000000000000e+00)+pow(q4,2.00000000000000000000e+00))*(-pow(q_des1,2.00000000000000000000e+00)+pow(q_des2,2.00000000000000000000e+00)-pow(q_des3,2.00000000000000000000e+00)+pow(q_des4,2.00000000000000000000e+00))+(-pow(q1,2.00000000000000000000e+00)-pow(q2,2.00000000000000000000e+00)+pow(q3,2.00000000000000000000e+00)+pow(q4,2.00000000000000000000e+00))*(-pow(q_des1,2.00000000000000000000e+00)-pow(q_des2,2.00000000000000000000e+00)+pow(q_des3,2.00000000000000000000e+00)+pow(q_des4,2.00000000000000000000e+00))+(-q1*q4+q2*q3)*(-q_des1*q_des4+q_des2*q_des3)*2.00000000000000000000e+00*2.00000000000000000000e+00+(pow(q1,2.00000000000000000000e+00)-pow(q2,2.00000000000000000000e+00)-pow(q3,2.00000000000000000000e+00)+pow(q4,2.00000000000000000000e+00))*(pow(q_des1,2.00000000000000000000e+00)-pow(q_des2,2.00000000000000000000e+00)-pow(q_des3,2.00000000000000000000e+00)+pow(q_des4,2.00000000000000000000e+00))+(q1*q2+q3*q4)*(q_des1*q_des2+q_des3*q_des4)*2.00000000000000000000e+00*2.00000000000000000000e+00+(q1*q2-q3*q4)*(q_des1*q_des2-q_des3*q_des4)*2.00000000000000000000e+00*2.00000000000000000000e+00+(q1*q3+q2*q4)*(q_des1*q_des3+q_des2*q_des4)*2.00000000000000000000e+00*2.00000000000000000000e+00+(q1*q3-q2*q4)*(q_des1*q_des3-q_des2*q_des4)*2.00000000000000000000e+00*2.00000000000000000000e+00+(q1*q4+q2*q3)*(q_des1*q_des4+q_des2*q_des3)*2.00000000000000000000e+00*2.00000000000000000000e+00+1.00000000000000000000e+00));
    acadodata_f2 << (w1-w_des1);
    acadodata_f2 << (w2-w_des2);
    acadodata_f2 << (w3-w_des3);
    OCP ocp1(0, 2, 10);
    ocp1.minimizeLSQ(acadodata_M1, acadodata_f1);
    ocp1.minimizeLSQEndTerm(acadodata_M2, acadodata_f2);
    ocp1.subjectTo((-3.49999999999999977796e-01) <= u1 <= 3.49999999999999977796e-01);
    ocp1.subjectTo((-3.49999999999999977796e-01) <= u2 <= 3.49999999999999977796e-01);
    ocp1.subjectTo((-3.49999999999999977796e-01) <= u3 <= 3.49999999999999977796e-01);
    ocp1.subjectTo((-3.50000000000000033307e-02) <= u4 <= 3.50000000000000033307e-02);
    ocp1.subjectTo((-3.50000000000000033307e-02) <= u5 <= 3.50000000000000033307e-02);
    ocp1.subjectTo((-3.50000000000000033307e-02) <= u6 <= 3.50000000000000033307e-02);
    DifferentialEquation acadodata_f3;
    acadodata_f3 << dot(r1) == v1;
    acadodata_f3 << dot(r2) == v2;
    acadodata_f3 << dot(r3) == v3;
    acadodata_f3 << dot(v1) == ((1/(((pow(roff1,2.00000000000000000000e+00)+pow(roff2,2.00000000000000000000e+00))*mass+I_zz)*mass*mass-mass*pow((-mass)*roff2,2.00000000000000000000e+00)-mass*pow(mass*roff1,2.00000000000000000000e+00))*(((pow(roff1,2.00000000000000000000e+00)+pow(roff2,2.00000000000000000000e+00))*mass+I_zz)*mass-pow((-mass)*roff2,2.00000000000000000000e+00))*(-(-pow(w3,2.00000000000000000000e+00))*mass*roff2+u2)+1/(((pow(roff1,2.00000000000000000000e+00)+pow(roff2,2.00000000000000000000e+00))*mass+I_zz)*mass*mass-mass*pow((-mass)*roff2,2.00000000000000000000e+00)-mass*pow(mass*roff1,2.00000000000000000000e+00))*(-(-pow(w3,2.00000000000000000000e+00))*mass*roff1+u1)*(-mass)*mass*roff1*roff2+1/(((pow(roff1,2.00000000000000000000e+00)+pow(roff2,2.00000000000000000000e+00))*mass+I_zz)*mass*mass-mass*pow((-mass)*roff2,2.00000000000000000000e+00)-mass*pow(mass*roff1,2.00000000000000000000e+00))*(-mass*mass*roff1)*u6)*(q1*q2-q3*q4)*2.00000000000000000000e+00+(1/(((pow(roff1,2.00000000000000000000e+00)+pow(roff2,2.00000000000000000000e+00))*mass+I_zz)*mass*mass-mass*pow((-mass)*roff2,2.00000000000000000000e+00)-mass*pow(mass*roff1,2.00000000000000000000e+00))*(((pow(roff1,2.00000000000000000000e+00)+pow(roff2,2.00000000000000000000e+00))*mass+I_zz)*mass-pow(mass*roff1,2.00000000000000000000e+00))*(-(-pow(w3,2.00000000000000000000e+00))*mass*roff1+u1)+1/(((pow(roff1,2.00000000000000000000e+00)+pow(roff2,2.00000000000000000000e+00))*mass+I_zz)*mass*mass-mass*pow((-mass)*roff2,2.00000000000000000000e+00)-mass*pow(mass*roff1,2.00000000000000000000e+00))*(-(-mass)*mass*roff2)*u6+1/(((pow(roff1,2.00000000000000000000e+00)+pow(roff2,2.00000000000000000000e+00))*mass+I_zz)*mass*mass-mass*pow((-mass)*roff2,2.00000000000000000000e+00)-mass*pow(mass*roff1,2.00000000000000000000e+00))*(-(-pow(w3,2.00000000000000000000e+00))*mass*roff2+u2)*(-mass)*mass*roff1*roff2)*(pow(q1,2.00000000000000000000e+00)-pow(q2,2.00000000000000000000e+00)-pow(q3,2.00000000000000000000e+00)+pow(q4,2.00000000000000000000e+00)));
    acadodata_f3 << dot(v2) == ((-pow(q1,2.00000000000000000000e+00)+pow(q2,2.00000000000000000000e+00)-pow(q3,2.00000000000000000000e+00)+pow(q4,2.00000000000000000000e+00))*(1/(((pow(roff1,2.00000000000000000000e+00)+pow(roff2,2.00000000000000000000e+00))*mass+I_zz)*mass*mass-mass*pow((-mass)*roff2,2.00000000000000000000e+00)-mass*pow(mass*roff1,2.00000000000000000000e+00))*(((pow(roff1,2.00000000000000000000e+00)+pow(roff2,2.00000000000000000000e+00))*mass+I_zz)*mass-pow((-mass)*roff2,2.00000000000000000000e+00))*(-(-pow(w3,2.00000000000000000000e+00))*mass*roff2+u2)+1/(((pow(roff1,2.00000000000000000000e+00)+pow(roff2,2.00000000000000000000e+00))*mass+I_zz)*mass*mass-mass*pow((-mass)*roff2,2.00000000000000000000e+00)-mass*pow(mass*roff1,2.00000000000000000000e+00))*(-(-pow(w3,2.00000000000000000000e+00))*mass*roff1+u1)*(-mass)*mass*roff1*roff2+1/(((pow(roff1,2.00000000000000000000e+00)+pow(roff2,2.00000000000000000000e+00))*mass+I_zz)*mass*mass-mass*pow((-mass)*roff2,2.00000000000000000000e+00)-mass*pow(mass*roff1,2.00000000000000000000e+00))*(-mass*mass*roff1)*u6)+(1/(((pow(roff1,2.00000000000000000000e+00)+pow(roff2,2.00000000000000000000e+00))*mass+I_zz)*mass*mass-mass*pow((-mass)*roff2,2.00000000000000000000e+00)-mass*pow(mass*roff1,2.00000000000000000000e+00))*(((pow(roff1,2.00000000000000000000e+00)+pow(roff2,2.00000000000000000000e+00))*mass+I_zz)*mass-pow(mass*roff1,2.00000000000000000000e+00))*(-(-pow(w3,2.00000000000000000000e+00))*mass*roff1+u1)+1/(((pow(roff1,2.00000000000000000000e+00)+pow(roff2,2.00000000000000000000e+00))*mass+I_zz)*mass*mass-mass*pow((-mass)*roff2,2.00000000000000000000e+00)-mass*pow(mass*roff1,2.00000000000000000000e+00))*(-(-mass)*mass*roff2)*u6+1/(((pow(roff1,2.00000000000000000000e+00)+pow(roff2,2.00000000000000000000e+00))*mass+I_zz)*mass*mass-mass*pow((-mass)*roff2,2.00000000000000000000e+00)-mass*pow(mass*roff1,2.00000000000000000000e+00))*(-(-pow(w3,2.00000000000000000000e+00))*mass*roff2+u2)*(-mass)*mass*roff1*roff2)*(q1*q2+q3*q4)*2.00000000000000000000e+00);
    acadodata_f3 << dot(v3) == ((1/(((pow(roff1,2.00000000000000000000e+00)+pow(roff2,2.00000000000000000000e+00))*mass+I_zz)*mass*mass-mass*pow((-mass)*roff2,2.00000000000000000000e+00)-mass*pow(mass*roff1,2.00000000000000000000e+00))*(((pow(roff1,2.00000000000000000000e+00)+pow(roff2,2.00000000000000000000e+00))*mass+I_zz)*mass-pow((-mass)*roff2,2.00000000000000000000e+00))*(-(-pow(w3,2.00000000000000000000e+00))*mass*roff2+u2)+1/(((pow(roff1,2.00000000000000000000e+00)+pow(roff2,2.00000000000000000000e+00))*mass+I_zz)*mass*mass-mass*pow((-mass)*roff2,2.00000000000000000000e+00)-mass*pow(mass*roff1,2.00000000000000000000e+00))*(-(-pow(w3,2.00000000000000000000e+00))*mass*roff1+u1)*(-mass)*mass*roff1*roff2+1/(((pow(roff1,2.00000000000000000000e+00)+pow(roff2,2.00000000000000000000e+00))*mass+I_zz)*mass*mass-mass*pow((-mass)*roff2,2.00000000000000000000e+00)-mass*pow(mass*roff1,2.00000000000000000000e+00))*(-mass*mass*roff1)*u6)*(q1*q4+q2*q3)*2.00000000000000000000e+00+(1/(((pow(roff1,2.00000000000000000000e+00)+pow(roff2,2.00000000000000000000e+00))*mass+I_zz)*mass*mass-mass*pow((-mass)*roff2,2.00000000000000000000e+00)-mass*pow(mass*roff1,2.00000000000000000000e+00))*(((pow(roff1,2.00000000000000000000e+00)+pow(roff2,2.00000000000000000000e+00))*mass+I_zz)*mass-pow(mass*roff1,2.00000000000000000000e+00))*(-(-pow(w3,2.00000000000000000000e+00))*mass*roff1+u1)+1/(((pow(roff1,2.00000000000000000000e+00)+pow(roff2,2.00000000000000000000e+00))*mass+I_zz)*mass*mass-mass*pow((-mass)*roff2,2.00000000000000000000e+00)-mass*pow(mass*roff1,2.00000000000000000000e+00))*(-(-mass)*mass*roff2)*u6+1/(((pow(roff1,2.00000000000000000000e+00)+pow(roff2,2.00000000000000000000e+00))*mass+I_zz)*mass*mass-mass*pow((-mass)*roff2,2.00000000000000000000e+00)-mass*pow(mass*roff1,2.00000000000000000000e+00))*(-(-pow(w3,2.00000000000000000000e+00))*mass*roff2+u2)*(-mass)*mass*roff1*roff2)*(q1*q3-q2*q4)*2.00000000000000000000e+00);
    acadodata_f3 << dot(q1) == ((-q3)*5.00000000000000000000e-01*w2+5.00000000000000000000e-01*q2*w3+5.00000000000000000000e-01*q4*w1);
    acadodata_f3 << dot(q2) == ((-q1)*5.00000000000000000000e-01*w3+5.00000000000000000000e-01*q3*w1+5.00000000000000000000e-01*q4*w2);
    acadodata_f3 << dot(q3) == ((-q2)*5.00000000000000000000e-01*w1+5.00000000000000000000e-01*q1*w2+5.00000000000000000000e-01*q4*w3);
    acadodata_f3 << dot(q4) == ((-q1)*5.00000000000000000000e-01*w1+(-q2)*5.00000000000000000000e-01*w2+(-q3)*5.00000000000000000000e-01*w3);
    acadodata_f3 << dot(w1) == 0.00000000000000000000e+00;
    acadodata_f3 << dot(w2) == 0.00000000000000000000e+00;
    acadodata_f3 << dot(w3) == (1/(((pow(roff1,2.00000000000000000000e+00)+pow(roff2,2.00000000000000000000e+00))*mass+I_zz)*mass*mass-mass*pow((-mass)*roff2,2.00000000000000000000e+00)-mass*pow(mass*roff1,2.00000000000000000000e+00))*(-(-mass)*mass*roff2)*(-(-pow(w3,2.00000000000000000000e+00))*mass*roff1+u1)+1/(((pow(roff1,2.00000000000000000000e+00)+pow(roff2,2.00000000000000000000e+00))*mass+I_zz)*mass*mass-mass*pow((-mass)*roff2,2.00000000000000000000e+00)-mass*pow(mass*roff1,2.00000000000000000000e+00))*(-(-pow(w3,2.00000000000000000000e+00))*mass*roff2+u2)*(-mass*mass*roff1)+1/(((pow(roff1,2.00000000000000000000e+00)+pow(roff2,2.00000000000000000000e+00))*mass+I_zz)*mass*mass-mass*pow((-mass)*roff2,2.00000000000000000000e+00)-mass*pow(mass*roff1,2.00000000000000000000e+00))*mass*mass*u6);

    ocp1.setModel( acadodata_f3 );


    ocp1.setNU( 6 );
    ocp1.setNP( 0 );
    ocp1.setNOD( 29 );
    OCPexport ExportModule1( ocp1 );
    ExportModule1.set( GENERATE_MATLAB_INTERFACE, 1 );
    uint options_flag;
    options_flag = ExportModule1.set( HESSIAN_APPROXIMATION, GAUSS_NEWTON );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: HESSIAN_APPROXIMATION");
    options_flag = ExportModule1.set( DISCRETIZATION_TYPE, MULTIPLE_SHOOTING );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: DISCRETIZATION_TYPE");
    options_flag = ExportModule1.set( SPARSE_QP_SOLUTION, FULL_CONDENSING_N2 );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: SPARSE_QP_SOLUTION");
    options_flag = ExportModule1.set( INTEGRATOR_TYPE, INT_IRK_GL4 );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: INTEGRATOR_TYPE");
    options_flag = ExportModule1.set( NUM_INTEGRATOR_STEPS, 20 );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: NUM_INTEGRATOR_STEPS");
    options_flag = ExportModule1.set( QP_SOLVER, QP_QPOASES );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: QP_SOLVER");
    options_flag = ExportModule1.set( HOTSTART_QP, YES );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: HOTSTART_QP");
    options_flag = ExportModule1.set( LEVENBERG_MARQUARDT, 1.000000E-05 );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: LEVENBERG_MARQUARDT");
    uint export_flag;
    export_flag = ExportModule1.exportCode( "../NMPC_export" );
    if(export_flag != 0) mexErrMsgTxt("ACADO export failed because of the above error(s)!");


    clearAllStaticCounters( ); 
 
} 

