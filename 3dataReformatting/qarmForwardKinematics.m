function [p1, p2, p3, p4, R04, T04, gemma, w4, Rw04,w4tot] = qarmForwardKinematics(phi, phi_dot)

%% QUANSER_ARM_FPK
% v 4.0 - 4th June 2020

% REFERENCE: 
% Chapter 3. Forward Kinematics
% Robot Dynamics and Control 
% Spong, Vidyasagar
% 1989

% INPUTS:
% phi     : Alternate joint angles vector 4 x 1

% OUTPUTS:
% p4      : End-effector frame {4} position vector expressed in base frame {0}
% R04     : rotation matrix from end-effector frame {4} to base frame {0}

%% Manipulator parameters:
L1 = 0.1400;
L2 = 0.3500;
L3 = 0.0500;
L4 = 0.2500;
L5 = 0.1500;

%% Alternate parameters
l1 = L1;
l2 = sqrt(L2^2+L3^2);
l3 = L4 + L5;
beta = atan(L3/L2);

%% From phi space to theta space 
theta = phi;
theta(1) = phi(1);
theta(2) = phi(2)+beta-(pi/2);
theta(3) = phi(3)-beta;
theta(4) = phi(4);

%% Transformation matrices for all frames:

% T{i-1}{i} = quanser_arm_DH(  a, alpha,  d,     theta );
        T01 = quanser_arm_DH(  0,     -pi/2,         l1,     theta(1));
        T12 = quanser_arm_DH(  l2,    0,     0,      theta(2));
        T23 = quanser_arm_DH(  0,     -pi/2,         0,      theta(3));
        T34 = quanser_arm_DH(  0,     0,     l3,     theta(4));
        
        T02 = T01*T12;
        T03 = T02*T23;
        T04 = T03*T34;

        W01 = quanser_arm_rot_velocity_DH(phi_dot(1));
        W12 = quanser_arm_rot_velocity_DH(phi_dot(2));
        W23 = quanser_arm_rot_velocity_DH(phi_dot(3));
        W34 = quanser_arm_rot_velocity_DH(phi_dot(4));
        
        W01A = eye(4)*W01*eye(4)^-1;
        W12A = T01*W12*T01^-1;
        W23A = T02*W23*T02^-1;
        W34A = T03*W34*T03^-1;
        
%         W01A = T01*W01*T01^-1;
%         W12A = T02*W12*T02^-1;
%         W23A = T03*W23*T03^-1;
%         W34A = T04*W34*T04^-1;
        
        W01 = W01A;
        W02 = W01+W12A;
        W03 = W02+W23A;
        W04 = W03+W34A;
%% Position of end-effector Transformation

% Extract the Position vector 
p1 = T01(1:3,4)';
p2 = T02(1:3,4)';
p3 = T03(1:3,4)';
p4 = T04(1:3,4)';

% Extract the Rotation matrix
R04 = T04(1:3,1:3);
gemma = theta(4);

%% extract velocity
w4 = W04(1:3,4)';

Rw04 = W04(1:3,1:3);
Rw = [-1*Rw04(2,3),Rw04(1,3),-1*Rw04(1,2)];

w4tot = w4 + Rw.*norm(p4);

end

function T = quanser_arm_rot_velocity_DH(phi_dot)
    T = zeros(4,4);
    T(1,2) = -1*phi_dot;
    T(2,1) = phi_dot;
end

function T = quanser_arm_DH(a,alpha,d,theta)

%% QUANSER_ARM_DH
% v 1.0 - 26th March 2019

% REFERENCE: 
% Chapter 3. Forward and Inverse Kinematics
% Robot Modeling and Control 
% Spong, Hutchinson, Vidyasagar
% 2006

% INPUTS:
% a       :   translation  : along : x_{i}   : from : z_{i-1} : to : z_{i}
% alpha   :      rotation  : about : x_{i}   : from : z_{i-1} : to : z_{i}
% d       :   translation  : along : z_{i-1} : from : x_{i-1} : to : x_{i}
% theta   :      rotation  : about : z_{i-1} : from : x_{i-1} : to : x_{i}
% NOTE: Standard DH Parameters

% OUTPUTS:
% T       : transformation                   : from :     {i} : to : {i-1}

%%
% Rotation Transformation about z axis by theta
T_R_z = [[cos(theta),   -sin(theta),    0,      0];...
         [sin(theta),   cos(theta),     0,      0];...
         [0,            0,              1,      0];...
         [0,            0,              0,      1]];
     
% Translation Transformation along z axis by d
T_T_z = [[1,0,0,0];...
         [0,1,0,0];...
         [0,0,1,d];...
         [0,0,0,1]];

% Translation Transformation along x axis by a
T_T_x = [[1,0,0,a];...
         [0,1,0,0];...
         [0,0,1,0];...
         [0,0,0,1]];
     
% Rotation Transformation about x axis by alpha
T_R_x = [[1,    0,          0,          0];...
         [0,    cos(alpha), -sin(alpha),0];...
         [0,    sin(alpha), cos(alpha), 0];...
         [0,    0,          0,          1]];

% For a transformation FROM frame {i} TO frame {i-1}:
% rotate about z
% then translate along z, 
% then translate along x, 
% then rotate about x, 

T = T_R_z*T_T_z*T_T_x*T_R_x;

end