function [omega,theta] = Rotation3D(flow)
%Rotation3D Computes a small rotation and translation between two point clouds 
%   R is approximated using the Rodrigues formula for small rotations
%   pc1: point cloud 1
%   pc2: point cloud 2
%   flow: the 3D normal flow between pc1 and pc2
%   omega: the rotation vector
%   theta: the angle of rotation
%   T: the translation vector; assumed to be 0
x = flow(:,1);
y = flow(:,2);
z = flow(:,3);
nx = flow(:,4);
ny = flow(:,5);
nz = flow(:,6);
nf = flow(:,7);
siz = size(flow);
B = zeros(siz(1),3);
B(:,1) = -ny.*z+nz.*y;
B(:,2) = nx.*z -nz.*x;
B(:,3) = -nx.*y + ny.*x;
% B(:,4) = nx;
% B(:,5) = ny;
% B(:,6) = nz;
u = B \ nf;     % the right division, the LS solution for u
% T = u(4:6);
theta = norm(u(1:3));
omega = u(1:3)/theta;
end
