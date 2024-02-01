% function [omega, theta, T] = Motion3D(flow)
% %Motion3D Computes a small rotation and translation between two point clouds 
% %   R is approximated using the Rodrigues formula for small rotations
% %   pc1: point cloud 1
% %   pc2: point cloud 2
% %   flow: the 3D normal flow between pc1 and pc2
% %   omega: the rotation vector
% %   theta: the angle of rotation
% %   T: the translation vector
% theta = norm(flow(:,1:3) \ flow(:,7));
% mk =  mean(flow(:,4:7));
% mk = mk(1:3)*mk(4);
% K = [0 -1*mk(3) mk(2);...
%     mk(3) 0 -1*mk(1);...
%     -1*mk(2) mk(1) 0];
% omega = eye(3)+sin(theta)*K+(1-cos(theta))*K^2;
% T = (flow(:,4:6) \ flow(:,7))';
% end

% function [omega,theta,T] = Motion3D(flow)
% Motion3D Computes a small rotation and translation between two point clouds 
%   R is approximated using the Rodrigues formula for small rotations
%   pc1: point cloud 1
%   pc2: point cloud 2
%   flow: the 3D normal flow between pc1 and pc2
%   omega: the rotation vector
%   theta: the angle of rotation
%   T: the translation vector
% x = flow(:,1);
% y = flow(:,2);
% z = flow(:,3);
% nx = flow(:,4);
% ny = flow(:,5);
% nz = flow(:,6);
% nf = flow(:,7);
% siz = size(flow);
% 
% dispX = -nz.*y + ny.*z;
% dispY = nz.*x - nx.*z;
% dispZ = -ny.*x + nx.*y;
% 
% b = nx.*dispX + ny.*dispY + nz.*dispZ;
% 
% A = zeros(siz(1),6);
% 
% A(:,1) = -ny.*z+nz.*y;
% A(:,2) = nx.*z -nz.*x;
% A(:,3) = -nx.*y + ny.*x;
% A(:,4) = nx;
% A(:,5) = ny;
% A(:,6) = nz;
% 
% u = lsqr(A,b);
% 
% theta = norm(u(1:3));
% omega = u(1:3)/theta;
% 
% T = u(4:6);
% u = B \ nf;     % the right division, the LS solution for u
% T = u(4:6);
% theta = norm(u(1:3));
% omega = u(1:3)/theta;
% end


function [omega,theta,T] = Motion3D(flow)
%Motion3D Computes a small rotation and translation between two point clouds 
%   R is approximated using the Rodrigues formula for small rotations
%   pc1: point cloud 1
%   pc2: point cloud 2
%   flow: the 3D normal flow between pc1 and pc2
%   omega: the rotation vector
%   theta: the angle of rotation
%   T: the translation vector
x = flow(:,1);
y = flow(:,2);
z = flow(:,3);
nx = flow(:,4);
ny = flow(:,5);
nz = flow(:,6);
nf = flow(:,7);
siz = size(flow);
B = zeros(siz(1),6);

B(:,1) = -ny.*z+nz.*y;
B(:,2) = nx.*z -nz.*x;
B(:,3) = -nx.*y + ny.*x;
B(:,4) = nx;
B(:,5) = ny;
B(:,6) = nz;
u = B \ nf;     % the right division, the LS solution for u
T = u(4:6);
theta = norm(u(1:3));
omega = u(1:3)/theta;
end

