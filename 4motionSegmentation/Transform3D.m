function [pcout] = Transform3D(pc,omega,theta,T)
%Transform3D Take a point cloud and transform it using R=(I+theta*K) & T, K
%   is the screw matrix made from omega
R = axang2rotm([omega; theta]');
T1 = repmat(T',[length(pc),1]);
pcout = double(pc)*R' + T1;
end

