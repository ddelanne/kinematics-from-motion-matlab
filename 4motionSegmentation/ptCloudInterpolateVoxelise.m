function [pcout] = ptCloudInterpolateVoxelise(pcin)
% ptCloudVoxelise takes a point cloud whose coordinates are double values
% and interpolates it to make it much denser; After that the point cloud is
% filtered to remove duplicates and voxelised; 
% All points are turned into non-negative integer values
% 
% min(pcin)
% pcin = pcin + shift;
% min(pcin)
x = pcin(:,1);
y = pcin(:,2);
z = pcin(:,3);
% sprintf('Interpolate')
% [min(x) min(y) min(z)]
% F = scatteredInterpolant(x,z,y);   % create interpolant
F = scatteredInterpolant(x,y,z);   % create interpolant
F.ExtrapolationMethod = 'nearest';
% k = boundary(y,z);
% mm = round(max(max(y),max(z))+5);
% bw = poly2mask(y(k),z(k),mm,mm);
% k = boundary(x,z);
% mm = round(max(max(x),max(z))+5);
% bw = poly2mask(x(k),z(k),mm,mm);

% k = boundary(x,z);
% mm = round(max(max(x),max(z))+5);
% bw = poly2mask(x(k),z(k),mm,mm);
% [z1,x1] = find(bw ~= 0);
% y1 = F(x1,z1);
% pc = [x1 y1 z1];

k = boundary(x,y);
mm = round(max(max(x),max(y))+5);
bw = poly2mask(x(k),y(k),mm,mm);
[y1,x1] = find(bw ~= 0);
z1 = F(x1,y1);
pc = [x1 y1 z1];


%sprintf('Hello %f',min(pc))
pcout = pc;
%pcout = int32(round(pc));
end