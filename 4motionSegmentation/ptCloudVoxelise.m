function [pcout] = ptCloudVoxelise(shift, pcin)
% ptCloudVoxelise takes a point cloud whose coordinates are double values
% and voxelises it; All points are turned into non-negative integer values
% 
% min(pcin)
pcin = pcin + shift;
% sprintf('Hello 2: %f',min(pcin))
pcout1 = pcin;
%pcout1 = int32(round(pcin));
ind = find((pcout1(:,1)>1) & (pcout1(:,2)>1) & (pcout1(:,3)>1));
pcout = pcout1(ind,:);
end