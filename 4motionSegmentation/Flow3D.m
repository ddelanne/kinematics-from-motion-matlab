function flow = Flow3D(pc1in, pc2in)
%Flow3D Compute distance transform based 3D flow between two point clouds
%   pc1: reference point cloud
%   pc2: second point cloud
%   

% find bounding box min12, max12
% min1 = min(pc1);
% min2 = min(pc2);
% min12 = min(min1,min2);  % min is (2,2,2) - 1 pixel padding
pc1 = round(pc1in);
pc2 = round(pc2in);
% pc1 = pc1in;
% pc2 = pc2in;
max1 = max(pc1);
max2 = max(pc2);
max12 = ceil(max(max1,max2))+1;   % add 1 pixel padding around max value

A = zeros(max12(1)+2,max12(2)+2,max12(3)+2,'uint8'); % min12=(2,2,2)
% turn pc1 and pc2 into indexes 
% size(A);
% whos A;
% min(pc1);
% max(pc1);
ind = sub2ind(size(A),pc1(:,1),pc1(:,2),pc1(:,3));
A(ind) = 1;

[D, idx1] = bwdist(A,'euclidean');  % compute DT for pc1
ind = sub2ind(size(A), pc2(:,1),pc2(:,2),pc2(:,3));
ind1 = int32(ind);
DX = 1;
DY =  max12(1)+2;     % added 2 - 6-26  3:24
DZ = (max12(2)+2) * (max12(1)+2);   % added 2  6-26 3:24
dx = D(ind1 + DX) - D(ind1 - DX);
dy = D(ind1 + DY) - D(ind1 - DY);
dz = D(ind1 + DZ) - D(ind1 - DZ);

m = sqrt(dx.^2 + dy.^2 + dz.^2);
indm = find(m < 0.01);
m(indm) = 1;

dx = dx ./ m;
dy = dy ./ m;
dz = dz ./ m;
nf = D(ind1);

% compute x, y, z from ind1
flow2 = [double(pc2(:,1)) double(pc2(:,2)) double(pc2(:,3)) double(-dx) double(-dy) double(-dz) double(nf)];

% stats = struct("rms", sqrt(sum(double(nf).^2)/length(nf)), ...
%     "mean", mean(nf), ...
%     "sigma", std(nf), ...
%     "max", max(nf));

% display(sprintf('RMS = %f5.2      mean = %f5.2     sigma = %f5.2    max = %f5.2', ...
    % sqrt(sum(double(nf).^2)/length(nf)), mean(nf), std(nf), max(nf)));
ind = find(nf < mean(nf)+1.5*std(nf));
flow = flow2(ind,:);
end

