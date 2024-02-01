function [flow, Z1, A1, ind1] = Flow3D(pc1, pc2)
%Flow3D Compute distance transform based 3D flow between two point clouds
%   pc1: reference point cloud
%   pc2: second point cloud
%   

% find bounding box min12, max12
min1 = min(pc1);
min2 = min(pc2);
min12 = min(min1,min2);  % min is (2,2,2)now - 1 pixel padding
max1 = max(pc1);
max2 = max(pc2);
max12 = max(max1,max2)+1;   % there is 1 pixel padding around max value

A = zeros(max12(1),max12(2),max12(3),255,'uint8'); % assume min12=(2,2,2)
% turn pc1 and pc2 into indexes 

A1 = zeros(bb(2)-bb(1)+1,bb(4)-bb(3)+1,256,'uint8');
img1 = img10(bb(1):bb(2),bb(3):bb(4));
ind = find((img1 >= bb(5)) & (img1 <= bb(6)));
val = double(img1(ind));
ind1 = ind + (val-1)*(bb(2)-bb(1)+1)*(bb(4)-bb(3)+1);
A1(ind1) = 1;
% for i=10:250,
%     im = (img1 == i);
%     A(:, :, i) = uint8(im);
% end

% ind2 = find(A ~= A1);

% Z1 = bwdist(A1,'euclidean');
Z1 = bwdist(A1,'euclidean');

% figure, imagesc(Z1(:,:,100));

img2 = img20(bb(1):bb(2),bb(3):bb(4));
% figure,imagesc(img2);
% A2 = zeros(d(1),d(2),255,'uint8');
ind = find((img2 >= 5) & (img2 <= 250));
val = double(img2(ind));
ind1 = ind + (val-1)*(bb(2)-bb(1)+1)*(bb(4)-bb(3)+1);

[i,j,k] = ind2sub(size(Z1),ind1);

DY = 1;
DX = bb(4) - bb(3) + 1;
DZ = DX * (bb(2) - bb(1) + 1);

imgx = zeros(size(img2),'uint8');
ind2 = sub2ind(size(img2),i,j);
imgx(ind2) = k;
% figure, imagesc(imgx);


dx = Z1(ind1 + DX) - Z1(ind1 - DX);
dy = Z1(ind1 + DY) - Z1(ind1 - DY);
dz = Z1(ind1 + DZ) - Z1(ind1 - DZ);

m = sqrt(dx.^2 + dy.^2 + dz.^2);
indm = find(m < 0.01);
m(indm) = 1;

dx = dx ./ m;
dy = dy ./ m;
dz = dz ./ m;
nf = Z1(ind1);

% compute x, y, z from ind1
[i,j,k] = ind2sub(size(Z1),ind1);
% figure, scatter3(i,j,k);

flow = [j+bb(3)-1 i+bb(1)-1  k+500 -dx.*nf -dy.*nf -dz.*nf];

% ind1 are indexes in A2 of points corresponding to z ~= 0

% Flow can be computed using A1 & ind1 only
% indexes in ind1 can be used to index into Z1

% for i=10:250,
%     im = (img2 == i);
%     A2(:, :, i) = uint8(im);
% end

end

