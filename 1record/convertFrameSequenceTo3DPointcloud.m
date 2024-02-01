function pointcloudFrames = convertFrameSequenceTo3DPointcloud(calibration,depthFrames,pointClouds,numberOfFrames)
%CONVERTTO3DPOINTCLOUD Summary of this function goes here


P = [calibration.cx,calibration.cy];
F = [calibration.fx,calibration.fy];
k = calibration.radDist';
p = calibration.tanDist';
% pointcloudFrames = pointCloud.empty(0,100);

for frameNumber = 1:numberOfFrames
    depthFrame = depthFrames(:,:,frameNumber);
    [i,j] = find(depthFrame);
    indicesOfInterest = [i,j];
    h = size(depthFrame,1);
    w = size(depthFrame,2);
    % [ind] = find(depthFrames(:,:,frameNumber)>0);
    % filter out 0 values
    pc = pointClouds(:,:,frameNumber);
    pc = pc(sum(pc,2)>0,:);
    % get point cloud interms of depth image
    pc = horzcat(pc,proj(pc,[0,0],P,F,k,p));
    %get valid point that are actually in the frame
    validXPoints = pc(:,4)>0 & pc(:,4)<w;
    validYPoints = pc(:,5)>0 & pc(:,5)<h;
    validPoints = pc(validXPoints==true & validYPoints==true,:);
    %compare depth frame to pointcloud depth frame
    validPoints = validPoints(ismember(indicesOfInterest(:,1),validPoints(:,4)) & ismember(indicesOfInterest(:,2),validPoints(:,5)),1:3);

    % ind = horzcat(i,j,v);
    % shoulder3d = proj(ind,P,F,k,p);
end

end

function coords = deproj(ind,P,F,k,p)
    x1 = ind(:,1)-P(1)/F(1);
    y1 = ind(:,2)-P(2)/F(2);
    coords1 = [x1,y1];
    %distCoords = coords1;
    distCoords = Dmodel(coords1, k, p);
    x = ind(:,1)*distCoords(:,1);
    y = ind(:,1)*distCoords(:,2);
    coords = [x,y];
end

function coords = proj(pc,s,P,F,k,p)
    x1 = pc(:,1)./pc(:,3);
    y1 = pc(:,2)./pc(:,3);
    coords1 = [x1,y1];
    %distCoords = coords1;
    distCoords = Dmodel(coords1, s, k, p);
    x = F(1)*distCoords(:,1)+P(1);
    y = F(2)*distCoords(:,2)+P(2);
    coords = [round(x),round(y)];

end

function coords = Dmodel(pc,c,k,p)
    xd = pc(:,1);
    yd = pc(:,2);
    xc = c(1);
    yc = c(2);

    r = sqrt((xd-xc).^2+(yd-yc).^2);
    radial = k(1)*r.^2+k(2)*r.^4+k(3)*r.^6+k(4)*r.^8+k(5)*r.^10+k(6)*r.^12;
    xt = p(1)*(r.^2+2*(xd-xc).^2)+2*p(2)*((xd-xc).*(yd-yc));
    yt = 2*p(1)*((xd-xc).*(yd-yc))+p(2)*(r.^2+2*(xd-xc).^2);
    xu = xd+(xd-xc).*radial+xt;
    yu = yd+(yd-yc).*radial+yt;
    coords = [xu,yu];
    % r = x^2+y^2;
    % f = 1 + k(1)*r + k(2)*r^2+k(5)*r^3;
    % coords = [0,0];
    % coords(1) = f*x+2*k(3)*f^2*x*y+k(4)*(r+2*x^2*f^2);
    % coords(2) = f*y+2*k(3)*f^2*x*y+k(4)*(r+2*y^2*f^2);
end

