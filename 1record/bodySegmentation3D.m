function [bodyKeypoints, armPointClouds, mean_arm, forearmPointClouds, mean_forearm] = bodySegmentation3D(bodyFrames, pointClouds, numberOfFrames, s)
    % bodySegmentation3D(bodyFrames, pointCloudFrames, 10, 92160)
    % right arm:
    % jointIndexList = [13, 14, 15];
    % left arm:
    jointIndexList = [6, 7, 8];
    i = 1;
    bodyKeypoints = zeros([32,3,numberOfFrames]);
    while i <= numberOfFrames
        % reframe keypoints
        % frameBodyKeyPoints = (rotx(-90)*bodyFrames(i).Position3d)';
        frameBodyKeyPoints = (bodyFrames(i).Position3d)';
        bodyKeypoints(:,:,i) = frameBodyKeyPoints;
        i=i+1;
    end
    [shoulderKeypoints, elbowKeypoints, wristKeypoints, oppositeShoulderKeypoints, middleTorsoKeypoints] = calculateSyntheticKeypoints(bodyKeypoints);

    bodyKeypoints(6,:,:) = shoulderKeypoints';
    bodyKeypoints(7,:,:) = elbowKeypoints';
    bodyKeypoints(8,:,:) = wristKeypoints';

    armPointClouds = cell(numberOfFrames, 1);
    
    % zeros(s,3,numberOfFrames);
    % armPointClouds(armPointClouds == 0) = NaN;
    mean_arm = zeros(numberOfFrames,3);

    forearmPointClouds = cell(numberOfFrames, 1);
    % zeros(s,3,numberOfFrames);
    % forearmPointClouds(armPointClouds == 0) = NaN;
    mean_forearm = zeros(numberOfFrames,3);
    i = 1;
    while i<=numberOfFrames   
        display(sprintf('i = %d',i));
        pc = pointClouds(:,:,i);
        % remove all zeros points:
        pc = pc(sum(pc,2)~=0,:);


        % hold on; xlabel("x");ylabel("y");zlabel("z");
        % test = (rotx(-90)*pc')';
        % pcshow(test);

        % jointKeypoints = (bodyFrames(i).Position3d(:,:))';

        % get arm points
        % armKeypointIndexes = [jointIndexList(1),jointIndexList(2)];
        % armKeypoints =  jointKeypoints(armKeypointIndexes,:);
        
        armKeypoints = [shoulderKeypoints(i,:);elbowKeypoints(i,:)];
        armpc = bonePoints(pc, armKeypoints, .3, 0, 0, 0, 0);
        % armpc = (rotx(-90)*armpc')';
        % filter point cloud
        j = inf;
        while j>0
            t1 = size(armpc,1);
            armpc = voxelDensityFilter(armpc);
            t2 = size(armpc,1);
            j = t1-t2;
        end
        % [pc1,pc2] = getBleedingEdges(armKeypoints,armpc);
        mean_arm(i,:) = mean(armpc, 1); 
        armPointClouds{i} = armpc;
        % armPointClouds(1:size(armpc,1),:,i) = armpc;

        % get forearm points
        % forearmKeypointIndexes = [jointIndexList(2),jointIndexList(3)];
        % forearmKeypoints = jointKeypoints(forearmKeypointIndexes,:);
        forearmKeypoints = [elbowKeypoints(i,:);wristKeypoints(i,:)];
        forearmpc = bonePoints(pc, forearmKeypoints,.45,0,0,0,0);
        % forearmpc = (rotx(-90)*forearmpc')';
        % filter point cloud
        j = inf;
        while j>0
            t1 = size(forearmpc,1);
            forearmpc = voxelDensityFilter(forearmpc);
            t2 = size(forearmpc,1);
            j = t1-t2;
        end
        mean_forearm(i,:) = mean(forearmpc, 1); 
        forearmPointClouds{i} = forearmpc;
        % forearmPointClouds(1:size(forearmpc,1),:,i) = forearmpc;

        i = i+1;
    end
    
    %clearvars depth depthHeight depthWidth i numberOfFrames validData
    % save(outputFolder+"\depthImages.mat")
    %clear all
    
end

function [pc1,pc2] = getBleedingEdges(keypoints,pc)
    % FOV filter
    % get mid point between keypoints
    %pc median
    % calcualte median with min average of pcdist
    midKeypoint = keypoints(1,:)+(keypoints(1,:)-keypoints(2,:));
    % midKeypoint = point_projection_to_line(meanPoint, keypoints(1,:), keypoints(2,:));

    % distanceMap = pdist2(pc,pc);
    % [M,I] = min(mean(distanceMap,2));
    % meanPoint = pc(I,:);


    % project points on to line between midpoint and pc median
    perpendicularProjectedpc = point_projection_to_line(pc, meanPoint, midKeypoint);
    
    distKPperpendicular = vecnorm(perpendicularProjectedpc-midKeypoint,2,2);
        
    l = distKPperpendicular>inPerpendicularLengthBuffer & distKPperpendicular<outPerpendicularLengthBuffer;
    pcPerpendicularKeypoints = pcBetweenKeypoints(l,:);
        
    % find points outside distance
    filteredpc = pcPerpendicularKeypoints;

end

function filteredpc = bonePoints(pc, keypoints, armWidthMax, keypointALengthBuffer, keypointBLengthBuffer, inPerpendicularLengthBuffer, outPerpendicularLengthBuffer)
        % get distance of all points to line:
        distToForearm = point_to_line(pc, keypoints(1,:), keypoints(2,:));
        % consider only points near line
        pcNearLine = pc(distToForearm<armWidthMax,:);
        % project remaining points onto line
        forearmProjectedpc = point_projection_to_line(pcNearLine, keypoints(1,:), keypoints(2,:));
        % check if projected points are between the 2 keypoints
        distKP = norm(keypoints(1,:)-keypoints(2,:));
        distKP1 = vecnorm(forearmProjectedpc-keypoints(1,:),2,2);
        distKP2 = vecnorm(forearmProjectedpc-keypoints(2,:),2,2);
        % consider only points between the 2 keypoints
        l = (distKP+keypointBLengthBuffer)-distKP1>0 & (distKP+keypointALengthBuffer)-distKP2>0;
        pcBetweenKeypoints = pcNearLine(l,:);

        filteredpc = pcBetweenKeypoints;
end

function d = point_to_line(pt, v1, v2)
      a = v1 - v2;
      a = repmat(a,size(pt,1),1);
      b = pt - v2;
      d = vecnorm(cross(a,b,2),2,2) / norm(a);
end

function ptout = point_projection_to_line(pt,v1,v2)
    v = v2-v1;
    a = pt-v1;
    ptout = v1 + (dot(a,repmat(v,size(pt,1),1),2) / dot(v,v,2)) * (v);
end

function pcout = voxelDensityFilter(pcin)
    voxelSize = 7.5;
    
    filteredBin = zeros([size(pcin,1),1]);
    for i  = -1*round(voxelSize/2):round(voxelSize/2)
        for j  = -1*round(voxelSize/2):round(voxelSize/2)
            for k  = -1*round(voxelSize/2):round(voxelSize/2)
                filteredBin = filteredBin + makeVoxelGrid(voxelSize, [i, j, k], pcin);
            end
        end
    end
    majority = (voxelSize*voxelSize*voxelSize)/2;
    filteredBin = filteredBin>majority;
    pcout = pcin(filteredBin,:);
    % hold on
    % pcshow(pointCloud(pcin,"Color","red"));
    % pcshow(pointCloud(pcout,"Color","blue"));
    % hold off
end

function filteredBin = makeVoxelGrid(voxelSize, offSet, pcin)
    voxelizedpc = zeros(size(pcin));
    x = pcin(:,1);
    edgesX = min(x)-voxelSize+offSet(1):voxelSize:max(x)+voxelSize;
    voxelizedpc(:,1)=discretize(x, edgesX);
    y = pcin(:,2);
    edgesY = min(y)-voxelSize+offSet(2):voxelSize:max(y)+voxelSize;
    voxelizedpc(:,2)=discretize(y, edgesY);
    z = pcin(:,3);
    edgesZ = min(z)-voxelSize+offSet(3):voxelSize:max(z)+voxelSize;
    voxelizedpc(:,3)=discretize(z, edgesZ);
    v = accumarray(voxelizedpc,1);
    ind = find(v>=2);
    v2 = sub2ind(size(v),voxelizedpc(:,1),voxelizedpc(:,2),voxelizedpc(:,3));
    filteredBin = ismember(v2,ind);
end

function makeVoxel(edgesX,edgesY,edgesZ,voxel)
    cubePoints = zeros(8,3);

    cubePoints([1,4,5,8],1) = edgesX(voxel(1));
    cubePoints([2,3,6,7],1) = edgesX(voxel(1)+1);
    
    cubePoints([1:2,5:6],2) = edgesY(voxel(2));
    cubePoints([3:4,7:8],2) = edgesY(voxel(2)+1);

    cubePoints(1:4,3) = edgesZ(voxel(3));
    cubePoints(5:8,3) = edgesZ(voxel(3)+1);

    idx = [4 8 5 1 4; 1 5 6 2 1; 2 6 7 3 2; 3 7 8 4 3; 5 8 7 6 5; 1 4 3 2 1]';

    % hold on
    % s = cubePoints(1,2,4,3,:);
    % plot3(s(:,1),s(:,2),s(:,3),'Color','b');
    % plot3(X,Y,Z,'Color','b');
    % plot3(X,Y,Z,'Color','b');
    % plot3(X,Y,Z,'Color','b');
    hold on
    for i = 1:size(idx,1)
        patch(cubePoints(idx(:,i),1),cubePoints(idx(:,i),2),cubePoints(idx(:,i),3),'yellow', 'facealpha', 0.1);
    end
end
function pc = temp(v,pcout,pcin)
    ind = find(v,1);
    v(ind) = 0;
    [i, j, k] = ind2sub(size(v), ind);
    v1 = ismember(pcout(:,1),i) & ismember(pcout(:,2),j) & ismember(pcout(:,3),k);
    pc = pcin(v1,:);
end

function pout = point_transformation(pc, p)
    T = eye(4);
    T(1:3,4) = p; 
    numberOfPoints = size(pc,1);
    pcH = ones(numberOfPoints,size(pc,2)+1);
    pcH(:,1:3) = pc;
    pout = zeros(numberOfPoints,3);
    for i = 1:numberOfPoints
        pH = (T*pcH(i,:)')';
        pout(i,:) = pH(1:3);
    end
    
end