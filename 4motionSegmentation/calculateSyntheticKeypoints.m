function [shoulderKeypoints, elbowKeypoints, wristKeypoints, oppositeShoulderKeypoints, middleTorsoKeypoints] = calculateSyntheticKeypoints(bodyKeypoints)
    numberOfFrames = size(bodyKeypoints,3);
    
    [originalClevicalKeypoints ,originalShoulderKeypoints, originalElbowKeypoints, originalWristKeypoints, originalOppositeShoulderKeypoints, originalMiddleTorsoKeypoints] = getArmKeypoints(bodyKeypoints);
    
    % hold on
    % pcshow(pointCloud(bodyKeypoints(:,:,1),"color", "blue"))
    % pcshow(pointCloud(originalShoulderKeypoints,"color", "green"));
    % pcshow(pointCloud(originalElbowKeypoints,"color", "green"));
    % pcshow(pointCloud(originalWristKeypoints,"color", "green"));

    % [syntheticShoulderKeypoints, syntheticElbowKeypoints, syntheticWristKeypoints] = generateSyntheticKeypointsFromRotation(originalShoulderKeypoints, originalElbowKeypoints, originalWristKeypoints, numberOfFrames);
    syntheticShoulderKeypoints = originalShoulderKeypoints;
    syntheticElbowKeypoints = originalElbowKeypoints;
    syntheticWristKeypoints = originalWristKeypoints;
    % minDistance = 5;
    % 
    % [shoulderKeypoints2, elbowKeypoints2, wristKeypoints2] = adjustShoulderPointsBasedProximity(shoulderKeypoints1, elbowKeypoints1, wristKeypoints1, numberOfFrames, minDistance);
    % 
    % [elbowKeypoints3, wristKeypoints3] = adjustElbowPointsBasedProximity(elbowKeypoints2, wristKeypoints2, numberOfFrames, minDistance);
    % 
    % [shoulderKeypoints4, elbowKeypoints4, wristKeypoints4] = adjustPointsBasedOnBoneLength(shoulderKeypoints2, elbowKeypoints3, wristKeypoints3);

    shoulderKeypoints = syntheticShoulderKeypoints;
    elbowKeypoints = syntheticElbowKeypoints;
    wristKeypoints = syntheticWristKeypoints;
    oppositeShoulderKeypoints = originalOppositeShoulderKeypoints;
    middleTorsoKeypoints = originalMiddleTorsoKeypoints;

    

end

function [syntheticShoulderKeypoints, syntheticElbowKeypoints, syntheticWristKeypoints] = generateSyntheticKeypointsFromRotation(originalShoulderKeypoints, originalElbowKeypoints, originalWristKeypoints, numberOfFrames)
    [shoulderKeypoints1, elbowKeypoints1, wristKeypoints1] = adjustPointsBasedOnBoneLength(originalShoulderKeypoints, originalElbowKeypoints, originalWristKeypoints);

    [generatedShoulderKeypoint,distances] = lineIntersect3D(shoulderKeypoints1, elbowKeypoints1);
    %assuming no or little translation in shoulder:
    shoulderKeypoints2 = repmat(generatedShoulderKeypoint,[numberOfFrames 1]);
    syntheticShoulderKeypoints = shoulderKeypoints2;
    
    shoulderRotationMatrices = zeros(numberOfFrames,3,3);
    shoulderRotationMatrices(1,:,:) = eye(3);
    
    shoulderKeypoints2 = zeros(numberOfFrames,3);
    elbowKeypoints2 = elbowKeypoints1-generatedShoulderKeypoint;
    wristKeypoints2 = wristKeypoints1-generatedShoulderKeypoint;

    [shoulderKeypoints3, elbowKeypoints3, wristKeypoints3] = adjustPointsBasedOnBoneLength(shoulderKeypoints2, elbowKeypoints2, wristKeypoints2);

    elbowKeypoints4 = zeros(numberOfFrames,3);
    elbowKeypoints4(1,:) = elbowKeypoints3(1,:);
    
    wristKeypoints4 = zeros(numberOfFrames,3);
    wristKeypoints4(1,:) = wristKeypoints3(1,:);
    
    for i = 2:numberOfFrames
        angle_axis = vrrotvec(elbowKeypoints3(i,:),elbowKeypoints3(1,:));
        R = axang2rotm(angle_axis);
        shoulderRotationMatrices(i,:,:) = R;
        elbowKeypoints4(i,:) = elbowKeypoints3(i,:)*R';
        wristKeypoints4(i,:) = wristKeypoints3(i,:)*R';
    end
    [generatedElbowKeypoint,distances] = lineIntersect3D(elbowKeypoints4, wristKeypoints4);
    
    elbowKeypoints4 = zeros(numberOfFrames,3);
    for i = 1:numberOfFrames
        R = squeeze(shoulderRotationMatrices(i,:,:));
        elbowKeypoints4(i,:) = generatedElbowKeypoint*inv(R')+generatedShoulderKeypoint;
    end
    syntheticElbowKeypoints = elbowKeypoints4;
    syntheticWristKeypoints = originalWristKeypoints;
    [syntheticShoulderKeypoints, syntheticElbowKeypoints, syntheticWristKeypoints] = adjustPointsBasedOnBoneLength(syntheticShoulderKeypoints, syntheticElbowKeypoints, syntheticWristKeypoints);

end

function [newShoulderKeypoints, newElbowKeypoints, newWristKeypoints] = adjustShoulderPointsBasedProximity(originalShoulderKeypoints, originalElbowKeypoints, originalWristKeypoints, numberOfFrames, minDistance)

    newShoulderKeypoints = zeros(numberOfFrames,3);
    newElbowKeypoints = zeros(numberOfFrames,3);
    newWristKeypoints = zeros(numberOfFrames,3);
    currentSet = false(numberOfFrames,1);
    for i = 1:numberOfFrames
        if sum(currentSet) == 0
            currentSet(i)=1;
            continue;
        end
        k = originalShoulderKeypoints(i,:);
        distance = pdist2(originalShoulderKeypoints(currentSet,:),k);
        if sum(distance > minDistance)>0 || i == numberOfFrames
            pointSet = originalShoulderKeypoints(currentSet,:);
            syntheticKeypoint = mean(pointSet,1);
            syntheticOffset = pointSet - syntheticKeypoint;
            newElbowKeypoints(currentSet,:) = originalElbowKeypoints(currentSet,:) - syntheticOffset; 
            newWristKeypoints(currentSet,:) = originalWristKeypoints(currentSet,:) - syntheticOffset; 
            newShoulderKeypoints(currentSet,:) = repmat(syntheticKeypoint,[sum(currentSet) 1]);
            currentSet = false(numberOfFrames,1);
            if i == numberOfFrames
                if sum(distance > minDistance)>0
                    syntheticKeypoint = originalShoulderKeypoints(numberOfFrames,:);
                end
                syntheticOffset = originalShoulderKeypoints(numberOfFrames,:) - syntheticKeypoint;
                newShoulderKeypoints(numberOfFrames,:) = syntheticKeypoint;
                newElbowKeypoints(numberOfFrames,:) = originalElbowKeypoints(numberOfFrames,:) - syntheticOffset; 
                newWristKeypoints(numberOfFrames,:) = originalWristKeypoints(numberOfFrames,:) - syntheticOffset; 
                break;
            end
        end
        currentSet(i)=1;
    end
end

function [newElbowKeypoints, newWristKeypoints] = adjustElbowPointsBasedProximity(originalElbowKeypoints, originalWristKeypoints, numberOfFrames, minDistance)

    newElbowKeypoints = zeros(numberOfFrames,3);
    newWristKeypoints = zeros(numberOfFrames,3);
    currentSet = false(numberOfFrames,1);
    for i = 1:numberOfFrames
        if sum(currentSet) == 0
            currentSet(i)=1;
            continue;
        end
        k = originalElbowKeypoints(i,:);
        distance = pdist2(originalElbowKeypoints(currentSet,:),k);
        if sum(distance > minDistance)>0 || i == numberOfFrames
            pointSet = originalElbowKeypoints(currentSet,:);
            syntheticKeypoint = mean(pointSet,1);
            syntheticOffset = pointSet - syntheticKeypoint;
            newWristKeypoints(currentSet,:) = originalWristKeypoints(currentSet,:) - syntheticOffset; 
            newElbowKeypoints(currentSet,:) = repmat(syntheticKeypoint,[sum(currentSet) 1]);
            currentSet = false(numberOfFrames,1);
            if i == numberOfFrames
                if sum(distance > minDistance)>0
                    syntheticKeypoint = originalElbowKeypoints(numberOfFrames,:);
                end
                syntheticOffset = originalElbowKeypoints(numberOfFrames,:) - syntheticKeypoint;
                newElbowKeypoints(numberOfFrames,:) = syntheticKeypoint;
                newWristKeypoints(numberOfFrames,:) = originalWristKeypoints(numberOfFrames,:) - syntheticOffset; 
                break;
            end
        end
        currentSet(i)=1;
    end
end

function [newShoulderKeypoints, newElbowKeypoints, newWristKeypoints] = adjustPointsBasedOnBoneLength(originalShoulderKeypoints, originalElbowKeypoints, originalWristKeypoints)
    arm_v = originalShoulderKeypoints-originalElbowKeypoints;
    armLengths = vecnorm(arm_v,2,2);
    avgArmLength = mean(armLengths);
    arm_v = arm_v./armLengths;
    elbowKeypoints1 = originalShoulderKeypoints-arm_v.*(armLengths+(avgArmLength-armLengths));
    wristKeypoints1 = originalWristKeypoints - (originalElbowKeypoints - elbowKeypoints1);
    %get average length of forearm and shift wrist based on arm length
    forearm_v = elbowKeypoints1-wristKeypoints1;
    forearmLengths = vecnorm(forearm_v,2,2);
    avgForearmLength = mean(forearmLengths);
    forearm_v = forearm_v./forearmLengths;
    wristKeypoints2 = elbowKeypoints1-forearm_v.*(forearmLengths+(avgForearmLength-forearmLengths));

    newShoulderKeypoints = originalShoulderKeypoints;
    newElbowKeypoints = elbowKeypoints1;
    newWristKeypoints = wristKeypoints2;
end

function [clevicalKeypoints, shoulderKeypoints, elbowKeypoints, wristKeypoints, oppositeShoulderKeypoints, middleTorsoKeypoints] = getArmKeypoints(bodyKeypoints)
    jointIndexList = [5, 6, 7, 8, 3, 1];
    keypoints = bodyKeypoints(jointIndexList,:, :);
    clevicalKeypoints = squeeze(keypoints(1,:,:))';
    shoulderKeypoints = squeeze(keypoints(2,:,:))';
    elbowKeypoints = squeeze(keypoints(3,:,:))';
    wristKeypoints = squeeze(keypoints(4,:,:))';
    oppositeShoulderKeypoints = squeeze(keypoints(5,:,:))';
    middleTorsoKeypoints = squeeze(keypoints(6,:,:))';
end
