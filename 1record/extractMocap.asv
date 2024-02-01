function [timestamp,armRotation,armVelocity,armVelocityW,forearmRotation,forearmVelocity, forearmVelocityW,Mmag]= extractMocap(filename,delay,maxTime,BodyFrames_orientation,kinectTimestamp)  
    T = readcell(filename);
    numberOfRows = size(T,1);

    H = T(1:6,:);
    H(cellfun(@(x) any(ismissing(x)), H))={''};
    
    r1 = find(contains(H(3,:), 'Unlabeled'));
    lastColumn = r1(1)-1;

    H = H(1:6,1:lastColumn);

    D = T(7:numberOfRows,1:lastColumn);
    % f = cellfun(@(x) any(ismissing(x)), D);
    D(cellfun(@(x) any(ismissing(x)), D))= {nan};
    % r = any(cellfun(@(x) any(isnan(x)),D),2);
    % D(r,:) = ;
    
    % D(sum(isnan(D), 2) == 1, :) = [];

    F = [H;D];
    
    % D = T(7:numberOfRows,:);
    % D(cellfun(@(x) any(ismissing(x)), D))={nan};

    F = crudeSync(F, delay, maxTime);
    numberOfRows = size(F,1);
    timestamp = cell2mat(F(7:numberOfRows,2));
    
    armRotation = nan;
    armVelocity = nan;
    armVelocityW = nan;
    forearmRotation = nan;
    forearmVelocity = nan;
    forearmVelocityW = nan;
    handVelocityW = nan;
    
    idx = ones(numberOfRows-6,1);
    % joint points
    [shoulderPositionX, nonNaNIdx] = extractData(F, "Left_ARM:Left_Anterior_Shoulder", "Position", "X");
    idx = idx & nonNaNIdx;
    [shoulderPositionY, nonNaNIdx] = extractData(F, "Left_ARM:Left_Anterior_Shoulder", "Position", "Y");
    idx = idx & nonNaNIdx;
    [shoulderPositionZ, nonNaNIdx] = extractData(F, "Left_ARM:Left_Anterior_Shoulder", "Position", "Z");
    idx = idx & nonNaNIdx;
    shoulderPosition = [shoulderPositionX,shoulderPositionY,shoulderPositionZ];

    [elbowPositionX, nonNaNIdx] = extractData(F, "Left_ARM:Left_Lateral_Elbow", "Position", "X");
    idx = idx & nonNaNIdx;
    [elbowPositionY, nonNaNIdx] = extractData(F, "Left_ARM:Left_Lateral_Elbow", "Position", "Y");
    idx = idx & nonNaNIdx;
    [elbowPositionZ, nonNaNIdx] = extractData(F, "Left_ARM:Left_Lateral_Elbow", "Position", "Z");
    idx = idx & nonNaNIdx;
    elbowPosition = [elbowPositionX,elbowPositionY,elbowPositionZ];

    [wristPositionX, nonNaNIdx] = extractData(F, "Left_ARM:Left_Midline_Wrist", "Position", "X");
    idx = idx & nonNaNIdx;
    [wristPositionY, nonNaNIdx] = extractData(F, "Left_ARM:Left_Midline_Wrist", "Position", "Y");
    idx = idx & nonNaNIdx;
    [wristPositionZ, nonNaNIdx] = extractData(F, "Left_ARM:Left_Midline_Wrist", "Position", "Z");
    idx = idx & nonNaNIdx;
    wristPosition = [wristPositionX,wristPositionY,wristPositionZ];
    
    
    shoulderPosition = shoulderPosition(idx,:);
    elbowPosition = elbowPosition(idx,:);
    wristPosition = wristPosition(idx,:);
    timestamp = timestamp(idx);

    shoulderOutliersIdx = identifyOutliers(shoulderPosition);
    elbowOutliersIdx = identifyOutliers(elbowPosition);
    wristOutliersIdx = identifyOutliers(wristPosition);
    idx = (~shoulderOutliersIdx & ~elbowOutliersIdx & ~wristOutliersIdx);
    shoulderPosition = shoulderPosition(idx,:);
    elbowPosition = elbowPosition(idx,:);
    wristPosition = wristPosition(idx,:);
    timestamp = timestamp(idx);
    
    numberOfFrames = size(shoulderPosition,1);
    armRotation = zeros(numberOfFrames-1,3);
    forearmRotation = zeros(numberOfFrames-1,3);
    for i = 1:numberOfFrames-1
        display(sprintf('i = %d',i));
        shoulderPosition_a = shoulderPosition(i,:); 
        shoulderPosition_b = shoulderPosition(i+1,:);
        elbowPosition_a = elbowPosition(i,:);
        elbowPosition_b = elbowPosition(i+1,:);
        wristPosition_a = wristPosition(i,:);
        wristPosition_b = wristPosition(i+1,:);

        [~, mappedKinectTimestampIndex] = min(abs(kinectTimestamp-timestamp(i)));
        B = squeeze(BodyFrames_orientation(mappedKinectTimestampIndex,:,:));

        % convert to shoulder frame
        elbowPosition1_a = elbowPosition_a - shoulderPosition_a;
        elbowPosition1_b = elbowPosition_b - shoulderPosition_b;
        wristPosition1_a = wristPosition_a - shoulderPosition_a;
        wristPosition1_b = wristPosition_b - shoulderPosition_b;

        angle_axis = vrrotvec(elbowPosition1_a,elbowPosition1_b);
        R = axang2rotm(angle_axis);
        armRotation(i,:) = rad2deg(rotmat2vec3d(R)*B');

        elbowPosition2_a = elbowPosition1_a * R';
        elbowPosition2_b = elbowPosition1_b;
        wristPosition2_a = wristPosition1_a * R' - elbowPosition2_a;
        wristPosition2_b = wristPosition1_b - elbowPosition2_b;
    
        angle_axis = vrrotvec(wristPosition2_a,wristPosition2_b);
        R = axang2rotm(angle_axis);
        forearmRotation(i,:) = rad2deg(rotmat2vec3d(R)*B');
    end
    timeDelta = diff(timestamp);
    timestamp = timestamp(1:numberOfFrames-1,1);

    [b,a] = butter(5,10/15);
    armRotation = filter(b,a,armRotation);
    forearmRotation = filter(b,a,forearmRotation);
    % for i = 1:numberOfFrames-1
    %     [~, mappedKinectTimestampIndex] = min(abs(kinectTimestamp-timestamp(i)));
    %     B = squeeze(BodyFrames_orientation(mappedKinectTimestampIndex,:,:));
    %     armRotation = armRotation*B';
    %     forearmRotation = forearmRotation*B';
    % end

    armVelocity = armRotation./timeDelta;
    forearmVelocity = forearmRotation./timeDelta;
    [b,a] = butter(5,3/15); 
    armVelocity = filter(b,a,armVelocity);
    forearmVelocity = filter(b,a,forearmVelocity);


    armRotation = cumsum(armRotation,1);
    forearmRotation = cumsum(forearmRotation,1);
    Mmag = vecnorm(armVelocity+forearmVelocity,2,2);

    % shoulderToElbowDistance = [elbowPositionX-shoulderPositionX,...
    %                            elbowPositionY-shoulderPositionY,...
    %                            elbowPositionZ-shoulderPositionZ,...
    %                           vecnorm(elbowPosition - shoulderPosition,2,2)];
    % elbowToWristDistance = [wristPositionX-elbowPositionX,...
    %                            wristPositionY-elbowPositionY,...
    %                            wristPositionZ-elbowPositionZ,...
    %                           vecnorm(wristPosition - elbowPosition,2,2)];
    % shoulderToWristDistance = [wristPositionX-shoulderPositionX,...
    %                            wristPositionY-shoulderPositionY,...
    %                            wristPositionZ-shoulderPositionZ,...
    %                           vecnorm(wristPosition - shoulderPosition,2,2)];

    % % rigid bodies
    % armVelocityX = extractData(F, "Left_Upper_Arm", "Rotation", "X")*0.1;
    % armVelocityY = extractData(F, "Left_Upper_Arm", "Rotation", "Y")*0.1;
    % armVelocityZ = extractData(F, "Left_Upper_Arm", "Rotation", "Z")*0.1;
    % armVelocity = [armVelocityX,armVelocityY,armVelocityZ];
    % 
    % armPositionX = extractData(F, "Left_Upper_Arm", "Position", "X");
    % armPositionY = extractData(F, "Left_Upper_Arm", "Position", "Y");
    % armPositionZ = extractData(F, "Left_Upper_Arm", "Position", "Z");
    % armPosition = [armPositionX,armPositionY,armPositionZ];
    % shoulderToArmDistance = [ abs(armPositionX-shoulderPositionX),...
    %                             abs(armPositionY-shoulderPositionY),...
    %                             abs(armPositionZ-shoulderPositionZ),...
    %                             vecnorm(armPosition - shoulderPosition,2,2)];
    % armVelocity = armVelocity./shoulderToArmDistance(:,1:3);
    % armVelocity = convangvel(armVelocity,'rad/s','deg/s');
    % 
    % armVelocityW = extractData(F, "Left_Upper_Arm", "Rotation", "W");
    % armVelocityW = armVelocityW./shoulderToArmDistance(:,4);
    % armVelocityW = convangvel(armVelocityW,'rad/s','deg/s');
    % 
    % forearmVelocityX = extractData(F, "LEft_FRM", "Rotation", "X");
    % forearmVelocityY = extractData(F, "LEft_FRM", "Rotation", "Y");
    % forearmVelocityZ = extractData(F, "LEft_FRM", "Rotation", "Z");
    % forearmVelocity = [forearmVelocityX,forearmVelocityY,forearmVelocityZ];
    % forearmRotation = forearmRotation./elbowToWristDistance(:,1:3);
    % forearmRotation = convangvel(forearmRotation,'rad/s','deg/s');
    % 
    % forearmRotationW = extractData(F, "LEft_FRM", "Rotation", "W");
    % forearmRotationW = forearmRotationW./elbowToWristDistance(:,4);
    % forearmRotationW = convangvel(forearmRotationW,'rad/s','deg/s');
    % 
    % handRotationX = extractData(F, "Left_Hand", "Rotation", "X");
    % handRotationY = extractData(F, "Left_Hand", "Rotation", "Y");
    % handRotationZ = extractData(F, "Left_Hand", "Rotation", "Z");
    % handRotation = [handRotationX,handRotationY,handRotationZ];
    % handRotation = convangvel(handRotation,'rad/s','deg/s');
    % handRotation = handRotation./shoulderToWristDistance(:,1:3);
    % handRotation = convangvel(handRotation,'rad/s','deg/s');
    % 
    % handRotationW = extractData(F, "Left_Hand", "Rotation", "W"); 
    % handRotationW = handRotationW./shoulderToWristDistance(:,4);
    % handRotationW = convangvel(handRotationW,'rad/s','deg/s');

end

function [A,nonNanIdx]= extractData(T, name, type, primitive)
    numberOfRows = size(T,1);
    numberOfColumns = size(T,2);
    B_name = ismember(T(3,1:numberOfColumns),name);
    B_type = ismember(T(5,1:numberOfColumns),type);
    B_primitive = ismember(T(6,1:numberOfColumns),primitive);
    i = B_name & B_type & B_primitive;
    A = cell2mat(T(7:numberOfRows,i));
    nonNanIdx = not(isnan(A));
end

function F = crudeSync(T, delay,maxTime)
    numberOfRows = size(T,1);
    timestamp = cell2mat(T(7:numberOfRows,2));
    timestamp = timestamp-delay;
    i = (timestamp>=0 & timestamp<=maxTime);
    H = T(1:6,:);
    T(7:numberOfRows,2) = num2cell(timestamp);
    D = T(7:numberOfRows,:);
    D = D(i,:);
    F = [H;D];
end

function idx = identifyOutliers(pc)
    dis = .01;
    %assumes first and last points are correct   
    numberOfPoints = size(pc,1);
    idx = zeros([numberOfPoints,1]);
    d = zeros([numberOfPoints,1]);
    % sumDistance = 0;
    % for i = 1:numberOfPoints-1
    % 
    % end
    for i = 2:numberOfPoints
        c = pc(i,:);
        a = pc(i-1,:);
        ac = norm(c-a);
        d(i) = ac;
        if idx(i-1) == 1 && ac>dis
            continue;
        elseif ac>dis
            idx(i) = 1;
        elseif idx(i-1) == 1 && ac<dis
            idx(i) = 1;
        end
    end
end