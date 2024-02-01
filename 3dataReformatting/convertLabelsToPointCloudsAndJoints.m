function convertLabelsToPointCloudsAndJoints(inputFolder, outputFolder)
% datafilePath = "C:\Users\main-portable\OneDrive - George Mason University - O365 Production\sharedProjectFolder\MotionEstimationOfRigidKinematicStructureViaPointClouds\robotData\dataset1";
% convertLabelsToPointCloudsAndJoints(datafilePath, datafilePath);
    load(inputFolder+"\labels.mat");
    uA = gTruth.LabelData.upperArm;
    lA = gTruth.LabelData.lowerArm;
    numberOfFrames = size(uA,1);

    wrist_kp = zeros(numberOfFrames,3);
    
    elbow_kp = zeros(numberOfFrames,3);

    shoulder_kp = zeros(numberOfFrames,3);
    
    
    for i = 1:numberOfFrames
        Uroi = cuboidModel(uA(i,:));
        Lroi = cuboidModel(lA(i,:));

        pcd = pcread(inputFolder+"\frame"+int2str(i)+".ply");
        Uindices = findPointsInsideCuboid(Uroi,pcd);
        Lindices = findPointsInsideCuboid(Lroi,pcd);
      
        Upcd = select(pcd,Uindices);
        Lpcd = select(pcd,Lindices);

        Upcd = scalePointCloud(Upcd,100);
        Lpcd = scalePointCloud(Lpcd,100);

        if i<numberOfFrames
            variableCreator ("upper_source_"+int2str(i), Upcd.Location);
            variableCreator ("lower_source_"+int2str(i), Upcd.Location);
        end

        if i>1
            variableCreator ("upper_target_"+int2str(i-1), Lpcd.Location);
            variableCreator ("lower_target_"+int2str(i-1), Lpcd.Location);
        end

        
        Uc = getCornerPoints(Uroi);
        [~,indices] = mink(vecnorm(Uc(1,:)-Uc,2,2),4);
        indicesUpperArm = ones(8,1);
        indicesUpperArm(indices) = 2;
        upperArmKeypoint1 = mean(Uc(indicesUpperArm==1,:));
        upperArmKeypoint2 = mean(Uc(indicesUpperArm==2,:));

        Lc = getCornerPoints(Lroi);
        [~,indices] = mink(vecnorm(Lc(1,:)-Lc,2,2),4);
        indicesLowerArm = ones(8,1);
        indicesLowerArm(indices) = 2;
        lowerArmKeypoint1 = mean(Lc(indicesLowerArm==1,:));
        lowerArmKeypoint2 = mean(Lc(indicesLowerArm==2,:));

        distances = [norm(upperArmKeypoint1-lowerArmKeypoint1),...
                    norm(upperArmKeypoint1-lowerArmKeypoint2),...
                    norm(upperArmKeypoint2-lowerArmKeypoint1),...
                    norm(upperArmKeypoint2-lowerArmKeypoint2)];
        [~,I] = min(distances);

        switch I
            case 1
                shoulder_kp(i,:) = upperArmKeypoint2;
                elbow_kp(i,:) = mean([upperArmKeypoint1;lowerArmKeypoint1]);
                wrist_kp(i,:) = lowerArmKeypoint2;
            case 2
                shoulder_kp(i,:) = upperArmKeypoint2;
                elbow_kp(i,:) = mean([upperArmKeypoint1;lowerArmKeypoint2]);
                wrist_kp(i,:) = lowerArmKeypoint1;
            case 3
                shoulder_kp(i,:) = upperArmKeypoint1;
                elbow_kp(i,:) = mean([upperArmKeypoint2;lowerArmKeypoint1]);
                wrist_kp(i,:) = lowerArmKeypoint2;
            case 4
                shoulder_kp(i,:) = upperArmKeypoint1;
                elbow_kp(i,:) = mean([upperArmKeypoint2;lowerArmKeypoint2]);
                wrist_kp(i,:) = lowerArmKeypoint1;
        end

    end
    clearvars distances ...
        gTruth...
        i...
        I...
        indices...
        indicesLowerArm...
        indicesUpperArm...
        inputFolder...
        lA...
        Lc...
        Lindices...
        lowerArmKeypoint1...
        lowerArmKeypoint2...
        Lpcd...
        Lroi...
        numberOfFrames...
        pcd...
        uA...
        Uc...
        Uindices...
        Upcd...
        upperArmKeypoint1...
        upperArmKeypoint2...
        Uroi;
    save(outputFolder+"\dataset.mat");
    clear;
end
function pcdOUT = scalePointCloud(pcdIN,s)
    pcd = pcdIN.Location * s;
    pcdOUT = pointCloud(pcd,"Color",pcdIN.Color);
end
function variableCreator ( newVar, variable )
    assignin ( 'caller', newVar, variable );
end