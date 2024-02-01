function playPointCloudFrames(pointClouds,numberOfFrames)
% datafilePath = "C:\Users\main-portable\OneDrive - George Mason University - O365 Production\sharedProjectFolder\MotionEstimationOfRigidKinematicStructureViaPointClouds\robotData\dataset1";
    for i = 1:numberOfFrames
        pcshow(pointClouds(:,:,i));
        title("frame"+int2str(i));
    end
end

