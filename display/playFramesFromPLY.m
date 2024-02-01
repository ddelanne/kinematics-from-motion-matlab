function playFramesFromPLY(datafilePath,numberOfFrames)
% datafilePath = "C:\Users\main-portable\OneDrive - George Mason University - O365 Production\sharedProjectFolder\MotionEstimationOfRigidKinematicStructureViaPointClouds\robotData\dataset1";
    for i = 1:numberOfFrames
        pcd = pcread(datafilePath+"\frame"+int2str(i)+".ply");
        
        pcshow(pcd);
        title("frame"+int2str(i));
        waitforbuttonpress
    end
end

