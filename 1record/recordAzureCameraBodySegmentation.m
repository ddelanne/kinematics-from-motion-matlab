function [calibration, depthFrames, bodyFrames, pointClouds, timestamps] = recordAzureCameraBodySegmentation(numberOfFrames)
    % datafilePath = "C:\Users\main-portable\OneDrive - George Mason University - O365 Production\sharedProjectFolder\MotionEstimationOfRigidKinematicStructureViaPointClouds\robotData\dataset3";
    % Create KinZ object and initialize it
    % Available options: 
    % '720p', '1080p', '1440p', '1535p', '2160p', '3072p'
    % 'binned' or 'unbinned'
    % 'wfov' or 'nfov'
    % 'imu_on' or 'imu_on'
    kz = KinZ('3072p','binned', 'nfov', 'imu_off', 'bodyTracking');
    % images sizes
    depthWidth = kz.DepthWidth; 
    depthHeight = kz.DepthHeight;
    calibration = kz.getcalibration();
    % Create matrices for the images
    depthFrames = zeros(depthHeight,depthWidth,numberOfFrames,'uint16');
    timestamps = zeros(numberOfFrames,1);
    bodyFrames(1,numberOfFrames) = struct("Id",nan,"Position3d",nan,"Position2d_rgb",nan,"Position2d_depth",nan,"Orientation",nan,"Confidence",nan);
    pointClouds = zeros(depthHeight*depthWidth,3,numberOfFrames);
    i = 1;
    pause;
    pause(5);
    while i<=numberOfFrames
        % Get frames from Kinect and save them on underlying buffer
        validData = kz.getframes('color','depth', 'bodies');
        
        % Before processing the data, we need to make sure that a valid
        % frame was acquired.
        if validData
            % Get the pointcloud with color from the Kinect
            % Select the output 'pointCloud' to use the MATLAB built-in
            % pointCloud object. 
            % For MATLAB versions older than 2015b, use 'output','raw' and use
            % scatter3 to plot the point cloud. See pointCloudDemo1.m
            [depth, depth_timestamp] = kz.getdepth();
            depthFrames(:,:,i) = depth;
            timestamps(i) = depth_timestamp;
            numBodies = kz.getnumbodies;
            [pc, timestamp] = kz.getpointcloud('output','raw','color','false');
            pointClouds(:,:,i) = pc;
            % if i == 1
            %     pointCloudFrames = [pc];
            % else
            %     pointCloudFrames(:,i) = cat(pointCloudFrames,)
            % end
            if numBodies == 0
                % bodyFrames() = [bodyFrames, struct("Id",0,"Position3d",zeros(3,32),"Position2d_rgb",uint32(zeros(2,32)),"Position2d_depth",uint32(zeros(2,32)),"Orientation",zeros(4,32),"Confidence",uint32(zeros(1,32)))];
            else
                bodyFrames(i) = kz.getbodies();
                % bodyFrames = [bodyFrames, kz.getbodies()];
            end
        end
        i = i+1;
    end
    kz.delete;

    i = 1;
    % while i<=numberOfFrames
    %     depth = depthFrames(:,:,i);
    %     body = bodyFrames(i);
    % 
    %     jointKeypointIndexes = [6,7,8];
    %     jointKeyPoints = body.Position2d_depth(:,jointKeypointIndexes);
    %     imshow(depth);
    %     viscircles(center, 10,'Color','b');
    %     i = i+1;
    % end
    %clearvars depth depthHeight depthWidth i numberOfFrames validData
    % save(outputFolder+"\depthImages.mat")
    %clear all
    
end

