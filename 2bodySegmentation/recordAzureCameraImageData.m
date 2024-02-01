% Create KinZ object and initialize it
% Available options: 
% '720p', '1080p', '1440p', '1535p', '2160p', '3072p'
% 'binned' or 'unbinned'
% 'wfov' or 'nfov'
function recordAzureCameraImageData(numberOfFrames, outputFolder)
    % datafilePath = "C:\Users\main-portable\OneDrive - George Mason University - O365 Production\sharedProjectFolder\MotionEstimationOfRigidKinematicStructureViaPointClouds\robotData\dataset1";
    % recordAzureCameraImageData(10, datafilePath)

    addpath('.\Mex');
    kz = KinZ('3072p', 'binned', 'nfov');
    
    % Create matrices for the images
    depthWidth = kz.DepthWidth; 
    depthHeight = kz.DepthHeight; 
    depths = zeros(depthHeight,depthWidth,numberOfFrames,'uint16');
    % Main Loop
    i = 1;
    while i<=numberOfFrames
        % Get frames from Kinect and save them on underlying buffer
        validData = kz.getframes('color','depth','imu');
        
        % Before processing the data, we need to make sure that a valid
        % frame was acquired.
        if validData
            % Get the pointcloud with color from the Kinect
            % Select the output 'pointCloud' to use the MATLAB built-in
            % pointCloud object. 
            % For MATLAB versions older than 2015b, use 'output','raw' and use
            % scatter3 to plot the point cloud. See pointCloudDemo1.m
            depth = kz.getdepth();
            depths(:,:,i) = depth; 
            i = i+1;
        end
    end
    kz.delete;
    clearvars depth depthHeight depthWidth i numberOfFrames validData kz
    save(outputFolder+"\depthImages.mat")
    clear all
end