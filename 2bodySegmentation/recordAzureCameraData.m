% Create KinZ object and initialize it
% Available options: 
% '720p', '1080p', '1440p', '1535p', '2160p', '3072p'
% 'binned' or 'unbinned'
% 'wfov' or 'nfov'
function recordAzureCameraData(numberOfFrames, outputFolder)
    % datafilePath = "C:\Users\main-portable\OneDrive - George Mason University - O365 Production\sharedProjectFolder\MotionEstimationOfRigidKinematicStructureViaPointClouds\robotData\dataset1";
    % recordAzureCameraData(10, datafilePath)

    addpath('.\Mex');
    kz = KinZ('3072p', 'binned', 'nfov', 'imu_on');
    
    % Create matrices for the images
    depthWidth = kz.DepthWidth; 
    depthHeight = kz.DepthHeight; 
    depth = zeros(depthHeight,depthWidth,'uint16');
    pc = pointCloud(zeros(depthHeight*depthWidth,3));

    imuData = zeros(numberOfFrames,9);

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
            pcdkz = kz.getpointcloud('output','pointCloud','color','true');
            % scale
            pcd = scalePCD(pcdkz, 1000);
            % reorient
            pcd = reorientPCD(pcd);
            % ROI
            pcd = ROIPCD(pcd, [0, 1.25, -1, 1, -inf, inf]);

            pcwrite(pcd, outputFolder+"\frame"+int2str(i)+".ply")
            sensorData = kz.getsensordata;
            imuData(i,1) = sensorData.temp;
            imuData(i,2) = sensorData.acc_x;
            imuData(i,3) = sensorData.acc_y;
            imuData(i,4) = sensorData.acc_z;
            imuData(i,5) = sensorData.acc_timestamp_usec;
            imuData(i,6) = sensorData.gyro_x;
            imuData(i,7) = sensorData.gyro_y;
            imuData(i,8) = sensorData.gyro_z;
            imuData(i,9) = sensorData.gyro_timestamp_usec;
            i = i+1;
        end
    end
    titles = {'temp','acc_x' 'acc_y','acc_z','acc_timestamp_usec','gyro_x','gyro_y','gyro_z', 'gyro_timestamp_usec'};
    C = [titles; num2cell(imuData)];
    writecell(C,outputFolder+'\IMUdata.csv')
    
    kz.delete;
end

function pcdOUT = scalePCD(pcdIN, scale)
    pcdLoc = pcdIN.Location/scale;
    pcdOUT = pointCloud(pcdLoc,"Color",pcdIN.Color);
end

function pcdOUT = ROIPCD(pcdIN, roi)
    indices = findPointsInROI(pcdIN,roi);
    pcdOUT = select(pcdIN,indices);
end

function pcdOUT = reorientPCD(pcdIN)
    %reorient
    pcdLoc = pcdIN.Location;
    c1 = pcdLoc(:,1);
    c2 = pcdLoc(:,2);
    c3 = pcdLoc(:,3);
    pcdLoc(:,1) = c3;
    pcdLoc(:,2) = c1;
    pcdLoc(:,3) = -1*c2;
    pcdOUT = pointCloud(pcdLoc,"Color",pcdIN.Color);
end