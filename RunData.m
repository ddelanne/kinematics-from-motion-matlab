clear;
%add path to all functions:
addpath('.\MexBodyTracking');
mfilename = matlab.desktop.editor.getActiveFilename;
folder = fileparts(which(mfilename));
addpath(genpath(folder));

%record azure data
numberOfFrames = 200;% FILL ME
d = "datasetA"; % FILL ME
% datasetName = "datasets\"+d+"\FILTEREDDEBUG"+d+"3072pbinnedNfov.mat";
% load(datasetName);
datasetFolder = "datasets/"+d+"/";
datasetName = d;
% get data
% pause(5);
[calibration, depthFrames, bodyFrames, pointClouds, timestamp] = recordAzureCameraBodySegmentation(numberOfFrames);

% save in correct format
clearvars ans i s mfilename folder
filename = datasetName+".mat";
save(datasetFolder+filename);

% function pointCloudOUT = reorientPointCloud(pointCloudIN)
% pointCloudOUT = zeros(size(pointCloudIN));
% for i = 1:size(pointCloudIN,3)
%     pc = pointCloudIN(:,:,i);
%     pc2 = (rotx(-90)*rotz(-90)*pc')';
%     pointCloudOUT(:,:,i) = pc2;
% end
% end