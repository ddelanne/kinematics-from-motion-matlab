clear;
%add path to all functions:
mfilename = matlab.desktop.editor.getActiveFilename;
folder = fileparts(which(mfilename));
addpath(genpath(folder));
% get data
d = "datasetB";
datasetFolder = "datasets/"+d+"/";
datasetName = d;

% filename = datasetName+'Mocap.csv';
% CSV
% mocapDatasetFolder = datasetFolder+filename;
% mocapDatasetFolder = datasetFolder+"test.csv";

% camera data
datasetName = d;
filename = datasetName+".mat";
% load(datasetFolder+"SEGMENTED"+filename);
load(datasetFolder+filename);
% numberOfFrames = 100;
% %segment azure data
s = size(depthFrames,1)*size(depthFrames,2);
[bodyKeypoints, armPointClouds, mean_arm, forearmPointClouds, mean_forearm] = bodySegmentation3D(bodyFrames, pointClouds, numberOfFrames, s);

% save in correct format
clearvars ans i s mfilename folder datafilePath
filename = datasetName+".mat";
save(datasetFolder+"SEGMENTED"+filename);
