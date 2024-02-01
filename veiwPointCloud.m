datafile = "datasetA3072pbinnedNfov";
robotDatafilePath = "datasets\datasetA\FILTERED"+datafile+".mat";
load(robotDatafilePath)
for i = 1:numberOfFrames
    hold on
    pcshow(pointCloud(forearmPointClouds(:,:,i),"color","red"));
    % pcshow(pointCloud(armPointClouds(:,:,i),"color","green"));
    title("frame"+int2str(i));
    waitforbuttonpress
    hold off
end