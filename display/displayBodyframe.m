function displayBodyframe(bodyKeypoints, color, markersize)
    hold on
    for i = 1:size(bodyKeypoints,3)
        pcshow(pointCloud(bodyKeypoints(:,:,i),"color", color),"MarkerSize",markersize);
    end
    hold off
end

