function [shoulderDepthFrames, elbowDepthFrames, wristDepthFrames] = bodySegmentation(bodyFrames,depthFrames,numberOfFrames)
    med_val_shoulder = nan;
    std_val_shoulder = nan;
    med_val_elbow = nan;
    std_val_elbow = nan;
    med_val_wrist = nan;
    std_val_wrist = nan;
    i = 1;
    shoulderDepthFrames = zeros(size(depthFrames));
    elbowDepthFrames = zeros(size(depthFrames));
    wristDepthFrames = zeros(size(depthFrames));
    while i<=numberOfFrames
        depth = rescale(double(depthFrames(:,:,i)),0,1);
        h = size(depth,1);
        w = size(depth,2);
        filter = [1,h-100,1,w,0.075,0.125];
        body = getBodyImage(depth, filter);

        jointKeypointIndexes = [13,14,15];
        jointKeypoints = (bodyFrames(i).Position2d_depth(:,:))';
        armJointKeypoints = jointKeypoints(jointKeypointIndexes,:);
        search_radius_denom = nan;
        num_std_dev = 3.5;

        % hold on
        % imshow(rescale(double(depth),0,1));
        % viscircles(jointKeypoints(jointKeypointIndexes,:), 10,'Color','b');
        % hold off
        [segmentedDepthShoulder, med_val_shoulder, std_val_shoulder] = segment_body_around_point(depth, body, armJointKeypoints(1,:), search_radius_denom, num_std_dev, med_val_shoulder, std_val_shoulder);
        [segmentedDepthElbow, med_val_elbow, std_val_elbow] = segment_body_around_point(depth, body, armJointKeypoints(2,:), search_radius_denom, num_std_dev, med_val_elbow, std_val_elbow);
        [segmentedDepthWrist, med_val_wrist, std_val_wrist] = segment_body_around_point(depth, body, armJointKeypoints(3,:), search_radius_denom, num_std_dev, med_val_wrist, std_val_wrist);
        shoulderDepthFrames(:,:,i) = segmentedDepthShoulder;
        elbowDepthFrames(:,:,i) = segmentedDepthElbow;
        wristDepthFrames(:,:,i) = segmentedDepthWrist;
        i = i+1;
        % subplot(1,3,1), imshow(segmentedDepthShoulder>0);
        % subplot(1,3,2), imshow(segmentedDepthElbow>0);
        % subplot(1,3,3), imshow(segmentedDepthWrist>0);

    end
    
    %clearvars depth depthHeight depthWidth i numberOfFrames validData
    % save(outputFolder+"\depthImages.mat")
    %clear all
    
end

function bin = getBodyImage(depth, filter)
    % filter: [xMin,xMax,yMin,yMax,depthMin,depthMax]
    % b = zeros(size(depth));
    % b(depth>filter(5) & depth<filter(6)) = depth(depth>filter(5) & depth<filter(6));
    b = depth>filter(5) & depth<filter(6);
    bin = zeros(size(b));
    bin(filter(1):filter(2),filter(3):filter(4)) = b(filter(1):filter(2),filter(3):filter(4));
end

function mask = create_circular_mask(h, w, center, radius)
    % Creates circular mask around a given point.
    % Source: https://stackoverflow.com/a/44874588

    if isnan(center) % use the middle of the image
        center = [round(w/2), round(h/2)];
    end
    if isnan(radius) % use the smallest distance between the center and image walls
        radius = min([center(1), center(2), w-center(1), h-center(2)]);
    end
    Y = (1:h)';
    X = 1:w;
    dist_from_center = sqrt((X - double(center(1))).^2 + (Y-double(center(2))).^2);
    % 
    mask = dist_from_center <= radius;
end

% class SegmentClass():
% 
%     def __init__(self, data):
%         self.std_val = None
%         self.med_val = None
%         self.blank_image = np.zeros(shape=(data['depth_frames'][0].shape[0], data['depth_frames'][0].shape[1])).astype(np.int16)
% 
function [segment_image, med_val, std_val] = segment_body_around_point(depth_img, body_indx_img, center, search_radius_denom, num_std_dev, med_val, std_val)
%         Segment human body around given point. Uses the body index frame.
% 
%         Params
%         ------
%         stats_radius_denom : float, default None
%             Radius around keypoint to compute summary statistics to determine pixel values to include
%             in body segmentation. Radius computed by dividing the image width by stats_radius_denom.
% 
%         search_radius_denom : float, default None
%             Search radius around keypoint to perform body segmentation. Radius computed by dividing the 
%             image width by search_radius_denom.
% 
%         num_std_dev : float, default 3.5
%             Number of standard deviations for median value in search_radius to look for relevant pixels.
% 
%         Returns
%         -------
%         Masked depth image with relevant body part within search_radius segmented.
%         """
%           search_radius_denom=None, num_std_dev=3.5, plot=True    
    blank_image = zeros(size(depth_img));
    h = size(depth_img,1);
    w = size(depth_img,2);
%         no body index found and no historical data
    if anynan(body_indx_img) && isnan(std_val) && isnan(med_val)
        segment_image = blank_image;
        return;
    end
    if isnan(search_radius_denom)
        search_radius = w/10;  % also seems to be a good value
    else
        search_radius = w/search_radius_denom;
    end
    if (any(body_indx_img) == false)
    %     std_val = self.std_val
    %     med_val = self.med_val
    else
        %   get small area around keypoint to identify relevant range of depth values for segmentation
        masked_depth_image = depth_img;
        masked_depth_image(~body_indx_img) = 0;
%         #     prox_image = masked_image.copy()  # for debugging
% 
%             # get area to search for relevant pixels
        search_mask = create_circular_mask(h, w, center, search_radius);
        masked_depth_image(~search_mask) = 0;
% 
%             # get statistics for segmentation
%             # ignore values of 0
        med_val = median(nonzeros(masked_depth_image));
        std_val = std(nonzeros(masked_depth_image));
    end

%   get area to search for relevant pixels
    search_mask = create_circular_mask(h, w, center, search_radius);
    masked_image = depth_img;
    masked_image(~search_mask) = 0;
% 
%         # use values within certain number of standard deviations from the mean
    seg_mask = (masked_image >= med_val - std_val*num_std_dev) & (masked_image <= med_val + std_val*num_std_dev);
    new_masked_image = masked_image;
    new_masked_image(~seg_mask) = 0;
    segment_image = new_masked_image;
% 
%         if plot is True:
%             plt.imshow(new_masked_image)
%         return 
end