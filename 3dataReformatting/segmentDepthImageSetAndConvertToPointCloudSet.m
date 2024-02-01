function segmentDepthImageSetAndConvertToPointCloudSet(depthImagesDirectoryPath)
%SEGMENTDEPTHIMAGESETANDCONVERTTOPOINTCLOUDSET Summary of this function goes here
%   Detailed explanation goes here
% datafilePath = "C:\Users\main-portable\OneDrive - George Mason University - O365 Production\sharedProjectFolder\MotionEstimationOfRigidKinematicStructureViaPointClouds\robotData\depthData";
% get data from mat file (numberOfFrames, listOfJointKeypoints, dictionary of depth images)
%load(depthImagesDirectoryPath+"\metadata.mat");
numberOfFrames = 1;
for i = 1:numberOfFrames
    %depthImage = depthImages("depthImage"+i);
    %jointKeypoints = listOfJointKeypoints(i);
    %body_indx_img = 0;
    depthImage = 0;
    jointKeypoints = 0;
    body_indx_img = 0;
    search_radius_denom = nan;
    num_std_dev = 3.5;
    plot = true;
    center = 0;
    % search_radius_denom=None, num_std_dev=3.5, plot=True
    segment_body_around_point(depthImage, body_indx_img, center, search_radius_denom, num_std_dev, plot);
end
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
    dist_from_center = sqrt((X - center(1)).^2 + (Y-center(2)).^2);
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
function segment_body_around_point(depth_img, body_indx_img, center, search_radius_denom, num_std_dev, plot)
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
    self.blank_image = np.zeros(shape=(data['depth_frames'][0].shape[0], data['depth_frames'][0].shape[1])).astype(np.int16)
    h = size(depth_img,1);
    w = size(depth_img,2);
%         no body index found and no historical data
    if (any(body_indx_img) == false) && (self.std_val is None) and (self.med_val is None):
        return self.blank_image
    end
    if isnan(search_radius_denom)
        search_radius = w/10;  % also seems to be a good value
    else
        search_radius = w/search_radius_denom;
    if (any(body_indx_img) == false)
        std_val = self.std_val
        med_val = self.med_val
    else
        %   get small area around keypoint to identify relevant range of depth values for segmentation
%             masked_depth_image = depth_img.copy()
%             body_mask = body_indx_img != 0
%             masked_depth_image[~np.any(body_mask, 2)] = 0
%         #     prox_image = masked_image.copy()  # for debugging
% 
%             # get area to search for relevant pixels
%             search_mask = create_circular_mask(h, w, center=center, radius=search_radius)
%             masked_depth_image[~search_mask] = 0
% 
%             # get statistics for segmentation
%             # ignore values of 0
%             med_val = np.median(masked_depth_image[masked_depth_image!=0])
%             std_val = masked_depth_image[masked_depth_image!=0].std()
% 
%             self.std_val = std_val
%             self.med_val = med_val
    end

%   get area to search for relevant pixels
%         search_mask = create_circular_mask(h, w, center=center, radius=search_radius)
%         masked_image = depth_img.copy()
%         masked_image[~search_mask] = 0
% 
%         # use values within certain number of standard deviations from the mean
%         seg_mask = (masked_image >= med_val - std_val*num_std_dev) & (masked_image <= med_val + std_val*num_std_dev)
%         new_masked_image = masked_image.copy()
%         new_masked_image[~seg_mask] = 0
% 
%         if plot is True:
%             plt.imshow(new_masked_image)
% 
%         return 
end