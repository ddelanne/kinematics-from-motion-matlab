function segmentArm(depthFrames, bodyFrames)
    % datafilePath = "C:\Users\main-portable\OneDrive - George Mason University - O365 Production\sharedProjectFolder\MotionEstimationOfRigidKinematicStructureViaPointClouds\robotData\mkdata\";
    % get images
    files = dir([inputFolder+'extract\depth\*.png']);
    % get joints
    fid = fopen(inputFolder+'dataset1.json'); % Opening the file
    raw = fread(fid,inf); % Reading the contents
    str = char(raw'); % Transformation
    fclose(fid); % Closing the file
    jointKeypoints = jsondecode(str); % Using the jsondecode function to parse JSON from string
    % get calibration
    fid = fopen(inputFolder+'extract\intrinsic.json'); % Opening the file
    raw = fread(fid,inf); % Reading the contents
    str = char(raw'); % Transformation
    fclose(fid); % Closing the file
    calibration = jsondecode(str); % Using the jsondecode function to parse JSON from string
    intrinsic_matrix = calibration.intrinsic_matrix;
    
    P = intrinsic_matrix([7,8]);
    F = intrinsic_matrix([1,5]);

    numberOfFrames = size(files,1);
    depths = nan;
    keypointsOfInterestTags = ["SHOULDER_LEFT","ELBOW_LEFT","WRIST_LEFT"];
    keypointsOfInterestIndices = [6,7,8];
    boneList = [8,9];
    for i = 1:numberOfFrames
        filename = files(i).name;
        folder = files(i).folder;
        filepath = append(folder,"\",filename);
        depthImage = rescale(double(imread(filepath)),0,1);
        h = size(depthImage,1);
        w = size(depthImage,2);
        bodyImage = getBodyImage(depthImage, [75,h-150,500,w]);
        search_radius_denom = nan;
        num_std_dev = 3.5;
        plot = true;

        % armJointKeypoints = jointKeypoints.frames(1).bodies.joint_positions(keypointsOfInterestIndices,:);
        
        armJointKeypoints = jointKeypoints.frames(i).bodies.joint_positions(:,:);

        center = proj(armJointKeypoints(:,:),P,F);
        hold on
        imshow(bodyImage);
        viscircles(center, 10,'Color','b');
        hold off
        segment_body_around_point(depthImage, bodyImage, armJointKeypoints(1,:), search_radius_denom, num_std_dev, plot);

    end
end

function bin = getBodyImage(depthImage, filter)
    % filter: [xMin,xMax,yMin,yMax]
    b = depthImage<.25 & depthImage>.05;
    bin = zeros(size(b));
    l = b(filter(1):filter(2),filter(3):filter(4));
    bin(filter(1):filter(2),filter(3):filter(4)) = l;
end

function coords = proj(p,P,F)
    x1 = p(:,1)./p(:,3);
    y1 = p(:,2)./p(:,3);
    coords1 = [x1,y1];
    distCoords = coords1;
    % distCoords = Dmodel(coords1, [0,0], k);
    x = F(1)*distCoords(:,1)+P(1);
    y = F(2)*distCoords(:,2)+P(2);
    coords = [x,y];
end

function coords = Dmodel(p,c,k)
    xd = p(1);
    yd = p(2);
    xc = c(1);
    yc = c(2);

    r = sqrt((xd-xc)^2+(yd-yc)^2);
    f = k(1)*r^2+k(2)*r^4+k(3)*r^6+k(4)*r^8+k(5)*r^10+k(6)*r^12;
    xu = xd+(xd-xc)*f;
    yu = yd+(yd-yc)*f;
    coords = [xu,yu];
    % r = x^2+y^2;
    % f = 1 + k(1)*r + k(2)*r^2+k(5)*r^3;
    % coords = [0,0];
    % coords(1) = f*x+2*k(3)*f^2*x*y+k(4)*(r+2*x^2*f^2);
    % coords(2) = f*y+2*k(3)*f^2*x*y+k(4)*(r+2*y^2*f^2);

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
function segmentImage = segment_body_around_point(depth_img, body_indx_img, center, search_radius_denom, num_std_dev, plot)
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
    % if ~isnan(body_indx_img) && ~isnan(std_val) && ~isnan(med_val)
    %     return blank_image;
    % end
    % if isnan(search_radius_denom)
    %     search_radius = w/10;  % also seems to be a good value
    % else
    %     search_radius = w/search_radius_denom;
    % if (any(body_indx_img) == false)
    %     std_val = self.std_val
    %     med_val = self.med_val
    % else
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
    % end

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