clear;
%add path to all functions:
mfilename = matlab.desktop.editor.getActiveFilename;
folder = fileparts(which(mfilename));
addpath(genpath(folder));
%get data from robot and azure

% datafilePath = "C:\Users\main-portable\OneDrive - George Mason University - O365 Production\sharedProjectFolder\MotionEstimationOfRigidKinematicStructureViaPointClouds\robotData\dataset1";
% robotDatafilePath = datafilePath+"\dataset.mat";
d = "datasetB";
datafile = d;
robotDatafilePath = "datasets\"+d+"\SEGMENTED"+datafile+".mat";
load(robotDatafilePath)
numberOfFrames = size(forearmPointClouds,1)-1;


% om_out = zeros(numberOfFrames-1,3);
% rot_out = zeros(numberOfFrames-1,1);
% T_out = zeros(numberOfFrames-1,3);
% om1_out = zeros(numberOfFrames-1,3);
% rot1_out = zeros(numberOfFrames-1,1);
% stats_out = repmat(struct("rms", 0), numberOfFrames-1,1);

y = 1;
s = 1:y:numberOfFrames;

ICP_arm_r_out = zeros(size(s,2)-1,3);
ICP_arm_rms_out = zeros(size(s,2)-1,1);
ICP_forearm_r_out = zeros(size(s,2)-1,3);
ICP_forearm_rms_out = zeros(size(s,2)-1,1);

DT_arm_r_out = zeros(size(s,2)-1,3);
DT_arm_rms_out = zeros(size(s,2)-1,1);
DT_forearm_r_out = zeros(size(s,2)-1,3);
DT_forearm_rms_out = zeros(size(s,2)-1,1);

keypoint_arm_r_out = zeros(size(s,2)-1,3);
keypoint_forearm_r_out = zeros(size(s,2)-1,3);

ICP_arm_w_out = zeros(size(s,2)-1,3);
ICP_forearm_w_out = zeros(size(s,2)-1,3);
DT_arm_w_out = zeros(size(s,2)-1,3);
DT_forearm_w_out = zeros(size(s,2)-1,3);
keypoint_arm_w_out = zeros(size(s,2)-1,3);
keypoint_forearm_w_out = zeros(size(s,2)-1,3);

ICP_w_mag_out = zeros(size(s,2)-1,1);
DT_w_mag_out = zeros(size(s,2)-1,1);
keypoint_w_mag_out = zeros(size(s,2)-1,1);


ICP_arm_th_out = zeros(size(s,2)-1,1);
ICP_forearm_th_out = zeros(size(s,2)-1,1);
DT_arm_th_out = zeros(size(s,2)-1,1);
DT_forearm_th_out = zeros(size(s,2)-1,1);
keypoint_arm_th_out = zeros(size(s,2)-1,1);
keypoint_forearm_th_out = zeros(size(s,2)-1,1);

ICP_arm_th_C_out = zeros(size(s,2)-1,1);
ICP_forearm_th_C_out = zeros(size(s,2)-1,1);
DT_arm_th_C_out = zeros(size(s,2)-1,1);
DT_forearm_th_C_out = zeros(size(s,2)-1,1);

ICP_arm_translation = zeros(size(s,2)-1,3);
ICP_forearm_translation = zeros(size(s,2)-1,3);

BodyFrames_orientation = zeros(size(s,2)-1,3,3);

% hold on;
% pcshow(pointCloud(bodyKeypoints(:,:,1),"Color","Red"),"MarkerSize",50);

jointIndexList = [6, 7, 8];
% [shoulderKeypoints, elbowKeypoints, wristKeypoints, oppositeShoulderKeypoints, middleTorsoKeypoints] = calculateSyntheticKeypoints(bodyKeypoints);
[shoulderKeypoints, elbowKeypoints, wristKeypoints, oppositeShoulderKeypoints, middleTorsoKeypoints] = getArmKeypoints(bodyKeypoints);
t = (timestamp(1:size(keypoint_forearm_r_out,1))-timestamp(1))*1e-9;
% v = [shoulderKeypoints(1,:); elbowKeypoints(1,:); wristKeypoints(1,:); oppositeShoulderKeypoints(1,:); middleTorsoKeypoints(1,:)];
% pcshow(pointCloud(v,"Color","Blue"),"MarkerSize",50);

for i=1:size(s,2)-1
% for i=1:100-2
    display(sprintf('i = %d',i));
    % get keypoints
    shoulderKeypoint_a = shoulderKeypoints(i,:);
    elbowKeypoint_a = elbowKeypoints(i,:);
    wristKeypoint_a = wristKeypoints(i,:);
    oppositeShoulderKeypoint_a = oppositeShoulderKeypoints(i,:);
    middleTorsoKeypoint_a = middleTorsoKeypoints(i,:);
    shoulderKeypoint_b = shoulderKeypoints(i+1,:);
    elbowKeypoint_b = elbowKeypoints(i+1,:);
    wristKeypoint_b = wristKeypoints(i+1,:);
    oppositeShoulderKeypoint_b = oppositeShoulderKeypoints(i+1,:);
    middleTorsoKeypoint_b = middleTorsoKeypoints(i+1,:);

    % [shoulderKeypoint_a, elbowKeypoint_a, wristKeypoint_a] = getArmKeypoints(bodyKeypoints,s(i));
    % [shoulderKeypoint_b, elbowKeypoint_b, wristKeypoint_b] = getArmKeypoints(bodyKeypoints,s(i+1));
    % [oppositeShoulderKeypoint, middleTorsoKeypoint] = getAuxKeypoints(bodyKeypoints,s(i+1));
    % [shoulderKeypoint_c, elbowKeypoint_c, wristKeypoint_c] = getArmKeypoints(bodyKeypoints,s(i-1));


    B_a = (calculateBodyFrame(shoulderKeypoint_a,oppositeShoulderKeypoint_a, middleTorsoKeypoint_a));

    % addCue(B_a,shoulderKeypoint_a, 10, 2.5);
    % addCue(eye(3),shoulderKeypoint_a, 10, 5);
    % pcshow(pointCloud(armPointClouds{s(i)},"color","red"));

    B_b = (calculateBodyFrame(shoulderKeypoint_b,oppositeShoulderKeypoint_b, middleTorsoKeypoint_b));


    BodyFrames_orientation(i,:,:) = B_a;
    BodyFrames_orientation(i+1,:,:) = B_b;

    % convert to shoulder frame
    elbowKeypoint1_a = elbowKeypoint_a - shoulderKeypoint_a;
    elbowKeypoint1_b = elbowKeypoint_b - shoulderKeypoint_b;
    wristKeypoint1_a = wristKeypoint_a - shoulderKeypoint_a;
    wristKeypoint1_b = wristKeypoint_b - shoulderKeypoint_b;
    % get point clouds in shoulder frame
    arm1_a = armPointClouds{s(i)} - shoulderKeypoint_a;
    arm1_b = armPointClouds{s(i+1)} - shoulderKeypoint_b;
    forearm1_a = forearmPointClouds{s(i)} - shoulderKeypoint_a;
    forearm1_b = forearmPointClouds{s(i+1)} - shoulderKeypoint_b;
    wholeArm1_a = vertcat(arm1_a,forearm1_a);
    wholeArm1_b = vertcat(arm1_b,forearm1_b);

    % hold on;
    % addCue(B_a,shoulderKeypoint_a, 10, 2.5);
    % pcshow(pointCloud(shoulderKeypoint_a,"color","yellow"),"MarkerSize",50);
    % pcshow(pointCloud(armPointClouds{s(i)},"color","red"));



    % calculate keypoint rotation of shoulder
    keypoint_arm_angle_axis = vrrotvec(elbowKeypoint1_a,elbowKeypoint1_b);
    keypoint_arm_R = axang2rotm(keypoint_arm_angle_axis);
    keypoint_arm_rotation_vector = rad2deg(rotmat2vec3d(keypoint_arm_R)*B_b');

    elbowKeypoint2_a = elbowKeypoint1_a * keypoint_arm_R';
    elbowKeypoint2_b = elbowKeypoint1_b;
    wristKeypoint2_a = wristKeypoint1_a * keypoint_arm_R' - elbowKeypoint2_a;
    wristKeypoint2_b = wristKeypoint1_b - elbowKeypoint2_b;

    keypoint_forearm_angle_axis = vrrotvec(wristKeypoint2_a,wristKeypoint2_b);
    keypoint_forearm_R = axang2rotm(keypoint_forearm_angle_axis);
    keypoint_forearm_rotation_vector = rad2deg(rotmat2vec3d(keypoint_forearm_R)*B_b');


    % assign rotation theta
    ICP_arm_angle_axis = zeros(4,1);
    ICP_forearm_angle_axis = zeros(4,1);
    DT_arm_angle_axis = zeros(4,1);
    DT_forearm_angle_axis = zeros(4,1);
    ICP_arm_angle_axis_Complete = zeros(4,1);
    ICP_forearm_angle_axis_Complete = zeros(4,1);
    DT_arm_angle_axis_Complete = zeros(4,1);
    DT_forearm_angle_axis_Complete = zeros(4,1);

    % assign rotation vector
    ICP_arm_rotation_vector = zeros(3,1);
    ICP_forearm_rotation_vector = zeros(3,1);
    DT_arm_rotation_vector = zeros(3,1);
    DT_forearm_rotation_vector = zeros(3,1);

    % assign RMS
    ICP_arm_rms = 0;
    ICP_forearm_rms = 0;
    DT_arm_rms = 0;
    DT_forearm_rms = 0;




    % check if whole arm is rigid
    if false
    % if sum(abs(rad2deg(rotmat2vec3d(keypoint_forearm_R))))<1
        wholeArm2_a = wholeArm1_a*keypoint_arm_R';
        wholeArm2_b = wholeArm1_b;
        % DT
        % adjust shoulder rotation estimation with DT
        [DT_arm_degrees_vector, DT_arm_angle_axis, DT_arm_R_A, DT_arm_rms] = DT(wholeArm2_a,wholeArm2_b);
        DT_arm_R = DT_arm_R_A * keypoint_arm_R;
        DT_arm_rotation_vector = rad2deg(rotmat2vec3d(DT_arm_R)*B_b');
        DT_arm_angle_axis_Complete = rotm2axang(DT_arm_R);


        % adjust shoulder rotation estimation with ICP
        [ICP_arm_R_A, pc, ICP_arm_rms] = pcregistericp(pointCloud(wholeArm2_a),pointCloud(wholeArm2_b), "Metric", "pointToPlane");
        ICP_arm_translation(i,:) = ICP_arm_R_A.Translation;
        ICP_arm_angle_axis = rotm2axang(ICP_arm_R_A.R);
        ICP_arm_R = ICP_arm_R_A.R * keypoint_arm_R;
        ICP_arm_rotation_vector = rad2deg(rotmat2vec3d(ICP_arm_R)*B_b');
        ICP_arm_angle_axis_Complete = rotm2axang(ICP_arm_R);

    else
        arm2_a = arm1_a*keypoint_arm_R';
        arm2_b = arm1_b;
        % DT
        % adjust shoulder rotation estimation with DT
        [DT_arm_degrees_vector, DT_arm_angle_axis, DT_arm_R_A, DT_arm_rms]= DT(arm2_a,arm2_b);
        DT_arm_R = DT_arm_R_A * keypoint_arm_R;
        DT_arm_rotation_vector = rad2deg(rotmat2vec3d(DT_arm_R)*B_b');
        DT_arm_angle_axis_Complete = rotm2axang(DT_arm_R);
        % transfrom forearm stuff relative to shoulder rotation estimation
        forearm2_a = forearm1_a*DT_arm_R';
        forearm2_b = forearm1_b;
        elbowKeypoint2_a = elbowKeypoint1_a*DT_arm_R';
        elbowKeypoint2_b = elbowKeypoint1_b;

        % compute elbow estimation with adjusted shoulder rotation estimation
        DT_wristKeypoint_a = wristKeypoint1_a * DT_arm_R' + elbowKeypoint2_a;
        DT_wristKeypoint_b = wristKeypoint1_b + elbowKeypoint2_b;

        keypoint_forearm_angle_axis = vrrotvec(DT_wristKeypoint_a,DT_wristKeypoint_b);
        keypoint_forearm_R_A = axang2rotm(keypoint_forearm_angle_axis);

        % adjust elbow rotation estimation with DT
        [DT_forearm_degrees_vector, DT_forearm_angle_axis, DT_forearm_R_A, DT_forearm_rms] = DT(forearm2_a,forearm2_b);
        DT_forearm_R = DT_forearm_R_A * keypoint_forearm_R_A;
        DT_forearm_rotation_vector = rad2deg(rotmat2vec3d(DT_forearm_R)*B_b');
        DT_forearm_angle_axis_Complete = rotm2axang(DT_forearm_R);
        % ICP
        % adjust shoulder rotation estimation with ICP
        [ICP_arm_R_A, pc, ICP_arm_rms] = pcregistericp(pointCloud(arm2_a),pointCloud(arm2_b), "Metric", "pointToPlane");
        ICP_arm_translation(i,:) = ICP_arm_R_A.Translation;
        ICP_arm_angle_axis = rotm2axang(ICP_arm_R_A.R);
        ICP_arm_R = ICP_arm_R_A.R * keypoint_arm_R;
        ICP_arm_rotation_vector = rad2deg(rotmat2vec3d(ICP_arm_R)*B_b');
        ICP_arm_angle_axis_Complete = rotm2axang(ICP_arm_R);
        % transfrom forearm stuff relative to shoulder rotation estimation
        forearm2_a = forearm1_a*ICP_arm_R';
        forearm2_b = forearm1_b;
        elbowKeypoint2_a = elbowKeypoint1_a*ICP_arm_R';
        elbowKeypoint2_b = elbowKeypoint1_b;

        % compute elbow estimation with adjusted shoulder rotation estimation
        ICP_wristKeypoint_a = wristKeypoint1_a * ICP_arm_R' + elbowKeypoint2_a;
        ICP_wristKeypoint_b = wristKeypoint1_b + elbowKeypoint2_b;

        keypoint_forearm_angle_axis = vrrotvec(ICP_wristKeypoint_a,ICP_wristKeypoint_b);
        keypoint_forearm_R_A = axang2rotm(keypoint_forearm_angle_axis);

        % adjust elbow rotation estimation with ICP
        [ICP_forearm_R_A, pc, ICP_forearm_rms] = pcregistericp(pointCloud(forearm2_a),pointCloud(forearm2_b), "Metric", "pointToPlane");
        ICP_forearm_translation(i,:) = ICP_forearm_R_A.Translation;
        ICP_forearm_angle_axis = rotm2axang(ICP_forearm_R_A.R);
        ICP_forearm_R = ICP_forearm_R_A.R * keypoint_forearm_R_A;
        ICP_forearm_angle_axis_Complete = rotm2axang(ICP_forearm_R);
        ICP_forearm_rotation_vector = rad2deg(rotmat2vec3d(ICP_forearm_R)*B_b');

    end
    % assign rotation theta
    ICP_arm_th_out(i) = ICP_arm_angle_axis(4);
    ICP_forearm_th_out(i) = ICP_forearm_angle_axis(4);
    DT_arm_th_out(i) = DT_arm_angle_axis(4);
    DT_forearm_th_out(i) = DT_forearm_angle_axis(4);
    ICP_arm_th_C_out(i) = ICP_arm_angle_axis_Complete(4);
    ICP_forearm_th_C_out(i) = ICP_forearm_angle_axis_Complete(4);
    DT_arm_th_C_out(i) = DT_arm_angle_axis_Complete(4);
    DT_forearm_th_C_out(i) = DT_forearm_angle_axis_Complete(4);
    keypoint_arm_th_out(i) = keypoint_arm_angle_axis(4);
    keypoint_forearm_th_out(i) = keypoint_forearm_angle_axis(4);

    % assign rotation vector
    ICP_arm_r_out(i,:) = ICP_arm_rotation_vector;
    ICP_forearm_r_out(i,:) = ICP_forearm_rotation_vector;
    DT_arm_r_out(i,:) = DT_arm_rotation_vector;
    DT_forearm_r_out(i,:) = DT_forearm_rotation_vector;
    keypoint_arm_r_out(i,:) = keypoint_arm_rotation_vector;
    keypoint_forearm_r_out(i,:) = keypoint_forearm_rotation_vector;

    % assign RMS
    ICP_arm_rms_out(i) = ICP_arm_rms;
    ICP_forearm_rms_out(i) = ICP_forearm_rms;
    DT_arm_rms_out(i) = DT_arm_rms;
    DT_forearm_rms_out(i) = DT_forearm_rms;
end
save("temp.mat");
load("temp.mat");
%filter Rotation
t = (timestamp(1:size(keypoint_forearm_r_out,1))-timestamp(1))*1e-9;
[b,a] = butter(5,10/15);
ICP_arm_r_out(:,1) = filter(b,a,ICP_arm_r_out(:,1));
ICP_arm_r_out(:,2) = filter(b,a,ICP_arm_r_out(:,2));
ICP_arm_r_out(:,3) = filter(b,a,ICP_arm_r_out(:,3));

ICP_forearm_r_out(:,1) = filter(b,a,ICP_forearm_r_out(:,1));
ICP_forearm_r_out(:,2) = filter(b,a,ICP_forearm_r_out(:,2));
ICP_forearm_r_out(:,3) = filter(b,a,ICP_forearm_r_out(:,3));

DT_arm_r_out(:,1) = filter(b,a,DT_arm_r_out(:,1));
DT_arm_r_out(:,2) = filter(b,a,DT_arm_r_out(:,2));
DT_arm_r_out(:,3) = filter(b,a,DT_arm_r_out(:,3));

DT_forearm_r_out(:,1) = filter(b,a,DT_forearm_r_out(:,1));
DT_forearm_r_out(:,2) = filter(b,a,DT_forearm_r_out(:,2));
DT_forearm_r_out(:,3) = filter(b,a,DT_forearm_r_out(:,3));

keypoint_arm_r_out(:,1) = filter(b,a,keypoint_arm_r_out(:,1));
keypoint_arm_r_out(:,2) = filter(b,a,keypoint_arm_r_out(:,2));
keypoint_arm_r_out(:,3) = filter(b,a,keypoint_arm_r_out(:,3));

keypoint_forearm_r_out(:,1) = filter(b,a,keypoint_forearm_r_out(:,1));
keypoint_forearm_r_out(:,2) = filter(b,a,keypoint_forearm_r_out(:,2));
keypoint_forearm_r_out(:,3) = filter(b,a,keypoint_forearm_r_out(:,3));
% 
% for i = 1:size(s,2)-1
%     B = squeeze(BodyFrames_orientation(i,:,:));
%     ICP_arm_r_out = ICP_arm_r_out*B';
%     ICP_forearm_r_out = ICP_forearm_r_out*B';
%     DT_arm_r_out = DT_arm_r_out*B';
%     DT_forearm_r_out = DT_forearm_r_out*B';
%     keypoint_arm_r_out = keypoint_arm_r_out*B';
%     keypoint_forearm_r_out = keypoint_forearm_r_out*B';
% 
% end

ICP_forearm_r_acc = cumsum(ICP_forearm_r_out,1);
ICP_arm_r_acc = cumsum(ICP_arm_r_out,1);
DT_arm_r_acc = cumsum(DT_arm_r_out,1);
DT_forearm_r_acc = cumsum(DT_forearm_r_out,1);
keypoint_arm_r_acc = cumsum(keypoint_arm_r_out,1);
keypoint_forearm_r_acc = cumsum(keypoint_forearm_r_out,1);

% assign velocity
% ICP_arm_r_Delta = diff(ICP_arm_r_out);
% ICP_arm_r_Delta = [[0 0 0]; ICP_arm_r_Delta];
% ICP_forearm_r_Delta = diff(ICP_forearm_r_out);
% ICP_forearm_r_Delta = [[0 0 0]; ICP_forearm_r_Delta];
% 
% DT_arm_r_Delta = diff(DT_arm_r_out);
% DT_arm_r_Delta = [[0 0 0]; DT_arm_r_Delta];
% DT_forearm_r_Delta = diff(DT_forearm_r_out);
% DT_forearm_r_Delta = [[0 0 0]; DT_forearm_r_Delta];
% 
% keypoint_arm_r_Delta = diff(keypoint_arm_r_out);
% keypoint_arm_r_Delta = [[0 0 0]; keypoint_arm_r_Delta];
% keypoint_forearm_r_Delta = diff(keypoint_forearm_r_out);
% keypoint_forearm_r_Delta = [[0 0 0]; keypoint_forearm_r_Delta];

timeDelta = (diff(timestamp(1:numberOfFrames,1))*1e-9);
ICP_arm_w_out = ICP_arm_r_out./timeDelta;
ICP_forearm_w_out = ICP_forearm_r_out./timeDelta;
DT_arm_w_out = DT_arm_r_out./timeDelta;
DT_forearm_w_out = DT_forearm_r_out./timeDelta;
keypoint_arm_w_out = keypoint_arm_r_out./timeDelta;
keypoint_forearm_w_out = keypoint_forearm_r_out./timeDelta;

% filter velocity
% plot(t,ICP_forearm_w_out,'g'); hold on
[b,a] = butter(5,3/15); 
ICP_arm_w_out(:,1) = filter(b,a,ICP_arm_w_out(:,1));
ICP_arm_w_out(:,2) = filter(b,a,ICP_arm_w_out(:,2));
ICP_arm_w_out(:,3) = filter(b,a,ICP_arm_w_out(:,3));
ICP_forearm_w_out(:,1) = filter(b,a,ICP_forearm_w_out(:,1));
ICP_forearm_w_out(:,2) = filter(b,a,ICP_forearm_w_out(:,2));
ICP_forearm_w_out(:,3) = filter(b,a,ICP_forearm_w_out(:,3));

DT_arm_w_out(:,1) = filter(b,a,DT_arm_w_out(:,1));
DT_arm_w_out(:,2) = filter(b,a,DT_arm_w_out(:,2));
DT_arm_w_out(:,3) = filter(b,a,DT_arm_w_out(:,3));
DT_forearm_w_out(:,1) = filter(b,a,DT_forearm_w_out(:,1));
DT_forearm_w_out(:,2) = filter(b,a,DT_forearm_w_out(:,2));
DT_forearm_w_out(:,3) = filter(b,a,DT_forearm_w_out(:,3));

keypoint_arm_w_out(:,1) = filter(b,a,keypoint_arm_w_out(:,1));
keypoint_arm_w_out(:,2) = filter(b,a,keypoint_arm_w_out(:,2));
keypoint_arm_w_out(:,3) = filter(b,a,keypoint_arm_w_out(:,3));
keypoint_forearm_w_out(:,1) = filter(b,a,keypoint_forearm_w_out(:,1));
keypoint_forearm_w_out(:,2) = filter(b,a,keypoint_forearm_w_out(:,2));
keypoint_forearm_w_out(:,3) = filter(b,a,keypoint_forearm_w_out(:,3));

ICP_w_mag_out = vecnorm(ICP_arm_w_out+ICP_forearm_w_out,2,2);
DT_w_mag_out = vecnorm(DT_arm_w_out+DT_forearm_w_out,2,2);
keypoint_w_mag_out = vecnorm(keypoint_arm_w_out+keypoint_forearm_w_out,2,2);


maxTime = t(size(t,1));
datasetName = d;
filename = datasetName+'Mocap.csv';
datasetFolder = "datasets/"+d+"/";
mocapDatasetFolder = datasetFolder+filename;
[Mtimestamp,MarmRotation,MarmVelocity,MarmVelocityW,MforearmRotation,MforearmVelocity, MforearmVelocityW,Mmag] = extractMocap(mocapDatasetFolder,3.9,maxTime,BodyFrames_orientation,t);  


filename = "RESULTS"+datasetName+".mat";
save(filename);

% function [shoulderKeypoint, elbowKeypoint, wristKeypoint] = getArmKeypoints(bodyKeypoints,i)
%     jointIndexList = [6, 7, 8];
%     jointKeypoints = bodyKeypoints(:,:,i);
%     keypoints = jointKeypoints(jointIndexList,:);
%     shoulderKeypoint = keypoints(1,:);
%     elbowKeypoint = keypoints(2,:);
%     wristKeypoint = keypoints(3,:);
% end
function [shoulderKeypoints, elbowKeypoints, wristKeypoints, oppositeShoulderKeypoints, middleTorsoKeypoints] = getArmKeypoints(bodyKeypoints)
    jointIndexList = [6, 7, 8, 13, 3];
    keypoints = bodyKeypoints(jointIndexList,:, :);
    shoulderKeypoints = squeeze(keypoints(1,:,:))';
    elbowKeypoints = squeeze(keypoints(2,:,:))';
    wristKeypoints = squeeze(keypoints(3,:,:))';
    oppositeShoulderKeypoints = squeeze(keypoints(4,:,:))';
    middleTorsoKeypoints = squeeze(keypoints(5,:,:))';
end

function [oppositeShoulderKeypoint, middleTorsoKeypoint] = getAuxKeypoints(bodyKeypoints,i)
    jointIndexList = [2, 5];
    jointKeypoints = bodyKeypoints(:,:,i);
    keypoints = jointKeypoints(jointIndexList,:);
    oppositeShoulderKeypoint = keypoints(1,:);
    middleTorsoKeypoint = keypoints(2,:);
end

function B = calculateBodyFrame(shoulderKeypoint,oppositeShoulderKeypoint, middleTorsoKeypoint)
    N = cross(oppositeShoulderKeypoint-shoulderKeypoint,middleTorsoKeypoint-shoulderKeypoint);
    N = N/norm(N);
    S = oppositeShoulderKeypoint-shoulderKeypoint;
    S = S/norm(S);
    H = cross(N,S);
    B = [N',S',H'];
end

function addCue(rotationMatrix,centerPoint, intensity,thickness)
        tform = rigidtform3d(rotationMatrix, centerPoint');
        pts = pctransform(pointCloud([[intensity 0 0];[0 intensity 0];[0 0 intensity]]),tform);
        a = [pts.Location(1,:);centerPoint];
        b = [pts.Location(2,:);centerPoint];
        c = [pts.Location(3,:);centerPoint];
        plot3(a(:,1),a(:,2),a(:,3),"LineWidth",thickness,"Color","red");
        plot3(b(:,1),b(:,2),b(:,3),"LineWidth",thickness,"Color","green");
        plot3(c(:,1),c(:,2),c(:,3),"LineWidth",thickness,"Color","blue");
end

function [arm_rotation_vector, arm_tform, arm_rms, forearm_rotation_vector, forearm_tform, forearm_rms] = runICP(arm1_a, arm1_b, actual_tform, elbowKeypoint1_a, elbowKeypoint1_b, forearm1_a, forearm1_b)
    A = pointCloud(arm1_a,"color","green");
    B = pointCloud(arm1_b,"color","blue");
    m1 = min(A.Location);
    m2 = min(B.Location);
    shift = min(m1,m2)-2.5;
    a = A.Location - shift;
    b = B.Location - shift;
    pc1 = pointCloud(ptCloudInterpolateVoxelise(a),"color","green");    % shift is negative
    pc2 = pointCloud(ptCloudInterpolateVoxelise(b),"color","blue");   %shift is negative
    % pc1 = A;
    % pc2 = B;
    
    [arm_tform, pc, arm_rms] = pcregistericp(pc1, pc2, "Metric", "pointToPlane");
    % arm_tform.Translation = arm_tform.Translation + shift;
    
    arm_rotation_vector = rad2deg(rotmat2vec3d(actual_tform.R*arm_tform.R));
    
    % elbowKeypoint2_a =  pctransform(pointCloud(elbowKeypoint1_a),arm_tform);
    % elbowKeypoint2_b =  pointCloud(elbowKeypoint1_b);
    % forearm2_a = pctransform(pointCloud(forearm1_a),arm_tform);
    % forearm2_b = pointCloud(forearm1_b);
    % 
    % forearm3_a = forearm2_a.Location-elbowKeypoint2_a.Location;
    % forearm3_b = forearm2_b.Location-elbowKeypoint2_b.Location;   
    % 
    % A = pointCloud(forearm3_a,"color","green");
    % B = pointCloud(forearm3_b,"color","blue");
    % 
    % m1 = min(A.Location);
    % m2 = min(B.Location);
    % shift = min(m1,m2)-2.5;
    % a = A.Location - shift;
    % b = B.Location - shift;
    % pc1 = pointCloud(ptCloudInterpolateVoxelise(a),"color","green");    % shift is negative
    % pc2 = pointCloud(ptCloudInterpolateVoxelise(b),"color","blue");   %shift is negative
    % pc1 = A;
    % pc2 = B;
    % [forearm_tform, pc, forearm_rms] = pcregistericp(pc1,pc2,"Metric","pointToPlane");
    % forearm_rotation_vector = rad2deg(rotmat2vec3d(forearm_tform.R));
    forearm_tform = nan;
    forearm_rms = 0;
    forearm_rotation_vector = [0 0 0];
end

function [arm_rotation_vector, arm_angle_axis, arm_R, arm_rms, forearm_rotation_vector, forearm_R, forearm_rms] = runDT(arm2_a,arm2_b,keypoint_arm_R, elbowKeypoint1_a, elbowKeypoint1_b, forearm2_a, forearm2_b, keypoint_forearm_R)
    m1 = min(arm2_a);
    m2 = min(arm2_b);
    shift = min([m1;m2])-2.5;
    arm2_a_shift = arm2_a - shift;
    arm2_b_shift = arm2_b - shift;
    arm2_a_V = ptCloudInterpolateVoxelise(arm2_a_shift);   % shift is negative
    arm2_b_V = ptCloudInterpolateVoxelise(arm2_b_shift);   %shift is negative

    % A
    [arm_om, arm_th, arm_T, arm_om1, arm_th1, arm_stats] = RunAll(arm2_a_V,arm2_b_V, shift); % eval(str2sym(sprintf('s_source_%i',i))),eval(str2sym(sprintf('s_source_%i',i+1))));
    arm_rms = arm_stats.rms;
    arm_angle_axis = [arm_om1; arm_th1]';
    arm_R_A = axang2rotm(arm_angle_axis)';
    arm_R = arm_R_A * keypoint_arm_R;
    arm_rotation_vector = rad2deg(rotmat2vec3d(arm_R));

    % 
    elbowKeypoint2_a = elbowKeypoint1_a*arm_R_A';
    elbowKeypoint2_b = elbowKeypoint1_b;
    forearm3_a = forearm2_a*arm_R_A';
    forearm3_b = forearm2_b;
    % 
    forearm4_a = forearm3_a-elbowKeypoint2_a;
    forearm4_b = forearm3_b-elbowKeypoint2_b;   
    % 
    m1 = min(forearm4_a);
    m2 = min(forearm4_b);
    shift = min(m1,m2)-2.5;
    forearm4_a_shift = forearm4_a - shift;
    forearm4_b_shift = forearm4_b - shift;
    forearm4_a_V = ptCloudInterpolateVoxelise(forearm4_a_shift);   % shift is negative
    forearm4_b_V = ptCloudInterpolateVoxelise(forearm4_b_shift);   %shift is negative

    % 
    [forearm_om, forearm_th, forearm_T, forearm_om1, forearm_th1, forearm_stats] = RunAll(forearm4_a_V, forearm4_b_V, shift); % eval(str2sym(sprintf('s_source_%i',i))),eval(str2sym(sprintf('s_source_%i',i+1))));
    forearm_rms = forearm_stats.rms;
    forearm_angle_axis = [forearm_om1; forearm_th1]';
    forearm_R_A = axang2rotm(forearm_angle_axis)';
    forearm_R = forearm_R_A * keypoint_forearm_R;
    forearm_rotation_vector = rad2deg(rotmat2vec3d(forearm_R));
    % forearm_tform = nan;
    % forearm_rms = 0;
    % forearm_rotation_vector = [0 0 0];
end
function [degrees_vector,angle_axis,R,rms]= DT(a,b)
    m1 = min(a);
    m2 = min(b);
    shift = min([m1;m2])-2.5;
    a_shift = a - shift;
    b_shift = b - shift;
    a_V = ptCloudInterpolateVoxelise(a_shift);   % shift is negative
    b_V = ptCloudInterpolateVoxelise(b_shift);   %shift is negative

    % A
    [om, th, T, om1, th1, stats] = RunAll(a_V,b_V, shift); % eval(str2sym(sprintf('s_source_%i',i))),eval(str2sym(sprintf('s_source_%i',i+1))));
    rms = stats.rms;
    angle_axis = [om1; th1]';
    R = axang2rotm(angle_axis)';
    degrees_vector = rad2deg(rotmat2vec3d(R));
end

% DT symmetry
% function [arm_rotation_vector, arm_tform, arm_rms, forearm_rotation_vector, forearm_tform, forearm_rms] = runDT(arm1_a,arm1_b,arm1_c)
%     m1 = min(arm1_a);
%     m2 = min(arm1_b);
%     m3 = min(arm1_c);
%     shift = min([m1;m2;m3])-2.5;
%     arm1_a_shift = arm1_a - shift;
%     arm1_b_shift = arm1_b - shift;
%     arm1_c_shift = arm1_c - shift;
%     arm1_a_V = ptCloudInterpolateVoxelise(arm1_a_shift);   % shift is negative
%     arm1_b_V = ptCloudInterpolateVoxelise(arm1_b_shift);   %shift is negative
%     arm1_c_V = ptCloudInterpolateVoxelise(arm1_c_shift);   %shift is negative
% 
%     % A
%     [arm_om, arm_th, arm_T, arm_om1, arm_th1, arm_stats] = RunAll(arm1_a_V,arm1_b_V, shift); % eval(str2sym(sprintf('s_source_%i',i))),eval(str2sym(sprintf('s_source_%i',i+1))));
%     arm_rms = arm_stats.rms;
%     arm_R = axang2rotm([arm_om1; arm_th1]');
%     arm_tform_AB = invert(rigidtform3d(arm_R , [0 0 0]));
%     arm_rotation_vector_AB = rad2deg(rotmat2vec3d(arm_tform_AB.R));
% 
%     [arm_om, arm_th, arm_T, arm_om1, arm_th1, arm_stats] = RunAll(arm1_c_V,arm1_b_V, shift); % eval(str2sym(sprintf('s_source_%i',i))),eval(str2sym(sprintf('s_source_%i',i+1))));
%     arm_rms = arm_stats.rms;
%     arm_R = axang2rotm([arm_om1; arm_th1]');
%     arm_tform_CB = invert(rigidtform3d(arm_R , [0 0 0]));
%     arm_rotation_vector_AC = rad2deg(rotmat2vec3d(arm_tform_CB.R));
%     % 
%     % elbowKeypoint2_a =  pctransform(pointCloud(elbowKeypoint1_a),arm_tform);
%     % forearm2_a = pctransform(pointCloud(forearm1_a),arm_tform);
%     % forearm2_b = pointCloud(forearm1_b);
%     % 
%     % forearm3_a = forearm2_a.Location-elbowKeypoint2_a.Location;
%     % forearm3_b = forearm2_b.Location-elbowKeypoint2_a.Location;   
%     % 
%     % m1 = min(forearm3_a);
%     % m2 = min(forearm3_a);
%     % shift = min(m1,m2)-2.5;
%     % pc1 = forearm3_a - shift;
%     % pc2 = forearm3_b - shift;
%     % pc1a = ptCloudInterpolateVoxelise(pc1);   % shift is negative
%     % pc2a = ptCloudInterpolateVoxelise(pc2);   %shift is negative
% 
%     % 
%     % [forearm_om, forearm_th, forearm_T, forearm_om1, forearm_th1, forearm_stats] = RunAll(pc1a,pc2a); % eval(str2sym(sprintf('s_source_%i',i))),eval(str2sym(sprintf('s_source_%i',i+1))));
%     % forearm_rms = forearm_stats.rms;
%     % forearm_R = axang2rotm([forearm_om1; forearm_th1]');
%     % forearm_tform = invert(rigidtform3d(forearm_R , [0 0 0]));
%     % forearm_rotation_vector =  rad2deg(rotmat2vec3d(forearm_tform.R));
%     forearm_tform = nan;
%     forearm_rms = 0;
%     forearm_rotation_vector = [0 0 0];
% end

% ITERATIVE DT
% function [arm_rotation_vector, arm_tform, arm_rms, forearm_rotation_vector, forearm_tform, forearm_rms] = runDT(arm1_a,arm1_b,elbowKeypoint1_a,forearm1_a,forearm1_b)
%     m1 = min(arm1_a);
%     m2 = min(arm1_b);
%     shift = min(m1,m2)-2.5;
%     pc1 = arm1_a - shift;
%     pc2 = arm1_b - shift;
%     pc1a = ptCloudInterpolateVoxelise(pc1);   % shift is negative
%     pc2a = ptCloudInterpolateVoxelise(pc2);   %shift is negative
%     T_arm_R = eye(3);
%     arm_rotation_vector = [NaN,NaN,NaN];
%     arm_rotation_vector_prev = [NaN,NaN,NaN];
%     d = 1
%     while d==1
%         [arm_om, arm_th, arm_T, arm_om1, arm_th1, arm_stats] = RunAll(pc1a,pc2a, shift); % eval(str2sym(sprintf('s_source_%i',i))),eval(str2sym(sprintf('s_source_%i',i+1))));
%         arm_rms = arm_stats.rms;
%         arm_R = axang2rotm([arm_om1; arm_th1]');
%         arm_tform = invert(rigidtform3d(arm_R , [0 0 0]));
% 
%         % unshift
%         pc1a = pc1a + shift;
%         pc2a = pc2a + shift;
%         % transfrom A to B
%         pc1a = pctransform(pointCloud(pc1a), arm_tform).Location;
%         % reshift
%         m1 = min(pc1a);
%         m2 = min(pc2a);
%         shift = min(m1,m2)-2.5;
%         pc1a = pc1a - shift;
%         pc2a = pc2a - shift;
%         %track change in rotation
%         T_arm_R = T_arm_R*arm_R;
%         arm_tformA = rigidtform3d(T_arm_R , [0 0 0]);
%         if isnan(arm_rotation_vector_prev)
%             arm_rotation_vector_prev = rad2deg(rotmat2vec3d(arm_tformA.R));
%             continue;
%         end
%         arm_rotation_vector = rad2deg(rotmat2vec3d(arm_tformA.R));
%         diff = sum(abs(arm_rotation_vector - arm_rotation_vector_prev));
%         arm_rotation_vector_prev = arm_rotation_vector;
%         if diff<.1
%             d = 0;            
%         end
%     end
%     arm_tform = (rigidtform3d(T_arm_R , [0 0 0]));
%     arm_rotation_vector = rad2deg(rotmat2vec3d(arm_tform.R));
% 
%     elbowKeypoint2_a =  pctransform(pointCloud(elbowKeypoint1_a),arm_tform);
%     forearm2_a = pctransform(pointCloud(forearm1_a),arm_tform);
%     forearm2_b = pointCloud(forearm1_b);
% 
%     forearm3_a = forearm2_a.Location-elbowKeypoint2_a.Location;
%     forearm3_b = forearm2_b.Location-elbowKeypoint2_a.Location;   
% 
%     m1 = min(forearm3_a);
%     m2 = min(forearm3_a);
%     shift = min(m1,m2)-2.5;
%     pc1 = forearm3_a - shift;
%     pc2 = forearm3_b - shift;
%     pc1a = ptCloudInterpolateVoxelise(pc1);   % shift is negative
%     pc2a = ptCloudInterpolateVoxelise(pc2);   %shift is negative
% 
%     % 
%     % [forearm_om, forearm_th, forearm_T, forearm_om1, forearm_th1, forearm_stats] = RunAll(pc1a,pc2a); % eval(str2sym(sprintf('s_source_%i',i))),eval(str2sym(sprintf('s_source_%i',i+1))));
%     % forearm_rms = forearm_stats.rms;
%     % forearm_R = axang2rotm([forearm_om1; forearm_th1]');
%     % forearm_tform = invert(rigidtform3d(forearm_R , [0 0 0]));
%     % forearm_rotation_vector =  rad2deg(rotmat2vec3d(forearm_tform.R));
%     forearm_tform = nan;
%     forearm_rms = 0;
%     forearm_rotation_vector = [0 0 0];
% end


