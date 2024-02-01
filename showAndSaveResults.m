%rotation in degrees
d = "datasetB";
datasetName = d;
sourceString = "RESULTS"+datasetName; 
outputFolder = "./results/"+d+"/"+datasetName;

load(sourceString+".mat");

% t = (timestamp(1:size(keypoint_forearm_r_out,1))-timestamp(1))*1e-9;
% maxTime = t(size(t,1));
% datasetName = d;
% filename = datasetName+'Mocap.csv';
% datasetFolder = "datasets/"+d+"/";
% mocapDatasetFolder = datasetFolder+filename;
% [Mtimestamp,MarmRotation,MarmVelocity,MarmVelocityW,MforearmRotation,MforearmVelocity, MforearmVelocityW,MhandVelocityW] = extractMocap(mocapDatasetFolder,4.2,maxTime);  

close all;
colororder({'r', 'g', 'b'});
% lineStyleOrder({"-","--",":","-."});
% plot(t,ICP_arm_translation);
% xlabel("Seconds"); ylabel("millimeters");
% title("ICP\_arm\_translation");
% savefig("./results/"+d+"/1ICP_arm_translation"+"_"+sourceString+".fig");
% clf;
% plot(t,ICP_forearm_translation);
% xlabel("Seconds"); ylabel("millimeters");
% title("ICP\_forearm\_translation");
% savefig("./results/"+d+"/1ICP_forearm_translation"+"_"+sourceString+".fig");
% clf;
% 
% 
% plot(t,ICP_arm_th_out);
% xlabel("Seconds"); ylabel("Radians");
% title("ICP\_arm\_th\_out");
% savefig("./results/"+d+"/1ICP_arm_th_out"+"_"+sourceString+".fig");
% clf;
% plot(t,ICP_forearm_th_out);
% xlabel("Seconds"); ylabel("Radians");
% title("ICP\_forearm\_th\_out");
% savefig("./results/"+d+"/1ICP_forearm_th_out"+"_"+sourceString+".fig");
% clf;
% plot(t,DT_arm_th_out);
% xlabel("Seconds"); ylabel("Radians");
% title("DT\_arm\_th\_out");
% savefig("./results/"+d+"/1DT_arm_th_out"+"_"+sourceString+".fig");
% clf;
% plot(t,DT_forearm_th_out);
% xlabel("Seconds"); ylabel("Radians");
% title("DT\_forearm\_th\_out");
% savefig("./results/"+d+"/1DT_forearm_th_out"+"_"+sourceString+".fig");
% clf;
% plot(t,ICP_arm_th_C_out);
% xlabel("Seconds"); ylabel("Radians");
% title("ICP\_arm\_th\_C\_out");
% savefig("./results/"+d+"/1ICP_arm_th_C_out"+"_"+sourceString+".fig");
% clf;
% plot(t,ICP_forearm_th_C_out);
% xlabel("Seconds"); ylabel("Radians");
% title("ICP\_forearm\_\th\_C\_out");
% savefig("./results/"+d+"/1ICP_forearm_th_C_out"+"_"+sourceString+".fig");
% clf;
% plot(t,DT_arm_th_C_out);
% xlabel("Seconds"); ylabel("Radians");
% title("DT\_arm\_th\_C\_out");
% savefig("./results/"+d+"/1DT_arm_th_C_out"+"_"+sourceString+".fig");
% clf;
% plot(t,DT_forearm_th_C_out);
% xlabel("Seconds"); ylabel("Radians");
% title("DT\_forearm\_th\_C\_out");
% savefig("./results/"+d+"/1DT_forearm_th_C_out"+"_"+sourceString+".fig");
% clf;
% plot(t,keypoint_arm_th_out);
% xlabel("Seconds"); ylabel("Radians");
% title("keypoint\_arm\_th\_out");
% savefig("./results/"+d+"/1keypoint_arm_th_out"+"_"+sourceString+".fig");
% clf;
% plot(t,keypoint_forearm_th_out);
% xlabel("Seconds"); ylabel("Radians");
% title("keypoint\_forearm\_th_out");
% savefig("./results/"+d+"/1keypoint_forearm_th_out"+"_"+sourceString+".fig");
% clf;

% ARM ROTATION 
tl = tiledlayout(4,1);
title(tl,'Arm Rotation')
xlabel(tl,'Time(Seconds)')
ylabel(tl,'Degrees')

% MOCAP
nexttile
hold on;
plot(Mtimestamp,MarmRotation(:,1),'--');
plot(Mtimestamp,MarmRotation(:,2),'-');
plot(Mtimestamp,MarmRotation(:,3),':','LineWidth',1);
legend("x","y","z",'Location','northoutside', "NumColumns", 3);
ylabel("Mocap");
% KEYPOINTS
nexttile;
hold on;
plot(t,keypoint_arm_r_acc(:,1),'--');
plot(t,keypoint_arm_r_acc(:,2),'-');
plot(t,keypoint_arm_r_acc(:,3),':','LineWidth',1);
ylabel("Keypoint");
% ICP
nexttile;
hold on;
plot(t,ICP_arm_r_acc(:,1),'--');
plot(t,ICP_arm_r_acc(:,2),'-');
plot(t,ICP_arm_r_acc(:,3),':','LineWidth',1)
ylabel("ICP");
% DT
nexttile;
hold on;
plot(t,DT_arm_r_acc(:,1),'--');
plot(t,DT_arm_r_acc(:,2),'-');
plot(t,DT_arm_r_acc(:,3),':','LineWidth',1)
ylabel("Distance Transform");
savefig("./results/"+d+"/rotation_ARM"+"_"+sourceString+".fig");
clf;

% FOREARM ROTATION
tl = tiledlayout(4,1);
title(tl,'Forearm Rotation')
xlabel(tl,'Time(Seconds)')
ylabel(tl,'Degrees')
% MOCAP
nexttile;
hold on;
plot(Mtimestamp,MforearmRotation(:,1),'--');
plot(Mtimestamp,MforearmRotation(:,2),'-');
plot(Mtimestamp,MforearmRotation(:,3),':','LineWidth',1);
legend("x","y","z",'Location','northoutside', "NumColumns", 3);
ylabel("Mocap");
% KEYPOINT
nexttile;
hold on;
plot(t,keypoint_forearm_r_acc(:,1),'--');
plot(t,keypoint_forearm_r_acc(:,2),'-');
plot(t,keypoint_forearm_r_acc(:,3),':','LineWidth',1);
ylabel("Keypoint");
% ICP
nexttile;
hold on;
plot(t,ICP_forearm_r_acc(:,1),'--');
plot(t,ICP_forearm_r_acc(:,2),'-');
plot(t,ICP_forearm_r_acc(:,3),':','LineWidth',1);
ylabel("ICP");
% DT
nexttile;
hold on;
plot(t,DT_forearm_r_acc(:,1),'--');
plot(t,DT_forearm_r_acc(:,2),'-');
plot(t,DT_forearm_r_acc(:,3),':','LineWidth',1);
ylabel("Distance Transform");

savefig("./results/"+d+"/rotation_FOREARM"+"_"+sourceString+".fig");
clf;

% ARM VELOCITY 
tl = tiledlayout(4,1);
title(tl,'Arm Velocity');
xlabel(tl,'Time(Seconds)');
ylabel(tl,'Degrees per Seconds');
% MOCAP
nexttile;
hold on;
plot(Mtimestamp,MarmVelocity(:,1),'--');
plot(Mtimestamp,MarmVelocity(:,2),'-');
plot(Mtimestamp,MarmVelocity(:,3),':','LineWidth',1);
legend("x","y","z",'Location','northoutside', "NumColumns", 3);
ylabel("Mocap");
% KEYPOINT
nexttile;
hold on;
plot(t,keypoint_arm_w_out(:,1),'--');
plot(t,keypoint_arm_w_out(:,2),'-');
plot(t,keypoint_arm_w_out(:,3),':','LineWidth',1);
ylabel("Keypoint");
%ICP
nexttile;
hold on;
plot(t,ICP_arm_w_out(:,1),'--');
plot(t,ICP_arm_w_out(:,2),'-');
plot(t,ICP_arm_w_out(:,3),':','LineWidth',1);
ylabel("rotational Velocity(degrees/second)");
ylabel("ICP");
%DT
nexttile;
hold on;
plot(t,DT_arm_w_out(:,1),'--');
plot(t,DT_arm_w_out(:,2),'-');
plot(t,DT_arm_w_out(:,3),':','LineWidth',1);
ylabel("Distance Transform");
savefig("./results/"+d+"/velocity_ARM"+"_"+sourceString+".fig");
clf;

% FOREARM VELOCITY 
tl = tiledlayout(4,1);
title(tl,'Forearm Velocity')
xlabel(tl,'Time(Seconds)')
ylabel(tl,'Degrees per Seconds')
% MOCAP
nexttile;
hold on;
plot(Mtimestamp,MforearmVelocity(:,1),'--');
plot(Mtimestamp,MforearmVelocity(:,2),'-');
plot(Mtimestamp,MforearmVelocity(:,3),':','LineWidth',1);
legend("x","y","z",'Location','northoutside', "NumColumns", 3);
ylabel("Mocap");
% KEYPOINT
nexttile;
hold on;
plot(t,keypoint_forearm_w_out(:,1),'--');
plot(t,keypoint_forearm_w_out(:,2),'-');
plot(t,keypoint_forearm_w_out(:,3),':','LineWidth',1);
ylabel("Keypoint");
% ICP
nexttile;
hold on;
plot(t,ICP_forearm_w_out(:,1),'--');
plot(t,ICP_forearm_w_out(:,2),'-');
plot(t,ICP_forearm_w_out(:,3),':','LineWidth',1);
ylabel("ICP");
% DT
nexttile;
hold on;
plot(t,DT_forearm_w_out(:,1),'--');
plot(t,DT_forearm_w_out(:,2),'-');
plot(t,DT_forearm_w_out(:,3),':','LineWidth',1);
ylabel("Distance Transform");
savefig("./results/"+d+"/velocity_FOREARM"+"_"+sourceString+".fig");
clf;

% %RMS

tl = tiledlayout(4,1);
colororder({'black'});
title(tl,'Root Mean Square Score')
nexttile
plot(t, ICP_arm_rms_out); 
xlabel("Time(Seconds)"); ylabel("RMS");
title("ICP Arm");
nexttile
plot(t, DT_arm_rms_out); 
xlabel("Time(Seconds)"); ylabel("RMS");
title("DT Arm");
nexttile
plot(t, ICP_forearm_rms_out);
xlabel("Time(Seconds)"); ylabel("RMS");
title("ICP Forearm");
nexttile
plot(t, DT_forearm_rms_out);
xlabel("Time(Seconds)"); ylabel("RMS");
title("DT Forearm");

savefig("./results/"+d+"/RMS"+"_"+sourceString+".fig");
clf;

% velocity
plot(Mtimestamp, Mmag,'green'); hold on
plot(t, ICP_w_mag_out,'blue');
plot(t, DT_w_mag_out,'red','LineWidth',1); 
plot(t, keypoint_w_mag_out,'black'); 
legend("mocap","ICP","DT","keypoint");
xlabel("Seconds"); ylabel("Velocity Magnitude(in Degrees/second)");
title("Angular Velocity");
savefig("./results/"+d+"/velocity_Magnitude"+"_"+sourceString+".fig");
clf;
% RMS box plot
boxplot([ICP_arm_rms_out, DT_arm_rms_out, ICP_forearm_rms_out, DT_forearm_rms_out],'Labels',{'ICP Arm','DT Arm','ICP Forearm','DT Forearm'});hold on;
ylabel("RMS");
title("Box Plot of Root Mean Square Score");
savefig("./results/"+d+"/RMS_Box_Plot"+"_"+sourceString+".fig");
clf;

% % ARM X axis Rotation direct comparison
% plot(t, ICP_arm_r_out(:,1),'blue'); hold on;
% plot(t, DT_arm_r_out(:,1),'red'); 
% plot(t, keypoint_arm_r_out(:,1),'black'); 
% legend("ICP","DT","keypoint");
% xlabel("Seconds"); ylabel("Rotation(in Degrees)");
% title("ARM X axis Rotation Direct Comparison");
% savefig("./results/"+d+"/directComparison_ARM_Xrotation"+"_"+sourceString+".fig");
% clf;
% % ARM Y axis Rotation direct comparison
% plot(t, ICP_arm_r_out(:,2),'blue'); hold on;
% plot(t, DT_arm_r_out(:,2),'red'); 
% plot(t, keypoint_arm_r_out(:,2),'black'); 
% legend("ICP","DT","keypoint");
% xlabel("Seconds"); ylabel("Rotation(in Degrees)");
% title("ARM Y axis Rotation Direct Comparison");
% savefig("./results/"+d+"/directComparison_ARM_Yrotation"+"_"+sourceString+".fig");
% clf;
% % ARM Z axis Rotation direct comparison
% plot(t, ICP_arm_r_out(:,3),'blue'); hold on;
% plot(t, DT_arm_r_out(:,3),'red'); 
% plot(t, keypoint_arm_r_out(:,3),'black'); 
% legend("ICP","DT","keypoint");
% xlabel("Seconds"); ylabel("Rotation(in Degrees)");
% title("ARM Z axis Rotation Direct Comparison");
% savefig("./results/"+d+"/directComparison_ARM_Zrotation"+"_"+sourceString+".fig");
% clf;
% % FOREARM X axis Rotation direct comparison
% plot(t, ICP_forearm_r_out(:,1),'blue'); hold on;
% plot(t, DT_forearm_r_out(:,1),'red'); 
% plot(t, keypoint_forearm_r_out(:,1),'black'); 
% legend("ICP","DT","keypoint");
% xlabel("Seconds"); ylabel("Rotation(in Degrees)");
% title("FOREARM X axis Rotation Direct Comparison");
% savefig("./results/"+d+"/directComparison_FOREARM_Xrotation"+"_"+sourceString+".fig");
% clf;
% % FOREARM Y axis Rotation direct comparison
% plot(t, ICP_forearm_r_out(:,2),'blue'); hold on;
% plot(t, DT_forearm_r_out(:,2),'red'); 
% plot(t, keypoint_forearm_r_out(:,2),'black'); 
% legend("ICP","DT","keypoint");
% xlabel("Seconds"); ylabel("Rotation(in Degrees)");
% title("FOREARM Y axis Rotation Direct Comparison");
% savefig("./results/"+d+"/directComparison_FOREARM_Yrotation"+"_"+sourceString+".fig");
% clf;
% % FOREARM Z axis Rotation direct comparison
% plot(t, ICP_forearm_r_out(:,3),'blue'); hold on;
% plot(t, DT_forearm_r_out(:,3),'red'); 
% plot(t, keypoint_forearm_r_out(:,3),'black'); 
% legend("ICP","DT","keypoint");
% xlabel("Seconds"); ylabel("Rotation(in Degrees)");
% title("FOREARM Z axis Rotation Direct Comparison");
% savefig("./results/"+d+"/directComparison_FOREARM_Zrotation"+"_"+sourceString+".fig");
% clf;
% 
% % ARM X axis Velocity direct comparison
% plot(t, ICP_arm_w_out(:,1),'blue'); hold on;
% plot(t, DT_arm_w_out(:,1),'red'); 
% plot(t, keypoint_arm_w_out(:,1),'black'); 
% legend("ICP","DT","keypoint");
% xlabel("Seconds"); ylabel("Velocity(in Degrees/second)");
% title("ARM X axis Velocity Direct Comparison");
% savefig("./results/"+d+"/directComparison_ARM_Xvelocity"+"_"+sourceString+".fig");
% clf;
% % ARM Y axis Velocity direct comparison
% plot(t, ICP_arm_w_out(:,2),'blue'); hold on;
% plot(t, DT_arm_w_out(:,2),'red'); 
% plot(t, keypoint_arm_w_out(:,2),'black'); 
% legend("ICP","DT","keypoint");
% xlabel("Seconds"); ylabel("Velocity(in Degrees/second)");
% title("ARM Y axis Velocity Direct Comparison");
% savefig("./results/"+d+"/directComparison_ARM_Yvelocity"+"_"+sourceString+".fig");
% clf;
% % ARM Z axis Velocity direct comparison
% plot(t, ICP_arm_w_out(:,3),'blue'); hold on;
% plot(t, DT_arm_w_out(:,3),'red'); 
% plot(t, keypoint_arm_w_out(:,3),'black'); 
% legend("ICP","DT","keypoint");
% xlabel("Seconds"); ylabel("Velocity(in Degrees/second)");
% title("ARM Z axis Velocity Direct Comparison");
% savefig("./results/"+d+"/directComparison_ARM_Zvelocity"+"_"+sourceString+".fig");
% clf;
% % FOREARM X axis Velocity direct comparison
% plot(t, ICP_forearm_w_out(:,1),'blue'); hold on;
% plot(t, DT_forearm_w_out(:,1),'red'); 
% plot(t, keypoint_forearm_w_out(:,1),'black'); 
% legend("ICP","DT","keypoint");
% xlabel("Seconds"); ylabel("Velocity(in Degrees/second)");
% title("FOREARM X axis Velocity Direct Comparison");
% savefig("./results/"+d+"/directComparison_FOREARM_Xvelocity"+"_"+sourceString+".fig");
% clf;
% % FOREARM Y axis Velocity direct comparison
% plot(t, ICP_forearm_w_out(:,2),'blue'); hold on;
% plot(t, DT_forearm_w_out(:,2),'red'); 
% plot(t, keypoint_forearm_w_out(:,2),'black'); 
% legend("ICP","DT","keypoint");
% xlabel("Seconds"); ylabel("Velocity(in Degrees/second)");
% title("FOREARM Y axis Velocity Direct Comparison");
% savefig("./results/"+d+"/directComparison_FOREARM_Yvelocity"+"_"+sourceString+".fig");
% clf;
% % FOREARM Z axis Velocity direct comparison
% plot(t, ICP_forearm_w_out(:,3),'blue'); hold on;
% plot(t, DT_forearm_w_out(:,3),'red'); 
% plot(t, keypoint_forearm_w_out(:,3),'black'); 
% legend("ICP","DT","keypoint");
% xlabel("Seconds"); ylabel("Velocity(in Degrees/second)");
% title("FOREARM Z axis Velocity Direct Comparison");
% savefig("./results/"+d+"/directComparison_FOREARM_Zvelocity"+"_"+sourceString+".fig");
% clf;
close all;