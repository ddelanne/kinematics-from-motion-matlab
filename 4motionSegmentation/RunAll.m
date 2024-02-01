function [omega,theta,T,omega1,theta1,stats] = RunAll(pc1in, pc2in, shift)
% RunAll:  Estimate motion between two point clouds, assume pure rotation; 
%   try translation + rotation too
% m1 = min(pc1in);
% m2 = min(pc2in);
% shift = min(m1,m2)-2.5;
% pc1 = pc1in - shift;
% pc2 = pc2in - shift;
% pc1 = ptCloudInterpolateVoxelise(pc1);   % shift is negative
% pc2 = ptCloudInterpolateVoxelise(pc2);   %shift is negative
pc1 = pc1in;
pc2 = pc2in;


% pc1 = pc1in - shift;
% pc2 = pc2in - shift;

% pc1 = pc1in;
% pc2 = pc2in;

% sprintf('Calling Flow3D')
% min(pc1), min(pc2)
flow = Flow3D(pc1,pc2);
% figure;
% hold on;
% [M,I] = max([abs(flow(:,4).*flow(:,7)),abs(flow(:,5).*flow(:,7)),abs(flow(:,6).*flow(:,7))],[],2);
% flowA = flow(I==1,:);
% flowB = flow(I==2,:);
% flowC = flow(I==3,:);
% quiver3(flowA(:,1),flowA(:,2),flowA(:,3),flowA(:,4).*flowA(:,7),flowA(:,5).*flowA(:,7),flowA(:,6).*flowA(:,7),0,"red");
% quiver3(flowB(:,1),flowB(:,2),flowB(:,3),flowB(:,4).*flowB(:,7),flowB(:,5).*flowB(:,7),flowB(:,6).*flowB(:,7),0,"green");
% quiver3(flowC(:,1),flowC(:,2),flowC(:,3),flowC(:,4).*flowC(:,7),flowC(:,5).*flowC(:,7),flowC(:,6).*flowC(:,7),0,"blue");
% figure;
% hold on;
% xlabel("x");ylabel("y");zlabel("z");
% pcshow(pointCloud(pc1,"color","red"));
% pcshow(pointCloud(pc2,"color","green"));
% figure, quiver3(flow(:,1),flow(:,2),flow(:,3),flow(:,4).*flow(:,7),flow(:,5).*flow(:,7),flow(:,6).*flow(:,7),0); hold on;
% hold on; scatter3(pc1(:,1),pc1(:,2),pc1(:,3));
% scatter3(pc2(:,1),pc2(:,2),pc2(:,3),'g');
% pc1 = pc1 + shift;
% pc2 = pc2 + shift;
% scatter3(pc1(:,1),pc1(:,2),pc1(:,3));
% scatter3(pc2(:,1),pc2(:,2),pc2(:,3),'g');

% close all hidden;

N = size(flow,1);
t1 = flow(:,1:3)-flow(:,4:6).*(flow(:,7));
t2 = flow(:,1:3);
stats = struct("rms", sqrt((1/N)*sum(vecnorm(t2-t1,2,2).^2)));
flow(:,1:3) = flow(:,1:3) + shift;
% flow(:,1:3) = flow(:,1:3);
[omega, theta, T] = Motion3D(flow);
[omega1, theta1] = Rotation3D(flow);

% pc20 = Transform3D(pc2in, omega1, theta1, [0 0 0]');
% figure, quiver3(flow(:,1),flow(:,2),flow(:,3),flow(:,4).*flow(:,7),flow(:,5).*flow(:,7),flow(:,6).*flow(:,7),0); hold on;
% hold on; scatter3(pc1(:,1),pc1(:,2),pc1(:,3));
% scatter3(pc20(:,1),pc20(:,2),pc20(:,3),'red');



% pc20 = Transform3D(pc2in, omega1, theta1, [0 0 0]');
% figure, Display3DFlow(flow,'b'); hold on;
% display('Residual flow');
% pc21 = ptCloudVoxelise(-shift,pc20);
% [flow1,stats1] = Flow3D(pc1,pc21);
% flow1(:,1:3) = flow1(:,1:3)+shift; 
% [r_omega, r_theta, r_T] = Motion3D(flow1);
% [r_omega1,r_theta1] = Rotation3D(flow1);
% pc30 = Transform3D(pc20, r_omega1, r_theta1,[0 0 0]');
% pc31 = ptCloudVoxelise(-shift, pc30);
% [flow2,stats2] = Flow3D(pc1, pc31);
% flow2(:,1:3) = flow2(:,1:3)+shift; 
% [r2_omega, r2_theta, r2_T] = Motion3D(flow2);
% [r2_omega1,r2_theta1] = Rotation3D(flow2);
% pc40 = Transform3D(pc30, r2_omega1, r2_theta1,[0 0 0]');
% pc41 = ptCloudVoxelise(-shift, pc40);
% [flow3,stats3] = Flow3D(pc1, pc41);

% omega = r2_omega;
% theta = r2_theta;
% T = r2_T;
% omega1 = r2_omega1;
% theta1 = r2_theta1;
% stats = stats3;

%Display3DFlow(flow1,'r');
%figure, scatter3(pc1(:,1),pc1(:,2),pc1(:,3));
%hold on;
%scatter3(pc10(:,1),pc10(:,2),pc10(:,3),'r');

% At this point transforming the second point cloud and repeating the

% process makes sense to get rid of the residuals 



end
 
