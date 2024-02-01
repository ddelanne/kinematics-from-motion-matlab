function [t,endEffectorCoordPosition,endEffectorPhi] = azureAndRobotDataSelector(robotData, azureData,T)
    %get robot data
    t = robotData(1,:)';
    endEffectorCoordPosition = robotData(2:4,:)';
    endEffectorPhi = robotData(5:8,:)';
    endEffectorPhiDot = robotData(9:12,:)';
    %get joint points
    [p1, p2, p3, p4, R04, T04, gemma, w4, Rw04,w4tot] = qarmForwardKinematics(endEffectorPhi, endEffectorPhiDot); 

    %shoulderJointPoint = p1*T;
   % armJointPoint=p2*T;
   % foreArmJointPoint=p3*T;
   % endEffectorPoint=p4*T;
    
end