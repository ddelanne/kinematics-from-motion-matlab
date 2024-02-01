function [] = Display3DFlow(flow,color)
%Display3DFlow: Displays 3D flow using quiver3 function
x = flow(:,1);
y = flow(:,2); 
z = flow(:,3);
u = flow(:,4).*flow(:,7);
v = flow(:,5).*flow(:,7);
w = flow(:,6).*flow(:,7);
quiver3(x,y,z,u,v,w,0,color); 
end