shifted = cell(150,1);
mean_sh = mean(shoulder_kp);
% read the source point clouds and shift them using diff
for i=20:149,
    pc = eval(str2sym(sprintf('upper_source_%i',i)));
    shifted{i} = pc - mean_sh;
end


