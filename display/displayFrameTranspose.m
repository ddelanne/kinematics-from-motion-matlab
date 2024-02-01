function displayFrameTranspose(pc1, pc2, R, transpose, flags)
    hold on 
    if flags(1) == 1 
        pcshow(pointCloud(pc1,"Color","green"));
    end
    if flags(2) == 1 
        pcshow(pointCloud(pc2,"Color","blue"));
    end
    if flags(3) == 1 
        pcshow(pointCloud(pc1*R',"Color","red"));
    end
    if flags(4) == 1 
        pcshow(pointCloud(pc1*R'+transpose,"Color","red"));
    end
    legend("pc A","pc B","pcA to pcB","Color","White");
    hold off

end

