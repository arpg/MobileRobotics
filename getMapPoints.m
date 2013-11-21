function getMapPoints()
global mapPoints;
global pixels;
plotCount = 0;
s = size(pixels);
mapPoints = zeros(60000,2);
for i=1:s(1)
    for j=1:s(2)        
        if pixels(i,j) ~= 0
            %means the cell is occupied
            plotCount = plotCount + 1;
            mapPoints(plotCount,1) = i;
            mapPoints(plotCount,2) = s(2) - j;
        end
    end
end
mapPoints = mapPoints(1:plotCount,:);
