function points = convertReadingsToPoints(x, readings)
points = zeros(size(readings,1),2);
for i=1:size(readings,1)
    dtheta = (i-size(readings,1)/2) * pi / 180 ;
    points(i,1) = x(1) + readings(i,2) * cos(dtheta + x(3));
    points(i,2) = x(2) + readings(i,2) * sin(dtheta + x(3));    
end