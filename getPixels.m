function getPixels( RGB )
global pixels;

s = size(RGB);
%pixels = zeros(s(1),s(2));
pixels = zeros(s(2),s(1));
for i=1:s(1)
    for j=1:s(2)
        if RGB(i,j,1) > 50 || RGB(i,j,2) > 50 || RGB(i,j,3) > 50
            %pixels(i,j) = 0;
            pixels(j,i) = 0;
        else
            %pixels(i,j) = 1;
            pixels(j,i) = 1;
        end
    end
end
