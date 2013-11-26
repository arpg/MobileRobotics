function [FINALMAP]=PotentialFields(sink)


u=imread('Dilatedmap.png');
u=rgb2gray(u);
imshow(u);
size(u);
 utest=imresize(u,0.3);
% utest=u
imshow(utest);
utest= imcomplement(utest);
imshow(utest);


[m,n]=size(utest);

for i=1:m
   for j=1:n
    if (utest(i,j)==41);
        utest(i,j)=0;
    end
    
    if(utest(i,j)>100);
        utest(i,j)=100;
    end
        
end
end

%  imshow(utest)
%   surf(double(utest)/120,'EdgeColor', 'none')

 G = fspecial('gaussian',[5 5],5);
 utest = imfilter(utest,G,'same');
%  imshow(utest)
% surf(double(utest))

for i=1:m
   for j=1:n
    if (utest(i,j)==41);
        utest(i,j)=0;
    end
    
    if(utest(i,j)>60);
        utest(i,j)=60;
    end
        
end
end
utest=utest/60;
 imshow(utest);
% surf(double(utest),'EdgeColor', 'none')
 
 distance=BFSws(utest,sink);
 C = max(distance(:));
 FINALMAP=(double(utest)*C+distance)/C;

end