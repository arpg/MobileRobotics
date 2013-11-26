 function [distance]=BFSws(A,source)


[n,m] = size(A);
[r,e]=size(source);
if(e>r)
source=source';
end


% colors: white, gray, black
white = 0; 
gray = 1; 
black = 2;

% initialize colors
color = zeros(n,m);
% initialize distances
distance = inf*ones(n,m);
% initialize branches
branch = zeros(n,m);



color(source(1),source(2)) = gray;
distance(source(1),source(2)) = 0;
branch(source(1),source(2)) = -1;
Q = source;

while ~isempty(Q)

 u = Q(:,1);
v=NearN(A,u(1),u(2));

[n,m]=size(v);
for ii=1:m

%  if (distance(v(1,ii),v(2,ii))==0)
%          distance(v(1,ii),v(2,ii)) = distance(u(1),u(2))+1;
%  end;
 
 if (color(v(1,ii),v(2,ii))==white);
         color(v(1,ii),v(2,ii)) = gray;
         distance(v(1,ii),v(2,ii)) = distance(u(1),u(2))+1;
         Q = [Q [v(1,ii),v(2,ii)]'];
 end

end
[k,l]=size(Q);

[k,l]=size(Q);

if (l==1)
    Q=[];
       color(u(1),u(2)) = black;

else if (l~=1);
Q = Q(:,2:length(Q));
   color(u(1),u(2)) = black;
    end
end
   
end









 end