function z= get_observations(x, rmax)
%tic()
z= get_visible_landmarks(x,rmax);
%z= compute_range_bearing(x,lm2);
%toc()
%
%

function z2 = get_visible_landmarks(x,rmax)
global mapPoints;
%% compute the observations
% Select set of landmarks that are visible within vehicle's semi-circular field-of-view
dx= mapPoints(:,1) - x(1);
dy= mapPoints(:,2) - x(2);
phi= x(3);

% incremental tests for bounding semi-circle
%      & (dx*cos(phi) + dy*sin(phi)) > 0 ...  % bounding line
ii= find(abs(dx) < rmax & abs(dy) < rmax & (dx.^2 + dy.^2) < rmax^2);           % bounding circle
% Note: the bounding box test is unnecessary but illustrates a possible speedup technique
% as it quickly eliminates distant points. Ordering the landmark set would make this operation
% O(logN) rather that O(N).
  
lm2= mapPoints(ii,:);

%size(mapPoints)
dx = lm2(:,1) - x(1);
dy = lm2(:,2) - x(2);


theta_r = [floor((atan2(dy,dx)-x(3))*180/pi + 0.5)+90, sqrt(dx.^2 + dy.^2)];

%ii = find(theta_r(:,1) <= 180 & theta_r(:,1) > 0);
theta_r = (theta_r(find(theta_r(:,1) <= 180 & theta_r(:,1) > 0),:));%theta_r(ii,:);

theta_r = sortrows(theta_r);
temp = transpose([0 transpose(theta_r(:,1))]);
temp2 = transpose([transpose(theta_r(:,1)) 100000]);
temp = temp - temp2;
temp = temp(1:size(temp)-1);
z2 = theta_r(find(temp<0),:);
%z2 = z1(:,2);

