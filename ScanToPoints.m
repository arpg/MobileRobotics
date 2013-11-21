
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function p = ScanToPoints( z, pose, fov, nbeams )
    th = -fov/2:fov/(nbeams-1):fov/2;
    T = Cart2T(pose);
    p =  T(1:2,:) * [ z'.*cos(th);...
                         z'.*sin(th);...
                         ones(1,numel(z))];
end