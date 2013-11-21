function TestReadLaser
    % load a robot
    robot = SetupRobot('Bob','r');

    % load a map
    map = Map();
    map.Init( 'DilatedMap.png', 0, 0, 0.1 );
    h = map.Draw();

%    xwr = [100;40;0];    
    xwr = [ 38.6000; 22.7000; 0 ];

    nbeams = 91;
    fov = 90*(pi/180);
    maxrange = 40;
    h = plot( 0 );
    w = 0;
    v = 0;

    % so we can capture 'button presses'
	set( gcf, 'WindowKeyPressFcn', @(h_obj,evt)KeyPressCallback(h_obj,evt) );

    while 1
        tic;
        robot.SetPose( xwr );
        robot.Draw();

        % prep for mex call to generate laser scan:
        z = MyReadLaser( uint8(map.m_MapImage),...
                       map.m_dMetersPerPixel,...
                       map.m_dLeft,...% in meteres
                       map.m_dTop,...% in meteres
                       xwr, maxrange, fov, nbeams );

        p = ScanToPoints( z, xwr, fov, nbeams );

        delete(h);
        h = plot( [xwr(1) p(1,:) xwr(1)], [xwr(2) p(2,:) xwr(2)], 'r-' );

        k = double(get( gcf, 'CurrentCharacter' ));
        if k == 'a'
            w = w-0.01;
        elseif k == 's'
            v = v-0.05;
        elseif k == 'd'
            w = w+0.01;
        elseif k == 'w'
            v = v+0.05;
        elseif k == ' '
            v = 0;
            w = 0;
        end
        xwr = ProcessModel( xwr, [v;w] );
        set(gcf,'currentch',char(1)); % reset CurrentCharacter

        drawnow();
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function p = ScanToPoints( z, pose, fov, nbeams )
    th = -fov/2:fov/(nbeams-1):fov/2;
    T = Cart2T(pose);
    p =  T(1:2,:) * [ z'.*cos(th);...
                         z'.*sin(th);...
                         ones(1,numel(z))];
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function z = MyReadLaser( img, mpp, left, top, pose, dMaxRange, fov, n_beams )
    z = zeros(n_beams,1);
    ii = 1;
    % get pose, in pixel space
    pose = [ (pose(1:2)-[left;top]) / mpp; pose(3) ];
    dth = fov/(n_beams-1);
    for th = -fov/2:dth:fov/2
        th*180/pi
        z(ii) = RayCast( img, pose, th, dMaxRange, mpp );
        ii = ii+1;
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function z = RayCast( img, pose, theta, max_range, mpp )
    % dir is the iteration num for the laser measurement
    theta = pose(3) + theta;
    w = size(img,2);
    h = size(img,1);
    ray = [cos(theta); sin(theta)];
    z = max_range;
    for range = 1:0.5:max_range/mpp
        pt = range*ray + pose(1:2);
        if pt(1) <= 0 || pt(1) >= w || pt(2) <= 0 || pt(2) >= h
            continue
        end
        pr = round(pt);
%        fprintf( 'range = %f, pt = %f, %f, pr = %d, %d\n', range, pt(1), pt(2), pr(1), pr(2) );
        if img(pr(2)+1,pr(1)+1) < 250
%            fprintf( 'ray %f %f, range %f my intersection at %f, %f\n', ...
%                ray(1), ray(2), range,  pt(1), pt(2) );
            z = norm(pt-pose(1:2))*mpp;
            return;
        end
    end
end

function KeyPressCallback(src,eventdata)
%    src
%    eventdata
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function r = SetupRobot( sName, sColor )
    xvl = [0;0;0]; % laser position on the vehicle
    r = Robot();
    r.SetName( sName );

    mesh.verts = 2*[ [-0.2 0]; [1 0]; [-0.2 0.3]; [-0.2 -0.3] ];
    mesh.faces = [ 1 3 2; 1 2 4 ];
    mesh.colors = sColor;

    r.SetRenderGeometry( mesh );
    r.SetCollisionGeometry( mesh );

    l = Laser();
    l.SetName( 'MyLaser' );
    l.SetPose(  xvl );

    mesh.verts = [ [0.2 0.1]; [0.2 -0.1]; [0 -0.1]; [0 0.1] ];
    mesh.faces = [ 1 2 4; 4 2 3 ];
    mesh.colors = 'b';

    l.SetRenderGeometry( mesh );
    l.SetCollisionGeometry( mesh );

    r.AddSensor( l );

    r.m_LinearVelocity = 0.5;

end