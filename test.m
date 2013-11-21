function TestReadLaser
    % load a robot
    [robot,laser] = SetupRobot('Bob','r');

    xwr = [0;0;0];
    h = plot( 0 );
    w = 0;
    v = 0;

    % so we can capture 'button presses'
%	set( gcf, 'WindowKeyPressFcn', @(h_obj,evt)KeyPressCallback(h_obj,evt) );
	set( gcf, 'WindowKeyPressFcn', @(h_obj,evt) [] );

    while 1
        tic;
%        robot.Draw();

        z = laser.GetMeasurement();

        p = ScanToPoints( z, xwr, laser.fov, laser.nbeams );

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
function KeyPressCallback(src,eventdata)
%    src
%    eventdata
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [r,l] = SetupRobot( sName, sColor )
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
    
    l.SetFoV( 180 );
    l.SetNumBeams( 181 );

    mesh.verts = [ [0.2 0.1]; [0.2 -0.1]; [0 -0.1]; [0 0.1] ];
    mesh.faces = [ 1 2 4; 4 2 3 ];
    mesh.colors = 'b';

    l.SetRenderGeometry( mesh );
    l.SetCollisionGeometry( mesh );

    r.AddSensor( l );

    r.m_LinearVelocity = 0.5;

    % whoa, secrets
    r.m_WorldProxy = WorldProxy( r, 'localhost', 9999 );
    l.m_WorldProxy = r.m_WorldProxy;
end