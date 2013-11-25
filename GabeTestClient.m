%% usage: 
%% close all; clear all; GabeTestClient('lumaaaa','r',8888);
function GabeTestClient( sName, sColor, ProxyPortNum )

    [robot,laser] = SetupMyRobot( sName, sColor, ProxyPortNum );
 	
    h = plot( 0 );
    set( gcf, 'WindowKeyPressFcn', @(h_obj,evt) [] );
    set( gca, 'ZDir', 'reverse', 'YDir', 'reverse' );
    axis tight;
    xlabel( 'x' );
    ylabel( 'y' );
    
    w = 0;
    v = 0;

    xwr = [0;0;0];

    while 1
        t = tic();
        % 1) Sense
        z = GetMeasurementNew(laser, robot);

        % 2) Plan
        p = ScanToPoints( z, xwr, laser.fov, laser.nbeams );

        delete(h);
%        h = plot( [xwr(1) p(1,:) xwr(1)], [xwr(2) p(2,:) xwr(2)], 'r-' );
        h = plot( p(1,:), p(2,:), 'r.' );
        axis equal;

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
        set(gcf,'currentch',char(1)); % reset CurrentCharacter        

        u = [v;w];
        
        % 3) Act
%        u = [robot.m_LinearVelocity;robot.m_AngularVelocity];
%        u = u + [0.01*randn+0.05; 0.05*randn];
       ApplyCommandNew( robot,u );
%        fprintf('Updating at %5.2fhz\r\r\r\r\r\r\r\r\r', 1/toc(t) );
       drawnow();
       pause(0.1); % wait 
    end
end
 
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% add robot to world proxy
function [r,l] = SetupMyRobot( sName, sColor, ProxtPortNum )
    xvl = [0;0;0]; % laser position on the vehicle
    r = Robot();
    r.SetName( sName );

    mesh.verts = [ [-0.2 0]; [1 0]; [-0.2 0.3]; [-0.2 -0.3] ];
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

    % whoa, secrets
    % add robot to world proxy
    if AddRobotToWorldProxy(r, 'localhost', ProxtPortNum) ~= 1
        error('Fatal error! fail to add robot! exit.');
    end
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function bSuccess = AddRobotToWorldProxy(robot, sHost, nPort)
    % connect and send message to worldproxy
    ProxySocket = pnet( 'tcpconnect', sHost, nPort );
    msg.type  = 'ADD_ROBOT';
    msg.robot = robot;
    SendMsg(ProxySocket, msg );
    fprintf('Adding new robot to Server by proxy, waitting for world proxy to response...\n' );   
      
    bSuccess = 0;
    while bSuccess == 0        
        % wait for worldproxy to reply
        rmsg = RecvMsg(ProxySocket, 1 );
        if ~isempty(rmsg)         
            if strcmp(rmsg.type, 'WELCOME_ROBOT') == 1
               fprintf('Add new robot by proxy to Server Success.\n' );
               bSuccess = 1;  
               robot.m_Socket = ProxySocket;
            else
               rmsg.type
            end   
        else
%             fprintf('get no response from proxy\n');
        end   
    end
end



