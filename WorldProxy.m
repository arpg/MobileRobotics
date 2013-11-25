classdef WorldProxy
    properties
        m_sProxyHost;
        m_nProxyPort;
        m_ProxySocket;

        m_sServerHost;
        m_nServerPort;
        m_ServerSocket;
        
        m_WorldState;
        m_Map;
        m_sMapname;
        m_Robot;
        m_RobotWaitTime; 
        
        m_nTimeStep;
    end
    
    methods
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % usage:   close all; clear all;w=WorldProxy(8888,'localhost');w.Run();
        function obj = WorldProxy(nPortNum, sServerHost)
            obj.m_sProxyHost = '0.0.0.0';
            obj.m_nProxyPort = nPortNum;
            obj.m_Robot = [];
            obj.m_ProxySocket = pnet( 'tcpsocket', obj.m_nProxyPort );  
            obj.m_sServerHost = sServerHost;
            obj.m_nServerPort = 9999;
        end
        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % input server's sHost and nPort
        function  Run( obj )
            %% connect to server
            fprintf('Connecting to %s:%d... ', obj.m_sServerHost ,obj.m_nServerPort );
            obj.m_ServerSocket = pnet( 'tcpconnect', obj.m_sServerHost ,obj.m_nServerPort );

            msg.type = 'CLIENT_CONNECT';
            msg.timestep = 0;
            SendMsg( obj.m_ServerSocket, msg );
            fprintf('done.\nFetching map and world state...' );
            
            % get the world state (and the map too, if we don't have it)
            bSuccess = 0;
            while bSuccess == 0
                rmsg = RecvMsg( obj.m_ServerSocket, 5);
                if isempty(rmsg)
                    fprintf( 'No response from the server\n' );
                else
                    if strcmp(rmsg.type, 'WELCOME_CLIENT') == 0
                        fprintf( 'Connect to server success, but get wrong response..\n' );
                    else
                        obj.m_nTimeStep = msg.timestep;
                        obj.m_Map = rmsg.map;
                        obj.m_WorldState = rmsg.worldstate;% get the compete world state
                        fprintf('Connect to Server success.\n');
                        bSuccess = 1;
                    end
                end
            
            end

            
            %% sync with server
            while(1)     
                obj = CheckIfAddNewRobot(obj);
                                
                obj = CheckRequestFromRobot(obj);
                
                SyncWorldStateWithServer(obj);
                
                obj = CheckIfDeleteRobot(obj);   
                
                obj.m_nTimeStep = obj.m_nTimeStep+1;
                
                pause(0.07) % for it to be 20 fps
            end
        end
        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % delete robot if we cannot hear from robot for seconds
        function newobj = CheckIfDeleteRobot(obj)
            if size(obj.m_Robot,2) > 0 
                fprintf('Does not hear from robot for %f\n',obj.m_RobotWaitTime);
                if obj.m_RobotWaitTime >2
                    obj.m_Robot = []; 
                    fprintf('Delete lost robot. Total wait time is %f \n', obj.m_RobotWaitTime);
                    obj.m_RobotWaitTime = 0;
                end
            end
            
            newobj = obj;
        end
        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % the following check if there are any request from robot
        function newobj = CheckRequestFromRobot(obj)
            % check if we have robot
            if size(obj.m_Robot,2) ~= 0
                % recv msg from robot
                time = tic();
                msg = RecvMsg( obj.m_Robot.m_Socket, 2 );
                if ~isempty(msg)
                    obj.m_RobotWaitTime = 0;
                    
                    if strcmp(msg.type, 'REQ_APPLYCOMMAND') == 1
%                        fprintf('detect robot request for apply command.\n');
                       rmsg = ApplyCommand(obj, msg);
                       obj.m_Robot.m_dPose = rmsg.dPose;

                       SendMsg( obj.m_Robot.m_Socket, rmsg ); 
                       fprintf('Send pose after apply command to Robot Success.\n');
                       
                    elseif strcmp(msg.type, 'REQ_MEASUREMENT') == 1
%                        fprintf('detect robot request for new measurement command.\n');
                       rmsg = GenMeasurement(obj, msg);
                                                   
                       SendMsg( obj.m_Robot.m_Socket, rmsg );
                       fprintf('Send Measurement to Robot Success.\n');
                    end
                    
                else
                    obj.m_RobotWaitTime = obj.m_RobotWaitTime + toc(time);
                end
            end
            
            newobj = obj;
        end
        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % check if robot request for apply command
        function rmsg = ApplyCommand(obj, msg)
            % check if we have robot
             command = msg.vel;
             pose = ProcessModel( obj.m_Robot.m_dPose, command );
             while( ObjectCollides(obj.m_Map, obj.m_Robot, pose) )
                  pose = ProcessModel( obj.m_Robot.m_dPose, command );
                  command = command + [0.1*randn; 0.5*randn];
             end
             
             rmsg.type = 'REP_APPLYCOMMAND';
             rmsg.dPose = pose;
             rmsg.LinearVelocity = command(1);
             rmsg.AngularVelocity = command(2);  
        end
        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % check if robot request for new measurement
        function rmsg = GenMeasurement(obj, msg)
            dMaxRange = msg.dMaxRange; 
            fov = msg.fov; 
            nbeams = msg.nbeams;
                       
            % set reply msg
            rmsg.type = 'REP_MEASUREMENT';
            xwr = obj.m_Robot.m_dPose;
            rmsg.z  = ReadLaser( uint8(obj.m_Map.m_MapImage),...
                                 obj.m_Map.m_dMetersPerPixel,...
                                 obj.m_Map.m_dLeft,...% in meteres
                                 obj.m_Map.m_dTop,...% in meteres
                                 xwr, dMaxRange, fov, nbeams );
        end
        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function msg = RecvLatestMsg(obj, Socket, CurTimeStep, RecurseNum)
            RecurseNum = RecurseNum +1;
            msg = RecvMsg( Socket, 0 );% use socket
            % set max recurse num 
            if(RecurseNum>=495)
               fprintf('type is %s, timestep is %d, desire timestep %d\n',msg.type, msg.timestep,CurTimeStep);
               return 
            end
%             fprintf('in %d loop.\n',RecurseNum);
            if ~isempty(msg)

               if(msg.timestep ~= CurTimeStep)
%                    fprintf('read msg again\n');
                   obj.RecvLatestMsg(Socket, CurTimeStep, RecurseNum);
               else
                   fprintf('get latest msg success.\n');
                   fprintf('type is %s, timestep is %d, desire timestep %d\n',msg.type, msg.timestep,CurTimeStep);
               end
            end
        end

        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % always send info to server
        function  SyncWorldStateWithServer(obj)
            msg.UpdateRobot = ~isempty(obj.m_Robot);
            msg.type = 'UPDATE_ROBOT';
            msg.timestep = obj.m_nTimeStep;
                
            %% send robot state to robot
            % if we do not have any robot in proxy, send empty state
            if(size(obj.m_Robot,2)==0)
                SendMsg( obj.m_ServerSocket, msg );
%                 fprintf('Send Empty Msg to Server success.\n' );
            % if we have robot in proxy, send robot state
            else
                msg.robot = obj.m_Robot(end);
                msg.pose = obj.m_Robot(end).m_dPose;
                SendMsg( obj.m_ServerSocket, msg );
%                 fprintf('Send Robot Msg to Server success.\n' );
            end
            
            
            %% wait for server to reponse world state
            bSuccess = 0;
            while bSuccess == 0
%                 rmsg = RecvMsg(obj.m_ServerSocket, 0.001);
                rmsg = obj.RecvLatestMsg(obj.m_ServerSocket, obj.m_nTimeStep, 0);

                if ~isempty(rmsg)
                    if strcmp(rmsg.type, 'WORLD_STATE')
                        obj.m_WorldState = rmsg.worldstate;% get the compete world state
                        fprintf('Get Latest World State success. Size of World State is %d.\n', size(obj.m_WorldState,2) );
                        bSuccess = 1;
                    end
                end            
            end
            
        end
        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function  bSuccess=AddNewRobotToServer(obj, robot)
            %% add robot to server
            msg.type = 'ADD_PROXY_ROBOT';
            msg.timestep = obj.m_nTimeStep;
            msg.robot = robot;
            SendMsg( obj.m_ServerSocket, msg );

            %% wait for server to reponse
            bSuccess = 0;
            while bSuccess == 0
                rmsg = RecvMsg(obj.m_ServerSocket, 2);
                if ~isempty(rmsg)
                    if strcmp(rmsg.type, 'WELCOME_PROXY_ROBOT')
                        obj.m_WorldState = rmsg.worldstate;% get the compete world state
                        obj.m_Robot(1).SetPose( rmsg.startpose );
                        fprintf('proxy add robot to server success.\n' );
                        bSuccess = 1;
                    else
%                         rmsg.type
                    end
                else
%                     fprintf( 'No response from the server. Add new robot fail.\n' );
                end            
            end
        end
        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function  newobj = CheckIfAddNewRobot(obj)
            %% only allow add one robot to world proxy
            if size(obj.m_Robot,2) == 1
                newobj = obj;
                return
            end
            
            %% check if need to add new robot
            % 1) check if we need to add new robot
            con = pnet( obj.m_ProxySocket, 'tcplisten', 'noblock' );
            if con == -1
                newobj = obj;
                return
            end
            
            msg = RecvMsg( con, 0.1 );
            if ~isempty(msg)
                if strcmp(msg.type,'ADD_ROBOT') == 0
%                     pnet( con, 'close' );
%                     pnet( con, 'delete' );
                    fprintf('does not need to add new robot\n');
                else
                    obj.m_Robot = [obj.m_Robot, msg.robot];
                    obj.m_Robot(end).m_Socket = con;
                    obj.m_RobotWaitTime = 0;
                    fprintf('proxy add new robot success. Try to add robot to server.\n')
                    
                    % add robot to server
                    if AddNewRobotToServer(obj, obj.m_Robot(end)) == 1
                        
                        % reply robot (client)
                        rmsg.type = 'WELCOME_ROBOT';
                        SendMsg(obj.m_Robot(end).m_Socket, rmsg );
                        fprintf('reply robot(TestClient) success.\n');
                    else
                        fprintf('reply robot(TestClient) Fail.\n');
                    end
                end
            end
            newobj = obj;
            
            fprintf('finish add new robot. Current Size of Robot is %d\n', size(obj.m_Robot,2));
        end
   
    end
end




