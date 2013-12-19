
classdef Server < handle
    properties
        m_sHostname;
        m_nHostport;
        m_vClients;
        m_Map;
        m_nSocket;% server socket
        m_ImageHandle;
        
        m_vClientWaitTime;
        m_vClientMaxWaitTime;
        
        m_vClientWaitNum;
        m_nClientMaxWaitNum;
        
        m_nTimeStep;
    end
    methods
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % clear all;clear all;server = Server('DilatedMap.png');server.Run();
        function obj = Server( sMapfile )
            obj.m_sHostname         = '0.0.0.0';
            obj.m_nHostport         = 9999;
            obj.m_Map               = Map();
            obj.m_Map.Init( sMapfile, 0, 0, 0.1 ); % top, left, meters per pixel
            
            obj.m_vClients          = [];
            obj.m_nSocket           = pnet( 'tcpsocket', obj.m_nHostport );
            obj.m_nTimeStep         = 0;
            obj.m_nClientMaxWaitNum = 100;
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function Run( obj )
            obj.m_vClients = [];
            obj.m_Map.Draw();
            while 1                
                % check if we want to add new client.
                CheckIfAddNewProxy(obj);
                
                % sync with all client
                obj=UpdatesAllClients(obj);
                
                % draw state
                DrawState(obj);
                
                obj.m_nTimeStep = obj.m_nTimeStep +1;  
                
                % wait for sometime to avoid network delay.
                pause(0.03) 
            end
        end

        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function UpdateClient(obj, client, msg)
            if (msg.UpdateRobot == 1)
                client.m_nTimeStep = obj.m_nTimeStep;
                client.m_Robot.m_dPose = msg.pose;
%                 fprintf('Update client %s success. \n', client.m_Robot.m_sName );
            else
%                 fprintf('Receive Msg from client Success. But Does not get any robot info.\n');
            end
        end
        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function AddProxyRobot(obj, client, msg)
             % 1) try to add new robot
             client.m_Robot = msg.robot;
             g = msg.robot.m_CollisionGeom.verts';
             p = obj.GetInitialPose(g);
             client.m_Robot.SetPose( p );

             % 2) All good? Send state to client.
             rmsg.type = 'WELCOME_PROXY_ROBOT';
             rmsg.worldstate = obj.GetCompleteWorldState();
             rmsg.startpose  = p;

             % 3) t = hlp_deserialize( hlp_serialize(rmsg) )
             SendMsg( client.m_Socket, rmsg );
             fprintf('Server Add new robot from proxy success.\n'); 
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function msg = RecvLatestMsg(obj, Socket, CurTimeStep, RecurseNum)
            RecurseNum = RecurseNum +1;
            msg = RecvMsg( Socket, 0.002);% use socket
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
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function newobj = UpdatesAllClients(obj)

            %% 1, try to get data from each client
            for ii = 1:size(obj.m_vClients,2)
                WaitTime = tic();
 
                %% try to get data from client, if fail, wait
                c = obj.m_vClients(ii);
%                 msg = RecvMsg( c.m_Socket, 0.001 );% use socket
                msg = obj.RecvLatestMsg(c.m_Socket, obj.m_nTimeStep, 0);
                
                %% try to get state from client
                if ~isempty(msg)
                    % try to get robot state from client
                    if strcmp(msg.type, 'UPDATE_ROBOT') ==1
                       obj.m_vClientWaitTime(ii) = 0;
                       obj.m_vClientWaitNum(ii) = 0;
                       obj.UpdateClient(c, msg);
                    % client request add new robot
                    elseif strcmp(msg.type,'ADD_PROXY_ROBOT') == 1
                       obj.m_vClientWaitTime(ii) = 0;
                       obj.m_vClientWaitNum(ii) = 0;
                       obj.AddProxyRobot(c, msg);
                    else
                       obj.m_vClientWaitTime(ii) = obj.m_vClientWaitTime(ii) + toc(WaitTime);
                       obj.m_vClientWaitNum(ii) = obj.m_vClientWaitNum(ii) + 1;
                    end
                else
                    obj.m_vClientWaitTime(ii) = obj.m_vClientWaitTime(ii) + toc(WaitTime);
                    obj.m_vClientWaitNum(ii) = obj.m_vClientWaitNum(ii) + 1;
%                     fprintf('Cannot receive any msg from client.. Total waited %f for client %d \n', obj.m_vClientWaitTime(ii) , ii);
                    fprintf('Cannot receive any msg from client.. Total waited %d Times for client %d \n', obj.m_vClientWaitNum(ii) , ii);
                end
            end
            
            
            %% Send world state to all client
            worldstate = obj.GetWorldState();
            fprintf('Size of world state is %d\n ', size(worldstate, 2)); 

            for ii = 1:size(obj.m_vClients,2)
                % reply world state to client
                rmsg.type = 'WORLD_STATE';
                rmsg.worldstate = worldstate;
                rmsg.timestep = obj.m_nTimeStep;
                SendMsg( obj.m_vClients(ii).m_Socket, rmsg ); 
            end
            
            
            %% delete client if we lost it
            for ii = 1:size(obj.m_vClients,2)
                if(obj.m_vClientWaitTime(ii) >0.6) || obj.m_vClientWaitNum(ii)>obj.m_nClientMaxWaitNum
                    % delete lost client
                    fprintf('Delete lost client! No Response Over %f seconds.\n',obj.m_vClientWaitTime(ii));
                    obj.m_vClients(ii) = [];
                    obj.m_vClientWaitTime(ii) = 0; 
                end
            end
                
            
            %% update obj
            newobj = obj;
%             fprintf('GetUpdatesfFromAllClients(%d) Success.\n', size(obj.m_vClients,2));
        end

      
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
        function vTardy = TardyClients( obj, nTimestep )
            vTardy = {};
            for jj = 1:size(obj.m_vClients,2)
                c = obj.m_vClients(jj);
                if c.m_nTimeStep < nTimestep
                    vTardy{size(vTardy,2)+1} = c.m_Robot.m_sName;
                end
            end
        end

        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function DrawState( obj )
             t = tic();
%            if ~ishandle( obj.m_ImageHandle )
 %               return;
 %           end
%             imshow( obj.m_Map );

            for ii = 1:size(obj.m_vClients, 2)
                if ~isempty(obj.m_vClients(ii).m_Robot)
                    r = obj.m_vClients(ii).m_Robot;
                    r.Draw();
%                     fprintf('DrawState() took %f\n', toc(t));
                end
            end    
        end
        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % send back robot poses
        function vState = GetWorldState( obj )
            vState = [];
            for ii = 1:size(obj.m_vClients,2)
                if  size(obj.m_vClients(ii).m_Robot, 2) ~= 0 
                    vState = [vState  obj.m_vClients(ii).m_Robot.m_dPose];
%                   fprintf('get %d world state element..\n',size(vState, 2)); 
                end
            end
        end

        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % just cat all the robots together...
        function vState = GetCompleteWorldState( obj )
            vState = [];
            for ii = 1:size(obj.m_vClients,2)
                c = obj.m_vClients(ii);
                vState = [ vState c.m_Robot ];
%                 fprintf('get %d world state element..\n',size(vState, 2)); 
            end
        end

        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function p = GetInitialPose( obj, pts )
            th = 2*pi*rand;
            R = [cos(th) -sin(th); sin(th) cos(th)];
            pxy = R*pts(1:2,:);
            xy = obj.m_Map.RandomFreeSpace( pxy );
            p = [xy;th];
        end
        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function CheckIfAddNewProxy(obj)
            % 1) wait for client to connect and CLIENT_CONNECT msg
            con = pnet( obj.m_nSocket, 'tcplisten', 'noblock' );
            if con == -1
                return
            end
                        
            msg = RecvMsg( con, 0.001 );
            if ~isempty(msg)
                if  strcmp(msg.type,'CLIENT_CONNECT') == 0
                    warning( 'Failed to hear from client!\n');
                    pnet( con, 'close' );
                    pnet( con, 'delete' );
                    return;
                end
            end

            fprintf('Client trying to connect...\n');

            c = Client();
            c.m_Socket = con;% save socket for later usages
            obj.m_vClients = [obj.m_vClients c ];
            obj.m_vClientWaitTime = [obj.m_vClientWaitTime 0 ];
            obj.m_vClientWaitNum = [obj.m_vClientWaitNum 0];

            % 2) All good? Send state to client.
            rmsg.type = 'WELCOME_CLIENT';
            rmsg.timestep = obj.m_nTimeStep;
            rmsg.map = obj.m_Map;
            rmsg.worldstate = obj.GetCompleteWorldState();

            SendMsg( c.m_Socket, rmsg );
            
            fprintf('Add NewProxy success. Num of Connected Proxy is %d\n', size(obj.m_vClients,2));
        end
                
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    end
end
