
classdef Server < handle
    properties
        m_sHostname;
        m_nHostport;
        m_vClients;
        m_Map;
        m_nSocket;
        m_ImageHandle;
    end
    methods
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function obj = Server( sMapfile )
            pnet('closeall');
            obj.m_sHostname = '0.0.0.0';
            obj.m_nHostport = 9999;
            obj.m_Map = Map();
            obj.m_Map.Init( sMapfile );
            obj.m_vClients = [];
            obj.m_nSocket = pnet( 'tcpsocket', obj.m_nHostport );
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function Run( obj )
            obj.m_vClients = [];
            obj.m_Map.Draw();
            nTimeStep = 0;
            while 1
                t = tic();
                obj.AcceptNewClient();
                obj.GetUpdatesfFromAllClients(nTimeStep);
                obj.DrawState();
                nTimeStep = nTimeStep +1;
                fprintf('Running at %.2fhz\n', 1/toc(t));
            end
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function GetUpdatesfFromAllClients(obj,nTimestep)
            nClients = size(obj.m_vClients,2);
            nClientCount = 0;
            t = tic;
            while nClientCount ~= nClients
                for ii = 1:nClients
                    c = obj.m_vClients(ii);
                    msg = RecvMsg( c.m_Socket, 0 );
                    if ~isempty(msg)
                        c.m_nTimeStep = nTimestep;
                        r = c.m_Robot;
                        r.m_dPose = msg.pose;
                        fprintf('heard from %s\n', r.m_sName );
                        nClientCount = nClientCount+1;
                        rmsg.type = 'STATE_UPDATE';
                        rmsg.pose = c.m_Robot.m_dPose;
                        SendMsg( c.m_Socket, rmsg );
                    end
                end
                % wait 0.5 seconds to hear from all clients
                if toc(t) > 0.5
                    fprintf('Tardy Clients:\n');
                    vTardy = obj.TardyClients(nTimestep);
                    for jj = 1:size(vTardy,2)
                        for kk = 1:size(obj.m_vClients,2)
                            if( obj.m_vClients(kk) == vTardy{jj} )
                                fprintf( 'Kicking %s\n', jj, vTardy{jj}.m_Robot.m_sName );
                                obj.m_vClients(kk).Disconnect();
                                obj.m_vClients(kk) = [];
                            end
                        end
                    end
                    return;
                end
            end            
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
        function vTardy = TardyClients( obj, nTimestep )
            vTardy = {};
            for jj = 1:size(obj.m_vClients,2)
                c = obj.m_vClients(jj);
                if c.m_nTimeStep < nTimestep
                    vTardy{size(vTardy,2)+1} = c;
                end
            end
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function DrawState( obj )
            for ii = 1:size(obj.m_vClients,2)
                r = obj.m_vClients(ii).m_Robot;
                r.Draw();
            end
            drawnow();
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % send back robot poses
        function vState = GetWorldState( obj )
            vState = [];
            for ii = 1:size(obj.m_vClients,2)
                c = obj.m_vClients(ii);
                vState = [vState c.m_Robot.GetPose()];
            end
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % just cat all the robots together...
        function vState = GetCompleteWorldState( obj )
            vState = [];
            for ii = 1:size(obj.m_vClients,2)
                c = obj.m_vClients(ii);
                vState = [ vState c.m_Robot ];
            end
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function p = GetInitialPose( obj, pts )
            th = 2*pi*rand;
            R = [cos(th) -sin(th); sin(th) cos(th)];
            pxy = R*pts(1:2,:);
            xy = obj.m_Map.RandomFreeSpace( pxy );
            
            xy = [240;145];
            p = [xy;th];
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function AcceptNewClient( obj )
            % 1) wait for client to connect and CLIENT_CONNECT msg
            con = pnet( obj.m_nSocket, 'tcplisten', 'noblock' );
            if con == -1
                return
            end
%            fprintf('Client trying to connect...');
            msg = RecvMsg( con, 1 );
            if isempty(msg)
                return;
            end

            c = Client();
            c.m_Socket = con;
            c.m_Robot = msg.robot;
            g = msg.robot.m_CollisionGeom.verts';
            p = obj.GetInitialPose(g);
            c.m_Robot.SetPose( p );
            obj.m_vClients = [obj.m_vClients c ];

            % 2) All good? Send state to client.
            rmsg.type = 'WELCOME_MSG';
            rmsg.map = obj.m_Map;
            rmsg.worldstate = obj.GetCompleteWorldState();
            rmsg.startpose = p;

%            t = hlp_deserialize( hlp_serialize(rmsg) )

            SendMsg( con, rmsg );
        end
    end
end
