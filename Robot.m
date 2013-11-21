classdef Robot < Entity
    properties
        m_LinearVelocity;
        m_AngularVelocity;
        m_MaxAbsLinearVelocity;
        m_MaxAbsAngularVelocity;
        m_vSensors;
        m_WorldProxy;
        m_Socket;
    end
    methods
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function obj = Robot()
            obj.m_LinearVelocity = 0;
            obj.m_AngularVelocity = 0;
            obj.m_MaxAbsLinearVelocity = 0.5;
            obj.m_MaxAbsAngularVelocity = 5*pi/180;
            obj.m_dPose = [0;0;0];
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function obj = AddSensor( obj, s )
            obj.m_vSensors{size(obj.m_vSensors,2)+1} = s;
            obj.Attach( s );
        end

        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function ApplyCommand( obj, command )
            if abs(command(1)) > obj.m_MaxAbsLinearVelocity
                command(1) = sign(command(1))*obj.m_MaxAbsLinearVelocity;
            end
            if abs(command(2)) > obj.m_MaxAbsAngularVelocity
                command(2) = sign(command(2))*obj.m_MaxAbsAngularVelocity;
            end

            map = obj.m_WorldProxy.m_Map;
            pose = obj.ProcessModel( obj.m_dPose, command );
            while( map.ObjectCollides(obj,pose) )
                pose = obj.ProcessModel( obj.m_dPose, command );
                command = command + [0.1*randn; 0.5*randn];
            end
            obj.m_dPose = pose;
            obj.m_LinearVelocity = command(1);
            obj.m_AngularVelocity = command(2);

            msg.type = 'CLIENT_UPDATE';
            msg.pose = pose;
            msg.vel = command;
            msg.name = obj.m_sName;
            % send command msg for new pose
            SendMsg( obj.m_WorldProxy.m_Socket, msg );

            % apply pose 
            rmsg = RecvMsg( obj.m_WorldProxy.m_Socket );
            obj.m_dPose = rmsg.pose;
        end
        
       
        function ApplyCommandNew( obj, command )
            if abs(command(1)) > obj.m_MaxAbsLinearVelocity
                command(1) = sign(command(1))*obj.m_MaxAbsLinearVelocity;
            end
            if abs(command(2)) > obj.m_MaxAbsAngularVelocity
                command(2) = sign(command(2))*obj.m_MaxAbsAngularVelocity;
            end
            
            % send command msg for new pose
            msg.type = 'REQ_APPLYCOMMAND';
            msg.pose = obj.m_dPose;
            msg.vel = command;
            msg.name = obj.m_sName;
            SendMsg( obj.m_Socket, msg );
            fprintf('[ApplyCommand] send req for apply command. Waitting for world proxy to reponse..\n');

            % wait for reply (lock)
            bSuccess = 0;
            while bSuccess == 0
%                 fprintf('waitting for apply command result from world proxy..\n');
                rmsg = RecvMsg( obj.m_Socket, 0.01);
                if ~isempty(rmsg)    
                    if strcmp(rmsg.type, 'REP_APPLYCOMMAND')==1
                       obj.m_dPose = rmsg.dPose;
                       obj.m_LinearVelocity = rmsg.LinearVelocity;
                       obj.m_AngularVelocity = rmsg.AngularVelocity;
                       bSuccess = 1;
                       fprintf('[ApplyCommand] update command success.\n');
                    end
                end
            end
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function [f,Fx,Fu] = ProcessModel( obj, x, u )
            f = [ x(1) + u(1)*cos( x(3)+u(2) );...
                x(2) + u(1)*sin( x(3)+u(2) );...
                x(3) + u(2) ];
            if nargout >= 2
                Fx = [ 1 0 -u(1)*sin( x(3)+u(2) );...
                    0 1  u(1)*cos( x(3)+u(2) );...
                    0 0  1 ];
            end
            if nargout == 3
                Fu = [ cos( x(3)+u(2) )  -sin( x(3)+u(2) ) ;...
                    sin( x(3)+u(2) )   cos( x(3)+u(2) ) ;...
                    0                  1];
            end
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function z = ReadSensor( obj, sName )
            z = [];
            for ii = 1:size(obj.m_vSensors,2)
                if strcmp(obj.m_vSensors{ii}.GetName(), sName)
                    z = obj.m_vSensors{ii}.GetMeasurement();
                end
            end
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function S = saveobj(obj)
            % Call superclass saveobj
            % Save property values in struct
            S = saveobj@Entity(obj);
            S.m_LinearVelocity = obj.m_LinearVelocity;
            S.m_AngularVelocity = obj.m_AngularVelocity;
            S.m_vSensors = obj.m_vSensors;
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function obj = reload(obj,S)
            % Call superclass reload method
            % Assign subclass property value
            % Called by loadobj
            obj = reload@Entity(obj,S);
            obj.m_LinearVelocity = S.m_LinearVelocity;
            obj.m_AngularVelocity = S.m_AngularVelocity;
            obj.m_vSensors = S.m_vSensors;
        end
    end
    
    methods (Static)

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function obj = loadobj(S)
            % Create object of Robot class
            % Assign property value retrived from struct
            % loadobj must be Static so it can be called without object
            obj = Robot;
            obj = reload(obj,S);
        end
    end
end
