classdef Laser < Sensor
    properties
        m_vScan;
        m_dMaxRange;
        fov;
        nbeams;
    end
    methods
        function obj = Laser()
            obj.m_dMaxRange = 40; 
            obj.fov = 120*pi/180;
            obj.nbeams = 121;
        end

        function z = GetMeasurement( obj )            
            map = obj.m_WorldProxy.m_Map;
            xwr = obj.m_WorldProxy.m_Robot.m_dPose;
            z = ReadLaser( uint8(map.m_MapImage),...
                map.m_dMetersPerPixel,...
                map.m_dLeft,...% in meteres
                map.m_dTop,...% in meteres
                xwr, obj.m_dMaxRange, obj.fov, obj.nbeams );
        end
        
        
        function z = GetMeasurementNew( obj,robot)  
           % send command to request measurement
            msg.type = 'REQ_MEASUREMENT';
            msg.pose = robot.m_dPose;
            msg.dMaxRange = obj.m_dMaxRange; 
            msg.fov       = obj.fov; 
            msg.nbeams    = obj.nbeams;
            SendMsg( robot.m_Socket, msg );
            fprintf('[GetMeasurement] send request for new measurement, wait for proxy to reponse.\n');

            % wait for reply
            bSuccess = 0;
            while bSuccess == 0  
%                 fprintf('waitting for measurement from world proxy..\n');
                rmsg = RecvMsg( robot.m_Socket, 0.01);
                if ~isempty(rmsg)    
                    if strcmp(rmsg.type, 'REP_MEASUREMENT')==1
                       z = rmsg.z;
                       bSuccess = 1;
                       fprintf('[GetMeasurement] Get new measurement from world proxy success!\n');
                    end
                end
            end
                     
        end
        
        
        
        function SetFoV( obj, fov )
            obj.fov = fov;
        end
        
        function SetNumBeams( obj, nbeams )
            obj.nbeams = nbeams;
        end
        
        function S = saveobj(obj)
            % Call superclass saveobj
            % Save property values in struct
            S = saveobj@Sensor(obj);
            S.m_sKind = obj.m_sKind;
        end
        function obj = reload(obj,S)
            % Call superclass reload method
            % Assign subclass property value
            % Called by loadobj
            obj = reload@Sensor(obj,S);
            obj.m_sKind = S.m_sKind;
        end
    end

    methods (Static)
        function obj = loadobj(S)
            % Create object of Robot class
            % Assign property value retrived from struct
            % loadobj must be Static so it can be called without object
            obj = Laser;
            obj = reload(obj,S);
        end
    end
end
