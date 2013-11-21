classdef Sensor < Entity
    properties
        m_sKind;
        m_WorldProxy;
    end
    methods
        function z = GetMeasurement( obj )
            z = [];
        end
        
        function S = saveobj(obj)
            % Call superclass saveobj
            % Save property values in struct
            S = saveobj@Entity(obj);
            S.m_sKind = obj.m_sKind;
        end
        function obj = reload(obj,S)
            % Call superclass reload method
            % Assign subclass property value
            % Called by loadobj
            obj = reload@Entity(obj,S);
            obj.m_sKind = S.m_sKind;
        end
    end
    methods (Static)
        function obj = loadobj(S)
            % Create object of Robot class
            % Assign property value retrived from struct
            % loadobj must be Static so it can be called without object
            obj = Sensor;
            obj = reload(obj,S);
        end
    end
end
