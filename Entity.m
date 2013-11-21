classdef Entity < handle
%    properties(GetAccess='private')
    properties
        m_dPose;
        m_vChildren;
        m_nId;
        m_sName;
        m_RenderGeom;
        m_CollisionGeom;
        m_GraphicsHandles;
    end
    methods
        function obj = Entity()
            persistent nIdPool;
            if isempty( nIdPool )
                nIdPool = 1;
            end
            obj.m_nId = nIdPool;
            nIdPool = nIdPool+1;
            obj.m_GraphicsHandles = [];
        end
                
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function nId = SetName( obj, sName )
            obj.m_sName = sName;
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function sName = GetName( obj )
            sName = obj.m_sName;
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
        function nId = GetId( obj )
            nId = obj.m_nId;
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
        function p = GetPose(obj)
           p = obj.m_dPose;
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function obj = SetPose(obj, p)
           	obj.m_dPose = p;
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function pts = CollisionPoints( obj, p )
            if nargin == 1
                p = obj.m_dPose;
            end
            pts = obj.m_RenderGeom.verts';
            T = [cos(p(3)), -sin(p(3)), p(1);...
                sin(p(3)),  cos(p(3)), p(2)];
            pts = T*[pts;ones(1,size(pts,2))];
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function h = Draw( obj )
            if ishandle(obj.m_GraphicsHandles)
                delete(obj.m_GraphicsHandles)
                obj.m_GraphicsHandles = [];
            end

%            h = plot( obj.m_dPose(1), obj.m_dPose(2), '.', 'MarkerEdgeColor', obj.m_RenderGeom.colors );
            pts = obj.CollisionPoints();
            h = patch( 'Faces', obj.m_RenderGeom.faces, ...
                'Vertices', pts', ...
                'FaceColor', obj.m_RenderGeom.colors );

            x = obj.m_dPose(1)-length(obj.m_sName)/2;
            y = obj.m_dPose(2)-1;
            h = [h text( x, y, obj.m_sName,...
                'FontSize', 8 )];

            % draw children recursivley
            for ii = 1:size(obj.m_vChildren,2)
                h = [h obj.m_vChildren(ii).Draw()];
            end
            obj.m_GraphicsHandles = h;
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function Clear( obj )
            if ishandle(obj.m_GraphicsHandles)
                delete(obj.m_GraphicsHandles)
                obj.m_GraphicsHandles = [];
            end
            for ii = 1:size(obj.m_vChildren,2)
               obj.m_vChildren(ii).Clear();
            end
        end

        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function obj = Attach( obj, Child )
            % thing
            obj.m_vChildren = [obj.m_vChildren, Child ];
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function obj = SetRenderGeometry( obj, geom )
            obj.m_RenderGeom = geom;
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function obj = SetCollisionGeometry( obj, geom )
            obj.m_CollisionGeom = geom;
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % needed for serialization
        function S = saveobj(obj)
            S.m_dPose = obj.m_dPose;
            S.m_vChildren = obj.m_vChildren;
            S.m_nId = obj.m_nId;
            S.m_sName = obj.m_sName;
            S.m_RenderGeom = obj.m_RenderGeom;
            S.m_CollisionGeom = obj.m_CollisionGeom;
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % needed for serialization
        function obj = reload(obj,S)
            obj.m_dPose = S.m_dPose;
            obj.m_vChildren = S.m_vChildren;
            obj.m_nId = S.m_nId;
            obj.m_sName = S.m_sName;
            obj.m_RenderGeom = S.m_RenderGeom;
            obj.m_CollisionGeom = S.m_CollisionGeom;
        end
    end
    methods (Static)
        % needed for serialization
        function obj = loadobj(S)
            obj = Entity;
            obj = reload(obj,S);
        end
    end
end
