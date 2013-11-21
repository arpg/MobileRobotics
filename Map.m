classdef Map < handle
    properties
        m_MapImage;
        m_sMapName;
        m_dTop;
        m_dLeft;
        m_dMetersPerPixel;
        m_ImageHandle;
    end
    methods
        function obj = Map()
        end
        
        function obj = Init( obj, sName, dTop, dLeft, dMetersPerPixel )
            obj.m_sMapName = sName;        
            im = imread( sName );
            if ndims(im) == 2
                obj.m_MapImage = uint8(im);
            else
                obj.m_MapImage = rgb2gray(im);
            end
            obj.m_dMetersPerPixel = dMetersPerPixel;
            obj.m_dTop = dTop;
            obj.m_dLeft = dLeft;
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % find a random pose where all points in pts are in free space
        function p = RandomPose( obj, ent )
            pts = ent.m_CollisionGeom.verts';
            th = 2*pi*rand;
            R = [cos(th) -sin(th); sin(th) cos(th)];
            pxy = R*pts(1:2,:);
            xy = obj.RandomFreeSpace( pxy );
            p = [xy;th];
        end

        % find a place where all points in pts are in free space
        function p = RandomFreeSpace( obj, pts )
            p = obj.RandomPointInFreeSpace();
            g = pts+repmat(p,1,size(pts,2));
            while obj.CollisionExists( g )
                p = obj.RandomPointInFreeSpace();
                g = pts+repmat(p,1,size(pts,2));
            end
        end
        
        function p = RandomPointInFreeSpace( obj )
            w = size(obj.m_MapImage,2);
            h = size(obj.m_MapImage,1);
            x = max(1,round(w*rand));
            y = max(1,round(h*rand));
            while obj.m_MapImage(y,x) < 250;
                x = max(1,round(w*rand));
                y = max(1,round(h*rand));
            end
            p = [obj.m_dLeft + x*obj.m_dMetersPerPixel;...
                 obj.m_dTop + y*obj.m_dMetersPerPixel];
        end

        function cx = CenterX( obj )
            cx = obj.m_dTop + obj.Width()/2;
        end

        function cy = CenterY( obj )
            cy = obj.m_dLeft + obj.Height()/2;
        end

        function w = Width( obj )
            w = size(obj.m_MapImage,2)*obj.m_dMetersPerPixel;
        end

        function h = Height( obj )
            h = size(obj.m_MapImage,1)*obj.m_dMetersPerPixel;
        end
        
        function bRes = ObjectCollides( obj, ent, pose )
            pts = ent.CollisionPoints(pose);
            bRes = obj.CollisionExists( pts );
        end
        
        function bRes = CollisionExists( obj, pts )
            bRes = false;
            for ii = 1:size(pts,2)
               v = round(pts(:,ii));
               x = (v(1) - obj.m_dLeft)/obj.m_dMetersPerPixel;
               y = (v(2) - obj.m_dTop)/obj.m_dMetersPerPixel;

               if obj.m_MapImage(y,x) < 250
%                   plot( v(1), v(2), 'r.' );
                   bRes = true;
%                   waitforbuttonpress();
               end
            end
        end
       
        function pix = MToPix( obj, m )
            pix = (m-[obj.m_dLeft;obj.m_dTop]) / obj.m_dMetersPerPixel;
        end

        function m = PixToM( obj, pix )
            m = [ obj.m_dLeft + pix(1)*obj.m_dMetersPerPixel;...
                  obj.m_dTop  + pix(2)*obj.m_dMetersPerPixel];
        end

        function h = Draw( obj )
            hold on;
            x = obj.m_dLeft;
            y = obj.m_dTop;
            width = obj.Width();
            height = obj.Height();

            [X,Y] = meshgrid( [x,x+width], [y,y+height] );
            Z = zeros(size(X));

            im = double( imresize( obj.m_MapImage, 0.2 ) );
            obj.m_ImageHandle = surface( X, Y, Z, im,...
               'FaceColor','texturemap','EdgeColor','none',...
               'CDataMapping','scaled');
            colormap(gray);

            % NASA convention
            set( gca, 'ZDir', 'reverse', 'YDir', 'reverse' );
            axis tight;
            xlabel( 'x' );
            ylabel( 'y' );
            h = obj.m_ImageHandle;
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function S = saveobj(obj)
            S.m_MapImage        = obj.m_MapImage;
            S.m_sMapName        = obj.m_sMapName;
            S.m_dTop            = obj.m_dTop;
            S.m_dLeft           = obj.m_dLeft;
            S.m_dMetersPerPixel = obj.m_dMetersPerPixel;
            S.m_ImageHandle     = obj.m_ImageHandle;
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function obj = reload(obj,S)
            obj.m_MapImage        = S.m_MapImage;
            obj.m_sMapName        = S.m_sMapName;
            obj.m_dTop            = S.m_dTop;
            obj.m_dLeft           = S.m_dLeft;
            obj.m_dMetersPerPixel = S.m_dMetersPerPixel;
            obj.m_ImageHandle     = S.m_ImageHandle;
        end
    end

    methods (Static)
        function obj = loadobj(S)
            obj = Map;
            obj = reload(obj,S);
        end 
    end
end
