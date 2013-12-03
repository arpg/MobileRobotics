%% call 
% ParticleFiltering([],[],nbeams_,maxrange_,fov_,map_,N_particles_,threshold_) 
% first
%% For all other calls use:
% ParticleFiltering(scan_,command_)
%% output
% pose = 3 x 1 (estimated position of robot)
function pose = ParticleFiltering(scan_,command_,nbeams_,maxrange_,fov_,map_,N_particles_,threshold_)
    persistent particles;
    persistent map;
    persistent nbeams;
    persistent maxrange;
    persistent fov;
    persistent N_particles;
    persistent threshold;
    if nargin == 8
        N_particles = N_particles_;
        threshold = threshold_;
        particles = [];
        map = map_;
        nbeams = nbeams_;
        maxrange = maxrange_;
        fov = fov_;
        for ii = 1:N_particles
            particles = [particles,RandomPose(map,robot)];
        end
        particles = [particles;1/N_particles * ones(1,N_particles)];
        pose = weighted_robust_mean(particles);
        return;
    end
    
    % prep for mex call to generate laser scan:
    z = scan_

    p = ScanToPoints2( z, fov, nbeams );
    for ii = 1:size(particles,2)
        for i = 1:10
            particles = [particles, particles(:,ii)+randn(4,1)];
        end
    end

    % Sample particle sensors from Map
    particles = PropegateParticles(particles,command_,[0.01,0.01]);
    particles = WeighParticles(particles,p,map,maxrange, fov, nbeams,threshold );
    pose = weighted_robust_mean(particles);
end

function pose = weighted_robust_mean(particles)
% Find most likely particle
[~,idx] = max(particles(4,:));
mlp_xy = particles(1:2,idx);
mlp_th = particles(3,idx);

% Find all particles within epsilon

% Arbitrary thresholds for point grouping. May need to change.
epsilon_xy = 10;
epsilon_th = pi/4;

dist_xy = sum((particles(1:2,:) - repmat(mlp_xy,1, size(particles,2))).^2);
dist_th = abs(particles(3,:) - mlp_th);
xy_epsilon_idx = dist_xy < epsilon_xy;
th_epsilon_idx = dist_th < epsilon_th;
pt_epsilon = xy_epsilon_idx & th_epsilon_idx;

particles_epsilon = particles(:, pt_epsilon);

% Compute weighted mean
weighted_pose = particles_epsilon(1:3,:).*repmat(particles_epsilon(4,:), 3, 1);
pose = sum (weighted_pose,2)/sum(particles_epsilon,2);

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% MEX THESE FILES
function particles = PropegateParticles(particles,U,Q)
     for ii =1:size(particles,2)
         noise = diag(Q)*randn(size(U));
         u = U+noise;
         x = particles(1:3,ii);
         particles(1:3,ii) = [ x(1) + u(1)*cos( x(3)+u(2) );
                               x(2) + u(1)*sin( x(3)+u(2) );
                               x(3) + u(2) ];
%          particles(1:3,ii) = ProcessModel( particles(1:3,ii), u+noise);
     end
end

function particles = WeighParticles(particles,pr,map,maxrange, fov, nbeams,threshold)
% wparticles = [weights;particles]
% particles: particle poses
% p: robot
    for ii =1:size(particles,2)
        zp = ReadLaser( uint8(map.m_MapImage),...
           map.m_dMetersPerPixel,...
           map.m_dLeft,...% in meteres
           map.m_dTop,...% in meteres
           particles(1:3,ii), maxrange, fov, nbeams );

       % agnostic (robot frame) [r,b] -> [x,y]_robot
        pp = ScanToPoints2( zp, fov, nbeams );
        
        res = inf*ones(size(pp,1),1);
        for i = 1:size(pp,1);
            for j = 1:size(pr,1);
                if(norm(pp(i,:)-pr(j,:)) < res(i)), res(i) = norm(pp(i,:)-pr(j,:)); end
            end
        end
        
        weight = exp(-0.5*norm(res)/10);
        % set weight
        particles(4,ii) = weight;
    end
    particles = sortrows(particles',4)';
    particles = particles(:,end-threshold:end);
end
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function p = ScanToPoints2( z, fov, nbeams )
    th = -fov/2:fov/(nbeams-1):fov/2;
    p =  [ z'.*cos(th);
           z'.*sin(th)];
end