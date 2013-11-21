function T = Cart2T( l )
if( size(l,1) == 6 )
T = epose_to_hpose( l );
elseif( size(l,1) == 3 )
th = l(3);
T = [ cos(th),   sin(th),   l(1);...
     -sin(th),   cos(th),   l(1);...
     0,         0,      1];
else
error('Cart2T -- incorrect vector size, must be 6dof or 3dof');
end
