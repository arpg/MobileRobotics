function [poses,u_i] = GenerateCommands(Map, initX, maxv, maxw)

z = Map;

[Fx, Fy] = gradient(z);
Fx = -Fx;
Fy = -Fy;

L = sqrt(Fx.^2 + Fy.^2); % for linear vel
W = atan2(Fy,Fx); % for angular vel

x = initX;
x_i = floor(initX);
ii=1;
Disp = 1000;
while (L(x_i(2), x_i(1))>0.001)&&(ii<50000)
    w_t = W(x_i(2), x_i(1)) - x(3);
    if(abs(w_t)> pi)
        w_t = -1*sign(w_t)*(2*pi - abs(w_t));
    end
    w = sign(w_t) * min(maxw, abs(w_t) );
    v = min(maxv, L(x_i(2), x_i(1))/(abs(w)+0.1));
    
    x = ProcessModel(x, [v w]);
    x_i = floor(x);
    w_t = W(x_i(2), x_i(1)) - x(3);
    if(abs(w_t)> pi)
        w_t = -1*sign(w_t)*(2*pi - abs(w_t));
    end
    w = sign(w_t) * min(maxw, abs(w_t) );
    v = min(maxv, L(x_i(2), x_i(1))/(abs(w)+0.1));
    PrevX = x;
    x = ProcessModel(x, [v w]);
%     Disp = norm(x-PrevX,1)
    x_i = floor(x);
    poses(:,ii) = x_i';
    u_i(:,ii) = [v w];
    ii = ii+1;
end

end