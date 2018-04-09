function [phi, gp] = pure_pursuit(k, L, v, g, pos)
    ld = k*v;
    
    d = abs(sqrt((g(1,:)-pos(1)).^2+(g(2,:)-pos(2)).^2));
    d = abs(d-ld);
    d = fliplr(d);
    [~,i] = min(d);
    i = size(g,2) -i + 1;
    alpha = atan2(g(2,i)-pos(2),g(1,i)-pos(1)) - pos(3);
%     alpha = atan2(gy(i)-y(i-1),gx(i)-x(i-1)) - theta(i-1);
    phi = atan2(2*L*sin(alpha),ld);
    gp = g(:,i);
end