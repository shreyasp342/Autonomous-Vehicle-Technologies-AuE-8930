% function [phi, gp] = pure_pursuit(k, L, v, g, pos)
function phi = pure_pursuit(k, L, v, g,pos)
    
Xcf = g(1,1);
Zcf = g(1,3);
Xcn = g(2,1);
Zcn = g(2,3);
theta = -atan((Zcf-Zcn)/(Xcf-Xcn))-90;
    
%     d = abs(sqrt((g(1,:)-pos(1)).^2+(g(2,:)-pos(2)).^2));
%     d = abs(d-ld);
%     d = fliplr(d);
%     [~,i] = min(d);
%     i = size(g,2) -i + 1;
%     alpha = atan2(g(2,i)-pos(2),g(1,i)-pos(1)) - pos(3);
    ld = k*v;
    alpha = atan2(g(1,3)-pos(3),g(1,1)-pos(1)) - theta;
%     alpha = atan2(gy(i)-y(i-1),gx(i)-x(i-1)) - theta(i-1);
    phi = atan2(2*L*sin(alpha),ld);
%     gp = g(:,i);
end