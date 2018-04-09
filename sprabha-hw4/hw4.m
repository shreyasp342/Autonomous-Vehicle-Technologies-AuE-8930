close all
clear all

L = 1;
dt = 0.01;
v = 1;

T = 6;

tt= 0;

t = 0:dt:T+2*tt;

for i = 1:length(t)
    if t(i)<=tt
        gphi(i) = 0;
        gx(i) = t(i);
        gy(i) = 1;
    elseif t(i)>T+tt
        gphi(i) = 0;
        gx(i) = t(i);
        gy(i) = 2;
    else
        gphi(i) = atan2(1,T);
        gx(i) = gx(i-1) + cos(gphi(i))*dt;
        gy(i) = gy(i-1) + sin(gphi(i))*dt;
    end
end

x(1) = 0;
y(1) = 1;
theta(1) = 0;


% k = 0.05;
k = 0.0:0.01:1;
for j = 1:length(k)
    gp = [0;1];
    clear x y theta
    x(1) = 0;
    y(1) = 1;
    theta(1) = 0;
    for i = 2:length(t)
        alpha = atan2(gy(i)-y(i-1),gx(i)-x(i-1)) - theta(i-1);
        [phi(i), ggp] = pure_pursuit(k(j), L ,v, [gx;gy], [x(i-1),y(i-1),theta(i-1)]);
        gp = [gp, ggp];
        x(i) = x(i-1) + (v*cos(theta(i-1))*dt);
        y(i) = y(i-1) + (v*sin(theta(i-1))*dt);
        theta(i) = theta(i-1) + (tan(phi(i))*dt*(v/L)*dt);
    end
    
    % figure;
    % for i = 1:length(t)
    % hold on
    % plot(gx,gy,'b');
    % plot(x(i),y(i),'k*');
    % plot(gx(i+1),gy(i+1),'r*');
    % plot(gp(1,i),gp(2,i),'g*');
    
    % pause(0.05);
    % hold off
    % clf
    % end
    
    e = sqrt((gx(t>tt&t<T+tt)-x(t>tt&t<T+tt)).^2+(gy(t>tt&t<T+tt)-y(t>tt&t<T+tt)).^2);
    err(j) = sqrt((gx(end)-x(end)).^2 + (gy(end)-y(end)).^2);
    error(j) = max(e);
    % hold on
    % plot(e);
    % pause(1);
end

clear x y theta
x(1) = 0;
y(1) = 1;
theta(1) = 0;
k2 = k(err<=0.05);
err(err<=0.05)
for j = 1:length(k2)
    gp = [0;1];
    clear x y theta
    x(1) = 0;
    y(1) = 1;
    theta(1) = 0;
    for i = 2:length(t)
        alpha = atan2(gy(i)-y(i-1),gx(i)-x(i-1)) - theta(i-1);
        [phi(i), ggp] = pure_pursuit(k2(j), L ,v, [gx;gy], [x(i-1),y(i-1),theta(i-1)]);
        gp = [gp, ggp];
        x(i) = x(i-1) + (v*cos(theta(i-1))*dt);
        y(i) = y(i-1) + (v*sin(theta(i-1))*dt);
        theta(i) = theta(i-1) + (tan(phi(i))*dt*(v/L)*dt);
    end
    
%     figure;
%     for i = 1:length(t)
%         hold on
%         plot(gx,gy,'b');
%         plot(x(i),y(i),'k*');
%         plot(gx(i),gy(i),'r*');
%         plot(gp(1,i),gp(2,i),'g*');
%         pause(0.01);
%         hold off
%         clf
%     end
    
    e = sqrt((gx(t>tt&t<T+tt)-x(t>tt&t<T+tt)).^2+(gy(t>tt&t<T+tt)-y(t>tt&t<T+tt)).^2);
    err(j) = sqrt((gx(end)-x(end)).^2 + (gy(end)-y(end)).^2);
    error(j) = max(e);
%     hold on
%     plot(e);
%     pause(1);
end

k = min(k2);
clear x y theta
x(1) = 0;
y(1) = 1;
theta(1) = 0;
for i = 2:length(t)
    alpha = atan2(gy(i)-y(i-1),gx(i)-x(i-1)) - theta(i-1);
    [phi(i), ggp] = pure_pursuit(k2(j), L ,v, [gx;gy], [x(i-1),y(i-1),theta(i-1)]);
    gp = [gp, ggp];
    x(i) = x(i-1) + (v*cos(theta(i-1))*dt);
    y(i) = y(i-1) + (v*sin(theta(i-1))*dt);
    theta(i) = theta(i-1) + (tan(phi(i))*dt*(v/L)*dt);
end
figure;plot(phi);
figure;plot(theta);
figure;
hold on
plot(gx,gy,'b');
plot(x,y,'r');
hold off
