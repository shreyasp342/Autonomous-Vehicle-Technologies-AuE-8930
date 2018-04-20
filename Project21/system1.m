clc
clear
close all
i=0;
K1=1;
K2=1;

IPsystem1 = '192.168.1.249';
IPsystem2 = '192.168.1.250';
portSystem1 = 9090;
portSystem2 = 9091;
dataqueue = zeros(1,5);
udpSystem1 = udp(ipSystem2,portSystem2,'LocalPort',portSystem1);

Cam=webcam(1);
Cam.Resolution='1920x1080'
a=arduino('COM5','UNO','Libraries','servo')
V=servo(a,'D12','MinPulseDuration',1e-3,'MaxPulseDuration',2e-3);
S=servo(a,'D13','MinPulseDuration',1e-3,'MaxPulseDuration',2e-3);
Left =[0,0];Right =[0,0];
EC=zeros(4,4);
vel=0.57;
vel2 = 0.56;
EQ=[0.19,0.0,0.0,0.0;0,0.09,00,00;0.0,0.0,0.08,0.0;0,00,0,0.0001;];
ER=[0.029,0.0,0.0,0.0;0,0.001,00,00;0.0,0,0.0029,0.0;0,00,0,0.0002;];
while (1)
    i=i+1;
    p=snapshot(Cam);
    sh=size(p);
    p=imresize(p,0.25);
    shape=size(p);
    p1=rgb2gray(p);
    
    BW=edge(p1,'canny',[0.31,0.36]);%detect edges
    
    a=[shape(2)*1.5/6,shape(2)*4.5/6,shape(2),0]; %Region Masking
    b=[shape(1)*3.5/7,shape(1)*3.5/7,shape(1),shape(1)];
    bw=roipoly(p,a,b);
    
    BW=(BW(:,:)&bw);
    sh=size(BW);
    [H,theta,rho]=hough(BW); %Transform Display
    
    P = houghpeaks(H,4,'threshold',0.5*max(max((H))));
    %first number of peaks to identify Threshold- minimum value to be
    %considered a peak
    
    lines = houghlines(BW,theta,rho,P,'FillGap',50,'MinLength',15);
    %Minimum -lines shorter than that are omitted
    %When the distance between the line segments is less than the value
    %specified, the houghlines function merges the line segments into
    %a single line segment.
    
    figure(01)
    imshow(BW), hold on
    pp=1;
    q=1;
    
    for k = 1:length(lines)
        xy(k,:) = [lines(k).point1,lines(k).point2];
        theta=lines(k).theta;
        if theta>0
            L(pp,:)=xy(k,:);
            pp=pp+1;
        else
            R(q,:)=xy(k,:);
            q=q+1;
        end
    end
    Lx=[L(:,1);L(:,3)];
    Ly=[L(:,2);L(:,4)];
    Rx=[R(:,1);R(:,3)];
    Ry=[R(:,2);R(:,4)];
    if i==1
        X=[Left,Right]';
    end
    Left=polyfit(Lx,Ly,1);
    Right=polyfit(Rx,Ry,1);
    temp(i)=Right(2);
    
    %     Kalman Filter
    if i~=1
        X0=X;
        X1=[Left,Right]';
        EC=EC+EQ;
        K=EC*pinv(EC+ER);
        X=X0+K*(X1-X0);
        EC=(eye(4)-K)*EC;
        Left=[X(1),X(2)];
        Right=[X(3),X(4)];
    end
    %         X1=[Left,Right]';
    %     EC=EC+EQ;
    %     K=EC*pinv(EC+ER);
    %     X=X0+K*(X1-X0);
    %     EC=(eye(4)-K)*EC;
    %     Left=[X(1),X(2)];
    %     Right=[X(3),X(4)];
    %     Kalman Filter End
    Lxx=min(Lx):max(Lx);
    Rxx=min(Rx):max(Rx);
    Mx(1)=0.5*(min(Lx)+max(Rx));
    Mx(2)=0.5*(max(Lx)+min(Rx));
    My(1)=0.5*(polyval(Left,min(Lx))+polyval(Right,max(Rx)));
    My(2)=0.5*(polyval(Left,max(Lx))+polyval(Right,min(Rx)));
    Middle=polyfit(Mx,My,1);
    Mxx=min(Mx):0.1:max(Mx);
    plot(Lxx,polyval(Left,Lxx),'LineWidth',2,'Color','green')
    plot(Rxx,polyval(Right,Rxx),'LineWidth',2,'Color','green')
    plot(Mx,polyval(Middle,Mx),'LineWidth',2,'Color','blue')
    Xr=sh(2)*0.5;
    Yr=sh(1)*0.5;
    m=Middle(1);
    plot([Xr,Xr],[0,Yr],'LineWidth',2,'Color','red');
    angle=sign(m)*(90-sign(m)*atand(m))/90;
    Xm=((Yr)-Middle(2))/Middle(1);
    Xl=((Yr)-Left(2))/Left(1);
    Xri=((Yr)-Right(2))/Right(1);
    D=(Xr-Xm)
    Steer=-K1*angle+K2*D;
    Steer=(Steer+90)/180;
    writePosition(V,vel);
    writePosition(S,Steer);
    drawnow
end
