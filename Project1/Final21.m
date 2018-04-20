clc
clear
close all

i=1;
maxD = 0;
minD= 10000000;
maxA = 0;
minA= 10000000;
maxS = 0;
minS= 10000000;
% Cam=webcam(1);
% Cam.Resolution='352x288';
% % {'640x480','160x120','176x144','320x240','352x288','800x600','1280x720','1920x1080'}
% a=arduino('COM5','UNO','Libraries','servo');
% V=servo(a,'D13','MinPulseDuration',1e-3,'MaxPulseDuration',2e-3);
% S=servo(a,'D12','MinPulseDuration',1e-3,'MaxPulseDuration',2e-3);
% writePosition(V, 0.5);
% writePosition(S, 0.5);
% pause(0.05);

Left =[];Right =[];L=[];R=[];Ltheta=zeros(7,1);Rtheta=Ltheta;
vel=0.575;
Mx=zeros(1,2);
My=Mx;
% %KALMAN FILTER FOR STEERING
% PS=0;QS=0.003;RS=0.003;Steer=0.37;


% Errors for Kalman gain for Left and Right Lanes;
EC=[0.001,0.0,0.0,0.0;
    0,0.005,00,00;
    0.0,0.0,0.001,0.0;
    0,00,0,0.0005;];
EP=EC;
EQ=0.035*[0.001,0.0,0.0,0.0;
    0,0.00655,00,00;
    0.0,0.0,0.001,0.0;
    0,00,0,0.00654;];
ER=0.00008*[1.2,0.0,0.0,0.0;
    0,2,00,00;
    0.0,0,1.2,0.0;
    0,00,0,2;];

v = VideoReader('sample_video.mp4');
figure;
while hasFrame(v)
    tic
    p = readFrame(v);


% while (1)
%     tic
%     p=snapshot(Cam);
%     p=imresize(p,1);
    shape=size(p);
    
    p1=rgb2gray(p);
% % % % %     detect edges
%     for q=1:shape(2)
%         for j=1:shape(1)
%             if p1(j,q)<150
%                 p1(j,q)=0;
%             end
%         end
%     end
%     
%     BW1=edge(p1,'canny',[0.3,0.95]);
    BW1=edge(p1,'canny',[0.31,0.35]);
    %Region Masking
    a1=0.01/6;
    a2=5/6;
    b1=3.5/7;
    b2=b1;
    a=[shape(2)*a1,shape(2)*a2,shape(2),0];
    b=[shape(1)*b1,shape(1)*b2,shape(1),shape(1)];
    bw=roipoly(p,a,b);
    BW=(BW1(:,:)&bw);
    pp=1;qq=1;pt=1;qt=1;
    [H,theta,rho]=hough(BW,'Theta',-60:60); %Transform Display
    
    P = houghpeaks(H,4,'threshold',0.3*max(max((H))));
    %first number of peaks to identify Threshold- minimum value to be
    %considered a peak
    
    lines = houghlines(BW,theta,rho,P,'FillGap',20,'MinLength',5);
    %Minimum -lines shorter than that are omitted
    %When the distance between the line segments is less than the value
    %specified, the houghlines function merges the line segments into
    %a single line segment.
    
    
    xy=zeros(length(lines),4);
    theta=zeros(length(lines),1);
    for ui=1:length(lines)
        theta(ui)=lines(ui).theta;
    end
    TL=theta>=0;
    Tl=sum(TL);
    Tr=length(lines)-Tl;
    L=zeros(2*Tl,2);R=zeros(2*Tr,2);Ltheta=zeros(Tl);Rtheta=zeros(Tr);
    for k = 1:length(lines)
        xy(k,:) = [lines(k).point1,lines(k).point2];
        theta=lines(k).theta;
        if theta>=0
            L(pp,:)=[lines(k).point1];
            L(pp+1,:)=[lines(k).point2]; %points for left line
            Ltheta(pt)=[lines(k).theta];
            pp=pp+2;pt=pt+1;
        else
            R(qq,:)=[lines(k).point1];
            R(qq+1,:)=[lines(k).point2];   %points for right line
            Rtheta(qt)=[lines(k).theta];
            qq=qq+2;qt=qt+1;
        end
    end
    if ~isempty(L)
        Lx=L(:,1);
        Ly=L(:,2);
        Left=polyfit(L(:,1),L(:,2),1);
    else
        Lx = 0;
        Ly = 1;
        Left=polyfit(L(:,1),L(:,2),1);
    end
    if ~isempty(R)
        Rx=R(:,1);
        Ry=R(:,2);
        Right=polyfit(R(:,1),R(:,2),1);
    else
        Rx = 0;
        Ry = 1;
        Right=polyfit(R(:,1),R(:,2),1);
    end
    Lxx=min(Lx):max(Lx);
    Rxx=min(Rx):max(Rx);
%     %%     Kalaman Filter for Left and Right
    if i==1
        X=[Left,Right]';
    end
    X0=X;%prediction
    X1=[Left,Right]';%measurement
    EC=EC+EQ;%error covariance
    K=EC/(EC+ER);%kalman gain
    X=X0+K*(X1-X0);%correction
    EC=(eye(4)-K)*EC;%error
    Left=[X(1),X(2)];
    Right=[X(3),X(4)];
%     %kalman filter for left and right end
    
    if ~isempty(Lx)
        if ~isempty(Rx)
            %MID POINT CALCULATION
            %             Mx(1)=0.5*(min(Lx)+max(Rx));
            %             Mx(2)=0.5*(max(Lx)+min(Rx));
            %             My(1)=0.5*(polyval(Left,min(Lx))+polyval(Right,max(Rx)));
            %             My(2)=0.5*(polyval(Left,max(Lx))+polyval(Right,min(Rx)));
            %             Middle=polyfit(Mx,My,1);
            
            %ALTERNATE MIDPOINT CALCULATION
            Mlx1=(b1*shape(1)-Left(2))/Left(1);
            Mly1=b1*shape(1);
            Mlx2=(shape(1)-Left(2))/Left(1);
            Mly2=shape(1);
            Mrx1=(b1*shape(1)-Right(2))/Right(1);
            Mry1=b1*shape(1);
            Mrx2=(shape(1)-Right(2))/Right(1);
            Mry2=shape(1);
            Mx(1)=0.5*(Mlx1+Mrx1);
            Mx(2)=0.5*(Mlx2+Mrx2);
            My(1)=0.5*(Mly1+Mry1);
            My(2)=0.5*(Mly2+Mry2);
            Middle=polyfit(Mx,My,1);
        end
    end
    
    Xr=shape(2)*0.5;
    Yr=shape(1);
    m=Middle(1);
    
    %% Plotting
%             figure(01)
%             imshow(BW), hold on
%             plot(Lxx,polyval(Left,Lxx),'LineWidth',2,'Color','green')
%             plot(Rxx,polyval(Right,Rxx),'LineWidth',2,'Color','green')
%             plot(Mx,polyval(Middle,Mx),'LineWidth',2,'Color','blue')
%             plot([Xr,Xr],[0,Yr],'LineWidth',2,'Color','red');
%             pause(0.05);
    %%  Control Logic
    if (max(Mx)-min(Mx))==0
        angle=0;
    else
        angle=atand((max(Mx)-min(Mx))/(polyval(Middle,max(Mx))-polyval(Middle,min(Mx))));
    end
    %distance
    Xm=((Yr)-Middle(2))/Middle(1);
    Xl=((Yr)-Left(2))/Left(1);
    Xri=((Yr)-Right(2))/Right(1);
    D=(Xr-Xm)/(Xl-Xri);
    %Steer Angle
    %0.5<Steer<1 for RIGHT
    % 0<steer<0.5 for Left
    K1=0.00765;K2=0.8;
    asdaf=K1*angle+K2*D;
    Steer=0.5+0.5*(K1*angle+K2*D);
    Steer
    minA = min(angle,minA);
    maxA = max(angle,maxA);
    minD = min(D,minD);
    maxD = max(D,maxD);
    minS = min(Steer,minS);
    maxS = max(Steer,maxS);
    if Steer>0.8
        Steer=0.8;
    elseif Steer<0.0505
        Steer=0.1;
    end
%     writePosition(S,Steer);
%     writePosition(V,vel);
    drawnow
    vel=0.5688;
    i=i+1;
   toc
end

