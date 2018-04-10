clc
clear
close all
i=1;K1=0.08;K2=1;
Cam=webcam(1);
Cam.Resolution='640x480';
a=arduino('COM5','UNO','Libraries','servo');
V=servo(a,'D13','MinPulseDuration',1e-3,'MaxPulseDuration',2e-3);
S=servo(a,'D12','MinPulseDuration',1e-3,'MaxPulseDuration',2e-3);
writePosition(V, 0.5);
writePosition(S, 0.5);
pause(0.05);
Left =[];Right =[];L=[];R=[];Ltheta=zeros(7,1);Rtheta=Ltheta;
vel=0.556;
%KALMAN FILTER FOR STEERING
PS=0;QS=0.003;RS=0.003;Steer=0.37;


% Errors for Kalman gain for Left and Right Lanes;
% EC=zeros(4,4);
% EQ=[0.05,0.0,0.0,0.0;
%     0,0.09,00,00;
%     0.0,0.0,0.08,0.0;
%     0,00,0,0.0001;];
% ER=[29,0.0,0.0,0.0;
%     0,0.01,00,00;
%     0.0,0,29,0.0;
%     0,00,0,0.02;];

% %Kalman gains for Middle Lane;
% EC=zeros(2,2);
% EQ=[1,0;0,1];ER=[1,0;0,1];
while (1)
    tic
    p=snapshot(Cam);
    p=imresize(p,1);
    shape=size(p);
    p1=rgb2gray(p);
    %detect edges
    for q=1:shape(2)
        for j=1:shape(1)
            if p1(j,q)<220
                p1(j,q)=0;
            end
        end
    end
    
    BW=edge(p1,'canny',[0.31,0.36]);
    %Region Masking
    a=[shape(2)*0.5/6,shape(2)*5/6,shape(2),0];
    b=[shape(1)*2/7,shape(1)*2/7,shape(1),shape(1)];
    bw=roipoly(p,a,b);
    BW=(BW(:,:)&bw);
    pp=1;qq=1;pt=1;qt=1;
    [H,theta,rho]=hough(BW,'Theta',-60:60); %Transform Display
    
    P = houghpeaks(H,4,'threshold',0.3*max(max((H))));
    %first number of peaks to identify Threshold- minimum value to be
    %considered a peak
    
    lines = houghlines(BW,theta,rho,P,'FillGap',20,'MinLength',20);
    %Minimum -lines shorter than that are omitted
    %When the distance between the line segments is less than the value
    %specified, the houghlines function merges the line segments into
    %a single line segment.
    
   figure(01)
    imshow(p), hold on
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
%     if length(Ltheta)>1
%         Lsigma=std(Ltheta);
%         Lbar=mean(Ltheta);
%         Ldiff=abs(Lbar-Ltheta);
%         CheckL=abs(Ldiff<Lsigma)
%         L(CheckL,:)=[];
%     end
%     if length(Rtheta)>1
%         Rsigma=std(Rtheta);
%         Rbar=mean(Rtheta);
%         Rdiff=abs(Rbar-Rtheta);
%         RCheck=abs(Rdiff<Lsigma)
%         L(RCheck,:)=[];
%     end
    %     if i~=1
    %         X=[Left,Right]';
    %     end
    if ~isempty(L)
        Lx=L(:,1);
        Ly=L(:,2);
        Left=polyfit(L(:,1),L(:,2),1);
    end
    if ~isempty(R)
        Rx=R(:,1);
        Ry=R(:,2);
        Right=polyfit(R(:,1),R(:,2),1);
    end
%     %         Kalman Filter for Left and Right
%     if i==1
%         X=[Left,Right]';
%     end
%     X0=X;
%     X1=[Left,Right]';
%     EC=EC+EQ;
%     K=EC/(EC+ER);
%     X=X0+K*(X1-X0);
%     EC=(eye(4)-K)*EC;
%     Left=[X(1),X(2)];
%     Right=[X(3),X(4)];
    %     Kalman Filter End Left and Right
    Lxx=min(Lx):max(Lx);
    Rxx=min(Rx):max(Rx);
    if ~isempty(Lx)
        if ~isempty(Rx)
    Mx(1)=0.5*(min(Lx)+max(Rx));
    Mx(2)=0.5*(max(Lx)+min(Rx));
    My(1)=0.5*(polyval(Left,min(Lx))+polyval(Right,max(Rx)));
    My(2)=0.5*(polyval(Left,max(Lx))+polyval(Right,min(Rx)));
    Middle=polyfit(Mx,My,1);
        end
    end
        %     Kalman gain for Middle Line
    %     if i~=1
    %         X0=X;
    %         X1=[Middle]';
    %         EC=EC+EQ;
    %         K=EC*pinv(EC+ER);
    %         X=X0+K*(X1-X0);
    %         EC=(eye(2)-K)*EC;
    %         Middle=X';
    %     end
    Mxx=min(Mx):0.1:max(Mx);
    plot(Lxx,polyval(Left,Lxx),'LineWidth',2,'Color','green')
    plot(Rxx,polyval(Right,Rxx),'LineWidth',2,'Color','green')
    plot(Mx,polyval(Middle,Mx),'LineWidth',2,'Color','blue')
    Xr=shape(2)*0.5;
    Yr=shape(1)*0.5;
    m=Middle(1);
    plot([Xr,Xr],[0,Yr],'LineWidth',2,'Color','red');
    %Control Logic
    if (max(Mx)-min(Mx))==0
        angle=0;
    else
        angle=atan((max(Mx)-min(Mx))/(polyval(Middle,max(Mx))-polyval(Middle,min(Mx))));
    end
    %distance
    Xm=((Yr*0.5)-Middle(2))/Middle(1);
    Xl=((Yr*0.5)-Left(2))/Left(1);
    Xri=((Yr*0.5)-Right(2))/Right(1);
    D=(Xr-Xm)/(Xl-Xri);
    
    Sp=Steer;
    %Steer Angle
    K2=1.98;
    Steer=0.48+0.1*(K1*angle+K2*D);
    
%     if rem(i,10)==0
%         PS=0;
%     end
      %yyyyyzzz Kalman Filter
%     %prediction
%     PS=PS+QS;
%     %correction
%     K=PS/(PS+RS);
%     Steer=Sp+K*(Steer-Sp);
%     PS=(1-K)*PS;
    if rem(i,5)==0
        vel=0.555;
    else
        vel=0.559;
    end
    
    if Steer>0.55
        Steer=0.55;
    elseif Steer<0.25
        Steer=0.25;
    end
    writePosition(V,vel);
    Steer;
    writePosition(S,Steer);
    pause(0.01);
    drawnow
    i=i+1;
    toc
end

