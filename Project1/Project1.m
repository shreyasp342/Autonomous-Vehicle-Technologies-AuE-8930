close all
clear



% % auE 8930 - Team 8
% % Project 1
% 
% Steps - 
% 1. Caliberation - 
%     Intrinsic Parameters
%     Extrinsic parameters
% 2. Hough transform
%     Get the center line of Lane
%     
% 3. Yc = Hc (height) - set manually
%     Get Xc, Zc
%     
%     Set velocity based on slop of line found above
% 3. Kalman Filter ( I guess, not sure how to and where to use). Optional but highly recommended
% 4. Inverse Projection (whatever that means)
% 5. Motion Control
%     options - 
%         Pure pursuit
%         Stanley
%         Modified Stanley
%         Some basic one from slides (with that theta and D?)
        
%% Initialization

k1=1;
k2=1;
%height 
Hc = 10;
Yc = Hc;
Cpos = 0;
% 
% A = [1 1 0 0; 0 1 0 0; 0 0 1 1; 0 0 0 1];
% H = [1 0 0 0; 0 0 1 0];
% kalmanLf = vision.KalmanFilter(A,H);
% kalmanRf = vision.KalmanFilter(A,H);
% kalmanLn = vision.KalmanFilter(A,H);
% kalmanRn = vision.KalmanFilter(A,H);

left = zeros(1,4);
right = zeros(1,4);

param = getDefaultParameters();         % get parameters that work well
param.motionModel = 'ConstantVelocity'; % switch from ConstantAcceleration
                                        % to ConstantVelocity
% After switching motion models, drop noise specification entries
% corresponding to acceleration.
param.initialEstimateError = param.initialEstimateError(1:2);
param.motionNoise          = param.motionNoise(1:2);

initialLocation = [0,0];
kalmanFilterL = configureKalmanFilter(param.motionModel, ...
          initialLocation, param.initialEstimateError, ...
          param.motionNoise, param.measurementNoise);
kalmanFilterR = configureKalmanFilter(param.motionModel, ...
          initialLocation, param.initialEstimateError, ...
          param.motionNoise, param.measurementNoise);


% Cam=webcam(1);
% % Cam.Resolution='1920x1080';
% a=arduino('COM5','UNO','Libraries','servo');
% V=servo(a,'D12','MinPulseDuration',1e-3,'MaxPulseDuration',2e-3);
% S=servo(a,'D13','MinPulseDuration',1e-3,'MaxPulseDuration',2e-3);
% writePosition(V, 0.5);
% writePosition(S, 0.5);
iter = 0;

%% Caliberation
% load('intrinsic.mat');
% load('extrinsic.mat');

%%
v = VideoReader('sample_video.mp4');

% figure;
while hasFrame(v)
    video = readFrame(v);
% while (1)
    tic
%     video=snapshot(Cam);
%     video=imresize(video,1);
    % 1. Get left and right line of lane in (u,v) coordinates
%     lf = predict(kalmanLf);
%     rf = predict(kalmanRf);
%     ln = predict(kalmanLn);
%     rn = predict(kalmanRn);
    
    [left1, right1] = Hough(video);
    
    if(sum(isnan(right1)) == 0)
        right = right1;
    end
    if(sum(isnan(left1)) == 0)
        left = left1;
    end  
%     lf = correct(kalmanLf,left(1:2));
%     rf = correct(kalmanRf,right(1:2));
%     ln = correct(kalmanLn,left(3:4));
%     rn = correct(kalmanRn,right(3:4));
%     left = [lf,ln];
%     right = [rf,rn];

%         predict(kalmanFilter);
%         trackedLocation = correct(kalmanFilter, [left,right]);
%         left = trackedLocation(1:4);
%         right = trackedLocation(5:8);
    
    pl1 = polyfit([left(2),left(4)],[left(1),left(3)],1);
    pl = pl1;
%         predict(kalmanFilterL);
%         pl = correct(kalmanFilterL, pl1);
    L(2) = min(left(2),right(2));
    L(4) = max(left(4),right(4));
    L(1) = polyval(pl,L(2));
    L(3) = polyval(pl,L(4));
    
    pr1 = polyfit([right(2),right(4)],[right(1),right(3)],1);
    pr = pr1;
%         predict(kalmanFilterR);
%         pr = correct(kalmanFilterR, pr1);
    R(2) = L(2);
    R(4) = L(4);
    R(1) = polyval(pr,R(2));
    R(3) = polyval(pr,R(4));
    
    mid = (L+R)/2;
    
%     clf;
%     imshow(video);
%     hold on
%     plot([0,size(video,2)],[size(video,1)/2,size(video,1)/2],'LineWidth',2,'Color','red');
%     plot([size(video,2)/2,size(video,2)/2],[0,size(video,1)],'LineWidth',2,'Color','red');
%     plot([left(1),left(3)],[left(2),left(4)],'LineWidth',2,'Color','blue');
%     plot([right(1),right(3)],[right(2),right(4)],'LineWidth',2,'Color','blue');
%     plot([L(1),L(3)],[L(2),L(4)],'LineWidth',2,'Color','black');
%     plot([R(1),R(3)],[R(2),R(4)],'LineWidth',2,'Color','black');
%     
%     mid = (left+right)/2;
%     plot([mid(1),mid(3)],[mid(2),mid(4)],'LineWidth',2,'Color','blue');
    m = atan((mid(4)-mid(2))/(mid(3)-mid(1)));
    
%     mid = [mid(1), mid(2);mid(3),mid(4)];
    
%     4. Inverse Projection
    mid = inverseProjection(left, right, Yc);
    
%     5. Control
%     [phi,vel] = control(mid, Cpos,k1,k2);
%     [phi] = control(mid, Cpos,k1,k2);
%     phi = (phi+90)/90;
%     
%     pm = polyfit([mid(2),mid(4)],[mid(1),mid(3)],1);
%     
%     
% %     writePosition(V,vel);
%     if iter >= 5
%         vel = 0.7;
%         iter = 0;
%     else
%         vel = 0.5;
%         iter = iter+1;
%     end
% %     writePosition(S,phi);
% %     writePosition(V,vel);
% 
% iter
% phi
%     pause(0.05);
% %     clf;
% %     hold off
%     
    toc
%     
end
    