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
%     Yc = Hc (height) - set manually
%     Get Xc, Zc
%     
%     Set velocity based on slop of line found above
% 3. Kalman Filter ( I guess, not sure). Optional but highly recommended
% 4. Inverse Projection (whatever that means)
% 5. Motion Control
%     options - 
%         Pure pursuit
%         Stanley
%         Modified Stanley
%         Some basic one from slides (with that theta and D?)
        
%% Caliberation
% load('intrinsic.mat');
% load('extrinsic.mat');

%height 
Hc = 10;
Yc = Hc;

v = VideoReader('sample_video.mp4');
left = zeros(1,4);
right = zeros(1,4);
while hasFrame(v)
    video = readFrame(v);
    % Get left and right line of lane in (u,v) coordinates
    [left1, right1] = Hough(video);
    if(sum(isnan(right1)) == 0)
        right = right1;
    end
    if(sum(isnan(left1)) == 0)
        left = left1;
    end    
    imshow(video);
    hold on
    plot([left(1),left(3)],[left(2),left(4)],'LineWidth',2,'Color','blue');
    plot([right(1),right(3)],[right(2),right(4)],'LineWidth',2,'Color','blue');
%     pause(0.05);
    clf;
end
    