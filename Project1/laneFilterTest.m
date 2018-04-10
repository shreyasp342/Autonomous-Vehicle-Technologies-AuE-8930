% %Edge Detection
% I = imread('../exit-ramp.jpg');
% Ig = rgb2gray(I);
% 
% Ie = edge(Ig,'Canny',[0.31,0.36]);
% 
% imshow(Ie);

% v = VideoReader('sample_video.mp4');
% while hasFrame(v)
%     video = readFrame(v);
%     imshow(video);
% end



%edge detection
pic = imread('exit-ramp.jpg');
% pic = imread('highway-construction-service-500x500.jpg');
shape = size(pic);
gray_pic = rgb2gray(pic);
edge_pic = edge(gray_pic,'canny',[0.31,0.36]);

%region masking
target_x=0.4; 
target_y=0.6*shape(1);
a=[shape(2)*target_x,shape(2)*(1-target_x),shape(2),0];
b=[target_y,target_y,shape(1),shape(1)];
bw=roipoly(pic,a,b);
BW=(edge_pic(:,:,1)&bw);
imshow(BW);

%hough line detection
[H,T,R] = hough(BW);
P=houghpeaks(H,3);
lines = houghlines(BW,T,R,P,'FillGap',4,'MinLength',5);
imagesc(pic);
hold on;
for i= 1:length(lines)
    plot([lines(i).point1(1),lines(i).point2(1)],[lines(i).point1(2),lines(i).point2(2)],'LineWidth',2,'Color','red');
end