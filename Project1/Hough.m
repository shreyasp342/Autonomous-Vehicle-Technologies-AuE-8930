function [left, right] = Hough(pic)
%edge detection
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
% imshow(BW);

%hough line detection
[H,T,R] = hough(BW);
P=houghpeaks(H,3);
lines = houghlines(BW,T,R,P,'FillGap',4,'MinLength',5);
% imagesc(pic);
% hold on;
% for i= 1:length(lines)
%     plot([lines(i).point1(1),lines(i).point2(1)],[lines(i).point1(2),lines(i).point2(2)],'LineWidth',2,'Color','red');
% end

%group lines with same theta
thetas = [];
linePoints = {};
for i = 1:length(lines)
    foundTheta = false;
    for j = 1:length(thetas)
        if thetas(j) == lines(i).theta
            linePoints{j} = [linePoints{j};lines(i).point1, lines(i).point2];
            foundTheta = true;
            break;
        end
    end
    if foundTheta == false
        thetas = [thetas,lines(i).theta];
        linePoints{length(thetas)} = [lines(i).point1, lines(i).point2];
    end
end

%merge the grouped lines - i.e. extend the line.
line = zeros(length(thetas),4);
for i = 1:length(thetas)
    dist = 0;
    for j = 1: size(linePoints{i},1)
        for k = 1: size(linePoints{i},1)
            dist1 = sqrt((linePoints{i}(j,1)-linePoints{i}(k,3)).^2 + (linePoints{i}(j,2)-linePoints{i}(k,4)).^2);
            if dist < dist1
                line(i,:) = [linePoints{i}(j,1), linePoints{i}(j,2), linePoints{i}(k,3), linePoints{i}(k,4)];
                dist = dist1;
            end
        end
    end
    
end

% merge multiple left and right lines to get single left and single right
% line
thetaPos = thetas > 0;
thetaNeg = thetas < 0;
left = [sum(line(thetaPos,1))/sum(thetaPos), sum(line(thetaPos,2))/sum(thetaPos), sum(line(thetaPos,3))/sum(thetaPos), sum(line(thetaPos,4))/sum(thetaPos)];
right = [sum(line(thetaNeg,1))/sum(thetaNeg), sum(line(thetaNeg,2))/sum(thetaNeg), sum(line(thetaNeg,3))/sum(thetaNeg), sum(line(thetaNeg,4))/sum(thetaNeg)];
            
% plot([left(1),left(3)],[left(2),left(4)],'LineWidth',2,'Color','blue');
% plot([right(1),right(3)],[right(2),right(4)],'LineWidth',2,'Color','blue');
  
end