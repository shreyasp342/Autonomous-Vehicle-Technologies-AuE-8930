clear 
close all

% IPsystem1 = '192.168.1.249';
% IPsystem2 = '192.168.1.250';
% portSystem1 = 9090;
% portSystem2 = 9091;
% udpSystem2 = udp(IPsystem1,portSystem1,'LocalPort',portSystem2);
% fopen(udpSystem2);

cam = webcam(1);
cam.Resolution= '320x240';
videoFrame= snapshot(cam);
frameSize= size(videoFrame);
videoPlayer= vision.VideoPlayer('Position', [100 100 [frameSize(2), frameSize(1)]+30]);

falseStop = 0.1;
cascadeStop = 10;
falseSchool = 0.1;
cascadeSchool = 10;
xmlStop = char(strcat("stop1","_",num2str(falseStop),"_",num2str(cascadeStop),".xml"));           
xmlSchool = char(strcat("school1","_",num2str(falseSchool),"_",num2str(cascadeSchool),".xml"));
detectorStop = vision.CascadeObjectDetector(xmlStop);
detectorSchool = vision.CascadeObjectDetector(xmlSchool);

flagStop = false;
flagSchool = false;
flagNothing = false;

while (1)
    p=snapshot(cam);
    sh=size(p);
    p=imresize(p,0.25);
    gray = rgb2gray(p);
    
    bboxStop = step(detectorStop,gray);
    bbStop = size(bboxStop,1);
    videoFrameStop = insertObjectAnnotation(videoFrame,'rectangle',bboxStop,'stop sign');
    step(videoPlayer, videoFrameStop);
    
%     bboxSchool = step(detectorSchool,gray);    
%     bbSchool = size(bboxSchool,1);
%     videoFrameSchool = insertObjectAnnotation(videoFrame,'rectangle',bboxSchool,'stop sign');
%     step(videoPlayer, videoFrameSchool);
    
%     if(bbStop == 0 && bbSchool == 0)
%         falgNothing = true;
%     elseif(bbStop == 1 && bbSchool == 0)
%         flagStop = true;
%     elseif(bbStop == 0 && bbSchool == 1)
%         flagSchool = true;
%     else
%         % There are multiple bounding boxes in the scene. Reduce them.
%     end
    
%     if flagStop == true
%         fwrite(udpB,1);
%     elseif flagSchool == true
%         fwrite(udpSystem2,2);
%     else
%         fwrite(udpSystem2,0);
%     end    
    
end
