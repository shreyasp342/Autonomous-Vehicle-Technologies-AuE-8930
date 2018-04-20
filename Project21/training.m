clear
close all
load('stoptable.mat');
negativeFolder = imageDatastore({'NegativeImages_stopSign\*.jpg','school\*.jpg','other\*.jpg'});
% negativeFolder = ['NegativeImages_stopSign','school'];
trainCascadeObjectDetector('xyz.xml', stop1, negativeFolder, 'FalseAlarmRate', 0.1, 'NumCascadeStages', 20);

% imds = imageDatastore({'NegativeImages_stopSign\*.jpg','school\*.jpg'});
detector = vision.CascadeObjectDetector('xyz.xml');
% img = imread('stop\WIN_20180417_17_08_25_Pro.jpg');
img = imread('school.jpg');
gray = rgb2gray(img);
bbox = step(detector,gray);
detectedImg = insertObjectAnnotation(img,'rectangle',bbox,'stop sign');
figure; imshow(detectedImg);

img = imread('test.jpg');
gray = rgb2gray(img);
bbox = step(detector,gray);
detectedImg = insertObjectAnnotation(img,'rectangle',bbox,'stop sign');
figure; imshow(detectedImg);
