load('stop.mat');
negativeFolder = 'NegativeImages_stopSign';
trainCascadeObjectDetector('xyz.xml', STOP, negativeFolder, 'FalseAlarmRate', 0.3, 'NumCascadeStages', 5);

detector = vision.CascadeObjectDetector('xyz.xml');
img = imread('test.jpg');
gray = rgb2gray(img);
bbox = step(detector,gray);
detectedImg = insertObjectAnnotation(img,'rectangle',bbox,'stop sign');
figure; imshow(detectedImg);
