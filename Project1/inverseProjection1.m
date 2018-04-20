function mid = inverseProjection1(left, right, Zc)

% load('intrinsic.mat');
% load('extrinsic.mat');
load cam1.mat;
intrinsic = cameraParams.IntrinsicMatrix;
extrinsic = [[rotationMatrix;zeros(1,3)],[translationVector';1]];

Xc = Zc*(left(1)-intrinsic(1,3))./intrinsic(1,1);
Yc = Zc*(left(2)-intrinsic(2,3))./intrinsic(2,2);
Lf = extrinsic * [Xc;Yc;Zc;1];

Xc = Zc*(left(3)-intrinsic(1,3))./intrinsic(1,1);
Yc = Zc*(left(4)-intrinsic(2,3))./intrinsic(2,2);
Ln = extrinsic * [Xc;Yc;Zc;1];

Xc = Zc*(right(1)-intrinsic(1,3))./intrinsic(1,1);
Yc = Zc*(right(2)-intrinsic(2,3))./intrinsic(2,2);
Rf = extrinsic * [Xc;Yc;Zc;1];

Xc = Zc*(right(3)-intrinsic(1,3))./intrinsic(1,1);
Yc = Zc*(right(4)-intrinsic(2,3))./intrinsic(2,2);
Rn = extrinsic * [Xc;Yc;Zc;1];

Xcf = (Rf(1)+Lf(1))/2;
Zcf = (Rf(3)+Lf(3))/2;
Xcn = (Rn(1)+Ln(1))/2;
Zcn = (Rn(3)+Ln(3))/2;
% 
% Xcf = (Xrfc+Xlfc)/2;
% Zcf = (Zrfc+Zlfc)/2;
% 
% Xcn = (Xrnc+Xlnc)/2;
% Zcn = (Zrnc+Zlnc)/2;

mid = [Xcf,Yc,Zcf;Xcn,Yc,Zcn];
clf
% figure;
hold on;
plot([Rf(1),Rn(1)],[Rf(2),Rn(2)]);
plot([Lf(1),Ln(1)],[Lf(2),Ln(2)]);
plot([mid(1,1),mid(2,1)],[mid(1,3),mid(2,3)]);
pause(0.05);
end