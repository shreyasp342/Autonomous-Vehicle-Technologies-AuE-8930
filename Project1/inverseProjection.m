function mid = inverseProjection(left, right, Yc)

% load('intrinsic.mat');
% load('extrinsic.mat');
load cam1.mat;
intrinsic = cameraParams.IntrinsicMatrix;
extrinsic = [[rotationMatrix;zeros(1,3)],[translationVector';1]];

Zc = (intrinsic(2,2)*Yc)./(left(2)-intrinsic(2,3));
Xc = Zc*(left(1)-intrinsic(1,3))./intrinsic(1,1);
Lf = extrinsic * [Xc;Yc;Zc;1];

Zc = (intrinsic(2,2)*Yc)./(left(4)-intrinsic(2,3));
Xc = Zc*(left(3)-intrinsic(1,3))./intrinsic(1,1);
Ln = extrinsic * [Xc;Yc;Zc;1];

Zc = (intrinsic(2,2)*Yc)./(right(2)-intrinsic(2,3));
Xc = Zc*(right(1)-intrinsic(1,3))./intrinsic(1,1);
Rf = extrinsic * [Xc;Yc;Zc;1];

Zc = (intrinsic(2,2)*Yc)./(right(4)-intrinsic(2,3));
Xc = Zc*(right(3)-intrinsic(1,3))./intrinsic(1,1);
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

% 
% clf
% % figure;
% hold on;
% plot([Rf(1),Rn(1)],[Rf(3),Rn(3)]);
% plot([Lf(1),Ln(1)],[Lf(3),Ln(3)]);
% plot([mid(1,1),mid(2,1)],[mid(1,3),mid(2,3)]);
% pause(0.05);


end