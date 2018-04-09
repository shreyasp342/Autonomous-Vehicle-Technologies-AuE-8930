function mid = inverseProjection(left, right, Yc)

% load('intrinsic.mat');
% load('extrinsic.mat');

Zlf = (intrinsic(2,2)*Yc)./(left(2)-intrinsic(2,3));
Xlf = Zlf*(left(1)-intrinsic(1,3))./intrinsic(1,1);
Zln = (intrinsic(2,2)*Yc)./(left(4)-intrinsic(2,3));
Xln = Zlf*(left(3)-intrinsic(1,3))./intrinsic(1,1);

Zrf = (intrinsic(2,2)*Yc)./(right(2)-intrinsic(2,3));
Xrf = Zlf*(right(1)-intrinsic(1,3))./intrinsic(1,1);
Zrn = (intrinsic(2,2)*Yc)./(right(4)-intrinsic(2,3));
Xrn = Zlf*(right(3)-intrinsic(1,3))./intrinsic(1,1);

Xcf = (Xrf+Xlf)/2;
Zcf = (Zrf+Zlf)/2;

Xcn = (Xrn+Xln)/2;
Zcn = (Zrn+Zln)/2;

mid = [Xcf,Yc,Zcf;Xcn,Yc,Zcn];


end