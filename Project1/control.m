function [phi,v] = control(mid,Cpos,k1,k2)

%Control from slides 

Xcf = mid(1,1);
Zcf = mid(1,3);
Xcn = mid(2,1);
Zcn = mid(2,3);

theta = -atan((Zcf-Zcn)/(Xcf-Xcn))-90;

D = Zcn + (((Xcf-Xcn)/(Zcf-Xcn)).*(Xcn-Cpos));
phi = (-k1.*theta)+(-k2.*D);

%velocity v proportional to theta?
v = 1;

end