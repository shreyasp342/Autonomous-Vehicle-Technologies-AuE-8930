function [phi] = control(mid,Cpos,k1,k2)
% function [phi,v] = control(mid,Cpos,k1,k2)

%Control from slides 
% 
Xcf = mid(1,1);
Zcf = mid(1,3);
Xcn = mid(2,1);
Zcn = mid(2,3);

% Xcf = mid(1,1);
% Zcf = mid(1,2);
% Xcn = mid(1,3);
% Zcn = mid(1,4);

% theta = -atan((Zcf-Zcn)/(Xcf-Xcn))-90;
theta = -atan((Zcf-Zcn)/(Xcf-Xcn))-90;

D = Zcn + (((Xcf-Xcn)/(Zcf-Xcn)).*(Xcn-Cpos));
phi = (-k1.*theta)+(-k2.*D);

while phi >= 90 || phi<= -90
    if phi > 90
        phi = phi -90;
    end
    if phi < -90
        phi = phi+90;
    end
end
    

%velocity v proportional to theta?
v = 1;

end