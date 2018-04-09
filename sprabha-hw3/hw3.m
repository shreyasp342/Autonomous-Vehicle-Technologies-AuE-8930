close all
clear all
load s1.mat
load s2.mat

s{1} = s1;
s{2} = s2;
clear s1 s2

mu0 = [0, 0, 0, 147, 102, 98, 53]';
lengthMu = size(mu0,1);
numLandmarks = (lengthMu - 3)/2;
sigma0 = zeros(lengthMu);
for i = (lengthMu - (numLandmarks*2) + 1):7
    sigma0(i,i) = 10^150;
end

vt = 1; %velocity in  m/s
wt = 10^-150;%radial velocity (straight line)
dt = 1;%sampling time in s

R = zeros(lengthMu);
for i=1:3
    R(i,i) = 0.1;
end
Q = zeros(lengthMu);
Q((lengthMu - (numLandmarks*2) + 1): lengthMu, (lengthMu - (numLandmarks*2) + 1): lengthMu) = diag([0.1,0.01,0.1,0.01]);

for i = 1:size(s{1},2)
    Qt{i} = Q(3+((i-1)*2+(1:2)),3+((i-1)*2+(1:2)));
end

mut1 = mu0;
sigmat1 = sigma0;
    Fx = [eye(3),zeros(3,numLandmarks*2)];
    vw = vt/wt;
    wdt = wt*dt;
    
    
for i = 1:size(s{1},1)
    %%
    %Prediction
    
    temp = [(-vw*sin(mut1(3)))+(vw*sin(mut1(3)+wdt));
        (-vw*sin(mut1(3)))+(vw*sin(mut1(3)+wdt));
        wdt];
    mut = mut1 + Fx' * temp;
    temp(3,1) = 0;
    temp = [zeros(3,2), temp];
    Gt = eye(lengthMu) + Fx' * temp * Fx; 
    sigmat = (Gt*sigmat1*Gt') + R;
    
    %%
    %Correction
    for j = 1:numLandmarks
     delta = [mut(3+((j-1)*2+(1:2))) - mut(1:2)];
     q = delta' * delta;
     zt = [sqrt(q); atan2(delta(2),delta(1))-mut(3)];
     Fxj = [Fx; zeros(2,3),zeros(2,2*j-2),eye(2),zeros(2,2*size(s{1},2)-2*j)];  
     Ht = (1/q .* [-sqrt(q)*delta(1), -sqrt(q)*delta(2), 0, sqrt(q)*delta(1), sqrt(q)*delta(2); delta(2), -delta(1), -q, -delta(2), delta(1)]) * Fxj;
     Kt = sigmat*Ht'*pinv(Ht*sigmat*Ht' + Q(3+((j-1)*2+(1:2)),3+((j-1)*2+(1:2))));
     
     mut = mut + (Kt*(s{j}(i,:)' - zt));
     sigmat = (eye(7) - Kt*Ht)*sigmat;
     clear delta q zt Fxj Ht Kt
    end
    
    mut1 = mut;
    sigmat1 = sigmat;
    mu{i} = mut;
    muu(i,:) = mut;
    sigma{i} = sigmat;
    clear mut sigmat
     
    
end

figure;
hold on
    
    plot(mu0(1), mu0(2), 'kd', 'LineWidth',2,'MarkerSize',10);
    plot(mu0(4), mu0(5), 'bd', 'LineWidth',2,'MarkerSize',10);
    plot(mu0(6), mu0(7), 'rd', 'LineWidth',2,'MarkerSize',10);
    
    plot(muu(100,1), muu(100,2), 'ko', 'LineWidth',1.5,'MarkerSize',10);
    plot(muu(100,4), muu(100,5), 'bo', 'LineWidth',1.5,'MarkerSize',10);
    plot(muu(100,6), muu(100,7), 'ro', 'LineWidth',1.5,'MarkerSize',10);
    
for i = 1:size(muu,1)
    plot(muu(i,1), muu(i,2), 'k*','MarkerSize',5);
    plot(muu(i,4), muu(i,5), 'b*','MarkerSize',5);
    plot(muu(i,6), muu(i,7), 'r*','MarkerSize',5);
end
hold off
title("map");
legend("vehicle initial location", "landmark 1 initial location","vehicle final location", "landmark 1 final location", "landmark 2 final location", "landmark 2 initial location", "vehicle", "landmark 1", "landmark 2",'Location','northwest');

clear mu
mu = muu;
T = table(mu);
writetable(T,'mu.txt','Delimiter', ' ')


