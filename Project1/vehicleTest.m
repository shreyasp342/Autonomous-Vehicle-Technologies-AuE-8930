clear;

vel1 = 0.75;
iter = 0;
a=arduino('COM5','UNO','Libraries','servo');
V=servo(a,'D12','MinPulseDuration',1e-3,'MaxPulseDuration',2e-3);
S=servo(a,'D13','MinPulseDuration',1e-3,'MaxPulseDuration',2e-3);
writePosition(V, vel1);
writePosition(S, 0.5);

while(1)
    if iter >= 3
        iter = 0;
        vel = vel1;
    else
        iter = iter + 1;
        vel = 0.5;
    end
    writePosition(V, vel);
    writePosition(S, 0.5);
end
