function x = param(worldXYZ,imageUV)

for i = 1 : size(worldXYZ,2)
    A((i*2)-1,:) = [worldXYZ{1,i}, 1, 0, 0, 0, 0, -(worldXYZ{1,i}*imageUV{1,i}(1))];
    A((i*2),:) = [0, 0, 0, 0, worldXYZ{1,i}, 1, -(worldXYZ{1,i}*imageUV{1,i}(2))];
    B((i*2)-1,:) = imageUV{1,i}(1);
    B((i*2),:) = imageUV{1,i}(2);    
end
x = pinv(A)*B;

end