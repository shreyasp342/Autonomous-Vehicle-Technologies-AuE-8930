close all
clear all

left = imread('Left.jpg');
right = imread('Right.jpg');

figure(1);imshow(left);
figure(2);imshow(right);

%%
%Data

%For question 1 and 2 - Calculating the parameters of camera

worldMoldXYZ{1,1} = [0,0,0];
worldMoldXYZ{1,2} = [0,35,0];
worldMoldXYZ{1,3} = [100,35,0];
worldMoldXYZ{1,4} = [100,0,0];
worldMoldXYZ{1,5} = [75,0,-40];
worldMoldXYZ{1,6} = [25,0,-40];
worldMoldXYZ{1,7} = [50,0,-30];

leftMoldUV{1,1} = [2544, 2423];
leftMoldUV{1,2} = [2563, 2350];
leftMoldUV{1,3} = [3060, 2375];
leftMoldUV{1,4} = [3064, 2456];
leftMoldUV{1,5} = [2937, 2637];
leftMoldUV{1,6} = [2678, 2617];
leftMoldUV{1,7} = [2808, 2587];

rightMoldUV{1,1} = [512, 3477];
rightMoldUV{1,2} = [680, 3352];
rightMoldUV{1,3} = [1240, 3584];
rightMoldUV{1,4} = [1094, 3744];
rightMoldUV{1,5} = [939, 3931];
rightMoldUV{1,6} = [652, 3789];
rightMoldUV{1,7} = [794, 3795];

%For Question 3 - Finding world coordinates of vertices of mold

leftMoldNewUV{1,1} = [2545, 2469];
leftMoldNewUV{1,2} = [3065, 2499];
leftMoldNewUV{1,3} = [2643, 2353];
leftMoldNewUV{1,4} = [2565, 2355];
leftMoldNewUV{1,5} = [2959, 2369];
leftMoldNewUV{1,6} = [2960, 2451];
leftMoldNewUV{1,7} = [2938, 2545];
leftMoldNewUV{1,8} = [2677, 2531];
leftMoldNewUV{1,9} = [2616, 2472];
leftMoldNewUV{1,10} = [2992, 2494];

rightMoldNewUV{1,1} = [513, 3550];
rightMoldNewUV{1,2} = [1099, 3811];
rightMoldNewUV{1,3} = [613, 3528];
rightMoldNewUV{1,4} = [783, 3402];
rightMoldNewUV{1,5} = [1125, 3536];
rightMoldNewUV{1,6} = [972, 3693];
rightMoldNewUV{1,7} = [933, 3806];
rightMoldNewUV{1,8} = [652, 3675];
rightMoldNewUV{1,9} = [586, 3574];
rightMoldNewUV{1,10} = [995, 3767];

%For question 4 - Finding the dimensions of the bar

leftBarUV{1,1} = [3253, 1708];
leftBarUV{1,2} = [3313, 1730];
leftBarUV{1,3} = [3384, 1716];
leftBarUV{1,4} = [3322, 1693];
leftBarUV{1,5} = [3259, 2286];
leftBarUV{1,6} = [3218, 2318];
leftBarUV{1,7} = [3387, 2298];

rightBarUV{1,1} = [1969, 2615];
rightBarUV{1,2} = [2002, 2678];
rightBarUV{1,3} = [2131, 2673];
rightBarUV{1,4} = [2094, 2615];
rightBarUV{1,5} = [1916, 3409];
rightBarUV{1,6} = [1948, 3489];
rightBarUV{1,7} = [2067, 3484];

%Question 5 - Finding the height of students

leftMaleUV{1,1} = [671, 224];
leftMaleUV{1,2} = [1140, 2102];
rightMaleUV{1,1} = [508, 129];
rightMaleUV{1,2} = [790, 2319];

leftFemaleUV{1,1} = [2001, 302];
leftFemaleUV{1,2} = [2117, 1973];
rightFemaleUV{1,1} = [2014, 389];
rightFemaleUV{1,2} = [1846, 2379];

%%
%Questions 1 and 2

b = param(worldMoldXYZ,leftMoldUV);
c = param(worldMoldXYZ,rightMoldUV);

%% 
%Question 3 - To find world coordinates of other vertices of Mold

for i = 1: size(rightMoldNewUV,2)
    worldMoldNewXYZ{1,i} = worldCoord(leftMoldNewUV{1,i}, b, rightMoldNewUV{1,i}, c);
end

%%
%Question 4 - Find the dimesions of the bar - w, h, l

for i = 1: size(rightBarUV,2)
    worldBarXYZ{1,i} = worldCoord(leftBarUV{1,i}, b, rightBarUV{1,i}, c);
end

AB = sqrt(sum((worldBarXYZ{1,1} - worldBarXYZ{1,2}).^2));
BC = sqrt(sum((worldBarXYZ{1,2} - worldBarXYZ{1,3}).^2));
CD = sqrt(sum((worldBarXYZ{1,3} - worldBarXYZ{1,4}).^2));
AD = sqrt(sum((worldBarXYZ{1,1} - worldBarXYZ{1,4}).^2));
AE = sqrt(sum((worldBarXYZ{1,1} - worldBarXYZ{1,5}).^2));
BF = sqrt(sum((worldBarXYZ{1,2} - worldBarXYZ{1,6}).^2));
CG = sqrt(sum((worldBarXYZ{1,3} - worldBarXYZ{1,7}).^2));

%%
%Question 5 - Find the height of the two students

for i = 1: size(leftMaleUV,2)
    worldMaleXYZ{1,i} = worldCoord(leftMaleUV{1,i}, b, rightMaleUV{1,i}, c);
end
heightMale = sqrt(sum((worldMaleXYZ{1,1} - worldMaleXYZ{1,2}).^2));

for i = 1: size(leftFemaleUV,2)
    worldFemaleXYZ{1,i} = worldCoord(leftFemaleUV{1,i}, b, rightFemaleUV{1,i}, c);
end
heightFemale = sqrt(sum((worldFemaleXYZ{1,1} - worldFemaleXYZ{1,2}).^2));

%in inches
heightMale = heightMale / 25.4;
heightFemale = heightFemale / 25.4;

