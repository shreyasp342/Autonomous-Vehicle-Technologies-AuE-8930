files = dir('other');
neg = table;
for i = 3:size(files,1)
    filename = strcat(files(i).folder,'\',files(i).name);
    neg(i-2,1) = {filename};
end

l = size(neg,1);
clear files
files = dir('school');
for i = 3:size(files,1)
    filename = strcat(files(i).folder,'\',files(i).name);
    neg(l+i-2,1) = {filename};
end
clear i l files filename

fAlarm = [0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9];
cascades = [5,7,10,13,15,18,20,25,30];
r = 5;

for rr = 1:r
    stop = stop1;
    rng(0);
    k = randperm(size(stop,1),ceil(size(stop,1)*testSize));
    test = stop(k,1);
    stop(k,:)=[];
    for ff = 1:size(fAlarm,2)
        parfor cc = 1:size(cascades,2)
            xmlFile = char(strcat("stop",num2str(rr),"_",num2str(fAlarm(ff)),"_",num2str(cascades(cc)),".xml"));
            detector = vision.CascadeObjectDetector(xmlFile);
            
            
%             bb = zeros(1,size(test,1));
%             b00 = 0;
%             b11 = 0;
%             bg11 = 0;
%             for tt = 1: size(test,1)
%                 img = imread(char(test{tt,1}));
%                 gray = rgb2gray(img);
%                 bbox = step(detector,gray);
%                 bb(tt) = size(bbox,1);
%                 if bb(tt) ==0
%                     b00 = b00 + 1;
%                 elseif bb(tt) == 1
%                     b11 = b11 + 1;
%                 else
%                     bg11 = bg11 + 1;
%                 end
%             end
%             b000(ff,cc) = b00;
%             b111(ff,cc) = b11;
%             bg111(ff,cc) = bg11;
%             ss{ff,cc} = bb;
%             
%             
            nn = zeros(1,size(neg,1));
            n00 = 0;
            n11 = 0;
            ng11 = 0;
            for tt = 1: size(neg,1)
                img = imread(char(neg{tt,1}));
                gray = rgb2gray(img);
                bbox = step(detector,gray);
                nn(tt) = size(bbox,1);
                if nn(tt) ==0
                    n00 = n00 + 1;
                elseif nn(tt) == 1
                    n11 = n11 + 1;
                else
                    ng11 = ng11 + 1;
                end
                fprintf("tt=%d\n",tt);
            end
            n000(ff,cc) = n00;
            n111(ff,cc) = n11;
            ng111(ff,cc) = ng11;
            nss{ff,cc} = nn;
        fprintf("rand=%d\tcascade=%d\tfalse rate=%d\n",rr,cascades(cc),fAlarm(ff));   
        end
%         b0{rr} = b000;
%         b1{rr} = b111;
%         bg1{rr} = bg111;
%         stopSign{rr} = ss;
%         
        n0{rr} = n000;
        n1{rr} = n111;
        ng1{rr} = ng111;
        nstopSign{rr} = nss;
        clear b000 b111 bg111 ss n000 n111 ng111 nss
    end
    clear stop test k
    
end
