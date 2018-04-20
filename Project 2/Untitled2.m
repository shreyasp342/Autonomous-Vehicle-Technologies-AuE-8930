clear
close all

delete *.xml
fold = dir('*Temp');
for i = 1:size(fold,1)
    rmdir(fold(i).name,'s');
end

clear
close all

load('stoptable.mat');
negativeStop = imageDatastore({'school\*.jpg','other\*.jpg'});
% negativeFolder = ['NegativeImages_stopSign','school'];
fAlarm = 0.9:-0.1:0.1;
% cascades = [5,7,10,13,15,18,20,25,30];
cascades = [30,28,25,23,20,18,15,13,10,7,5];
r = 1;

testSize = 0.2;

% fAlarm = [0.3,0.5];
% cascades = [5,3];
rng(0);
%%
for rr = 1:r
    stop = stop1;
    k = randperm(size(stop,1),ceil(size(stop,1)*testSize));
    test = stop(k,1);
    stop(k,:)=[];
    for ff = 1:size(fAlarm,2)
        parfor cc = 1:size(cascades,2)
            xmlFile = char(strcat("stop",num2str(rr),"_",num2str(fAlarm(ff)),"_",num2str(cascades(cc)),".xml"));
            trainCascadeObjectDetector(xmlFile, stop, negativeStop, 'FalseAlarmRate', fAlarm(ff), 'NumCascadeStages', cascades(cc));
            detector = vision.CascadeObjectDetector(xmlFile);
            
            fprintf("1 rand=%d\tcascade=%d\tfalse rate=%d\n",rr,cascades(cc),fAlarm(ff));
            
            
            bb = zeros(1,size(test,1));
            b00 = 0;
            b11 = 0;
            bg11 = 0;
            for tt = 1: size(test,1)
                img = imread(char(test{tt,1}));
                gray = rgb2gray(img);
                bbox = step(detector,gray);
                bb(tt) = size(bbox,1);
                if bb(tt) ==0
                    b00 = b00 + 1;
                elseif bb(tt) == 1
                    b11 = b11 + 1;
                else
                    bg11 = bg11 + 1;
                end
                fprintf("1 rand=%d\tcascade=%d\tfalse rate=%d\t tt=%d\n",rr,cascades(cc),fAlarm(ff),tt);
            end
            b000(ff,cc) = b00;
            b111(ff,cc) = b11;
            bg111(ff,cc) = bg11;
            ss{ff,cc} = bb;
            
            
            %             nn = zeros(1,size(neg,1));
            %             n00 = 0;
            %             n11 = 0;
            %             ng11 = 0;
            %             for tt = 1: size(neg,1)
            %                 img = imread(char(neg{tt,1}));
            %                 gray = rgb2gray(img);
            %                 bbox = step(detector,gray);
            %                 nn(tt) = size(bbox,1);
            %                 if nn(tt) ==0
            %                     n00 = n00 + 1;
            %                 elseif nn(tt) == 1
            %                     n11 = n11 + 1;
            %                 else
            %                     ng11 = ng11 + 1;
            %                 end
            %                 fprintf("tt=%d\n",tt);
            %             end
            %             n000(ff,cc) = n00;
            %             n111(ff,cc) = n11;
            %             ng111(ff,cc) = ng11;
            %             nss{ff,cc} = nn;
            fprintf("1 rand=%d\tcascade=%d\tfalse rate=%d\n",rr,cascades(cc),fAlarm(ff));
        end
        
    end
    b0{rr} = b000;
        b1{rr} = b111;
        bg1{rr} = bg111;
        stopSign{rr} = ss;
        
        %         n0{rr} = n000;
        %         n1{rr} = n111;
        %         ng1{rr} = ng111;
        %         nstopSign{rr} = nss;
        clear b000 b111 bg111 ss n000 n111 ng111 nss
    clear stop test k
    
end

save('1.mat','b0','b1','bg1','stopSign');
% 
% 
% %% School Sign from Class
% clear neg
% clear
% fAlarm = 0.9:-0.1:0.1;
% % cascades = [5,7,10,13,15,18,20,25,30];
% cascades = [30,28,25,23,20,18,15,13,10,7,5];
% r = 3;
% 
% testSize = 0.2;
% 
% % fAlarm = [0.3,0.5];
% % cascades = [5,3];
% 
% 
% neg = table;
% files = dir('NegativeImages_stopSign');
% for i = 3:size(files,1)
%     filename = strcat(files(i).folder,'\',files(i).name);
%     neg(i-2,1) = {filename};
% end
% clear i l files filename
% 
% for rr = 1:r
%     for ff = 1:size(fAlarm,2)
%         parfor cc = 1:size(cascades,2)
%             xmlFile = char(strcat("stop",num2str(rr),"_",num2str(fAlarm(ff)),"_",num2str(cascades(cc)),".xml"));
%             detector = vision.CascadeObjectDetector(xmlFile);
%             
%             cnn = zeros(1,size(neg,1));
%             cn00 = 0;
%             cn11 = 0;
%             cng11 = 0;
%             for tt = 1: size(neg,1)
%                 img = imread(char(neg{tt,1}));
%                 gray = rgb2gray(img);
%                 bbox = step(detector,gray);
%                 cnn(tt) = size(bbox,1);
%                 if cnn(tt) ==0
%                     cn00 = cn00 + 1;
%                 elseif cnn(tt) == 1
%                     cn11 = cn11 + 1;
%                 else
%                     cng11 = cng11 + 1;
%                 end
%                 fprintf("2 negative class test rand=%d\tcascade=%d\tfalse rate=%d\t tt=%d\n",rr,cascades(cc),fAlarm(ff),tt);
%             end
%             cn000(ff,cc) = cn00;
%             cn111(ff,cc) = cn11;
%             cng111(ff,cc) = cng11;
%             cnss{ff,cc} = cnn;
%             %             clear cn00 cn11 cng11 cnn
%             fprintf("2negative class test rand=%d\tcascade=%d\tfalse rate=%d\n",rr,cascades(cc),fAlarm(ff));
%         end
%         
%         
%     end
%     cn0{rr} = cn000;
%         cn1{rr} = cn111;
%         cng1{rr} = cng111;
%         cnstopSign{rr} = cnss;
%         clear b000 b111 bg111 ss cn000 cn111 cng111 cnss
%     clear stop test k
%     
% end
% save('2.mat','cn0','cn1','cng1','cnstopSign');
% 
% 
% %% Stop Sign from class
% clear neg
% clear
% fAlarm = 0.9:-0.1:0.1;
% % cascades = [5,7,10,13,15,18,20,25,30];
% cascades = [30,28,25,23,20,18,15,13,10,7,5];
% r = 3;
% 
% testSize = 0.2;
% 
% % fAlarm = [0.3,0.5];
% % cascades = [5,3];
% 
% 
% neg = table;
% files = dir('PositiveImages_stopSign');
% for i = 3:size(files,1)
%     filename = strcat(files(i).folder,'\',files(i).name);
%     neg(i-2,1) = {filename};
% end
% clear i l files filename
% 
% for rr = 1:r
%     for ff = 1:size(fAlarm,2)
%         parfor cc = 1:size(cascades,2)
%             xmlFile = char(strcat("stop",num2str(rr),"_",num2str(fAlarm(ff)),"_",num2str(cascades(cc)),".xml"));
%             detector = vision.CascadeObjectDetector(xmlFile);
%             
%             tnn = zeros(1,size(neg,1));
%             tn00 = 0;
%             tn11 = 0;
%             tng11 = 0;
%             for tt = 1: size(neg,1)
%                 img = imread(char(neg{tt,1}));
%                 gray = rgb2gray(img);
%                 bbox = step(detector,gray);
%                 tnn(tt) = size(bbox,1);
%                 if tnn(tt) ==0
%                     tn00 = tn00 + 1;
%                 elseif tnn(tt) == 1
%                     tn11 = tn11 + 1;
%                 else
%                     tng11 = tng11 + 1;
%                 end
%                 fprintf("3 stop class test rand=%d\tcascade=%d\tfalse rate=%d\t tt=%d\n",rr,cascades(cc),fAlarm(ff),tt);
%             end
%             tn000(ff,cc) = tn00;
%             tn111(ff,cc) = tn11;
%             tng111(ff,cc) = tng11;
%             tnss{ff,cc} = tnn;
%             clear tn00 tn11 tng11 tnn
%             fprintf("3 stop class test rand=%d\tcascade=%d\tfalse rate=%d\n",rr,cascades(cc),fAlarm(ff));
%         end
%         
%         
%     end
%     tn0{rr} = tn000;
%         tn1{rr} = tn111;
%         tng1{rr} = tng111;
%         tnstopSign{rr} = tnss;
%         clear b000 b111 bg111 ss tn000 tn111 tng111 tnss
%     clear stop test k
%     
% end
% save('3.mat','tn0','tn1','tng1','tnstopSign');
% 
% 
% %% School Sign and random for test
% clear neg
% clear
% fAlarm = 0.9:-0.1:0.1;
% % cascades = [5,7,10,13,15,18,20,25,30];
% cascades = [30,28,25,23,20,18,15,13,10,7,5];
% r = 3;
% 
% testSize = 0.2;
% 
% % fAlarm = [0.3,0.5];
% % cascades = [5,3];
% 
% 
% files = dir('other');
% neg = table;
% for i = 3:size(files,1)
%     filename = strcat(files(i).folder,'\',files(i).name);
%     neg(i-2,1) = {filename};
% end
% 
% l = size(neg,1);
% clear files
% files = dir('school');
% for i = 3:size(files,1)
%     filename = strcat(files(i).folder,'\',files(i).name);
%     neg(l+i-2,1) = {filename};
% end
% 
% % l = size(neg,1);
% % clear files
% % files = dir('NegativeImages_stopSign');
% % for i = 3:size(files,1)
% %     filename = strcat(files(i).folder,'\',files(i).name);
% %     neg(l+i-2,1) = {filename};
% % end
% clear i l files filename
% 
% for rr = 1:r
%     for ff = 1:size(fAlarm,2)
%         parfor cc = 1:size(cascades,2)
%             xmlFile = char(strcat("stop",num2str(rr),"_",num2str(fAlarm(ff)),"_",num2str(cascades(cc)),".xml"));
%             detector = vision.CascadeObjectDetector(xmlFile);
%             
%             nn = zeros(1,size(neg,1));
%             n00 = 0;
%             n11 = 0;
%             ng11 = 0;
%             for tt = 1: size(neg,1)
%                 img = imread(char(neg{tt,1}));
%                 gray = rgb2gray(img);
%                 bbox = step(detector,gray);
%                 nn(tt) = size(bbox,1);
%                 if nn(tt) ==0
%                     n00 = n00 + 1;
%                 elseif nn(tt) == 1
%                     n11 = n11 + 1;
%                 else
%                     ng11 = ng11 + 1;
%                 end
%                 fprintf("4 negative train rand=%d\tcascade=%d\tfalse rate=%d\t tt=%d\n",rr,cascades(cc),fAlarm(ff),tt);
%             end
%             n000(ff,cc) = n00;
%             n111(ff,cc) = n11;
%             ng111(ff,cc) = ng11;
%             nss{ff,cc} = nn;
%             clear n00 n11 ng11 nn
%             fprintf("4 negative train rand=%d\tcascade=%d\tfalse rate=%d\n",rr,cascades(cc),fAlarm(ff));
%         end
%     end
%         
%         n0{rr} = n000;
%         n1{rr} = n111;
%         ng1{rr} = ng111;
%         nstopSign{rr} = nss;
%         clear b000 b111 bg111 ss n000 n111 ng111 nss
%     clear stop test k
%     
% end
% save('4.mat','n0','n1','ng1','nstopSign');
% 
% 
% %%
% clear
% load 1.mat
% load 2.mat
% load 3.mat
% load 4.mat
% save('stopDone.mat')
mkdir xmlStop1
movefile('*.xml','xmlStop1');