% [handles.SBP, handles.DBP, handles.PWV, handles.PTT] = calculate_BP(block, fps, filterFCC);
clear; clc; close all;
load('mapa35.mat'); % test1 tem os melhores valores ate agora - 1 min para o usuario Joao Campos
BH = 1.72;
part = 3;
fp = [.75, 2];  % Bandpass frequencies
f = 30;
wp=(2/f).* fp;
filterFCC = fir1(15, wp); 

if part == 1
    block = block(:,1:3000);
elseif part == 2
    block = block(:,3001:6000);
else
    block = block(:,6001:9000);
end


% block = block(:,1800*2:end-1);

% block(1,:) = detrending(block(1,:)', 10)';
block(2,:) = detrending(block(2,:)', 10)';
% block(3,:) = detrending(block(3,:)', 10)';
% block(4,:) = detrending(block(4,:)', 10)';
block(5,:) = detrending(block(5,:)', 10)';
% block(6,:) = detrending(block(6,:)', 10)';

block_Forehead = block(2,:);%./sqrt(block(1,:).^2 + block(2,:).^2 + block(3,:).^2);
block_Cheek = block(5,:);%./sqrt(block(4,:).^2 + block(5,:).^2 + block(6,:).^2);

% block_Forehead = block_Forehead';
% block_Cheek = block_Cheek';

block_Forehead = filter(filterFCC,1,block_Forehead);
block_Cheek = filter(filterFCC,1,block_Cheek);
% 
% block_Forehead = smooth(block_Forehead, 5)';
% block_Cheek = smooth(block_Cheek, 5)';
% 
% % figure;
% % plot(block_Forehead);
% % hold on;
% % plot(block_Cheek);
% % hold off;
% 
% % x = 0:size(block_Forehead,2)-1;
% % xx = linspace(0,size(block_Forehead,2)-1,(size(block_Forehead,2)/30)*500);
% % block_Forehead = spline(x,block_Forehead,xx);
% % 
% % x = 0:size(block_Cheek,2)-1;
% % xx = linspace(0,size(block_Cheek,2)-1,(size(block_Cheek,2)/30)*500);
% % block_Cheek = spline(x,block_Cheek,xx);
% % 
% % fp = [.75, 4];  % Bandpass frequencies
% % f = 500;
% % wp=(2/f).* fp;
% % filterFCC = fir1(128, wp);
% % block_Forehead = filter(filterFCC,1,block_Forehead);
% % block_Cheek = filter(filterFCC,1,block_Cheek);
%             
% [aF,dF] = haart(block_Forehead,1);
% [aC,dC] = haart(block_Cheek,1);
% 
% % figure;
% % plot(aF);
% % hold on;
% % plot(aC);
% % hold off;
% 
% mean_block_Forehead = mean(aF);
% mean_block_Cheek = mean(aC);
% 
% std_block_Forehead = std(aF);
% std_block_Cheek = std(aC);
% 
% block_Forehead = (aF' - mean_block_Forehead)/std_block_Forehead;
% block_Cheek = (aC' - mean_block_Cheek)/std_block_Cheek;
% % 
% figure;
% plot(block_Forehead);
% hold on;
% plot(block_Cheek);
% hold off;

% block_Forehead = diff(block_Forehead);
% block_Cheek = diff(block_Cheek);
% 
% block_Forehead = smooth(block_Forehead, 5)';
% block_Cheek = smooth(block_Cheek, 5)';

figure;
plot(block_Forehead);
hold on;
plot(block_Cheek);
hold off;

ratio = .85;
[pF,lF] = findpeaks(block_Forehead,fps,'MinPeakProminence',std(block_Forehead)*ratio); 
[pF,lF] = findpeaks(block_Forehead,fps,'MinPeakProminence',std(block_Forehead)*ratio,'MinPeakDistance', mean(diff(lF))*.7); 
[pC,lC] = findpeaks(block_Cheek,fps,'MinPeakProminence',std(block_Cheek)*ratio); 
[pC,lC] = findpeaks(block_Cheek,fps,'MinPeakProminence',std(block_Cheek)*ratio,'MinPeakDistance', mean(diff(lC))*.7); 

figure;
plot(linspace(0, size(block_Forehead,2)/fps, size(block_Forehead,2)),block_Forehead)
hold on
plot(lF,pF,'vr')
% hold off
% 
% figure;
plot(linspace(0, size(block_Cheek,2)/fps, size(block_Cheek,2)),block_Cheek)
% hold on
plot(lC,pC,'vb')
hold off

if abs(size(lC,2) - size (lF,2)) <7
    sizeDiff = 7;
else
    sizeDiff = abs(size(lC,2) - size (lF,2));
end

%0.2667
timeDiff = 0.25;
n = 1;

if size(lC,2) <= size(lF,2)
    for i = 1 : size(lC,2)
        if i - sizeDiff <=0
            for j = 1 : i + sizeDiff
                if abs(lC(1,i)-lF(1,j)) <= timeDiff
                    lC2(1,n) = lC(1,i);
                    lF2(1,n) = lF(1,j);
                    pC2(1,n) = pC(1,i);
                    pF2(1,n) = pF(1,j);
                    n = n+1;
                end
            end
        elseif i + sizeDiff >= size(lF,2)
            disp('passei aqui')
            for j = i - sizeDiff : size(lF,2)
                disp('aqui')
                if abs(lC(1,i)-lF(1,j)) <= timeDiff
                    lC2(1,n) = lC(1,i);
                    lF2(1,n) = lF(1,j);
                    pC2(1,n) = pC(1,i);
                    pF2(1,n) = pF(1,j);
                    n = n+1;
                end
            end
        else
            for j = i - sizeDiff : i+ sizeDiff
                if abs(lC(1,i)-lF(1,j)) <= timeDiff
                    lC2(1,n) = lC(1,i);
                    lF2(1,n) = lF(1,j);
                    pC2(1,n) = pC(1,i);
                    pF2(1,n) = pF(1,j);
                    n = n+1;
                end
            end
        end
    end
else
    for i = 1 : size(lF,2)
        if i - sizeDiff <=0
            for j = 1 : i + sizeDiff
                if abs(lF(1,i)-lC(1,j)) <= timeDiff
                    lF2(1,n) = lF(1,i);
                    lC2(1,n) = lC(1,j);
                    pF2(1,n) = pF(1,i);
                    pC2(1,n) = pC(1,j);
                    n = n+1;
                end
            end
        elseif i + sizeDiff >= size(lC,2)
            disp('passei aqui')
            for j = i - sizeDiff : size(lC,2)
                disp(' aqui')
                if abs(lF(1,i)-lC(1,j)) <= timeDiff
                    lF2(1,n) = lF(1,i);
                    lC2(1,n) = lC(1,j);
                    pF2(1,n) = pF(1,i);
                    pC2(1,n) = pC(1,j);
                    n = n+1;
                end
            end
        else
            for j = i - sizeDiff : i + sizeDiff
                if abs(lF(1,i)-lC(1,j)) <= timeDiff
                    lF2(1,n) = lF(1,i);
                    lC2(1,n) = lC(1,j);
                    pF2(1,n) = pF(1,i);
                    pC2(1,n) = pC(1,j);
                    n = n+1;
                end
            end
        end
    end
end

n = n-1;

figure;
plot(linspace(0, size(block_Forehead,2)/fps, size(block_Forehead,2)),block_Forehead)
hold on
plot(lF2,pF2,'vr')
% hold off
% 
% figure;
plot(linspace(0, size(block_Cheek,2)/fps, size(block_Cheek,2)),block_Cheek)
% hold on
plot(lC2,pC2,'vb')
hold off

PTT = sum(abs(lF2-lC2))/n

PWV = (0.25 * BH) / PTT

SBP = 0.0054 * PWV^2 - 0.2197 * PWV + 115.5994
DBP = -0.0012 * PWV^2 + 0.5138 * PWV + 55.343
