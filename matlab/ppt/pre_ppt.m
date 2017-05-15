%% load 
load expdata1.csv
Fs = 50;

%%
rawHeart = expdata1(:,1);
rawStep = expdata1(:,2)-mean(expdata1(:,2));
outHR = expdata1(:,3);
outSR = expdata1(:,4);
threshH = expdata1(:,5);
threshS = expdata1(:,6);
N = length(rawHeart);
h_heart = firls(50,[0 2.5 4 25]/50*2,[1 1 0 0]);
h_step = firls(50,[0 2 4 25]/50*2,[1 1 0 0]);
filtStep = conv(h_step, rawStep);
filtStep = filtStep(1:N);
filtHeart = conv(h_heart, rawHeart);

%%
thresh_ppt_tresh = threshH(7140:7260);
filt_ppt_tresh = filtHeart(7140:7260);
raw_ppt_tresh = rawHeart(7140:7260);
hold on 
% plot(raw_ppt_tresh)
plot_thresh((1:121)/50, [thresh_ppt_tresh,filt_ppt_tresh])


%%
new_hr = [outHR(1)];
idx = [1];
thres = 20;
for i=2:length(outHR)
    if i>10000 && i<15000
        thres = 20;
    elseif i>15000
        thres = 20;
    end
    if outHR(i)-outHR(idx(end))<-thres || outHR(i)-outHR(idx(end))>thres
%         w = floor(rand()*100+50);
%         outHR(i) = mean(outHR(i-w-50:i-50));
%         outHR(i) = 0;
    else
        new_hr = [new_hr outHR(i)];
        idx = [idx i];
    end
end

%%
hold on
plot(idx, new_hr,'x')
% ylim([80 120])
p = polyfit(idx,new_hr,3);
y1 = polyval(p,idx);
plot(y1)