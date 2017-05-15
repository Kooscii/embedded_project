%%
h=firls(50,[0 4 5 25]/50*2,[1 1 0 0]);
load exp1.csv

%%
x = exp1(1:30000,1);
n = (1:length(x))/50;

%%
x = conv(x,h);
x = x(1:length(n));

peak = -100;
valley = 100;
baseline = 0;
threshold = 3;
state = 0;
flg_firststep = 0;
cross_zero = 0;
prev_x = 0;
step = 0;
step_period = ones(1,5)*2000;
step_periodidx = 1;
outHR = 0;

%
peaklist = [];
peakidx = [];
valleylist = [];
valleyidx = [];
baselinelist = zeros(1,length(x));
thresholdlist = zeros(1,length(x));
upperthresholdlist = zeros(1,length(x));
upperthresholdidx = zeros(1,length(x));
lowerthresholdlist = zeros(1,length(x));
lowerthresholdidx = zeros(1,length(x));

msTimCnt = 0;
step_timeout = msTimCnt;
step_mstick = msTimCnt;

outHRlist = zeros(1,length(x));

baseline = mean(x(1:50));

for i=51:length(x)
    if i>50 && state ~= 0
        threshold = std(x(i-10:i))*0.3;
        if threshold < 3
            threshold = 3;
        end
    end
    %
    baselinelist(i) = baseline;
    if state == 0 ||state == 3
        thresholdlist(i) = baseline+threshold;
        upperthresholdlist(i) = baseline+threshold;
        upperthresholdidx(i) = i;
    else
        thresholdlist(i) = baseline-threshold;
        lowerthresholdlist(i) = baseline-threshold;
        lowerthresholdidx(i) = i;
    end
    
    if state == 0
        if x(i) > baseline+threshold
            flg_firststep = 1;
            state = 2;
            peak = x(i);
            step_timeout = msTimCnt;
            %
            peakidx = [peakidx i];
            peaklist = [peaklist peak];
        end
    elseif state == 2
        if x(i)>peak
            peak = x(i);
            %
            peakidx(end) = i;
            peaklist(end) = peak;
        elseif x(i)<baseline-threshold
            if flg_firststep == 0
                step_period(step_periodidx) = msTimCnt - step_mstick;
                step_periodidx = mod(step_periodidx, 5)+1;
                step_mean = mean(step_period);
                outHR = 60000./step_mean;
            end
                flg_firststep = 0;
                state = 3;
                valley = x(i);
                step_timeout = msTimCnt;
                step_mstick = msTimCnt;
                %
                valleyidx = [valleyidx i];
                valleylist = [valleylist valley];
        elseif flg_firststep == 0
                baseline = (peak-valley)*2/3+valley;
        end
    elseif state == 3
        if x(i)<valley
            valley = x(i);
            baseline = (peak-valley)*2/3+valley;
            %
            valleyidx(end) = i;
            valleylist(end) = valley;
        elseif x(i)>baseline+threshold
                state = 2;
                peak = x(i);
                step_timeout = msTimCnt;
                %
                peaklist = [peaklist peak];
                peakidx = [peakidx i];
        else
				baseline = baseline - threshold*0.1;
        end
    end
    
    if ((state == 3) && (msTimCnt - step_timeout>2000)) || ((state == 2) && (msTimCnt - step_timeout>1000))
			state = 0;
            flg_firststep = 1;
			baseline = mean(x(i-25:i));
			threshold = 0.05;
			step_period = 2000*ones(1,5);
            outHR = 0;
    end
    
    prev_x = x(i);
    
    outHRlist(i) = outHR;
    
    msTimCnt = msTimCnt + 20;
end

%%
figure
box on
hold on
plot(n, x)
plot(peakidx/50, peaklist, '.')
plot(valleyidx/50, valleylist, '.')
plot(n, baselinelist, ':')
plot(n, thresholdlist, ':')
legend('signal', 'peak', 'valley', 'baseline', 'threshold')
xlabel('Time (s)')
ylim([0 255])

%%
figure
box on
hold on
plot(outSRlist(1:end), '.')
plot(outHRlist(1:end), '.')
legend('SPM','BPM')
xlabel('Time (s)')
% plot(lowerthresholdidx, lowerthresholdlist,'.')
% plot(upperthresholdidx, upperthresholdlist,'.')