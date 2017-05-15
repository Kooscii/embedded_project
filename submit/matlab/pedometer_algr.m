%%
h=firls(50,[0 2 4 25]/50*2,[1 1 0 0]);
load expdata1.csv

%%
x = expdata1(9000:59000,2);
n = (9000:59000)/50;

x_g = mean(x(1:100));
x = x/x_g - 1;

% plot(abs(fft(x)))
figure

x = conv(x,h);
x = x(1:length(n));

% x = rawH - mean(rawH);

peak = -100;
valley = 100;
baseline = 0;
threshold = 0.05;
state = 0;
flg_firststep = 0;
cross_zero = 0;
prev_x = 0;
step = 0;
step_period = ones(1,5)*2000;
step_periodidx = 1;

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

outSRlist = zeros(1,length(x));
outSR = 0;

for i=1:length(x)
    if i>50 && state ~= 0
        threshold = std(x(i-40:i))*0.5;
        if threshold < 0.05
            threshold = 0.05;
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
                outSR = 60000./step_mean;
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
                baseline = (peak+valley)/2;
        end
    elseif state == 3
        if x(i)<valley
            valley = x(i);
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
            baseline = (peak+valley)/2;
        end
    end
    
    if ((state == 3) && (msTimCnt - step_timeout>3000)) || ((state == 2) && (msTimCnt - step_timeout>3000))
			state = 0;
            flg_firststep = 1;
			baseline = 0;
			threshold = 0.05;
			step_period = 2000*ones(1,5);
            outSR = 0;
    end
    
    prev_x = x(i);
    
    outSRlist(i) = outSR;
    
    msTimCnt = msTimCnt + 20;
end

%%
hold on
plot(x)
plot(peakidx, peaklist,'.')
plot(valleyidx, valleylist,'.')
plot(baselinelist,':')
plot(thresholdlist,':')

figure
plot(outSRlist,'+')
% plot(lowerthresholdidx, lowerthresholdlist,'.')
% plot(upperthresholdidx, upperthresholdlist,'.')