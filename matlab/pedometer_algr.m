%%


%%
j = (-1)^(1/2);

h=firls(50,[0 2 4 25]/50*2,[1 1 0 0]);

%%
x = csvread('step28.csv', 0, 2);



x_g = mean(x(1:300));
x = x/x_g - 1;
plot(abs(fft(x)))
figure

x = x(1:2:end);
x = conv(x,h);

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

%
peaklist = [];
peakidx = [];
valleylist = [];
valleyidx = [];
baselinelist = [];
thresholdlist = [];
upperthresholdlist = [];
upperthresholdidx = [];
lowerthresholdlist = [];
lowerthresholdidx = [];


for i=1:length(x)
    if i>50 && state ~= 0
        threshold = std(x(i-40:i))*0.3;
        if threshold < 0.05
            threshold = 0.05;
        end
    end
    %
    baselinelist = [baselinelist baseline];
    if state == 0 ||state == 3
        thresholdlist = [thresholdlist baseline+threshold];
        upperthresholdlist = [upperthresholdlist baseline+threshold];
        upperthresholdidx = [upperthresholdidx i];
    else
        thresholdlist = [thresholdlist baseline-threshold];
        lowerthresholdlist = [lowerthresholdlist baseline-threshold];
        lowerthresholdidx = [lowerthresholdidx i];
    end
    
    if state == 0
        if x(i) > baseline+threshold
            flg_firststep = 1;
            state = 2;
            peak = x(i);
            cross_zero = 0;
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
                flg_firststep = 0;
                state = 3;
                valley = x(i);
                cross_zero = 0;
                step = step + 1;
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
                cross_zero = 0;
                %
                peaklist = [peaklist peak];
                peakidx = [peakidx i];
        else
            baseline = (peak+valley)/2;
        end
    end
    
    if state ~= 0 && x(i) * prev_x <0
        cross_zero = cross_zero + 1;
        if cross_zero > 5
            state = 0;
            baseline = 0;
        end
    end
    
    prev_x = x(i);
end

%%
hold on
plot(x)
plot(peakidx, peaklist,'*')
plot(valleyidx, valleylist,'*')
plot(baselinelist,':')
plot(thresholdlist,':')
% plot(lowerthresholdidx, lowerthresholdlist,'.')
% plot(upperthresholdidx, upperthresholdlist,'.')