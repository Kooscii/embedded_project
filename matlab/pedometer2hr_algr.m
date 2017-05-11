%%


%%
load raw_data.csv
rawH = raw_data(:,1);

% h1=firls(50,[0 0.2 1 25]/50*2,[0 0 1 1]);
h2=firls(50, [0 2.5 4 25]/50*2,[1 1 0 0]);
h = conv(h1,h2);
%%

% x = rawH - mean(rawH);
% plot(abs(fft(x)))

x = rawH;
figure

% x = x(1:2:end);
% x = conv(x,h1);
x = conv(x,h2);

% x = rawH - mean(rawH);

peak = -100;
valley = 100;
baseline = 0;
threshold = 10;
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
    y = x(i);
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
        if y > baseline+threshold
            flg_firststep = 1;
            state = 2;
            peak = y;
            cross_zero = 0;
            %
            peakidx = [peakidx i];
            peaklist = [peaklist peak];
        end
    elseif state == 2
        if y>peak
            peak = y;
            %
            peakidx(end) = i;
            peaklist(end) = peak;
        elseif y<baseline-threshold
                flg_firststep = 0;
                state = 3;
                valley = y;
                cross_zero = 0;
                step = step + 1;
                %
                valleyidx = [valleyidx i];
                valleylist = [valleylist valley];
        elseif flg_firststep == 0
                baseline = (peak+valley)*2/3;
        end
    elseif state == 3
        if y<valley
            valley = y;
            %
            valleyidx(end) = i;
            valleylist(end) = valley;
        elseif y>baseline+threshold
                state = 2;
                peak = y;
                cross_zero = 0;
                %
                peaklist = [peaklist peak];
                peakidx = [peakidx i];
        else
            baseline = (peak+valley)*2/3;
        end
    end
    
    if state ~= 0 && y * prev_x <0
        cross_zero = cross_zero + 1;
        if cross_zero > 5
            state = 0;
            baseline = 0;
        end
    end
    
    prev_x = y;
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