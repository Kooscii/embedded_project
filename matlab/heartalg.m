load raw_data.csv

rawH = raw_data(:,1);
baselinelist = [];
thresholdlist = [];
upperthresholdlist = [];
upperthresholdidx = [];
lowerthresholdlist = [];
lowerthresholdidx = [];
averagelist = [];

h1=firls(50,[0 0.2 1 25]/50*2,[0 0 1 1]);
h2=firls(50, [0 2.5 4 25]/50*2,[1 1 0 0]);


%%
% load raw_data_conti.csv
% hold on
% rawH = raw_data_conti(:,1);
% plot(rawH)
rawH = conv(rawH, h1);
rawH = conv(rawH, h2);
% plot(rawH)

%%
currHR_val = rawH;
rawHR_stackIdx = 0;
prevHR_val = 255;
hr_state = 0;
hr_peak = 0;
hr_baseline = 0;
hr_threshold = 0;
hr_pulsecnt = 0;
hr_period = [0,0,0];
hr_mstick = 0;
hr_std = 0;
hr_mean = 0;
flg_esta = 0;
outHR = 0;
flg_start = 0;
rawHR_stack = zeros(length(currHR_val),1);
outHR = [];
cntH = 0;

tic

hr_datacnt = 0;

for i = 1:length(currHR_val)
    if i>51
        averagelist = [averagelist mean(currHR_val(i-50:i))];
    else
        averagelist = [averagelist mean(currHR_val(1:i))];
    end
    baselinelist = [baselinelist hr_baseline];
    if hr_state == 0 || hr_state == 1
        thresholdlist = [thresholdlist hr_baseline + hr_threshold];
%         upperthresholdlist = [upperthresholdlist baseline+threshold];
%         upperthresholdidx = [upperthresholdidx i];
    else
        thresholdlist = [thresholdlist hr_peak - hr_threshold];
%         lowerthresholdlist = [lowerthresholdlist baseline-threshold];
%         lowerthresholdidx = [lowerthresholdidx i];
    end
    
    rawHR_stack(i,:) = currHR_val(i);

    if hr_state == 0
        hr_baseline = min(rawHR_stack(i-hr_datacnt:i));
        hr_peak = max(rawHR_stack(i-hr_datacnt:i));
        if hr_datacnt < 100
            hr_datacnt = hr_datacnt + 1;
        end
        hr_threshold = (hr_peak-hr_baseline)/1.5;
        if hr_peak-hr_baseline>50 && currHR_val(i) > hr_baseline + hr_threshold
            hr_state = 2;
            hr_mstick = round(toc * 20000);
            hr_pulsecnt = 0;
        end
    elseif hr_state == 1
        if currHR_val(i) > hr_baseline + hr_threshold
            hr_state = 2;   
            hr_period(hr_pulsecnt) = round(toc * 20000)-hr_mstick;
            hr_mstick = round(toc * 20000);
        else 
            if (round(toc * 20000) - hr_mstick > 2000) 
                hr_state = 0;
                flg_esta = 0;
            end
        end
    elseif hr_state == 2
        if currHR_val(i) < hr_peak - hr_threshold
            if flg_esta == 0
                hr_state = 1;
                if hr_pulsecnt>0
                    hr_period(hr_pulsecnt) = hr_period(hr_pulsecnt) + (round(toc * 20000)-hr_mstick);
                    if (hr_pulsecnt == 3) 
                        hr_std = std(hr_period);
                        hr_mean = mean(hr_period);
                        if hr_mean > 200 && hr_mean < 3000 && hr_std < 500
                            hr_state = 1;
                            flg_esta = 1;
                        else 
                            hr_state = 0;
                            flg_esta = 0;
                        end
                    end
                else
                    hr_baseline = min(rawHR_stack(i-hr_datacnt:i));
                    hr_peak = max(rawHR_stack(i-hr_datacnt:i));
                    hr_threshold = (hr_peak-hr_baseline)/1.5;
                end
                hr_mstick = round(toc * 20000);
                hr_pulsecnt = mod(hr_pulsecnt, 3) + 1;
            else 
                hr_state = 1;
                flg_esta = 1;
                hr_baseline = min(rawHR_stack(i-50:i));
                hr_peak = max(rawHR_stack(i-hr_datacnt:i));
                hr_threshold = (hr_peak-hr_baseline)/1.5;
                hr_period(hr_pulsecnt) = hr_period(hr_pulsecnt) + round(toc * 20000)-hr_mstick;
                hr_mstick = round(toc * 20000);
                hr_pulsecnt = mod(hr_pulsecnt, 3) + 1;
                hr_std = std(hr_period);
                hr_mean = mean(hr_period);
                if hr_mean > 200 && hr_mean < 3000 && hr_std < 500
                    cntH = cntH + 1;
                    outHR = [outHR 60000./hr_mean];
                else 
                    hr_state = 0;
                    flg_esta = 0;
                end
            end
        else 
            if round(toc * 20000) - hr_mstick > 1000
                hr_state = 0;
                flg_esta = 0;
            end
        end
    end

    prevHR_val = currHR_val(i);
    pause(0.001)
end

%%
hold on
plot(rawH)
% plot(averagelist)
plot(baselinelist,':')
plot(thresholdlist,':')