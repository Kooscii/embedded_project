%%

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
currHR_val = [heart heart heart];
flg_start = 0;
rawHR_stack = zeros(length(currHR_val),1);
tic

hr_datacnt = 0;

for i = 1:length(currHR_val)

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
            hr_mstick = round(toc * 1000);
            hr_pulsecnt = 0;
        end
    elseif hr_state == 1
        if currHR_val(i) > hr_baseline + hr_threshold
            hr_state = 2;   
            hr_period(hr_pulsecnt) = round(toc * 1000)-hr_mstick;
            hr_mstick = round(toc * 1000);
        else 
            if (round(toc * 1000) - hr_mstick > 1000000) 
                hr_state = 0;
            end
        end
    elseif hr_state == 2
        if currHR_val(i) < hr_peak - hr_threshold
            if flg_esta == 0
                hr_state = 1;
                if hr_pulsecnt>0
                    hr_period(hr_pulsecnt) = hr_period(hr_pulsecnt) + (round(toc * 1000)-hr_mstick);
                    if (hr_pulsecnt == 3) 
                        hr_std = std(hr_period);
                        hr_mean = mean(hr_period);
                        if hr_mean > 500 && hr_mean < 3500 && hr_std < 500
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
                hr_mstick = round(toc * 1000);
                hr_pulsecnt = hr_pulsecnt + 1;
            else 
                hr_state = 1;
                flg_esta = 1;
                hr_baseline = min(rawHR_stack(i-50:i));
                hr_peak = max(rawHR_stack(i-hr_datacnt:i));
                hr_threshold = (hr_peak-hr_baseline)/1.5;
                hr_period(1) = hr_period(1) + round(toc * 1000)-hr_mstick;
                outHR = outHR + 1;
            end
        else 
            if round(toc * 1000) - hr_mstick > 100000
                hr_state = 0;
                flg_esta = 0;
            end
        end
    end

    prevHR_val = currHR_val(i);
    pause(0.01)
end