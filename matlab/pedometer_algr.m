%%
clear

%%
j = (-1)^(1/2);

% N1 = 10;
% k1 = 0:N1-1;
% M1 = (N1-1)/2;
% W = exp(j*2*pi/N1);
% Ak1 = [1 1 0.7 0 zeros(1, 3) 0 -0.7 -1];
% h1 = real(ifft(Ak1.*W.^(-M1*k1)));
% 
% N2 = 49;
% k2 = 0:N2-1;
% M2 = (N2-1)/2;
% W = exp(j*2*pi/N2);
% Ak2 = [1 0.9 0.05 zeros(1, 44) 0.05 0.9];
% h2 = real(ifft(Ak2.*W.^(-M2*k2)));
% 
% N5 = 9;
% k5 = 0:N5-1;
% M5 = (N5-1)/2;
% W = exp(j*2*pi/N5);
% Ak5 = [0 0.7 ones(1, 6) 0.7];
% h5 = real(ifft(Ak5.*W.^(-M5*k5)));

N1 = 99;
k1 = 0:N1-1;
M1 = (N1-1)/2;
W = exp(j*2*pi/N1);
Ak1 = [1 1 0.7 0 zeros(1, 92) 0 0.7 1];
h1 = real(ifft(Ak1.*W.^(-M1*k1)));

N2 = 49;
k2 = 0:N2-1;
M2 = (N2-1)/2;
W = exp(j*2*pi/N2);
Ak2 = [1 0.9 0.05 zeros(1, 44) 0.05 0.9];
h2 = real(ifft(Ak2.*W.^(-M2*k2)));

N5 = 99;
k5 = 0:N5-1;
M5 = (N5-1)/2;
W = exp(j*2*pi/N1);
Ak5 = [0 0.7 ones(1, 96) 0.7];
h5 = real(ifft(Ak5.*W.^(-M5*k5)));
% 
h3 = [1:2 2:-1:1];
% 
h4 = ones(1,3);

%%
x = csvread('step28.csv', 0, 2);

x_g = mean(x(1:300));
x = x/x_g - 1;

x = x(1:2:end);
x = conv(x,h);

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

for i=1:length(x)
    %
    baselinelist = [baselinelist baseline];
    
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
            if flg_firststep == 0
                baseline = (peak+valley)/2;
            end
        elseif x(i)<baseline-threshold
                flg_firststep = 0;
                state = 3;
                valley = x(i);
                baseline = (peak+valley)/2;
                cross_zero = 0;
                step = step + 1;
                %
                valleyidx = [valleyidx i];
                valleylist = [valleylist peak];
        end
    elseif state == 3
        if x(i)<valley
            valley = x(i);
            baseline = (peak+valley)/2;
            %
            valleyidx(end) = i;
            valleylist(end) = valley;
        elseif x(i)>baseline+threshold
                state = 2;
                peak = x(i);
                baseline = (peak+valley)/2;
                cross_zero = 0;
                %
                peaklist = [peaklist peak];
                peakidx = [peakidx i];
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
plot(peakidx, peaklist,'o')
plot(valleyidx, valleylist,'o')
plot(baselinelist,':')
plot(baselinelist-threshold,'--')
plot(baselinelist+threshold,'--')