%% load 
clear
load expdata1.csv
Fs = 50;

h_step = firls(50,[0 2 4 25]/50*2,[1 1 0 0]);
rawStep = expdata1(:,2)-mean(expdata1(:,2));
N = length(rawStep);
filtStep = conv(h_step, rawStep);
filtStep = filtStep(1:N);

%% plot rawdata only
plot_rawdata((0*50:2*50)/50, [rawStep(350*50:352*50)])
legend('raw')
print -depsc raw_data.eps

%% plot raw spectrum only
plot_rawfft((1:N)*Fs/N-Fs/2,abs(fftshift(fft(rawStep))))
xlim([-Fs/2, Fs/2])
ylim([0, 3e4])
print -depsc raw_spectrum.eps

%% plot freqz response
[H, w] = freqz(h_step,1);
plot_filter(w/2/pi*Fs, abs(H))
print -depsc freqz.eps

%% plot raw and filtered spectrum
plot_rawfft((1:N)*Fs/N-Fs/2,[abs(fftshift(fft(rawStep))),abs(fftshift(fft(filtStep)))])
xlim([-Fs/2, Fs/2])
ylim([0, 3e4])
print -depsc filtered_spectrum.eps

%% plot rawdata and filtered data
plot_rawdata((0*50:2*50)/50, [rawStep(350*50:352*50), filtStep(350*50:352*50)])
legend('raw','filtered')
print -depsc filtered_data.eps


