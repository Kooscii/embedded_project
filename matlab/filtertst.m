%%
clear
j = (-1)^(1/2);

%%
N1 = 10;
k1 = 0:N1-1;
M1 = (N1-1)/2;
W = exp(j*2*pi/N1);
Ak1 = [1 1 0.7 0 zeros(1, 3) 0 -0.7 -1];
h1 = real(ifft(Ak1.*W.^(-M1*k1)));

N2 = 49;
k2 = 0:N2-1;
M2 = (N2-1)/2;
W = exp(j*2*pi/N2);
Ak2 = [1 0.9 0.05 zeros(1, 44) 0.05 0.9];
h2 = real(ifft(Ak2.*W.^(-M2*k2)));

N5 = 9;
k5 = 0:N5-1;
M5 = (N5-1)/2;
W = exp(j*2*pi/N5);
Ak5 = [0 0.7 ones(1, 6) 0.7];
h5 = real(ifft(Ak5.*W.^(-M5*k5)));


%% plot A(w)
L = 2^10;
WL = exp(j*2*pi/L);
kL = 0:L-1;
w = kL.*2*pi/L;

hold on
grid on
box on

H = fft(h1,L);
A1 = real(H.*WL.^(M1*kL));
plot(w/2/pi, A1)
plot(k1/N1, Ak1, '.')

% H = fft(h2,L);
% A2 = real(H.*WL.^(M2*kL));
% plot(w/2/pi, A2)
% plot(k2/N2, Ak2, '.')

H = fft(h5,L);
A5 = real(H.*WL.^(M5*kL));
plot(w/2/pi, A5)
plot(k5/N5, Ak5, '.')
% xlim([0, 0.1])
% ylim([0.995, 1.005])