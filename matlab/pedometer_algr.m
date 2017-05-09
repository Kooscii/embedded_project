%%
clear

%%
j = (-1)^(1/2);

N1 = 99;
k1 = 0:N1-1;
M1 = (N1-1)/2;
W = exp(j*2*pi/N1);
Ak1 = [1 1 0.8 0.1 zeros(1, 92) 0.1 0.8 1];
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

h3 = [1:10 10:-1:1]/80;

h4 = ones(1,20)/20;

%%
x = csvread('sr-1hz.csv', 0, 3);

x_g = mean(x(1:300));
x = x/x_g - 1;

hold on

% plot(abs(fft(x)))

plot(x)
x = conv(x,h1);
plot(x)
x = conv(x,h5);
plot(x)
% plot(conv(x,h3))
% plot(conv(x,h4))

% stem(abs(fft(x)), '.')

% xlim([600 1800])


% % averaging
% w = 50;
% b2 = ones(1,w)*(1/w);
% y2 = filter(b2,a,y);
% 
% hold on
% % plot(data)
% plot(y)
% plot(y2)
% 
% curr_isgt = y(399)>y2(399);
% detectx = [];
% detecty = [];
% for i=400:n
%     prev_isgt = curr_isgt;
%     curr_isgt = y(i)>y2(i);
%     if (curr_isgt && ~prev_isgt)
%         detectx = [detectx, i];
%         detecty = [detecty, y2(i)];
%     end
% end
% 
% plot(detectx, detecty, 'ro')
% 
% length(detectx)


