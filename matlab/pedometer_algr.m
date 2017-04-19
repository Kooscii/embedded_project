load sr-2hz.csv
x2 = sr_2hz(:,1);
y2 = sr_2hz(:,2);
z2 = sr_2hz(:,3);
rms2 = sr_2hz(:,4);

load sr-1hz2.csv
x1 = sr_1hz2(:,1);
y1 = sr_1hz2(:,2);
z1 = sr_1hz2(:,3);
rms1 = sr_1hz2(:,4);

% 
% n = length(x2);
% 
% smoothing
w = 10;
b = ones(1,w)*(1/w);
a = [1];
s = filter(b,a,x2);

plot(s)

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


