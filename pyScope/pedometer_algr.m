load data.txt

n = length(data);

% smoothing
w = 20;
b = ones(1,w)*(1/w);
a = [1];
y = filter(b,a,data);

% averaging
w = 50;
b2 = ones(1,w)*(1/w);
y2 = filter(b2,a,y);

hold on
% plot(data)
plot(y)
plot(y2)

curr_isgt = y(399)>y2(399);
detectx = [];
detecty = [];
for i=400:n
    prev_isgt = curr_isgt;
    curr_isgt = y(i)>y2(i);
    if (curr_isgt && ~prev_isgt)
        detectx = [detectx, i];
        detecty = [detecty, y2(i)];
    end
end

plot(detectx, detecty, 'ro')

length(detectx)