%%
load raw_data_conti.csv
load rate_data_conti.csv

rawH = raw_data_conti(:,1);
rawS = sqrt((raw_data_conti(:,2).^2+raw_data_conti(:,3).^2+raw_data_conti(:,4).^2));
hr = rate_data_conti(:,1);
sr = rate_data_conti(:,2);

%%
hold on
plot(sr,'x')
plot(hr,'+')