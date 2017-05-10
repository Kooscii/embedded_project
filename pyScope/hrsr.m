%%
load raw_data.csv
load rate_data.csv

rawH = raw_data(:,1);
rawS = sqrt((raw_data(:,2).^2+raw_data(:,3).^2+raw_data(:,4).^2));
hr = rate_data(:,1);
sr = rate_data(:,2);

%%
plot(rawH)