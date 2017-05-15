function plot_filter(X1, Y1)
%CREATEFIGURE(X1, Y1)
%  X1:  vector of x data
%  Y1:  vector of y data

%  Auto-generated by MATLAB on 14-May-2017 22:38:38

% Create figure
figure1 = figure;

% Create axes
axes1 = axes('Parent',figure1);
hold(axes1,'on');

% Create plot
plot(X1,Y1,'LineWidth',3);

% Create xlabel
xlabel('Physical Frequency (Hz)');

% Create ylabel
ylabel('Magnitude Response');

% Uncomment the following line to preserve the X-limits of the axes
% xlim(axes1,[0 15]);
box(axes1,'on');
% Set the remaining axes properties
set(axes1,'FontSize',16);
