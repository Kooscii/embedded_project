function plot_rawdata(X1, YMatrix1)
%CREATEFIGURE(X1, YMATRIX1)
%  X1:  vector of x data
%  YMATRIX1:  matrix of y data

%  Auto-generated by MATLAB on 14-May-2017 18:48:46

% Create figure
figure1 = figure;

% Create axes
axes1 = axes('Parent',figure1);
hold(axes1,'on');

% Create multiple lines using matrix input to plot
plot1 = plot(X1,YMatrix1,'LineWidth',3);
set(plot1(1),'Color',[0 0.447058826684952 0.74117648601532]);

% Create xlabel
xlabel('Time (s)');

% Create ylabel
% ylabel('raw data');

box(axes1,'on');
% Set the remaining axes properties
set(axes1,'FontSize',16);