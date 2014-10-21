%% EurlerAccel.m
%
%  Yannik Kübli - 29.09.2014
%
%%
function [phi theta] = EulerAccel(ax, ay)
%
%
g = 9.81;

theta = asin (ax / g );
phi = asin ( -ay / (g*cos(theta)));