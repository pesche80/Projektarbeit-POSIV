%% LPF.m
%%
%  Yannik Kübli - 14.10.2014
%  Low-pass filter
%
%% 
function xlpf = LPF(x)
%
%
persistent prevX
persistent firstRun

if isempty(firstRun)
    prevX = x;
    firstRun =1;
end

alpha =0.8;
xlpf = alpha*prevX + (1-alpha)*x;

prevX = xlpf;

