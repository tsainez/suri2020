clear all
close all;
clc;

load('garage_tennis_clock_offset.mat');

real_time = (timestamp1 - timestamp1(1))/1000000;
vs1 = vs1 - mean(vs1);
figure
plot(real_time, vs1);
xlabel('Time (s)');
ylabel('Vibration amplitude')