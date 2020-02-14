clear all
close all
clc
%%
num = [40];
den = [1, 6, 11, 6];
sys = tf(num,den)
%%
figure
hold on
nyquist(sys)
grid on

figure
hold on
bode(sys)
grid on

