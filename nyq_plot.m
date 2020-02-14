clear all
close all
clc
%%
num = [1.169e11, 5.26e14, 1.373e17, 6.177e20 ];
den = [1, 5.455e04, 1.408e09, 6.592e12, 3.089e16, 4.727e16, 3.861e18];
sys = tf(num,den)
%%
figure
hold on
nyquist(sys)
grid on

