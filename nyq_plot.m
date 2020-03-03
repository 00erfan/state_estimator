clear all
close all
clc
%%
num = [5.05e08 , 1.102e14 , 8.654e18 , 3.129e23 , 6.252e27 , 4.637e31 , 2.214e35 , 5.346e38 , 4.896e41 , -1.175e44 , 1.115e47 , 2.942e48 , 1.826e49 , 9.428e38 ];
den = [ 1, 1.166e06 , 2.248e11 , 1.702e16 , 6.252e20 ,  1.206e25 ,9.373e28 , 4.767e32 , 1.327e36 , 1.875e39 , 1.33e42 , 7.821e44 , 8.512e45  , 1.083e47 , 8.969e47 , 6.317e37 ];
sys = tf(num,den);
%%
figure
hold on
%set(findall(gcf,'type','line'),'linewidth',2)
h = nyquistplot(sys);
grid on;
set(findall(gcf,'type','line'),'linewidth',2);
opt = getoptions(h);
opt.Title.FontSize = 25;
opt.XLabel.FontSize = 25;
opt.YLabel.FontSize = 25;
% opt.TickLabel.FontSize = 25;
% opt.ConfidenceRegionDisplaySpacing = 50;
opt.FreqUnits = 'Hz';
opt.TickLabel.FontSize = 25;
plot(-1,0,'-r*','MarkerSize', 15);


figure
hold on
grid on
P = nyquistoptions;
P.InputLabels.FontSize = 25;
P.OutputLabels.FontSize = 25;
P.Title.FontSize = 25;
P.XLabel.FontSize = 25;
P.YLabel.FontSize = 25;
P.TickLabel.FontSize = 12;
P.Grid = 'on';
nyquistplot(sys,P)
set(findall(gcf,'type','line'),'linewidth',2);
plot(-1,0,'-r*','MarkerSize', 25);

%%
OLS = sys
CLS = 1+sys

P = pole(OLS)
Z = zero(CLS)

%%
[A,B,C,D] = tf2ss(num,den)

%%
[Gm,Pm] = margin(sys)



