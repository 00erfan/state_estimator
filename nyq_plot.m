clear all
close all
clc
%%
num = [9.469e05 , 1.664e11 , 1.198e16 , 4.3e20 , 8.256e24 , 6.749e28 , 3.598e32 , 1.095e36 , 1.839e39 , 1.313e42 , 4.624e44 , 7.694e46 , 1.935e48 , 1.211e49 , 2.331e37 ];
den = [1 , 1.751e05 , 1.254e10 , 4.463e14 , 8.452e18 , 6.621e22 , 3.401e26 , 9.521e29 , 1.353e33 , 4.827e35 , 9.452e36, 7.297e37 , 1.07e39 , -1.402e28 , 0 , 0];
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



