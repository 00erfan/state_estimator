clear all
close all
clc
%%
num = [-1125 , -2.466e08 , -1.945e13 , -7.278e17 , -1.421e22 , -1.105e26 , -5.518e29 , -1.508e33 , -2.004e36 , -1.02e39 , -1.162e41 , -6.674e42 , -9.087e43 , 2.043e32 , 0 , 0 ];   
den = [ 1 , 2.192e05 , 1.729e10 , 6.472e14 , 1.265e19 , 9.894e22 , 5.03e26 , 1.393e30 , 1.865e33 , 9.529e35 , 1.09e38 , 6.667e39 , 2.36e41 , 3.708e42 , 1.916e43 , - 6.797e30   , 0 ];
sys = tf(num,den);

dbdrop = -3.5;
fb = bandwidth(sys,dbdrop)


figure
hold on
grid on
h = bodeplot(sys , logspace(0,3));
p = getoptions(h); 
p.Title.FontSize = 25;
p.XLabel.FontSize = 25;
p.YLabel.FontSize = 25;
% opt.TickLabel.FontSize = 25;
% opt.ConfidenceRegionDisplaySpacing = 50;
p.FreqUnits = 'Hz';
p.TickLabel.FontSize = 15;
set(findall(gcf,'type','line'),'linewidth',2);

p.PhaseMatching = 'on'; 
p.PhaseMatchingFreq = 1; 
p.PhaseMatchingValue = 90;
setoptions(h,p);
