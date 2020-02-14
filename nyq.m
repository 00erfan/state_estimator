Ts = mean( diff (smlk_input.time) );

y = smlk_output.signals.values;
u = smlk_input.signals.values;

data_sim = iddata(y,u,Ts , 'InterSample','zoh');
sys_sim = tfest(data_sim,6)
%%
figure
hold on
nyquist(sys_sim)
grid on
%%
figure
hold on
bode(sys_sim)
grid on

