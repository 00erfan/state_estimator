% close all
% clear all
% clc
%% System Parametes from Data sheet

scale = 1;
Kt = 109e-3;                       % Torque Constant [Nm/A] 
% Kv = 88 * pi/30;                   % Speed Constant [rpm/V] => [rad/V.Sec]
% Ra = 0.522;                        % BDC motor Armiture Resistance [ohm]
% La = 0.625e-3;                     % BDC motor Armiture Inductance [mH]
% Js = 1.1e-3 ;                      % Total Torque Sensor Inertia [Kg.m2]

Jtot = [0.000306 +  0.28200e-04];    % Inertia of motor rotor + harmonic drive , kilogram metre squared [kg. m2]
%Jtot = [0.000306 +  0.09e-04];    % Inertia of motor rotor + harmonic drive , kilogram metre squared [kg. m2]

rg = 160;                           % Gear Ratio of harmonic drive
%Jsint = [1.1e-4 ] + [0.5 * 0.32 * ( 0.0460^2 )];  % Inertia of Internal Torque Sensor Ring + Metal Coupling
Jsint = 1.1e-4 ;  % Inertia of Internal Torque Sensor Ring + Metal Coupling  
Jsint = scale*Jsint;
%Jsint = 1 * Jsint;
%Jtot = Jtot + (Jsint/(rg^2) );
%Jtot = 1 * Jtot;
Jsout = 9.58e-4 ;                   % Inertia of External Torque Sensor Ring
Jsout = scale*Jsout;
kg = 2.7e4;                         % Harmonic Drive Stiffness  [Nm/rad]
%ks = 8.1853e+3;                   % Torque Sensor Stiffness  [Nm/rad]
%ks = 6e4;
%ks = 1.64e4/2;
%ks = 8.1967e+03;
ks = 8.1853e3;
Jl =  0.04 + Jsout  ;               % load Inertia [Kg.m2]
%Jl =  0.0178 + Jsout  ;               % load Inertia [Kg.m2]
%Jl = 1 * Jl;
ds = 0;                           % Torque Sensor Damping Ratio [Nm.Sec/rad]
T_sweep = 130;                  % Duration of input chirp signal (sweep) [sec]
f0 = 0.1;                           % Chirp signal initial frequency [Hz]
f1 = 130;                            % Final Frequency [Hz]
f_log = 1000;                       % Sampling frequency
%desiredFs = f_log;
% Amp = .23;                          % Input Torque Signal Amplitiude in motor side [Nm]
I_cur = 2.3;                        % Motor current
Vel_mot_nom = 263;                  % nominal motor speed [rad/sec]

dgr = 25;           % Harmonic Drive Damping Ratio [Nm.Sec/rad]
dgs = 7;            % Damping Ratio between torque sensor and harmonic drive [Nm.Sec/rad]

%%%%%%%%%%%%%% GAMC modifications
%this matches resonance from experimental response
ks = 8.1853e4*1.4;     
Jl = 1.2 * Jl;

%Erfan's dgr value is far too high and the simulated motor velocity reflected at output of gearbox does not match experimental response
dgr=0.65;           
dgs = 6;           

%%
Data_exp = csvread('test_23_130_130_v1.csv' , 1 , 0 );  % Reading Experimental Data
Data_exp(end , :) = [];

%%
samples = 1:length( Data_exp(:,1) ) ;
tempo = Data_exp( samples , 1);
tempo=tempo-tempo(1);
in_cur = Data_exp( samples , 2);
out_cur = Data_exp( samples , 3);
out_vel = Data_exp( samples , 4);
out_trq = Data_exp( samples , 5);
%%
originalFs = 1 / mean ( diff( tempo ) )    % Real Sampling Frequency [Hz]

%% Removing Torque Sensor Off_set
out_trq_off = out_trq - mean( out_trq );

%%  Resample Data %%%%%%%%%%%%%%%%%%

desiredFs = 1000;
%desiredFs = originalFs + (.0*originalFs);
[p,q] = rat(desiredFs / originalFs);
%[p,q] = rat(originalFs / originalFs);

cur_mes_rspl = resample( out_cur , p , q );     % Resample Motor Current
vel_mes_rspl = resample( out_vel , p , q );     % Resample Motor Velocity
trq_mes_rspl = resample( out_trq_off , p , q );     % Resample Sensor Torque

time_rspl = (0:numel(cur_mes_rspl)-1)/desiredFs;
t_dur = tempo(end);            % Duration Time of Experiment [sec]

%% LOW PASS FILTER 

Fs = desiredFs;       % Sampling Frequency [Hz]
%Fs = originalFs;
Fc = 130;               % Cut of Frequency [Hz]
% 
[b,a] = butter( 6 , Fc/(Fs/2) );                 % Butterworth filter of order 6
trq_flt = filtfilt( b , a , trq_mes_rspl );     % Will be the filtered measured torque signal
cur_flt = filtfilt( b , a , cur_mes_rspl );     % Will be the filtered measured current signal

% sys_exp_input  =  cur_flt .* Kt .* rg;
% sys_exp_output =   trq_flt;

sys_exp_input  =  cur_mes_rspl .* Kt .* rg;    %this is wrong, rg is part of the system hardware you do not need to multiply by rg here
sys_exp_input  =  cur_mes_rspl .* Kt;          %motor torque

sys_exp_output =  trq_mes_rspl;

[Txy_exp,F_exp] = tfestimate( sys_exp_input , -sys_exp_output , 1024 , [] , [] , desiredFs);
systfest_exp = frd(Txy_exp,2*pi*F_exp);

fignum=11;

figure(fignum)
hold on;
h_exp = bodeplot(systfest_exp,'r',systfest_exp.Frequency );
setoptions(h_exp,'Xlim',[1,130],'FreqUnits','Hz')
grid on;

%%
Torque_input_ref = [ time_rspl' , ( cur_mes_rspl .* Kt) ];  % Input torque is calculated as motor curret * torque constant

sim('sim_exp_sim')
%sim('sim_sim_spl_exp')

trq_sen_sim =  sim_oupt.Data;
trq_mot_exp = sim_inpt.Data;
[Txy_exp_sim,F_exp_sim] = tfestimate(  trq_mot_exp .* rg , trq_sen_sim , 1024 , [] , [] , desiredFs); %this is wrong, rg is part of the simulink block diagram you do not need to multiply by rg here
[Txy_exp_sim,F_exp_sim] = tfestimate(  trq_mot_exp , trq_sen_sim , 1024 , [] , [] , desiredFs);
systfest_exp_sim = frd(Txy_exp_sim,2*pi*F_exp_sim);

figure(fignum)
hold on;
h_exp_sim = bodeplot(systfest_exp_sim,'k',systfest_exp_sim.Frequency );
setoptions(h_exp_sim,'Xlim',[1,130],'FreqUnits','Hz')
grid on;
hold
legend('exp frequency response', 'sim frequency response')

%compare motor velocities to ensure dgr is a suitable value
figure(fignum+1)
plot(time_rspl,vel_mes_rspl );grid;shg
hold
plot(tempo,out_vel,'r');shg
plot(sim_time.Data,sim_oupt_vel.Data,'c');shg
hold
legend('exp data resampled','exp data','sim data')
title('motor velocities reflected at gearbox output')

%compare torques sensor
figure(fignum+2)
plot(time_rspl,trq_mes_rspl );grid;shg
hold
plot(tempo,out_trq_off,'r');shg
plot(sim_time.Data,-sim_oupt.Data,'c');shg
hold
legend('exp data resampled','exp data','sim data')
title('torque sensor')
