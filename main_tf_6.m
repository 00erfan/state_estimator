clear all
close all
clc
%%
syms  Mh Jh           % Mass and Moment Inertia of Trunk  
syms  Ms_int Js_int   % Mass and Moment of inertia of torque sensor (Internal Ring)
syms  Ms_ext Js_ext   % Mass and Moment of inertia of torque sensor (External Ring)
syms  M_act J_act     % Mass and Moment Inertia of Actuator
syms  g               % Gravity acceleration
syms  rh              % Raidal Displacement of Central mass of trunk with respect rotational axis         
syms  Kh dh dhs       % Trunk Stiffness , Trunk Damping , Trk_trq Damping
syms  Ks ds           % Stiffness of torque sensor , Damping of torque sensor
syms  Kg dg dgs       % Gearbox Stiffness , Gearbox Damping , Gea_Trq Damping
syms  teta_h tetad_h tetadd_h            % Trunk Position , Trunk Velocity , Trunk Acceleration with respect to vertical axis 
syms  teta_ext tetad_ext tetadd_ext      % torque sensor Position,  Velocity ,  Acceleration (external ring)
syms  teta_int tetad_int tetadd_int      % torque sensor Position,  Velocity ,  Acceleration (internal ring)
syms  teta_act tetad_act tetadd_act      % BDC motor shaft Position , BDC motor shaft Velocity , BDC motor shaft Acceleration, data from encoder
syms  tau_h tau_m                        % Torque by Trunk , Torque by BDC motor 
syms  rg         % Gearbox ratio 
syms  bm         % Friction between motor and gearbox
syms  Tm         % Motor Torque 
%%
syms TETA_H TETAD_H TETADD_H
syms TETA_EXT TETAD_EXT TETADD_EXT
syms TETA_INT TETAD_INT TETADD_INT
syms TETA_ACT TETAD_ACT TETADD_ACT
%% Equations of Motion
% eqns = [ Tm - Bf*tetadm - Jtot*TETADDM - (Kg/rg)*( (tetam/rg) - tetas ) - (Dg/rg)*( (tetadm/rg) - tetads ) == 0 ,
%          Kg*( (tetam/rg) - tetas ) + Dg*( (tetadm/rg) - tetads ) - Js*TETADDS - Ks*(tetas - tetal) - Ds*(tetads - tetadl) == 0 ,
%          Ks*(  tetas - tetal ) + Ds*( tetads - tetadl ) - Jl*TETADDL  - Kl*(tetal - tetah) - Dl*(tetadl - tetadh) == 0  ];   

eqns = [ Tm - bm*tetad_act - J_act*TETADD_ACT - (Kg/rg)*( (teta_act/rg) - teta_int ) - (dgs/rg)*( (tetad_act/rg) - tetad_int ) - (dg/rg)*(tetad_act/rg) == 0 ,
         Kg*( (teta_act/rg) - teta_int ) + dgs*( (tetad_act/rg) - tetad_int ) - Js_int*TETADD_INT - Ks*(teta_int - teta_ext) - ds*(tetad_int - tetad_ext) == 0 ,
         Ks*(teta_int - teta_ext) + ds*(tetad_int - tetad_ext) - Js_ext*TETADD_EXT - Kh*(teta_ext - teta_h) - dhs*(tetad_ext - tetad_h) == 0 ,
         Kh*(teta_ext - teta_h) + dhs*(tetad_ext - tetad_h) - TETADD_H*Jh - tetad_h*dh == 0 ];
 
vars   = [ TETADD_ACT TETADD_INT TETADD_EXT TETADD_H ];
%%
eqns_mat = [ eqns ,  
             TETAD_ACT == tetad_act ,
             TETAD_INT == tetad_int ,
             TETAD_EXT == tetad_ext ,
             TETAD_H   == tetad_h ];  
          
vars_mat = [ TETAD_ACT TETADD_ACT TETAD_INT TETADD_INT TETAD_EXT TETADD_EXT TETAD_H TETADD_H];

[TETAD_ACT TETADD_ACT TETAD_INT TETADD_INT TETAD_EXT TETADD_EXT TETAD_H TETADD_H] = solve(eqns_mat, vars_mat)

%%
%STATE SPACE MODEL
%state =  [ motor position , motor velocity , Internal torque sensor position , Internal torque sensor velocity , External torque sensor position , External torque sensor velocity , Trunk position , Trunk Velocity ]
%inputs = [ motor torque  ]

eqns_mat_form = [ TETAD_ACT TETADD_ACT TETAD_INT TETADD_INT TETAD_EXT TETADD_EXT TETAD_H ];
vars_mat_form = [ teta_act tetad_act teta_int tetad_int teta_ext tetad_ext teta_h   ];
ns = length(vars_mat_form); % Number of States

[A,nB] = equationsToMatrix( eqns_mat_form , vars_mat_form ); 

A_eq = A;
Bt = -nB;

[B_eq,~] = equationsToMatrix( Bt , [ Tm , tetad_h  ] );
C_eq = [ 0 , 0  , Ks , ds , -Ks , -ds, 0  ]  ;
D_eq = [ 0 , 0 ]; 

%% Transfer Function 
syms s

% H(s) = Y(s) / U(s) = C*(sI-A)^(-1)*B + D
disp(' ')
disp('The symbolic transfer function is:')
TF = simplify(  C_eq*[ (s*eye(ns,ns) - A_eq )^(-1) ]*B_eq  + D_eq , 'Steps',100 );
pretty(TF)
disp(' ')

%% System Parametes from Data sheet
J_act = [0.000306 +  0.28200e-04];    % Inertia of motor rotor + harmonic drive , kilogram metre squared [kg. m2]
rg = 160;                             % Gear Ratio of harmonic drive
Js_int = 1.1e-4 ;                     % Inertia of Internal Torque Sensor Ring + Metal Coupling  
Js_ext = 9.58e-4 ;                    % Inertia of External Torque Sensor Ring
Kg = 2.7e4;                           % Harmonic Drive Stiffness  [Nm/rad]
ds = 0;                               % Torque Sensor Damping Ratio [Nm.Sec/rad]
Ks = 8.1853e4*1.4;                    % Torque Sensor Stiffness  [Nm/rad]
dg = 0.65;                            % Harmonic Drive Damping Ratio [Nm.Sec/rad]
dgs = 6;                              % Damping Ratio between torque sensor and harmonic drive [Nm.Sec/rad]
bm = 0.4e-3;                          % Friction between motor and harmonic drive [Nm.Sec/rad] 
dhs = 0;                              % Damping between trunk and torque sensor [Nm.Sec/rad]
Kh = 1125;                            % Trunk Stiffness [Nm/rad] 
W = 75;                               % Human Weight
Jh = 5.02;                            % Inertia of Trunk [kg. m2]
dh = 0.75;                            % Damping of Trunk [Nm.Sec/rad]
Vel_mot_nom = 263;                    % Nominal motor speed [rad/sec]
Trq_mot_nom = .56;                    % Nominal motor torque [Nm]

%%
A_eq 
A_lin = eval(A_eq);
B_eq 
B_lin = eval(B_eq);
C_eq
C_lin = eval(C_eq);
D_lin = zeros(1,2)
%%
A = eval(A_eq);
B = eval(B_eq);
C = [ 1 , 0  , 0  ,  0 ,  0  ,  0 , 0;
      0 , 0  , Ks , ds , -Ks , -ds, 0  ] ;
D = zeros(1,1);

%D = zeros(8,1);
%D = eval(D)
%%
Ts = -1;
%Plant = ss(A,B,C,D, Ts,'inputname','u' ,'outputname','y');
Plant = ss(A,[B B],C,D)
Q = .05*diag(ones(1,1)); % A number greater than zero
R = .05*diag(ones(1,2)); % A number greater than zero
% %[kalmf,L,~,M,Z] = kalman(Plant,Q,R,'delayed');
[kalmf,L,~,M,Z] = kalman(Plant,Q,R);
% 
% %% LQR CONTROL %%%%%%%%%%%%%%%
% % 
% % Q_lqr = zeros(50,50);l
% % Q_lqr(44,44) = 5*10e9;
% % Q_lqr(50,50) = 1*10e8;
% % 
% % R_lqr = 1e3;
% 
% 
% Q_lqr = zeros(7,7);
% Q_lqr(5,5) = 10e5;
% Q_lqr(7,7) = 10e5;
% 
% R_lqr = 1e2 .* eye(2,2);
% 
% %%%%%    u25   = x(44);
% %%%%%      q   = x(50);
% 
% [K_lqr , S_lqr , e_lqr ] = lqr( A_lin , B_lin , Q_lqr , R_lqr ) ;

