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
syms  Kh dh           % Trunk Stiffness , Trunk Damping 
syms  Ks ds           % Stiffness of torque sensor , Damping of torque sensor
syms  Kg dg           % Gearbox Stiffness , Gearbox Damping
syms  K_sh d_sh       % Stiffness between torque sensor and trunk , Damping between torque sensor and trunk
syms  teta_h tetad_h tetadd_h            % Trunk Position , Trunk Velocity , Trunk Acceleration with respect to vertical axis 
syms  teta_ext tetad_ext tetadd_ext      % torque sensor Position,  Velocity ,  Acceleration (external ring)
syms  teta_int tetad_int tetadd_int      % torque sensor Position,  Velocity ,  Acceleration (internal ring)
syms  teta_act tetad_act tetadd_act      % BDC motor shaft Position , BDC motor shaft Velocity , BDC motor shaft Acceleration, data from encoder
syms  tau_h tau_m                        % Torque by Trunk , Torque by BDC motor 
syms  rg         % Gearbox ratio 
syms  bm         % Friction between motor and gearbox         
%%
syms TETA_H TETAD_H TETADD_H
syms TETA_EXT TETAD_EXT TETADD_EXT
syms TETA_INT TETAD_INT TETADD_INT
syms TETA_ACT TETAD_ACT TETADD_ACT

%% Equations of Motion
% eqns = [ J_act*tetadd_act + (Kg*teta_act + dg*tetad_act - rg^2*tau_m + bm*rg^2*tetad_act - Kg*rg*teta_int - dg*rg*tetad_int)/rg^2
%        (Js_int*tetadd_int - (Kg*teta_act + dg*tetad_act - Kg*rg*teta_int - Ks*rg*teta_int + Ks*rg*teta_ext - dg*rg*tetad_int - ds*rg*tetad_int + ds*rg*tetad_ext)/rg)/rg
%        (Js_int*tetadd_ext - K_sh*teta_h + K_sh*teta_ext - Ks*teta_int + Ks*teta_ext - d_sh*tetad_h + d_sh*tetad_ext - ds*tetad_int + ds*tetad_ext)/rg
%        -(tau_h - K_sh*teta_h + K_sh*teta_ext - d_sh*tetad_h + d_sh*tetad_ext - dh*tetad_h - tetadd_h*(Mh*rh^2 + Jh) + Mh*g*rh*sin(teta_h))/rg
  
% Changing sin(teta_h) to teta_h  and making tau_h equal to zero
eqns = [  J_act*TETADD_ACT + (Kg*teta_act + dg*tetad_act - rg^2*tau_m + bm*rg^2*tetad_act - Kg*rg*teta_int - dg*rg*tetad_int)/rg^2 == 0,
        (Js_int*TETADD_INT - (Kg*teta_act + dg*tetad_act - Kg*rg*teta_int - Ks*rg*teta_int + Ks*rg*teta_ext - dg*rg*tetad_int - ds*rg*tetad_int + ds*rg*tetad_ext)/rg)/rg == 0,
        (Js_int*TETADD_EXT - K_sh*teta_h + K_sh*teta_ext - Ks*teta_int + Ks*teta_ext - d_sh*tetad_h + d_sh*tetad_ext - ds*tetad_int + ds*tetad_ext)/rg == 0,
        -(0 - K_sh*teta_h + K_sh*teta_ext - d_sh*tetad_h + d_sh*tetad_ext - dh*tetad_h - TETADD_H*(Mh*rh^2 + Jh) + Mh*g*rh*teta_h)/rg == 0 ];
 
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
%state =  [ motor position , motor velocity , Internal torque sensor position , Internal torque sensor velocity , External torque sensor position , External torque sensor velocity , Trunk position ]
%inputs = [ motor torque , Trunk velocity ] 

eqns_mat_form = [ TETAD_ACT TETADD_ACT TETAD_INT TETADD_INT TETAD_EXT TETADD_EXT TETAD_H ];
vars_mat_form = [ teta_act tetad_act teta_int tetad_int teta_ext tetad_ext teta_h  ];
ns = length(vars_mat_form); % Number of States

[A,nB] = equationsToMatrix( eqns_mat_form , vars_mat_form );

A = A
Bt = -nB;

[B,~] = equationsToMatrix( Bt , [ tau_m ,  tetad_h ] )
C = [ 0 , 0  , Ks , ds , -Ks , -ds, 0 ]  ;
D = [ 0 , 0 ]; 
% 
%% Transfer Function 
syms s

% H(s) = Y(s) / U(s) = C*(sI-A)^(-1)*B + D
disp(' ')
disp('The symbolic transfer function is:')
TF = simplify(  C*[ (s*eye(ns,ns) - A )^(-1) ]*B  + D , 'Steps',100 );
pretty(TF)
disp(' ')

%% Controller
syms ctrl
syms Trq_s  Trq_d  Trq_m  vel_h

%% Closed Loop System

% Trq_s = [ (Trq_d - (Trq_s/rg))*(ctrl) + Trq_d ]*TF(1)  +  [ (Trq_d - (Trq_s/rg))*(ctrl) + (vel_h*dh + vel_h*s*Mh*(rh^2) +  Jh*vel_h*s ) ]*TF(2);   

Trq_s = solve( Trq_s == [ ( (Trq_d - Trq_s)*(ctrl) + Trq_d ) / rg ]*TF(1)  +  [ ( (0 - Trq_s)*(ctrl) + (vel_h*dh + vel_h*s*Mh*(rh^2) +  Jh*vel_h*s ) )/ rg ]*TF(1) +  [ vel_h *TF(2) ] , Trq_s);
Trq_s = collect( Trq_s , [Trq_d , vel_h] );

disp(' ')
TF_tra = coeffs( Trq_s , vel_h  ,'All');
TF_tra = subs( TF_tra , Trq_d , 0 );
disp(' ')
disp('The transfer function in transparency mode is:')
TF_tra = simplify( TF_tra(1) );
pretty(TF_tra)
disp(' ')
% 
disp(' ')
TF_ass = coeffs( Trq_s , Trq_d , 'All'); 
TF_ass = subs( TF_ass , vel_h , 0 );
disp(' ')
disp('The transfer function in assisstive mode is:')
TF_ass = simplify( TF_ass(1) );
pretty(TF_ass)
disp(' ')

