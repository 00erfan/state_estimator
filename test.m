clear all
close all
clc
%%
A = [0 1 0;
     0 0 1;
     1 0 0];    
B = [ 0.3  1;
        0  1;
     -0.3  0.9];
C = [1.9 1.3 1];  
D = [0.53 -0.61];
sys = ss(A,B,C,D);

nx = 3;    %Number of states
ny = 1;    %Number of outputs
Qn = [4 2 0; 2 1 0; 0 0 1];
Rn = 0.7;
R = [1 0;0 2]
QXU = blkdiag(0.1*eye(nx),R);
QWV = blkdiag(Qn,Rn);
QI = eye(ny);

KLQG1 = lqg(sys,QXU,QWV,QI,'1dof')

%%
