A_ny = [       0             1             0             0             0             0
           -3156        -2.205     5.049e+05         112.2             0             0
               0             0             0             1             0             0
       1.534e+06         340.9    -1.287e+09    -5.455e+04     1.042e+09             0
               0             0             0             0             0             1
               0             0     1.196e+08             0    -1.208e+08             0]
           
           
           
B_ny = [ 0
         2992
         0
         0
         0
         0 ]
     
C_ny = [ 0             0             0             0          1125             0]

D_ny = 0


sys_nyk = ss( A_ny , B_ny , C_ny , D_ny )

figure
nyquist(sys_nyk)
hold on
grid on


figure
hold on
grid on
w = linspace(-10*pi,10*pi,512);
[re,im] = nyquist(sys_nyk,w  );
re = squeeze(re);
im = squeeze(im); 
plot(re,im,'b')
xlabel('Real Axis');
ylabel('Imaginary Axis');


     

           
  
