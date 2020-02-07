%%
REF_DATA = ref_data.signals.values;
EST_DATA = est_trq.signals.values;
ACT_DATA = act_trq.signals.values;
MOT_TRQ = motor_trq.signals.values;
TEMPO = ref_data.time;

INT_DATA = int_trq.signals.values;
INT_EST_DATA = int_est_trq.signals.values;


%%
figure
hold on
set(gca,'FontSize',25)
plot( TEMPO , REF_DATA  , '--k ' ,'LineWidth' , 5);grid;shg
plot( TEMPO , EST_DATA  , '-.b' , 'LineWidth' , 5);grid;shg
plot( TEMPO , ACT_DATA  , ':r','LineWidth' , 5);grid;shg

xlabel('Time [s]')
ylabel('Torque [Nm]')
legend('Reference Torque' , 'Estimated Torque' , 'Actual Torque ' )
grid on
%%
figure
hold on
set(gca,'FontSize',25)
%plot( TEMPO , ((ACT_DATA - EST_DATA)./ACT_DATA)*100  , 'k','LineWidth' , 5);grid;shg
plot( TEMPO , (ACT_DATA - EST_DATA)  , 'k','LineWidth' , 5);grid;shg

xlabel('Time [s]')
%ylabel('Error on \Delta \tau_{h} [%]')
ylabel('Error on \Delta y_{b}')
grid on
%%
figure
hold on
set(gca,'FontSize',25)
plot( TEMPO , MOT_TRQ   , 'k','LineWidth' , 5);grid;shg
xlabel('Time [s]')
ylabel('Torque [Nm]')
grid on

%%
figure
hold on
set(gca,'FontSize',25)
plot( TEMPO , INT_EST_DATA  , '-.b' , 'LineWidth' , 5);grid;shg
plot( TEMPO , INT_DATA  , ':r','LineWidth' , 5);grid;shg

xlabel('Time [s]')
ylabel('Velocity [rad/s]')
legend('Estimated Velocity' , 'Actual Velocity' )
grid on

figure
hold on
set(gca,'FontSize',25)
%plot( TEMPO , ((ACT_DATA - EST_DATA)./ACT_DATA)*100  , 'k','LineWidth' , 5);grid;shg
plot( TEMPO , (INT_DATA -INT_EST_DATA)  , 'k','LineWidth' , 5);grid;shg

xlabel('Time [s]')
%ylabel('Error on \Delta \tau_{h} [%]')
ylabel('Error on \Delta \theta_{s int}')
grid on