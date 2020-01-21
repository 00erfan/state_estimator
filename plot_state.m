%%
REF_DATA = ref_data.signals.values;
EST_DATA = est_trq.signals.values;
ACT_DATA = act_trq.signals.values;
MOT_TRQ = motor_trq.signals.values;
TEMPO = ref_data.time;



%%
figure
hold on
set(gca,'FontSize',25)
plot( TEMPO , REF_DATA  , '--r ' ,'LineWidth' , 5);grid;shg
plot( TEMPO , EST_DATA  , ':b' , 'LineWidth' , 5);grid;shg
plot( TEMPO , ACT_DATA  , 'k','LineWidth' , 5);grid;shg

xlabel('Time [s]')
ylabel('Torque [Nm]')
legend('Reference Torque' , 'Estimated Torque' , 'Actual Torque ' )
grid on
%%
figure
hold on
set(gca,'FontSize',25)
plot( TEMPO , ACT_DATA - EST_DATA   , 'k','LineWidth' , 5);grid;shg
xlabel('Time [s]')
ylabel('Error on \Delta \tau_{h} [Nm]')
grid on
%%
figure
hold on
set(gca,'FontSize',25)
plot( TEMPO , MOT_TRQ   , 'k','LineWidth' , 5);grid;shg
xlabel('Time [s]')
ylabel('Torque [Nm]')
grid on
