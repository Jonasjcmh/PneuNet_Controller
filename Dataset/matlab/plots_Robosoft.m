figure()
df=Steps;
plot(df.time, df.Filtered_angle)
hold on
plot(df.time, df.set_point)
xlabel('time [s]')
ylabel('angle [°]')
xlim([0 df.time(length(df.time))])
ylim([-5 70])
legStr = { 'Bending Angle' , 'Setpoint' };
legend( legStr );
%%
figure()
df=Sinoidal;
plot(df.time, df.Filtered_angle)
hold on
plot(df.time, df.set_point)
xlabel('time [s]')
ylabel('angle [°]')
xlim([0 df.time(length(df.time))])
ylim([min(df.Filtered_angle) max(df.Filtered_angle)])
legStr = { 'Bending Angle', 'Setpoint'  };
legend( legStr );

%yticks([0 50 100])
%yticklabels({'y = 0','y = 50','y = 100'})
%%
figure()
df=Ramps;
plot(df.time, df.Filtered_angle)
hold on
plot(df.time, df.set_point)
xlabel('time [s]')
ylabel('angle [°]')
xlim([0 df.time(length(df.time))])
ylim([min(df.Filtered_angle) max(df.Filtered_angle)])
legStr = { 'Bending Angle', 'Setpoint'  };
legend( legStr );

%%

figure()
df=NewStep;
plot(df.time, df.Filtered_angle)
hold on
plot(df.time, df.set_point)
xlabel('time [s]')
ylabel('angle [°]')
xlim([0 df.time(length(df.time))])
ylim([min(df.Filtered_angle) 90])
legStr = {'Bending Angle', 'Setpoint'  };
legend( legStr );

%%
figure()
df=NewSin;
plot(df.time, df.Filtered_angle)
hold on
plot(df.time, df.set_point)
xlabel('time [s]')
ylabel('angle [°]')
xlim([0 df.time(length(df.time))])
ylim([min(df.Filtered_angle) 180])
legStr = { 'Bending Angle', 'Setpoint' };
legend( legStr );

error=df.set_point-df.Filtered_angle
averageerror=sum(error)/length(error)
mean(error)

%yticks([0 50 100])
%yticklabels({'y = 0','y = 50','y = 100'})
%%
figure()
df=NewPressurewithout;
df2=NewPressuresensor;
plot(df.time, df.Filtered_pressure)
hold on
plot(df2.time, df2.Filtered_pressure)
xlabel('time [s]')
ylabel('pressure [kPa]')
xlim([0 60])
ylim([min(df.pressure) 25])
legStr = { 'Without sensor','With sensor'};
legend( legStr );

%%
figure()
df=NewRamp2;
plot(df.time, df.Filtered_angle)
hold on
plot(df.time, df.set_point)
xlabel('time [s]')
ylabel('angle [°]')
xlim([0 700])
ylim([min(df.Filtered_angle) 180])
legStr = { 'Bending Angle','Setpoint' };
legend( legStr );

error=df.set_point-df.Filtered_angle
averageerror=sum(error)/length(error)
mean(error)