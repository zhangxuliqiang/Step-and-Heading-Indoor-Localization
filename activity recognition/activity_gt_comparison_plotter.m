
 xaxis = datetime({'10/20/20 16:50:30.000', '10/20/20 16:53:00.000'},...
    'InputFormat','MM/dd/yy HH:mm:ss.SSS','TimeZone','Europe/Amsterdam');

figure
subplot(4,1,1)
hold on
plot(new_data.Time, new_data.accelerometerAccelerationXG)
plot([door_handle_use.elapsed, door_handle_use.elapsed],ylim, 'green')
xlim(xaxis)
title('Accelerometer x axis')
xlabel('Time')
ylabel('Acceleration (g)')
legend('smartwatch','door contact','Location','eastoutside')
hold off
subplot(4,1,2)
hold on
plot(new_data.Time, new_data.accelerometerAccelerationYG)
plot([door_handle_use.elapsed, door_handle_use.elapsed],ylim, 'green')
xlim(xaxis)
hold off
title('Accelerometer y axis')
xlabel('Time')
ylabel('Acceleration (g)')
legend('smartwatch','door contact','Location','eastoutside')
subplot(4,1,3)
hold on
plot(new_data.Time, new_data.accelerometerAccelerationZG)
plot([door_handle_use.elapsed, door_handle_use.elapsed],ylim, 'green')
xlim(xaxis)
hold off
title('Accelerometer z axis')
xlabel('Time')
ylabel('Acceleration (g)')
legend('smartwatch','door contact','Location','eastoutside')
subplot(4,1,4)
hold on
plot(shs.sl_components.Time +  record.StartTime,  shs.sl_components.standstill)
xlim(xaxis)
ylim([0,1.5])
plot([door_handle_use.elapsed, door_handle_use.elapsed],ylim, 'green')
title('Standstill detection from SHS')
xlabel('Time')
ylabel('Standstill detected')
legend('shs standstill \newline detection','door contact','Location','eastoutside')
hold off
sgtitle('Smartwatch Acceleration Signal and SHS Standstill Detection')

%%
