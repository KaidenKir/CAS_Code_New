clear
close all
clc

flightData = readtable("rocket_flight_data.csv");

massDiff = table2array(diff(flightData(:, "thrust")));
massDiff_filt = movmean(massDiff, 5);
burnoutIndex = find(massDiff_filt == 0, 1, 'first');
burnoutCoords = table2array([flightData(burnoutIndex, "positionX"), flightData(burnoutIndex, "positionY"), flightData(burnoutIndex, "positionZ")]);

figure 
plot3(flightData, "positionX", "positionY", "positionZ")
hold on
plot3(burnoutCoords(1), burnoutCoords(2), burnoutCoords(3), 'o')
axis equal

figure
%plot(flightData, "time", "roll")
hold on
plot(flightData, "time", "pitch")
plot(flightData, "time", "yaw")
plot(flightData, "time", "roll")
plot(flightData, "time", "velocity_magnitude")
hold off
figure
hold on
plot(flightData, "time", "positionX")
plot(flightData, "time", "positionY")
plot(flightData, "time", "positionZ")

figure
plot(flightData, "time", "pitch_rate")
hold on
plot(flightData, "time", "yaw_rate")
plot(flightData, "time", "roll_rate")
legend

figure
subplot(2, 2, 1)
plot(flightData, "time", "fin1")
subplot(2, 2, 2)
plot(flightData, "time", "fin2")
subplot(2, 2, 3)
plot(flightData, "time", "fin3")
subplot(2, 2, 4)
plot(flightData, "time", "fin4")

