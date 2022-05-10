%save data from pendulum accelerometers and pendulum potentiometer
%save into arrays
%to use with calibration_pend.m

%Author: Alex Boehm
%spring 2022 WIP senior design project

%decide name for file
matname = strcat('pend_pot_readings.mat');

%read from csv file
readings = readtable('pend_pot_readings.csv');

pot_readings = table2array(readings(:,7));
top_acc_readings = table2array(readings(:,1:3));
bottom_acc_readings = table2array(readings(:,4:6));

save(matname,'pot_readings','top_acc_readings','bottom_acc_readings');
