%save data from accelerometers for static calibration
%save into tables
%to use with calibration_static.m

%Author: Alex Boehm
%spring 2022 WIP senior design project

%make sure to save csv files with names like 'acc_negx.csv' to use with
%this code and save in folder named first part of csv files ('acc' in this
%example)

%decide name for mat file
matname = strcat(acc,'_readings.mat');

%first part of csv file names 
acc = 'cart';

addpath(acc)

negx = readtable(strcat(acc,'_negx.csv'));
negy = readtable(strcat(acc,'_negy.csv'));
negz = readtable(strcat(acc,'_negz.csv'));
posx = readtable(strcat(acc,'_posx.csv'));
posy = readtable(strcat(acc,'_posy.csv'));
posz = readtable(strcat(acc,'_posz.csv'));

save(matname,'negx','negy','negz','posx','posy','posz');

