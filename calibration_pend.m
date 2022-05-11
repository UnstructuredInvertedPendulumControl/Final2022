%uses least squares to find sensitivity matrices and offset vectors for
%acceleromters mounted to pendulum
%based on Cole's CalAccNotes.m
%data already saved to mat file using savedata_pend.m

%Author: Alex Boehm
%spring 2022 WIP senior design project

close all
clear

%get data from mat file
load('pend_pot_readings.mat')
%gives 3 arrays: pot_readings (Nx1), top_acc_readings(Nx3),
%bottom_acc_readings (Nx3);

%save accelerometer readings to structure
acc_readings = {top_acc_readings,bottom_acc_readings};

%num of data points
N = length(pot_readings);

%convert pot readings from deg to rad
theta = deg2rad(pot_readings);

%create b vector (actual acc readings, based on pot ang and grav)
g = 9.80665;    %m/s2
%check this (i think its right tho)
b_matrix = [-g*sin(theta'); g*cos(theta'); zeros(1,N)];
b = reshape(b_matrix,N*3,1);    %in form for x=A'*b

%intialize matrices for loops
x = zeros(12,2);    %sens+offset vector, columns = 2 accelerometers
err = zeros(N*3,2); %error for each reading
rmserr = [0,0];     %root mean square error for each accelerometer
S{1,2} = [];        %sensitivity matrices
O{1,2} = [];        %offset vectors

for acc = 1:2 %two accelerometers (1=top,2=bottom)
    
    %intialize A matrix (zeros)
    A = zeros(N*3,12);
    
    acc_data = acc_readings{acc};   %only taking one acc at a time
    
    for pt = 1:N  %runs through each data point
        %decompose vector from acc_data into components
        Vs_cell = num2cell(acc_data(pt,:));
        [Vx,Vy,Vz] = Vs_cell{:};
        
        %create individual A matrix for reading
        A_pt = [Vx Vy Vz 0 0 0 0 0 0 1 0 0;0 0 0 Vx Vy Vz 0 0 0 0 1 0;...
        0 0 0 0 0 0 Vx Vy Vz 0 0 1];
        
        %add to overall A matrix
        A(pt*3-2:pt*3,:) = A_pt;
    end
    
    %get sensitivity+offset vector
    x(:,acc) = pinv(A)*b;
    S{acc} = reshape(x(1:9,acc),[3,3]).';    %sensitivities
    O{acc} = x(10:12,acc);                   %offsets
    
    %find error for each reading (calc - actual)
    err(:,acc) = A*x(:,acc) - b;
    rmserr(acc) = rmse_calc(err(:,acc));
end

[S_top,S_bottom] = S{:};
[O_top,O_bottom] = O{:};


%calculates root mean square error
%stolen directly from Cole
function r = rmse_calc(err)
    r = 0;
    for i = 1:length(err)
        r = r + err(i)^2;
    end 
    r = r/length(err);
    r = sqrt(r);
end
        
        