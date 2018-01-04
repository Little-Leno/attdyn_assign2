% Example2.m
%
% This script demonstrates use of the MadgwickAHRS and MahonyAHRS algorithm
% classes with example data. ExampleBias.mat contains calibrated gyroscope,
% accelerometer and magnetometer data logged from an AHRS device (x-IMU)
% while it was sequentially rotated from 0 degrees, to +90 degree and then
% to -90 degrees around the X, Y and Z axis.  The script first plots the
% example sensor data, then processes the data through the algorithm and
% plots the output as Euler angles.
%
%
% Date          Author          Notes
% 28/09/2011    SOH Madgwick    Initial release
% 13/04/2012    SOH Madgwick    deg2rad function no longer used
% 06/11/2012    Seb Madgwick    radian to degrees calculation corrected

%% Start of script

addpath('quaternion_library');      % include quaternion library
%close all;                          % close all figures
clear;                              % clear all variables
clc;                                % clear the command terminal

%% Import and plot sensor data
load('ExampleBias.mat');
% figure('Name', 'Sensor Data');
% axis(1) = subplot(3,1,1);
% hold on;
% plot(time, Gyroscope(:,1), 'r');
% plot(time, Gyroscope(:,2), 'g');
% plot(time, Gyroscope(:,3), 'b');
% legend('X', 'Y', 'Z');
% xlabel('Time (s)');
% ylabel('Angular rate (deg/s)');
% title('Gyroscope');
% hold off;
% axis(2) = subplot(3,1,2);
% hold on;
% plot(time, Accelerometer(:,1), 'r');
% plot(time, Accelerometer(:,2), 'g');
% plot(time, Accelerometer(:,3), 'b');
% legend('X', 'Y', 'Z');
% xlabel('Time (s)');
% ylabel('Acceleration (g)');
% title('Accelerometer');
% hold off;
% axis(3) = subplot(3,1,3);
% hold on;
% plot(time, Magnetometer(:,1), 'r');
% plot(time, Magnetometer(:,2), 'g');
% plot(time, Magnetometer(:,3), 'b');
% legend('X', 'Y', 'Z');
% xlabel('Time (s)');
% ylabel('Flux (G)');
% title('Magnetometer');
% hold off;
% linkaxes(axis, 'x');

%% Process sensor data through algorithm


% AHRS = complAHRS('Kp_RP', 0.1, 'Ki_RP', 0.1, 'Kp_Yaw', 0.1*0.1, 'Ki_Yaw', 0.0001*256*0.1);
% dcm = zeros(3,3,length(time));
% euler = zeros(length(time),3);
% west = zeros(length(time), 3); 
% for t = 1:length(time)
%     AHRS.Update(Gyroscope(t,:) * (pi/180),  Accelerometer(t,:), Magnetometer(t,:));	% gyroscope units must be radians
%     dcm(:,:, t) = AHRS.DCM;
%     e = rotMat2euler(dcm(:, :, t))*(180/pi);
%     euler(t,:)=e;
%     west(t,:)=AHRS.Gyroest;
%end

%AHRS = complAHRS('Kp', 0.15*10, 'Ki', 0.000019*256*10, 'Kp_Y', 0.1*10, 'Ki_Y', 0.0001*256*10);
AHRS = complAHRS('Kp', 0.15 ,'Ki', 0.00019 , 'Kp_Y' ,0.1 , 'Ki_Y' , 0.0001) ;
euler = zeros(length(time),3);
west = zeros(length(time),3);
dcm_mat = zeros(3,3,length(time));
for t = 1:length(time)
    AHRS.Update(Gyroscope(t,:) * (pi/180), Accelerometer(t,:), Magnetometer(t,:));	% gyroscope units must be radians
    dcm_mat(:,:,t) = AHRS.DCM;
    e = rotMat2euler(dcm_mat(:,:,t)) * (180/pi);
    euler(t,:) = e;
    west(t,:) = AHRS.Gyroest;
end

%% Plot algorithm output as Euler angles
figure
subplot(3,1,1)
 
hold on;
plot(time, Gyroscope(:,1), 'r');
grid
plot(time, Gyroscope(:,2), 'g');
plot(time, Gyroscope(:,3), 'b');
legend('w_x', 'w_y', 'w_z');
xlabel('Time (s)');
ylabel('Angular rate (deg/s)');
title('Gyroscope');
west=west*180/pi;
plot(time, west(:,1), 'r:');
plot(time, west(:,2), 'g:');
plot(time, west(:,3), 'b:');
legend('w_x', 'w_y', 'w_z','w^{est}_x', 'w^{est}_y', 'w^{est}_z');

subplot(3,1,2)
 
hold on;
plot(time, euler(:,1), 'r');
grid
plot(time, euler(:,2), 'g');
plot(time, euler(:,3), 'b');
title('Euler angles');
xlabel('Time (s)');
ylabel('Angle (deg)');
legend('\phi', '\theta', '\psi');
hold off;

% subplot(3,1,3)
% hold on;
% plot(time, quaternion(:,1), 'k');
% plot(time, quaternion(:,2), 'r');
% plot(time, quaternion(:,3), 'g');
% plot(time, quaternion(:,4), 'b');
% plot(time,quaternion(:,2).^2+quaternion(:,3).^2+quaternion(:,4).^2+quaternion(:,1).^2,'k--');
% title('Quaternions');
% xlabel('Time (s)');
% legend('q4', 'q1', 'q2', 'q3', '|\bf{q}|');
% hold off;

%% End of script