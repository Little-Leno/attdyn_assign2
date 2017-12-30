% Example1.m
% Adopted from code by Madgwick
%
% ExampleData.mat contains calibrated gyroscope,
% accelerometer and magnetometer data logged from an AHRS device (x-IMU)
% while it was sequentially rotated from 0 degrees, to +90 degree and then
% to -90 degrees around the X, Y and Z axis.  The script first plots the
% example sensor data, then processes the data through the algorithm and
% plots the output as Euler angles.

%% Start of script

addpath('quaternion_library');      % include quaternion library
close all;                          % close all figures
clear;                              % clear all variables
clc;                                % clear the command terminal

%% Import and plot sensor data

load('ExampleBias.mat');
 
figure('Name', 'Sensor Data');
axis(1) = subplot(3,1,1);
hold on;
plot(time, Gyroscope(:,1), 'r');
plot(time, Gyroscope(:,2), 'g');
plot(time, Gyroscope(:,3), 'b');
legend('X', 'Y', 'Z');
xlabel('Time (s)');
ylabel('Angular rate (deg/s)');
title('Gyroscope');
hold off;
axis(2) = subplot(3,1,2);
hold on;
plot(time, Accelerometer(:,1), 'r');
plot(time, Accelerometer(:,2), 'g');
plot(time, Accelerometer(:,3), 'b');
legend('X', 'Y', 'Z');
xlabel('Time (s)');
ylabel('Acceleration (g)');
title('Accelerometer');
hold off;
axis(3) = subplot(3,1,3);
hold on;
plot(time, Magnetometer(:,1), 'r');
plot(time, Magnetometer(:,2), 'g');
plot(time, Magnetometer(:,3), 'b');
legend('X', 'Y', 'Z');
xlabel('Time (s)');
ylabel('Flux (G)');
title('Magnetometer');
%hold off;
linkaxes(axis, 'x');

%% Process sensor data through algorithm
AHRSquat = QuatAHRS('SamplePeriod', 1/256);
quaternion = zeros(length(time), 4);
for t = 1:length(time)
    AHRSquat.Update(Gyroscope(t,:) * (pi/180));	% gyroscope units must be radians
    quaternion(t, :) = AHRSquat.Quaternion;
    eQ=quatern2euler(quaternConj(quaternion(t, :))) * (180/pi);
    eulerQ(t,:)=eQ;
end

%% using direct cosine matrix/Euler
AHRSdcm = dcmAHRS('SamplePeriod', 1/256);
dcm_mat = zeros(length(time), 3);
for t = 1:length(time)
    AHRSdcm.Update(Gyroscope(t,:) * (pi/180));	% gyroscope units must be radians
    dcm_mat = AHRSdcm.DCM;
    edcm=rotMat2euler(dcm_mat) * (180/pi);
    eulerdcm(t,:)=edcm;
end

%% Plot algorithm output as Euler angles

figure
subplot(3,1,1)
 
hold on;
plot(time, Gyroscope(:,1), 'r');
plot(time, Gyroscope(:,2), 'g');
plot(time, Gyroscope(:,3), 'b');
legend('\omega_x', '\omega_y', '\omega_z');
xlabel('Time (s)');
ylabel('Angular rate (deg/s)');
title('Gyroscope');

subplot(3,1,2)
 
hold on;
plot(time, eulerdcm(:,1), 'r');
plot(time, eulerdcm(:,2), 'g');
plot(time, eulerdcm(:,3), 'b');
title('Euler angles');
xlabel('Time (s)');
ylabel('Angle (deg)');
legend('\phi', '\theta', '\psi');
hold off;

subplot(3,1,3)
hold on;
plot(time, quaternion(:,1), 'k');
plot(time, quaternion(:,2), 'r');
plot(time, quaternion(:,3), 'g');
plot(time, quaternion(:,4), 'b');
plot(time,quaternion(:,2).^2+quaternion(:,3).^2+quaternion(:,4).^2+quaternion(:,1).^2,'k--');
title('Quaternions');
xlabel('Time (s)');
legend('q4', 'q1', 'q2', 'q3', '|\bf{q}|');
hold off;

figure
hold on;
yyaxis left
plot(time, eulerdcm(:,1), 'r-');
plot(time, eulerdcm(:,2), 'g-');
plot(time, eulerdcm(:,3), 'b-');
yyaxis right
plot(time, quaternion(:,1), 'k--');
plot(time, quaternion(:,2), 'r--');
plot(time, quaternion(:,3), 'g--');
plot(time, quaternion(:,4), 'b--');
plot(time,quaternion(:,2).^2+quaternion(:,3).^2+quaternion(:,4).^2+quaternion(:,1).^2,'k-.');
title('Euler angles and Quaternions');
xlabel('Time (s)');
ylabel('Angle (deg)');
legend('\phi', '\theta', '\psi', 'q4', 'q1', 'q2', 'q3', '|\bf{q}|');
hold off;

%% End of script