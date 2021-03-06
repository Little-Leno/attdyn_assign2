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
AHRS = dcmAHRS('SamplePeriod', 1/256);
dcm_mat = zeros(length(time), 3);
for t = 1:length(time)
    AHRS.Update(Gyroscope(t,:) * (pi/180));	% gyroscope units must be radians
    dcm_mat = AHRS.DCM;
    e=rotMat2euler(dcm_mat) * (180/pi);
    euler(t,:)=e;
end

AHRS = QuatAHRS('SamplePeriod', 1/256);
quaternion = zeros(length(time), 4);
for t = 1:length(time)
    AHRS.Update(Gyroscope(t,:) * (pi/180));	% gyroscope units must be radians
    quaternion(t, :) = AHRS.Quaternion;
    e=quatern2euler(quaternConj(quaternion(t, :))) * (180/pi);
    euler(t,:)=e;
end



%% Plot algorithm output as Euler angles

figure
subplot(2,1,1)
 
hold on;
plot(time, Gyroscope(:,1), 'r');
plot(time, Gyroscope(:,2), 'g');
plot(time, Gyroscope(:,3), 'b');
legend('\omega_x', '\omega_y', '\omega_z');
xlabel('Time (s)');
ylabel('Angular rate (deg/s)');
title('Gyroscope');

subplot(2,1,2)
 
hold on;
plot(time, euler(:,1), 'r');
plot(time, euler(:,2), 'g');
plot(time, euler(:,3), 'b');
plot(time, dcm_mat(:,1), 'k--');
plot(time, dcm_mat(:,2), 'r--');
plot(time, dcm_mat(:,3), 'g--');
plot(time, dcm_mat(:,4), 'b--');
plot(time,dcm_mat(:,2).^2+dcm_mat(:,3).^2+dcm_mat(:,4).^2+dcm_mat(:,1).^2,'k-.');
title('Euler angles and quaternions');
xlabel('Time (s)');
ylabel('Angle (deg)');
legend('\phi', '\theta', '\psi', 'q4', 'q1', 'q2', 'q3', '|\bf{q}|');
hold off;

% subplot(3,1,3)
% hold on;
% plot(time, dcm_mat(:,1), 'k');
% plot(time, dcm_mat(:,2), 'r');
% plot(time, dcm_mat(:,3), 'g');
% plot(time, dcm_mat(:,4), 'b');
% plot(time,dcm_mat(:,2).^2+dcm_mat(:,3).^2+dcm_mat(:,4).^2+dcm_mat(:,1).^2,'k--');
% title('Quaternions');
% xlabel('Time (s)');
% legend('q4', 'q1', 'q2', 'q3', '|\bf{q}|');
% hold off;

%% End of script