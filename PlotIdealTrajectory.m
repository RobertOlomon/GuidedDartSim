%% PlotIdealTrajectory.m
% This script runs a GuidedDartSim simulation, computes an ideal orientation
% for the dart (with no yaw or roll so that the dart's z-axis is perfectly aligned
% with the vertical projection of its velocity), and then plots the ideal trajectory
% along with basis vectors (plotted every ~0.3 s).

clear; clc; close all;

%% Set Up Simulation Parameters (Example for Targeting Base)
Pitch = deg2rad(45);  
Yaw = 0.126689635808396;  % initial value (ignored in ideal orientation)
Angle = [Pitch; Yaw; 0];  % [pitch; yaw; roll]
DBVelocity = [0; 0; 16.38];  % dart basis velocity (along Z)
Position = [4.225; 0.94; 0.5852483];
Mass = 0.208;
InertiaMatrix = [0.00086057, 0, 0.00000021;
                 0, 0.00086107, 0.00000313;
                 0.00000021, 0.00000313, 0.00009604];

DisplayBasisVectors = false;
RunningOptimization = false;
TargetingBase = true;

% Create simulation object (assume GuidedDartSim is in the MATLAB path)
sim = GuidedDartSim(Angle, DBVelocity, Position, Mass, InertiaMatrix, ...
    'DisplayBasisVectors', DisplayBasisVectors, 'RunningOptimization', RunningOptimization, ...
    'TargetingBase', TargetingBase, 'MobileBase', 'fixed');


%% Run the Simulation
sim.run();

%% Extract True Trajectory Data
truePos = sim.PlotPosition;       % Nx3 matrix of dart positions (world coordinates)
timeArray = sim.PlotTime;         % simulation time vector
N = length(timeArray);
dt = sim.Timestep;

%% Compute Finite-Difference Velocity
velocity = zeros(N,3);
velocity(1,:) = (truePos(2,:) - truePos(1,:)) / dt;
for idx = 2:N-1
    velocity(idx,:) = (truePos(idx+1,:) - truePos(idx-1,:)) / (2*dt);
end
velocity(N,:) = (truePos(N,:) - truePos(N-1,:)) / dt;

%% Compute Ideal Orientation (No yaw or roll)
% For each timestep, we force the velocity's x-component to zero to ignore lateral motion.
idealBasis = cell(N,1);
idealPitch = zeros(N,1);  % ideal pitch angle for each timestep
for idx = 1:N
    v = velocity(idx,:)';
    % Force x-component to zero:
    v_forced = [0; v(2); v(3)];
    if norm(v_forced) < 1e-6
        theta = 0;
    else
        theta = atan2(v_forced(2), v_forced(3));
    end
    idealPitch(idx) = theta;
    % Construct ideal dart basis as a pure pitch rotation about the x-axis:
    % This yields: 
    %   xIdeal = [1; 0; 0]
    %   yIdeal = [0; cos(theta), -sin(theta)]
    %   zIdeal = [0; sin(theta), cos(theta)]
    R_pitch = [1,      0,         0;
               0, cos(theta), -sin(theta);
               0, sin(theta),  cos(theta)];
    idealBasis{idx} = R_pitch;
end

%% Plot the Ideal Trajectory with Basis Vectors
figure;
hold on; grid on; axis equal;
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('Ideal Dart Trajectory with Basis Vectors');

% Plot the trajectory (using the simulated positions)
plot3(truePos(:,1), truePos(:,2), truePos(:,3), 'b-', 'LineWidth', 2);

% For visualization, plot basis vectors every 0.3 seconds.
dt_basis = 0.3;   
indexInterval = round(dt_basis / dt);
scaleVec = 0.3;  % Increase as desired to make the arrows more visible

for idx = 1:indexInterval:N
    pos = truePos(idx,:)';
    R = idealBasis{idx};  % Ideal basis at this timestep
    % Plot the x-axis (red), y-axis (green), and z-axis (blue) of the dart
    quiver3(pos(1), pos(2), pos(3), scaleVec * R(1,1), scaleVec * R(2,1), scaleVec * R(3,1), 'Color', 'r', 'LineWidth', 2);
    quiver3(pos(1), pos(2), pos(3), scaleVec * R(1,2), scaleVec * R(2,2), scaleVec * R(3,2), 'Color', 'g', 'LineWidth', 2);
    quiver3(pos(1), pos(2), pos(3), scaleVec * R(1,3), scaleVec * R(2,3), scaleVec * R(3,3), 'Color', 'b', 'LineWidth', 2);
end

legend('Trajectory', 'x-axis', 'y-axis', 'z-axis');
