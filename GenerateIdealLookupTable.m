%% GenerateIdealLookupTable.m
% This script runs a GuidedDartSim simulation and generates an ideal lookup table 
% for the projected coordinates of a green light as seen by a virtual camera.
% In this ideal case, the dart’s orientation is computed such that its body z-axis 
% is perfectly aligned with the (full) velocity vector. In other words, the dart’s 
% orientation is the unique rotation (with roll forced to zero) that maps [0;0;1] to v/||v||.
% The lookup table is organized as [time, u, v, pitch, distance],
% where pitch is computed as acos(v_z) (from the normalized velocity).
%
% Note: This requires MATLAB's eul2rotm function.

clear; clc; close all;

%% Camera Intrinsics
imageWidth = 1280;
imageHeight = 800;
hfov = deg2rad(85);  % convert to radians
vfov = deg2rad(60);
fx = (imageWidth/2) / tan(hfov/2);
fy = (imageHeight/2) / tan(vfov/2);
cx = imageWidth/2;
cy = imageHeight/2;

%% Define Target Green Light World Coordinates
% For the "base" target, assume the target center is approximately:
%   [7.5; 1.077; 26.11] (in meters) with the green light 100 mm below the target.
targetGreenWorld = [7.5; 1.077 - 0.1; 26.11];

%% Set Up Simulation Parameters (Example for Targeting Base)
% (These parameters are used solely to generate the dart trajectory.)
Pitch = deg2rad(45);  
Yaw = 0.126689635808396;  % initial yaw (will be overridden by the ideal computation)
Angle = [Pitch; Yaw; 0];  % initial angle [pitch; yaw; roll]
DBVelocity = [0; 0; 16.38];  % dart basis velocity along Z
Position = [4.225; 0.94; 0.5852483];
Mass = 0.208;
InertiaMatrix = [0.00086057, 0.0, 0.00000021;
                 0.0, 0.00086107, 0.00000313;
                 0.00000021, 0.00000313, 0.00009604];

DisplayBasisVectors = false;
RunningOptimization = false;
TargetingBase = true;

% Create simulation object (assuming GuidedDartSim is in your MATLAB path)
sim = GuidedDartSim(Angle, DBVelocity, Position, Mass, InertiaMatrix, ...
    'DisplayBasisVectors', DisplayBasisVectors, ...
    'RunningOptimization', RunningOptimization, ...
    'TargetingBase', TargetingBase);

%% Run Simulation
% The simulation’s timestep is set to 0.001 s.
sim.run();

% After running, the following data is available:
%   sim.PlotTime: vector of time stamps (s)
%   sim.PlotPosition: N x 3 matrix of dart positions [X Y Z] (world coordinates)
%   sim.PlotDartBasis: cell array of 3x3 dart orientation matrices (but we'll recompute the ideal basis)
N = length(sim.PlotTime);

%% Compute Finite-Difference Velocity
velocity = zeros(N,3);
velocity(1,:) = (sim.PlotPosition(2,:) - sim.PlotPosition(1,:)) / sim.Timestep;
for idx = 2:N-1
    velocity(idx,:) = (sim.PlotPosition(idx+1,:) - sim.PlotPosition(idx-1,:)) / (2 * sim.Timestep);
end
velocity(N,:) = (sim.PlotPosition(N,:) - sim.PlotPosition(N-1,:)) / sim.Timestep;

%% Generate Ideal Lookup Table (Sample Every 5 ms)
dt_sample = 0.005;  % 5 ms sampling interval
lookupTable = [];  % Each row: [time, u, v, pitch, distance]
timeArray = sim.PlotTime;

% Sample every 5 simulation steps (if Timestep = 0.001 s)
sampleIndices = 1:5:N;
for idx = sampleIndices
    t = sim.PlotTime(idx);
    pos = sim.PlotPosition(idx,:)';   % dart position (3x1 vector)
    
    % Get velocity at this timestep.
    v = velocity(idx,:)';
    % Compute the normalized velocity vector.
    v_norm = v / norm(v);
    % Compute ideal Euler angles (using ZYX convention with roll=0) such that
    % R * [0; 0; 1] = v_norm.
    % From the relation:
    %    [v_x; v_y; v_z] = [ sin(pitch)*cos(yaw); sin(pitch)*sin(yaw); cos(pitch) ]
    % We obtain:
    idealPitch = acos(v_norm(3));   % pitch angle
    idealYaw = atan2(v_norm(2), v_norm(1));  % yaw angle
    % Set roll to zero.
    idealRoll = 0;
    
    % Construct the ideal dart basis using eul2rotm (angles in [yaw, pitch, roll])
    IdealBasis = eul2rotm([idealYaw, idealPitch, idealRoll], 'ZYX');
    
    % Compute camera position: 0.1 m ahead along the ideal dart's body z-axis.
    CameraPos = pos + 0.1 * IdealBasis(:,3);
    
    % Compute vector from camera to the green light in world coordinates.
    vec = targetGreenWorld - CameraPos;
    
    % Transform the vector into the camera coordinate system using the IdealBasis.
    X_cam = IdealBasis' * vec;  % [x_cam; y_cam; z_cam]
    
    % Project using the pinhole camera model (flip the y component to correct for image coordinate system).
    u = fx * (X_cam(1) / X_cam(3)) + cx;
    v = fy * (-X_cam(2) / X_cam(3)) + cy;
    
    % Append the row: [time, u, v, idealPitch, distance]
    distance = norm(vec);
    lookupTable = [lookupTable; t, u, v, idealPitch, distance];
end

% Save the ideal lookup table to a MAT-file.
save('LookupTable.mat', 'lookupTable');

%% Display the Ideal Lookup Table
disp('Ideal Lookup Table (columns: time, u, v, pitch (rad), distance (m)):' );
disp(lookupTable);

%% Plot Lookup Table Data
figure;
subplot(2,1,1);
plot(lookupTable(:,1), lookupTable(:,2), 'b-', 'LineWidth', 2); hold on;
plot(lookupTable(:,1), lookupTable(:,3), 'r-', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Pixel Coordinates');
legend('u','v');
title('Projected Green Light Coordinates vs. Time (Ideal)');

subplot(2,1,2);
plot(lookupTable(:,1), rad2deg(lookupTable(:,4)), 'k-', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Pitch Angle (deg)');
title('Dart Pitch Angle vs. Time (Ideal)');

figure;
plot(lookupTable(:,1), lookupTable(:,5), 'm-', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Distance (m)');
title('Distance from Camera to Green Light vs. Time (Ideal)');
