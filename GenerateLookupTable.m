%% GenerateLookupTable.m
% This script runs the GuidedDartSim simulation and generates a lookup table 
% for the projected coordinates of a green light as seen by a virtual camera.
% The camera is mounted 100 mm (0.1 m) in front of the dart’s origin (along the dart's z‑axis)
% and uses the following intrinsics: resolution 1280x800, HFOV = 85°, VFOV = 60°.
% The lookup table is organized as [time, u, v, pitch, distance].

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
% For the "base" target, the target center is approximately:
%   X_center = 7.5 m, Y_center = 1.077 m, Z_center = 26.11 m.
% With the green light 100 mm below the target, subtract 0.1 m from the Y-coordinate.
targetGreenWorld = [7.5; 1.077 - 0.1; 26.11];

%% Set Up Simulation Parameters (Example for Targeting Base)
Pitch = deg2rad(45);  
Yaw = 0.126689635808396;  % example value for base targeting
Angle = [Pitch; Yaw; 0];  % [pitch; yaw; roll]
DBVelocity = [0; 0; 16.38];  % dart basis velocity (Z-axis)
Position = [4.225; 0.94; 0.5852483];
Mass = 0.208;
InertiaMatrix = [0.00086057, 0.0, 0.00000021;
                 0.0, 0.00086107, 0.00000313;
                 0.00000021, 0.00000313, 0.00009604];

DisplayBasisVectors = false;
RunningOptimization = false;
TargetingBase = true;

% Create simulation object (make sure GuidedDartSim is in your MATLAB path)
sim = GuidedDartSim(Angle, DBVelocity, Position, Mass, InertiaMatrix, ...
    'DisplayBasisVectors', DisplayBasisVectors, ...
    'RunningOptimization', RunningOptimization, ...
    'TargetingBase', TargetingBase);

%% Run Simulation
% The simulation’s timestep is set to 0.001 s.
sim.run();

% After running, the following data is available:
%   sim.PlotTime      : vector of time stamps (s)
%   sim.PlotPosition  : N x 3 matrix of dart positions [X Y Z] in world coordinates
%   sim.PlotDartBasis : cell array where each cell is a 3x3 dart basis matrix

%% Generate Lookup Table (Sample Every 5 ms)
dt_sample = 0.005;  % 5 ms
lookupTable = [];  % Each row: [time, u, v, pitch, distance]
timeArray = sim.PlotTime;
N = length(timeArray);

% Since simulation timestep is 0.001 s, sample every 5 steps:
sampleIndices = 1:5:N;
for idx = sampleIndices
    t = sim.PlotTime(idx);
    pos = sim.PlotPosition(idx,:)';   % Column vector [x; y; z]
    DartBasis = sim.PlotDartBasis{idx}; % 3x3 orientation matrix (columns are basis vectors)
    
    % Compute camera position: 0.1 m ahead along the dart's local z-axis.
    CameraPos = pos + 0.1 * DartBasis(:,3);
    
    % Compute vector from camera to the green light in world coordinates.
    vec = targetGreenWorld - CameraPos;
    
    % Transform the vector into the camera coordinate system.
    % Since the camera's rotation is given by DartBasis, we have:
    X_cam = DartBasis' * vec;  % X_cam = [x_cam; y_cam; z_cam]
    
    % Project using the pinhole camera model (flip the y component).
    u = fx * (X_cam(1) / X_cam(3)) + cx;
    v = fy * (-X_cam(2) / X_cam(3)) + cy;  % flip y to correct upside-down projection
    
    % Estimate the dart’s pitch angle.
    % One simple approach: use the vertical component of the dart's forward (z-axis) vector.
    pitch = asin(DartBasis(2,3));  % assuming Y is the vertical axis.
    
    % Compute the straight-line distance from the camera to the green light.
    distance = norm(vec);
    
    % Append the data: [time, u, v, pitch, distance]
    lookupTable = [lookupTable; t, u, v, pitch, distance];
end

% Save the lookup table to a MAT-file.
save('LookupTable.mat', 'lookupTable');

%% Display the Lookup Table
disp('Lookup Table (columns: time, u, v, pitch (rad), distance (m)):');
disp(lookupTable);

%% Plot Lookup Table Data
figure;
subplot(2,1,1);
plot(lookupTable(:,1), lookupTable(:,2), 'b-', 'LineWidth', 2); hold on;
plot(lookupTable(:,1), lookupTable(:,3), 'r-', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Pixel Coordinates');
legend('u','v');
title('Projected Green Light Coordinates vs. Time');

subplot(2,1,2);
plot(lookupTable(:,1), rad2deg(lookupTable(:,4)), 'k-', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Pitch Angle (deg)');
title('Dart Pitch Angle vs. Time');

figure;
plot(lookupTable(:,1), lookupTable(:,5), 'm-', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Distance (m)');
title('Distance from Camera to Green Light vs. Time');
