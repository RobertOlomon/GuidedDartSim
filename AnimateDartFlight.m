%% AnimateDartFlightWithCameraFollow.m
% This script creates and runs a GuidedDartSim simulation, loads a dart STL file,
% scales it to the correct dart dimensions (length along local z-axis = 245.5 mm),
% animates the dart's flight by transforming the STL for each frame,
% and positions the camera to follow the dart.

clear; clc; close all;

%% Set Up Simulation Parameters (Example for Base Targeting)
Pitch = deg2rad(45);  
Yaw = 0.126689635808396;  % example value for base targeting
Angle = [Pitch; Yaw; 0];  % [pitch; yaw; roll]
DBVelocity = [0; 0; 16.38];  % dart basis velocity along Z
Position = [4.225; 0.94; 0.5852483];
Mass = 0.208;
InertiaMatrix = [0.00086057, 0.0, 0.00000021;
                 0.0, 0.00086107, 0.00000313;
                 0.00000021, 0.00000313, 0.00009604];

DisplayBasisVectors = false;
RunningOptimization = false;
TargetingBase = true;

% Create simulation object
sim = GuidedDartSim(Angle, DBVelocity, Position, Mass, InertiaMatrix, ...
    'DisplayBasisVectors', DisplayBasisVectors, ...
    'RunningOptimization', RunningOptimization, ...
    'TargetingBase', TargetingBase);


%% Run the Simulation to generate flight data
sim.run();

%% Load the Dart STL file and Scale it
% Load the STL file (assumed to be "dart.stl" in the working directory).
fv = stlread('dart.stl');  
% If the returned object is a triangulation, extract faces and vertices.
if isa(fv, 'triangulation')
    faces = fv.ConnectivityList;
    vertices = fv.Points;
else
    faces = fv.faces;
    vertices = fv.vertices;
end

% Compute the current dart length along its local z-axis.
z_min = min(vertices(:,3));
z_max = max(vertices(:,3));
currentLength = z_max - z_min;  % in STL units
desiredLength = 0.2455;         % 245.5 mm in meters
scaleFactor = desiredLength / currentLength;
% Scale the vertices
vertices = vertices * scaleFactor;

%% Set Up Figure and Initial Plot
figure;
axis equal;
grid on;
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
title('Dart Flight Animation with Camera Follow');
hold on;

% Optionally, plot the full flight path for reference.
plot3(sim.PlotPosition(:,1), sim.PlotPosition(:,2), sim.PlotPosition(:,3), 'b--', 'LineWidth', 1);

% Create a patch object for the dart.
hDart = patch('Faces', faces, 'Vertices', vertices, ...
              'FaceColor', 'red', 'EdgeColor', 'none', 'FaceLighting', 'gouraud');
          
% Set up lighting.
camlight('headlight');
material('dull');

%% Animation Loop with Camera Following
numFrames = size(sim.PlotPosition, 1);
for k = 1:numFrames
    % Get current simulation data.
    pos = sim.PlotPosition(k,:)';        % dart position (3x1 column vector)
    DartBasis = sim.PlotDartBasis{k};      % 3x3 orientation matrix (columns = basis vectors)
    
    % Transform the dart STL vertices from local to world coordinates.
    V_local = vertices;  % Nx3 matrix (already scaled)
    V_world = (DartBasis * V_local')';      % rotate vertices
    V_world = V_world + repmat(pos', size(V_world,1), 1);  % translate to current position
    
    % Update the patch object with the new vertices.
    set(hDart, 'Vertices', V_world);
    

    % Update the title with the current simulation time.
    title(sprintf('Dart Flight: t = %.3f s', sim.PlotTime(k)));
    
    drawnow;
    pause(0.005);  % adjust pause for desired animation speed
end
