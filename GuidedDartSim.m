%All angles in radians, units in mkgs
classdef GuidedDartSim < handle
    properties (AbortSet)
        Timestep;
        InitialAngle;
        Mass;
        InertiaMatrix;
        AirVelocity;
        AngularVelocity;
        Position;
        DBVelocity; % Dart Basis velocity -> Z is along longitudinal axis of dart
        DisplayBasisVectors;
        RunningOptimization;
        TargetingBase;
    end
    properties (Constant)
        ForceXInterpolant = load('Interpolants.mat').ForceXInterpolant;
        ForceYInterpolant = load('Interpolants.mat').ForceYInterpolant;
        ForceZInterpolant = load('Interpolants.mat').ForceZInterpolant;
        TorqueXInterpolant = load('Interpolants.mat').TorqueXInterpolant;
        TorqueYInterpolant = load('Interpolants.mat').TorqueYInterpolant;
        TorqueZInterpolant = load('Interpolants.mat').TorqueZInterpolant;
    end
    properties (SetAccess = private)
        PlotPitchAngularAcceleration;
        PlotMisalignment;
        PlotPosition;
        PlotHeadPosition;
        PlotDBVelocity;
        PlotTime;
        PlotAngularVelocity;
        TargetHit = false;
        Error;
        PlotDartBasis;  % Cell array to store full 3x3 dart orientation at each timestep.
        
        % New properties to store basis vectors for visualization every ~0.3 s.
        PlotDartBasisX; % Each entry is a 3x1 vector (x-axis of dart)
        PlotDartBasisY; % Each entry is a 3x1 vector (y-axis of dart)
        PlotDartBasisZ; % Each entry is a 3x1 vector (z-axis of dart)
        BasisTimes;     % Times at which the basis vectors were recorded.
    end
    properties
        MobileBase = 'fixed'; % New parameter: "fixed", "smooth_motion", or "random_motion"
    end

    methods
        % Class Constructor:
        function obj = GuidedDartSim(InitialAngle, DBVelocity, Position, Mass, InertiaMatrix, varargin)
            if nargin < 5
                error('Insufficient arguments');
            end
            DefaultTimestep = 0.001;
            DefaultAirVelocity = [0;0;0];
            DefaultAngularVelocity = [0;0;0];
            DefaultDisplayBasisVectors = false;
            DefaultRunningOptimization = false;
            DefaultTargetingBase = true;
            DefaultMobileBase = 'fixed';  % default behavior

            p = inputParser;
            addParameter(p,'Timestep',DefaultTimestep);
            addParameter(p,'AirVelocity',DefaultAirVelocity);
            addParameter(p,'AngularVelocity',DefaultAngularVelocity);
            addParameter(p,'DisplayBasisVectors',DefaultDisplayBasisVectors);
            addParameter(p,'RunningOptimization',DefaultRunningOptimization);
            addParameter(p,'TargetingBase',DefaultTargetingBase);
            addParameter(p,'MobileBase',DefaultMobileBase);
            parse(p, varargin{:});

            obj.InitialAngle = InitialAngle;
            obj.DBVelocity = DBVelocity;
            obj.Mass = Mass;
            obj.InertiaMatrix = InertiaMatrix;
            obj.Position = Position;
            obj.Timestep = p.Results.Timestep;
            obj.AirVelocity = p.Results.AirVelocity;
            obj.AngularVelocity = p.Results.AngularVelocity;
            obj.DisplayBasisVectors = p.Results.DisplayBasisVectors;
            obj.RunningOptimization = p.Results.RunningOptimization;
            obj.TargetingBase = p.Results.TargetingBase;
            obj.MobileBase = p.Results.MobileBase;
            
            % Initialize cell arrays.
            obj.PlotDartBasis = {};
            obj.PlotDartBasisX = {};
            obj.PlotDartBasisY = {};
            obj.PlotDartBasisZ = {};
            obj.BasisTimes = [];
        end

        % Runs the simulation for a given setup.
        % The run() method now uses the MobileBase parameter to simulate target motion.
        function run(obj)
            % Starting conditions
            DartBasis = AngleToDartBasis(obj.InitialAngle);
            Gravity = [0; -9.8083; 0]; % gravitational acceleration (m/s^2)
            HeadToCoMLength = 0.122;   % distance from dart position to head
            Velocity = DartBasisToWorldBasis(obj.DBVelocity, DartBasis);
            DecomposedInertiaMatrix = decomposition(obj.InertiaMatrix, 'lu');
            Time = 0;
            
            % Initialize target offset for mobile base modes (applied to the X-coordinate).
            targetOffset = 0;   % in meters
            if ~strcmp(obj.MobileBase, 'fixed')
                if strcmp(obj.MobileBase, 'smooth_motion')
                    offsetDir = 1;         % direction (+1 or -1)
                    offsetSpeed = 0.0005;  % m per timestep (adjustable)
                end
            end
            
            j = 1;
            while obj.Position(2) > 0
                % Record data for plotting.
                Misalignment = DartBasis(:,3) - Velocity./norm(Velocity);
                obj.PlotMisalignment(j,:) = Misalignment;
                obj.PlotPosition(j,:) = obj.Position;
                obj.PlotDBVelocity(j,:) = obj.DBVelocity;
                obj.PlotTime(j) = Time;
                obj.PlotAngularVelocity(j,:) = obj.AngularVelocity;
                obj.PlotDartBasis{j} = DartBasis;
                
                % Also store basis vectors every 0.3 seconds.
                if mod(Time, 0.3) < obj.Timestep
                    obj.PlotDartBasisX{end+1} = DartBasis(:,1);
                    obj.PlotDartBasisY{end+1} = DartBasis(:,2);
                    obj.PlotDartBasisZ{end+1} = DartBasis(:,3);
                    obj.BasisTimes(end+1) = Time;
                end
                
                % Interpolate aerodynamic forces and torques.
                DBAeroForceX = obj.ForceXInterpolant(obj.DBVelocity(1), obj.DBVelocity(2), obj.DBVelocity(3));
                DBAeroForceY = obj.ForceYInterpolant(obj.DBVelocity(1), obj.DBVelocity(2), obj.DBVelocity(3));
                DBAeroForceZ = obj.ForceZInterpolant(obj.DBVelocity(1), obj.DBVelocity(2), obj.DBVelocity(3));
                PitchTorque = obj.TorqueYInterpolant(obj.DBVelocity(1), obj.DBVelocity(2), obj.DBVelocity(3));
                YawTorque = obj.TorqueXInterpolant(obj.DBVelocity(1), obj.DBVelocity(2), obj.DBVelocity(3));
                RollTorque = obj.TorqueZInterpolant(obj.DBVelocity(1), obj.DBVelocity(2), obj.DBVelocity(3));
                
                % Translational Dynamics
                DBAeroForce = [DBAeroForceX; DBAeroForceY; DBAeroForceZ];
                AeroForce = DartBasisToWorldBasis(DBAeroForce, DartBasis);
                Acceleration = Gravity + (AeroForce ./ obj.Mass);
                obj.Position = obj.Position + Velocity * obj.Timestep + 0.5 * Acceleration * (obj.Timestep^2);
                HeadPosition = obj.Position + HeadToCoMLength * DartBasis(:,3);
                Velocity = Velocity + Acceleration * obj.Timestep;
                DBAirVelocity = WorldBasisToDartBasis(obj.AirVelocity, DartBasis);
                obj.DBVelocity = WorldBasisToDartBasis(Velocity, DartBasis) - DBAirVelocity;
                
                % Rotational Dynamics
                DBTorque = [PitchTorque; YawTorque; RollTorque];
                DBAngularAcceleration = DecomposedInertiaMatrix \ DBTorque;
                AngularAcceleration = DartBasisToWorldBasis(DBAngularAcceleration, DartBasis);
                DartBasis(:,1) = UpdateVector(obj.AngularVelocity, DartBasis(:,1), obj.Timestep);
                DartBasis(:,2) = UpdateVector(obj.AngularVelocity, DartBasis(:,2), obj.Timestep);
                DartBasis(:,3) = UpdateVector(obj.AngularVelocity, DartBasis(:,3), obj.Timestep);
                obj.AngularVelocity = obj.AngularVelocity + AngularAcceleration * obj.Timestep;
                
                % Update simulation time.
                Time = Time + obj.Timestep;
                
                % Update MobileBase target offset if applicable.
                if ~strcmp(obj.MobileBase, 'fixed')
                    if strcmp(obj.MobileBase, 'smooth_motion')
                        targetOffset = targetOffset + offsetDir * offsetSpeed;
                        if targetOffset >= 0.25
                            targetOffset = 0.25;
                            offsetDir = -1;
                        elseif targetOffset <= -0.25
                            targetOffset = -0.25;
                            offsetDir = 1;
                        end
                    elseif strcmp(obj.MobileBase, 'random_motion')
                        % Randomly update target offset (small random step)
                        targetOffset = targetOffset + 0.0005 * randn;
                        % Saturate within [-0.25, 0.25] meters.
                        targetOffset = max(min(targetOffset, 0.25), -0.25);
                    end
                end
                
                % Target Hit Condition for Base Target.
                % When MobileBase is active, shift the X boundaries by targetOffset.
                if obj.TargetingBase
                    x_lower = 7.4325335 + targetOffset;
                    x_upper = 7.5675337 + targetOffset;
                    if (x_lower < obj.Position(1)) && (obj.Position(1) < x_upper) && ...
                       (1.0484971 < obj.Position(2)) && (obj.Position(2) < 1.1062019) && ...
                       (26.0548107 < obj.Position(3)) && (obj.Position(3) < 26.1656942)
                        obj.TargetHit = true;
                        break;
                    end
                else
                    % Outpost target condition (unchanged)
                    if (2.354053 < obj.Position(1)) && (obj.Position(1) < 2.489053) && ...
                       (1.4989477 < obj.Position(2)) && (obj.Position(2) < 1.5566663) && ...
                       (16.6838448 < obj.Position(3)) && (obj.Position(3) < 16.7946805)
                        obj.TargetHit = true;
                        break;
                    end
                end
                
                obj.PlotPitchAngularAcceleration(j,:) = DBAngularAcceleration(1);
                obj.PlotHeadPosition(j,:) = HeadPosition;
                j = j + 1;
            end
        end
    end
end

% Helper function: Convert world basis vector to dart basis.
function f = WorldBasisToDartBasis(WorldBasisVector, DartBasis)
    f = DartBasis \ WorldBasisVector;
end

% Helper function: Convert dart basis vector to world basis.
function p = DartBasisToWorldBasis(DartBasisVector, DartBasis)
    p = DartBasis * DartBasisVector;
end

% Helper function: Update a vector using Rodrigues' rotation.
function d = UpdateVector(FixedAxisVector, Vector, Timestep)
    if all(FixedAxisVector == [0; 0; 0])
        RodriguesMatrix = eye(3);
    else
        Theta = Timestep * norm(FixedAxisVector);
        FixedAxisVector = FixedAxisVector / norm(FixedAxisVector);
        FixedAxisMatrix = [0, -FixedAxisVector(3), FixedAxisVector(2);
                           FixedAxisVector(3), 0, -FixedAxisVector(1);
                          -FixedAxisVector(2), FixedAxisVector(1), 0];
        RodriguesMatrix = eye(3) + FixedAxisMatrix*sin(Theta) + (FixedAxisMatrix^2)*(1-cos(Theta));
    end
    d = RodriguesMatrix * Vector;
end

% Helper function: Convert initial pitch and yaw angle to a dart basis.
function t = AngleToDartBasis(Angle)
    RotationMatrix = [cos(Angle(2)), 0, sin(Angle(2));
                      0, 1, 0;
                     -sin(Angle(2)), 0, cos(Angle(2))] * ...
                     [1, 0, 0;
                      0, cos(-Angle(1)),-sin(-Angle(1));
                      0, sin(-Angle(1)), cos(-Angle(1))];
    DartBasis(:,1) = -RotationMatrix * [1; 0; 0];
    DartBasis(:,2) = RotationMatrix * [0; 1; 0];
    DartBasis(:,3) = RotationMatrix * [0; 0; 1];
    t = DartBasis;
end
