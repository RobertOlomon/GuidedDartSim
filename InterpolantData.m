
%Import reference results from Solidworks Fluid Simulation:
load 3DSimulationData.mat ScatteredStudy
%Solidworks Simulation Data (dart-centric basis X left, Y up, Z forward)
RefVelocityX=-ScatteredStudy{2,:}'; %left-right
RefVelocityY=-ScatteredStudy{1,:}'; %up-down
RefVelocityZ=-ScatteredStudy{3,:}'; %forward-backward
RefForceX=ScatteredStudy{4,:}'; %left-right
RefForceY=ScatteredStudy{5,:}'; %up-down
RefForceZ=ScatteredStudy{6,:}'; %forward-backward
RefTorqueX=ScatteredStudy{7,:}'; %Yaw
RefTorqueY=ScatteredStudy{8,:}'; %Pitch
RefTorqueZ=ScatteredStudy{9,:}'; %Roll
            
%Create interpolants of sim data
ForceXInterpolant=scatteredInterpolant(RefVelocityX,RefVelocityY,RefVelocityZ,RefForceX);
ForceYInterpolant=scatteredInterpolant(RefVelocityX,RefVelocityY,RefVelocityZ,RefForceY);
ForceZInterpolant=scatteredInterpolant(RefVelocityX,RefVelocityY,RefVelocityZ,RefForceZ);
TorqueXInterpolant=scatteredInterpolant(RefVelocityX,RefVelocityY,RefVelocityZ,RefTorqueX);
TorqueYInterpolant=scatteredInterpolant(RefVelocityX,RefVelocityY,RefVelocityZ,RefTorqueY);
TorqueZInterpolant=scatteredInterpolant(RefVelocityX,RefVelocityY,RefVelocityZ,RefTorqueZ);

save("Interpolants.mat", "ForceXInterpolant", "ForceYInterpolant", "ForceZInterpolant", "TorqueXInterpolant", "TorqueYInterpolant", "TorqueZInterpolant");