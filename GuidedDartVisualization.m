%All angles in radians, units in mkgs
clear all
close all
clc

figure

iterNum = 1;
HitTracker=zeros(iterNum,1);

RandomError=false;
disp("Running...")
tic
for k = 1:iterNum
    %Starting conditions

    DisplayBasisVectors=true;
    TargetingBase=true;
    Pitch=deg2rad(45);

    if RandomError==true
        PitchAngleAlignmentError=normrnd(0,0.003);
        YawAngleAlignmentError=normrnd(0,0.003);
        YawPositionError=0.0003835*rand-0.0003835*rand;
        InitialVelocityError=normrnd(0,0.03);
        AirVelocity = [normrnd(0,0.1), 0, normrnd(0,0.1)];
    else
        PitchAngleAlignmentError=0;
        YawAngleAlignmentError=0;
        YawPositionError=0;
        InitialVelocityError=0;
    end
    
    if TargetingBase
        Angle=[Pitch+PitchAngleAlignmentError;0.126689635808396+YawPositionError+YawAngleAlignmentError;0]; %pitch,yaw,roll
        DBVelocity=[0;0; 16.38+InitialVelocityError]; %Dart Basis velocity-> Z is along longitudinal axis of dart
    else 
        Angle=[Pitch+PitchAngleAlignmentError;-0.10992110217290+YawPositionError+YawAngleAlignmentError;0]; %pitch,yaw,roll
        DBVelocity=[0;0;13.075933425262566+InitialVelocityError]; %Dart Basis velocity-> Z is along longitudinal axis of dart
    end
    
    Position=[4.225;0.94;0.5852483];
    Mass=.208;
    InertiaMatrix=[0.00086057,0.0,0.00000021
                   0.0,0.00086107,0.00000313
                   0.00000021,0.00000313,0.00009604]; %From SolidWorks
    
    %Initializes the simulation and runs it:
    CurrentSim = GuidedDartSim(Angle, DBVelocity, Position, Mass, InertiaMatrix, DisplayBasisVectors = DisplayBasisVectors);
    CurrentSim.run();
    
    %Plot current simulation run:
    plot3(CurrentSim.PlotPosition(:,1),CurrentSim.PlotPosition(:,3),CurrentSim.PlotPosition(:,2), 'blue')

    %plot(CurrentSim.PlotTime,CurrentSim.PlotMisalignment(:,2))
    hold on
    for basis = CurrentSim.PlotDartBasisX
        plot3(basis(1),basis(2),basis(3),'green','LineWidth',2); 
    end
    
    
    if CurrentSim.TargetHit 
        HitTracker(k)=1;
    end
end
toc

%Plot Targets
X=[2.354053,2.354053,2.489053,2.489053];
Z=[16.6838448,16.7946805,16.7946805,16.6838448];
Y=[1.4989477,1.5566663,1.5566663,1.4989477];
fill3(X,Z,Y,'r')
%Plot Green Light
[x,y,z] = sphere;
% Scale to desire radius.
radius = .04;
x = x * radius;
y = y * radius;
z = z * radius;
% Height offset from target
offset = -.1;
% Plot as surface.
surf(x+X(1)+(X(3)-X(1))/2,z+Z(1),y+Y(1)+offset, EdgeColor='g') 

X=[7.4325335,7.4325335,7.5675337,7.5675337];
Z=[26.0548107,26.1656942,26.1656942,26.0548107];
Y=[1.0484971,1.1062019,1.1062019,1.0484971];
fill3(X,Z,Y,'r')
%Plot Green Light
[x,y,z] = sphere;
% Scale to desire radius.
radius = .04;
x = x * radius;
y = y * radius;
z = z * radius;
% Height offset from target
offset = -.1;
% Plot as surface.
surf(x+X(1)+(X(3)-X(1))/2,z+Z(1),y+Y(1)+offset, EdgeColor='g') 

hold off
xlabel('X Position(m)') 
ylabel('Z Position(m)') 
zlabel('Y Position(m)') 
axis equal
grid on
xlim([0,15])
ylim([0,28])
zlim([0,8])

sans = sum(HitTracker)./iterNum;
disp(sans)

%hold off
%xlabel('Time')
%ylabel('X misalignment')
%{
figure
plot(PlotTime,PlotMisalignment(:,2))
xlabel('Time')
ylabel('Y misalignment')
%}

