function [wheel_vert_load, dyn_brake, dyn_outer_vert_wheel_load ... 
            effective_lat_weight_transfer, effective_corner_force] = ... 
            sus_forces(totalWeight, downforceMaxG, centerMass, trackWidth, ... 
            weightDistro, aero_wheel_loads, TEST_CORNER_RADIUS, GRAV, ... 
            long_axle_load)
%%#############################################################################
% Washington State University
% 2017-18 Formula SAE
%
% Suspension Design Program
% Suspension Force Calculations
%
% Created by Timm Von Derau
% Last Modified: 11/3/2017
%%#############################################################################

%% Table of Dynamic Multiplication Factors
DYN_MAX_VERT = 3.0;         % General vertical max dynamic load multiplier
DYN_MAX_TORSION = 1.3;      % On vertical loads, diagonally opposite wheels on 
                            %       high spots
DYN_MAX_GRIP = 1.3;         % Cornering, braking, acceleration
DYN_COM_VERT = 2.0;         % Combined Load, not common, lower multiplier
DYN_COM_OTHER = 1.1;        % Same
DYN_MU = 1.2;               % Dynamic tire friction coefficient
MU = 1.5;

%% Suspension Design
pushrodAngleF = 54.03;      % deg
pushrodAngleR = 41.52;      % deg
    rodTheta = [pushrodAngleF pushrodAngleR];
ucaAngleF = 11.47;          % deg
ucaAngleR = 17.13;          % deg
    ucaTheta = [ucaAngleF ucaAngleR];
frontH1 = 5.5867;           % in
rearH1 = 5.895;             % in
    h1 = [frontH1 rearH1];
frontH2 = 8.9716;           % in
rearH2 = 8.3138;            % in
    h2 = [frontH2 rearH2];
fr_rollingRad = 10.25;      % in
rr_rollingRad = 10.125;     % in
    rollingRadius = [fr_rollingRad rr_rollingRad];

    
    
%% Calculations
fprintf('##################################################################\n')
fprintf('               >>> Suspension Forces [Design] <<<\n')
fprintf('##################################################################\n\n')
% Maximum vertical load
vert_load_per_side = 0.5 * ((totalWeight * DYN_MAX_VERT) + (totalWeight * ... 
                                downforceMaxG * DYN_MAX_GRIP));     % lb
fr_wheel_vert_load = vert_load_per_side * weightDistro(1);          % lb
rr_wheel_vert_load = vert_load_per_side - fr_wheel_vert_load;       % lb
    wheel_vert_load = [fr_wheel_vert_load rr_wheel_vert_load];      % lb
fprintf('Dynamic vertical load per tire\n')
fprintf('    Front  %0.3f lb    Rear  %0.3f lb\n', ... 
            wheel_vert_load(1), wheel_vert_load(2))

% Maximum braking load
fr_brake_per_tire = aero_wheel_loads(1) * DYN_MAX_GRIP;           % lb
rr_brake_per_tire = aero_wheel_loads(2) * DYN_MAX_GRIP;           % lb
    brake_per_tire = [fr_brake_per_tire rr_brake_per_tire];
    dyn_brake = DYN_MU * [fr_brake_per_tire rr_brake_per_tire];
fprintf('Dynamic braking load per tire\n')
fprintf('    Front  %0.3f lb    Rear  %0.3f lb\n', ...
            dyn_brake(1), dyn_brake(2))
disp('(These are horizontal loads at the contact patch)')

% Maximum cornering load
effective_weight = totalWeight + (totalWeight * downforceMaxG);   % lb
fprintf('Total Effective Weight (with Aero) = %0.3f lb\n\n', effective_weight)
effective_corner_force = effective_weight * DYN_MU;
disp('>>> Design Cornering Forces <<<')
fprintf('Design lateral force = %0.3f lb\n', effective_corner_force)

fr_effective_lat_weight_transfer = (effective_corner_force * centerMass(2)) ...
                                        / trackWidth(1);
rr_effective_lat_weight_transfer = (effective_corner_force * centerMass(2)) ...
                                        / trackWidth(2);
    effective_lat_weight_transfer = [fr_effective_lat_weight_transfer ...
                                        rr_effective_lat_weight_transfer];
fprintf('Design lateral weight transfer\n    Front   %0.3f lb    Rear   %0.3f lb\n',...
            effective_lat_weight_transfer(1), effective_lat_weight_transfer(2))

fr_outer_vert_wheel_load = (0.5 * effective_weight * weightDistro(1)) + ...
                            (effective_lat_weight_transfer(1) * 0.625);
rr_outer_vert_wheel_load = (0.5 * effective_weight * weightDistro(2)) + ...
                            (effective_lat_weight_transfer(2) * 0.625);
outer_vert_wheel_load = DYN_MAX_GRIP * [fr_outer_vert_wheel_load rr_outer_vert_wheel_load];
    dyn_outer_vert_wheel_load = DYN_MAX_GRIP * DYN_MU * ...
                                    [fr_outer_vert_wheel_load ...
                                     rr_outer_vert_wheel_load];
fprintf('Design outer wheel lateral cornering force (applied at contact patch)\n')
fprintf('    Front   %0.3f lb    Rear   %0.3f lb\n', dyn_outer_vert_wheel_load(1),...
            dyn_outer_vert_wheel_load(2))
disp('Force goes INTO the suspension')
carMass = totalWeight / GRAV;
latCornerSpeedAero = sqrt((effective_corner_force * TEST_CORNER_RADIUS) / ...
                                carMass);
%fprintf('Test Corner Speed, WITH AERO = %0.3f ft/s  ->  %0.f mph\n\n', ...
%            latCornerSpeedAero, (latCornerSpeedAero * 3600.0) / 5280.0)

% Maximum Acceleration load
fprintf('\n>>> Design Acceleration Forces <<<\n')
rear_wheel_load = long_axle_load(2) / 2;             % lb, individual wheel load
accel_design_load = rear_wheel_load * DYN_MAX_GRIP;  % lb
design_accel_force = accel_design_load * DYN_MU;     % lb, design max accel force for FEA
                                                     % this force is applied at the HUB center
fprintf('Acceleration Design Load = %0.3f lb\n', accel_design_load)
fprintf('Design Acceleration Force = %0.3f lb\n', design_accel_force)
fprintf('    (This force is applied at the hub, not the contact patch)\n\n')

%% FEA Forces
disp('########################################################################')
disp('                      >>> Forces for FEA <<<')
disp('########################################################################')

fprintf('\n>>> Wishbone and Pushrod Forces <<<\n')
%% Pushrod
disp('The vertical load is handled through the pushrod (axial compression)')
pushrod_force = zeros(2,1);
for n = 1:2;
    pushrod_force(n) = wheel_vert_load(n) / sind(rodTheta(n));
end
fprintf('    Front - %0.3f lb    Rear - %0.3f lb\n', pushrod_force(1), ...
            pushrod_force(2))
fprintf('\nThe horizontal load from the pushrod is resisted by the LCA (tension)\n')
pushrod_hor = zeros(2,1);
for n = 1:2;
    pushrod_hor(n) = pushrod_force(n) * cosd(rodTheta(n));
end
fprintf('    Front - %0.3f lb    Rear - %0.3f lb\n\n', pushrod_hor(1), ...
            pushrod_hor(2))

%% Wishbones
disp('Lateral cornering forces are resisted by the LCA and UCA')
uca_force = zeros(2,1);
for n = 1:2
   uca_force(n) = ((h1(n)/h2(n)) * dyn_outer_vert_wheel_load(n)) / ...
                    cosd(ucaTheta(n));
end
fprintf('       Upper Control Arm Forces (tension)\n')
fprintf('    Front - %0.3f lb    Rear - %0.3f lb\n', ...
            uca_force(1), uca_force(2))
lca_force = zeros(2,1);            
for n = 1:2
    lca_force(n) = ((h1(n) + h2(n))/h2(n)) * dyn_outer_vert_wheel_load(n);
end
fprintf('\n       Lower Control Arm Forces (compression)\n')
fprintf('    Front - %0.3f lb    Rear - %0.3f lb\n\n', ...
            lca_force(1), lca_force(2))

% wishbone braking
disp('Braking Forces in UCA and LCA')
uca_brake_force = zeros(2,1);
for n = 1:2
   uca_brake_force(n) = ((h1(n)/h2(n)) * dyn_brake(n));
end
fprintf('       Upper Control Arm Forces (against brake vector)\n')
fprintf('    Front - %0.3f lb    Rear - %0.3f lb\n', ...
            uca_brake_force(1), uca_brake_force(2))
lca_brake_force = zeros(2,1);            
for n = 1:2
    lca_brake_force(n) = ((h1(n) + h2(n))/h2(n)) * dyn_brake(n);
end
fprintf('\n       Lower Control Arm Forces (with brake vector)\n')
fprintf('    Front - %0.3f lb    Rear - %0.3f lb\n\n', ...
            lca_brake_force(1), lca_brake_force(2))

% wishbone acceleration
accel_load = zeros(2,1);
for n = 1:2
    accel_load(n) = long_axle_load(n)/2 * DYN_MAX_GRIP * MU;
end
    disp('Acceleration Forces in UCA and LCA')
uca_accel_force = zeros(2,1);
for n = 1:2
   uca_accel_force(n) = ((h1(n)/h2(n)) * accel_load(n));
end
fprintf('       Upper Control Arm Forces (against accel vector)\n')
fprintf('    Front - %0.3f lb    Rear - %0.3f lb\n', ...
            uca_accel_force(1), uca_accel_force(2))
lca_accel_force = zeros(2,1);            
for n = 1:2
    lca_accel_force(n) = ((h1(n) + h2(n))/h2(n)) * accel_load(n);
end
fprintf('\n       Lower Control Arm Forces (with accel vector)\n')
fprintf('    Front - %0.3f lb    Rear - %0.3f lb\n\n', ...
            lca_accel_force(1), lca_accel_force(2))

%% Upright Forces
% Upright Cornering
rotorHatWidth = .25;
spacerWidth = .75;
tireWidth = 6.0;
wheelBackspace = -.4313;             % in
wheelOffset = rotorHatWidth + spacerWidth + wheelBackspace;   % in
bearingWidth = 10 / 25.4;           % in
del = [1.35 1];                    % in
outerBearingForce = zeros(2,1);
for n = 1:2
    outerBearingForce(n) = ((dyn_outer_vert_wheel_load(n) * rollingRadius(n)) - ... 
            (outer_vert_wheel_load(n) * (bearingWidth + del(n) + wheelOffset))) ...
            / (bearingWidth + del(n));
end
fprintf('  --------------- Upright Forces ---------------\n')
% Max Cornering
disp('     >>> Cornering Forces <<<')
fprintf('The outer bearing presses down into its race\n')
fprintf('    Front - %0.3f lb     Rear - %0.3f lb\n\n', outerBearingForce(1), ...
                outerBearingForce(2))
innerBearingForce = zeros(2,1);
for n = 1:2
    innerBearingForce(n) = outerBearingForce(n) + outer_vert_wheel_load(n);
end
fprintf('The inner bearing presses up into its race\n')
fprintf('    Front - %0.3f lb     Rear - %0.3f lb\n\n', innerBearingForce(1), ...
                innerBearingForce(2))
fprintf('The outer bearing presses into its retaining shoulder to resist lateral load\n')
fprintf('    Front - %0.3f lb     Rear - %0.3f lb\n\n', dyn_outer_vert_wheel_load(1), ...
            dyn_outer_vert_wheel_load(2))
% Max braking
disp('     >>> Braking Forces <<<')
l3 = 3.4961;            % in
l4 = 8 / 25.4;          % in
l5 = 3.3071;            % in
outVertBrakeBearingForce = zeros(2,1);
outHorBrakeBearingForce = zeros(2,1);
for n = 1:2
    outVertBrakeBearingForce(n) = (brake_per_tire(n) * (bearingWidth + del(n) + ...
                                wheelOffset)) / (bearingWidth + del(n));
    outHorBrakeBearingForce(n) = (dyn_brake(n) * (bearingWidth + del(n) + ...
                                wheelOffset)) / (bearingWidth + del(n));
end
inVertBrakeBearingForce = zeros(2,1);
inHorBrakeBearingForce = zeros(2,1);
for n = 1:2
    inVertBrakeBearingForce(n) = outVertBrakeBearingForce(n) - brake_per_tire(n);
    inHorBrakeBearingForce(n) = outHorBrakeBearingForce(n) - dyn_brake(n);
end
fprintf('The outer bearing presses up into the race\n')
fprintf('    Front - %0.3f lb     Rear - %0.3f lb\n\n', outVertBrakeBearingForce(1),...
            outVertBrakeBearingForce(2))
fprintf('The outer bearing presses towards the caliper mount\n')
fprintf('    Front - %0.3f lb     Rear - %0.3f lb\n\n', outHorBrakeBearingForce(1),...
            outHorBrakeBearingForce(2))
fprintf('The inner bearing presses down into its race\n')
fprintf('    Front - %0.3f lb     Rear - %0.3f lb\n\n', inVertBrakeBearingForce(1),...
            inVertBrakeBearingForce(2))
fprintf('The inner bearing presses away from the caliper mount\n')
fprintf('    Front - %0.3f lb     Rear - %0.3f lb\n\n', inHorBrakeBearingForce(1),....
             inHorBrakeBearingForce(2))
% Max Acceleration
disp('     >>> Acceleration Forces - Rear Wheels Only <<<')
outVertAccelBearingForce = (accel_load(2) / MU) * ((bearingWidth + del(2) + ...
                                wheelOffset)) / (bearingWidth + del(2));
outHorAccelBearingForce = accel_load(2) * ((bearingWidth + del(2) + ...
                                wheelOffset)) / (bearingWidth + del(2));             
inVertAccelBearingForce = outVertAccelBearingForce - (accel_load(2)/ MU);
inHorAccelBearingForce = outHorAccelBearingForce - accel_load(2);
fprintf('The outer bearing presses up into its race\n')
fprintf('    REAR ONLY - %0.3f lb\n\n', outVertAccelBearingForce)
fprintf('The outer bearing presses away from the caliper mount\n')
fprintf('    REAR ONLY - %0.3f lb\n\n', outHorAccelBearingForce)
fprintf('The inner bearing presses down into its race\n')
fprintf('    REAR ONLY - %0.3f lb\n\n', inVertAccelBearingForce)
fprintf('The inner bearing presses towards the caliper mount\n')
fprintf('    REAR ONLY - %0.3f lb\n\n', inHorAccelBearingForce)


disp('########################################################################')
disp('                       >>> End FEA Forces <<<')
disp('########################################################################')
end