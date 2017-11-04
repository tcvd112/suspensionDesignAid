function [long_axle_load, accelWeightTransfer, brakeWeightTransferNA, ...
            aero_wheel_loads, accelMax] ...
            = sus_longLoad(axle_loads, MU, centerMass, wheelBase, ...
                           GRAV, totalWeight, tireDia, downforceMaxG)
%%#############################################################################
% Washington State University
% 2017-18 Formula SAE
%
% Suspension Design Program
% Longitudinal Weight Transfer
%
% Created by Timm Von Derau
% Last Modified: 9/21/2017
%%#############################################################################

disp('>>> Max Forces for Longitudinal Weight Transfer <<<')
long_axle_load = zeros(4,1);	% 1 and 2 are accel F R, 3 4 are brake F R
% Acceleration
accelMax = (axle_loads(2)*MU)/(1 - (centerMass(2) * MU) / wheelBase);
accelWeightTransfer = zeros(2,1);
acceleration_transfer = accelMax * centerMass(2) / wheelBase;
accelWeightTransfer(1) = -acceleration_transfer;
accelWeightTransfer(2) = acceleration_transfer;
long_axle_load(1) = axle_loads(1) + accelWeightTransfer(1);
long_axle_load(2) = axle_loads(2) + accelWeightTransfer(2);
peakAccelTorqueRearWheels = long_axle_load(2) * (tireDia/2) * MU; % lb inches
% Braking NO AREO
brakeMaxNA = totalWeight * MU;
brakeWeightTransferNA = zeros(2,1);
brake_transfer = brakeMaxNA * centerMass(2) / wheelBase;
brakeWeightTransferNA(1) = brake_transfer;
brakeWeightTransferNA(2) = -brake_transfer;
long_axle_load(3) = axle_loads(1) + brakeWeightTransferNA(1);
long_axle_load(4) = axle_loads(2) + brakeWeightTransferNA(2);
% Total G's 
car_mass = totalWeight / GRAV;
maxAcceleration = accelMax / car_mass; maxAccelG = maxAcceleration / GRAV; 
maxBrakingNA = brakeMaxNA / car_mass; maxBrakeNAG = maxBrakingNA / GRAV;
fprintf('Max Acceleration (Traction limited) Force = %0.3f lbs\nWeight Transfer +/- %0.3f lbs\n', ...
			accelMax, acceleration_transfer)
fprintf('     Front Axle Load = %0.3f lbs\n     Rear Axle Load = %0.3f lbs\n', ...
			long_axle_load(1), long_axle_load(2))
fprintf('Peak Acceleration Torque = %0.3f lb*in\n', peakAccelTorqueRearWheels)
fprintf('Acceleration rate = %0.3f m/s^2  ->  %0.3f g\n\n', maxAcceleration, ... 
			maxAccelG)
disp('(non aero)')
fprintf('Max Braking Force = %0.3f lbs\nWeight Transfer +/- %0.3f lbs\n', ...
			brakeMaxNA, brake_transfer)
fprintf('     Front Axle Load = %0.3f lbs\n     Rear Axle Load = %0.3f lbs\n', ...
			long_axle_load(3), long_axle_load(4))
fprintf('Braking rate = %0.3f m/s^2  ->  %0.3f g\n\n', maxBrakingNA, maxBrakeNAG)


%% W/ Aero (Aero has litle to no effect on acceleration, and thus is neglected)

% Braking
effective_weight = totalWeight + (totalWeight * downforceMaxG);
rr_aero_axle_load = effective_weight * (centerMass(1) / wheelBase);
fr_aero_axle_load = effective_weight - rr_aero_axle_load;
    aero_axle_load = [fr_aero_axle_load rr_aero_axle_load];
DYN_MU = 1.2; aero_brake_max = effective_weight * DYN_MU;
aero_brake_transfer = aero_brake_max * centerMass(2) / wheelBase;
aeroBrakeWeightTransfer = zeros(2,1);
aeroBrakeWeightTransfer(1) = aero_brake_transfer;
aeroBrakeWeightTransfer(2) = -aero_brake_transfer;
aero_long_axle_load = zeros(2,1);
aero_long_axle_load(1) = aero_axle_load(1) + aeroBrakeWeightTransfer(1);
% Front weight transfer INCREASE
aero_long_axle_load(2) = aero_axle_load(2) + aeroBrakeWeightTransfer(2);
% Rear weight transfer DECREASE
aero_wheel_loads = zeros(2,1);
aero_wheel_loads(1) = aero_long_axle_load(1)/2; % Front left and right
aero_wheel_loads(2) = aero_long_axle_load(2)/2; % Rear left and right
% Drag braking
AIRBRAKE_DRAG = 0.7; % Gs
drag_force = totalWeight * AIRBRAKE_DRAG;
% total brake force
total_max_brake = drag_force + aero_brake_max;
% F=ma braking force
deceleration = total_max_brake / car_mass; decelG = deceleration / GRAV;

fprintf('Aero Braking (Theoretical)\n')
fprintf('Max Braking Force = %0.3f lbs\n', aero_brake_max)
fprintf('Weight Transfer +/- %0.3f lbs\n', aero_brake_transfer)
fprintf('    Front Axle Load = %0.3f lbs\n    Rear Axle Load = %0.3f lbs\n', ...
            aero_long_axle_load(1), aero_long_axle_load(2))
fprintf('Braking Rate = %0.3f m/s^2  ->  %0.3f G\n\n', ...
            deceleration, decelG)
end