%%#############################################################################
% Washington State University
% 2017-18 Formula SAE
%
% Suspension Calculations and Graphs
% To be put into simulink eventually (theoretically)
%
% Created by Timm Von Derau
% Last modified: 10/2/2017
%%#############################################################################

%% NOTES: Average FSAE speed 29-36 mph, avg max 66 mph 


clc; clear all
fprintf('\nWashington State University\n2017-18 Suspension Calculations\n\n')

%% Global variables
GRAV = 32.2;					% Gravity, ft/s^2
MU = 1.5;						% Tire Friction Coefficient
MIN_WHEELBASE = 60.0;			% SAE mandated wheelbase, inches
TRACK_PERCENT = 0.75;			% SAE rule smaller track at least 75% of larger
TEST_CORNER_RADIUS = 50.0;      % Radius for test calculations, ft

%% Preliminary car variable
% All front / rear variables, like track, will be FRONT first then REAR
wheelBase = 63.5;				% inches
frontTrackWidth = 46.61;        % inches
rearTrackWidth = 45.404;        % inches
    trackWidth = [frontTrackWidth rearTrackWidth];
centerGravityHeight = 10.0;		% inches
totalWeight = 743;              % lbs, car + driver
corneringGForce = 0.8;			% g's (For test corners NOT at max )
weightDistro = [0.48 0.52];     % Percent Weight distribution
centerMassX = wheelBase * weightDistro(2);
                                % inches, from front axle centerGravityHeight
                                % (wb * weight balance rear)
centerMassY = 10.0;				% inches, from bottom 
    centerMass = [centerMassX centerMassY];
downforceMaxG = .5;            % downforce G's (100lb downforce at 30mph)
rearSpringRate = 310.0;         % in*lb
frontSpringRate = 285.0;        % in*lb 
    springRate = [frontSpringRate rearSpringRate];
rearRockerRatio = 1.0;          % front and rear bellcrank motion ratio
frontRockerRatio = 1.0;         
    rockerRatio = [frontRockerRatio rearRockerRatio];
frontDamperTravel = 3.0;        % inches
rearDamperTravel = 3.0;         % inches
frontComp = 1.5; frontDroop = frontDamperTravel - frontComp;   % inches
rearComp = 1.2; rearDroop = rearDamperTravel - rearComp;       % inches
    wheelTravel = [frontComp frontDroop rearComp rearDroop];
tireDia = 20.5;                 % inches 

%& Prelim requirements met?
disp('>>> SAE Prelim Requirements <<<')
if wheelBase >= MIN_WHEELBASE
	fprintf('Wheelbase (%0.2f) long enough\n', wheelBase)
	else fprintf('Needs longer wheelbase\n'); return
end
if min(trackWidth) >= TRACK_PERCENT * max(trackWidth)
	fprintf('%0.2f in Trackwidth stagger (%0.2f / %0.2f -> %0.3f%%) is legal\n\n', ... 
				trackWidth(1) - trackWidth(2), trackWidth(1), ...
                trackWidth(2), trackWidth(2)/trackWidth(1)*100)
	else fprintf('Narrow track needs to be wider\n\n'); return
end
fprintf(' ----------- CAR MEETS SAE REQUIREMENTS ----------- \n\n')

disp('>>> Design Goals <<<')
disp('Track to wheelbase ratio should be between 1.2 - 1.5, 1.3 optimum')
if wheelBase / ((trackWidth(1) + trackWidth(2)) / 2) >= 1.5
	fprintf('Track to Wheelbase Ratio not Met\n'); return
	elseif wheelBase / ((trackWidth(1) + trackWidth(2)) / 2) <= 1.2
		fprintf('Track to Wheelbase Ratio not Met\n'); return
	else fprintf('Track to Wheelbase Ratio is %0.3f\n\n', wheelBase / ...
					((trackWidth(1) + trackWidth(2)) / 2))
end
fprintf(' ----------- DESIGN GOALS MET ----------- \n\n')

%% Calculations
% Axle Loads
axle_loads = sus_axleLoad(totalWeight, centerMass, wheelBase);
% Longitudinal Weight Transfer
[long_axle_load, accelWeightTransfer, brakeWeightTransferNA, ...
    aero_wheel_loads, accelMax] = ...
            sus_longLoad(axle_loads, MU, centerMass, wheelBase, GRAV,...
            totalWeight, tireDia, downforceMaxG);
% NO AERO Lateral Weight Transfer
[lat_load_NA, lat_axle_weight_transfer_NA] = sus_latLoad(totalWeight, MU, ...
            centerMass, trackWidth, GRAV, TEST_CORNER_RADIUS);
% Wheel Travel under Weight Transfer
%weight_transfer_wheel_travel = sus_wheelTravelWeightTransfer( ...
 %           lat_axle_weight_transfer_NA, rockerRatio, springRate, ...
  %          long_axle_load, accelWeightTransfer, brakeWeightTransferNA,...
   %         wheelTravel, frontComp, frontDroop, rearComp, rearDroop);
% Forces in Suspension Components
[wheel_vert_load, dyn_brake, dyn_outer_vert_wheel_load, ... 
            effective_lat_weight_transfer, effective_corner_force] = ...
            sus_forces(totalWeight, downforceMaxG, centerMass, ...
            trackWidth, weightDistro, aero_wheel_loads, TEST_CORNER_RADIUS, ...
            GRAV, long_axle_load);
boltShearForce = sus_hardware(dyn_outer_vert_wheel_load, ...
                dyn_brake, wheel_vert_load, effective_lat_weight_transfer, ...
                effective_corner_force);
