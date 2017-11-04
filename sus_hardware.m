function [boltShearForce] = sus_hardware(dyn_outer_vert_wheel_load, ...
          dyn_brake, wheel_vert_load, effective_lat_weight_transfer, ...
          effective_corner_force)
%%#############################################################################
% Washington State University
% 2017-18 Formula SAE
%
% Suspension Design Program
% Hardware Calculations
%
% Created by Timm Von Derau
% Last Modified: 10/2/2017
%%#############################################################################

%% Calculate the size of the tabs, the size of the hardware, and any other
%  hardware considerations for the suspension system.

% DESIGN GOAL: The reasoning behind this file it to find the best solution of
%               of weight savings and strength, specifically, moving from 5/16
%               to 1/4" hardware. 

%% ALL CALCULATIONS DONE FOR DOUBLE SHEAR WHEN APPLICABLE

fprintf('\n>>> Hardware Forces <<<\n')

%% Lookup values (POTENTIALLY NOT ALL NEEDED)
quarter_Sy = 130;                        % kpsi
fiveSixteen_Sy = 130;                    % kpsi
    Sy = [quarter_Sy fiveSixteen_Sy];
quarter_Sp = 120;                        % kpsi
fiveSixteen_Sp = 120;                    % kpsi
    Sp = [quarter_Sp fiveSixteen_Sp];
quarter_Sut = 150;                       % kpsi
fiveSixteen_Sut = 150;                   % kpsi
    Sut = [quarter_Sut fiveSixteen_Sut];
quarter_dia = 0.25;                      % in
fiveSixteen_dia = 0.3125;                % in
    diameter = [quarter_dia fiveSixteen_dia];
quarter_sph_thickness = 0.593;           % in
fiveSixteen_sph_thickness = 0.813;       % in
    spherical_thickness = [quarter_sph_thickness fiveSixteen_sph_thickness];

%% DESIGN SPECS - Tab thickeness and factor of safety desired
tabThickness = 0.1046;                   % in
factorSafety = 4;

%% calculations
boltShearForce = zeros(2,1);             % units will be KIPS, 
                                         % multiply by 1000 for lb
for n = 1:2;
    boltShearForce(n) = (0.557 * Sy(n) * 2 * pi * (diameter(n)/2).^2) ...
                            / factorSafety;
end
fprintf('Total force before exceeding %0.1fx factor of safety\n', ...
            factorSafety)
disp('- Double Shear')
fprintf('    1/4 inch - %0.3f lb\n    5/16 inch - %0.3f lb\n', ... 
            boltShearForce(1) * 1000, boltShearForce(2) * 1000)
disp('- Single Shear')
fprintf('    1/4 inch - %0.3f lb\n    5/16 inch - %0.3f lb\n\n', ... 
            boltShearForce(1) * 1000 / 2, boltShearForce(2) * 1000 / 2)
actualFactor = zeros(2,1);
for n = 1:2;
    actualFactor(n) = (1000 * 0.557 * Sy(n) * 2 * pi * (diameter(n)/2).^2) ...
                            / effective_corner_force;
end
            
%% Argument for 1/4 in
disp('--- CONCLUSIONS ---')
if effective_corner_force < boltShearForce(1) * 1000            
    disp('1/4 inch hardware acceptable for DOUBLE SHEAR uses')
    else
    disp('1/4 inch hardware NOT ACCEPTABLE for DOUBLE SHEAR uses')        
end
fprintf('Fastener factor of safety = %0.3f\n\n', actualFactor(1))
if effective_corner_force < boltShearForce(1) * 1000 / 2
    disp('1/4 inch hardware acceptable for SINGLE SHEAR uses')
    else
    disp('1/4 inch hardware NOT ACCEPTABLE for SINGLE SHEAR uses')
end
fprintf('Fastener factor of safety = %0.3f\n\n', actualFactor(1)/2)



end % function end