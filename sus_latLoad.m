function [lat_load_NA, lat_axle_weight_transfer_NA] = sus_latLoad( ...
            totalWeight, MU, centerMass, trackWidth, GRAV, TEST_CORNER_RADIUS)
%%#############################################################################
% Washington State University
% 2017-18 Formula SAE
%
% Suspension Design Program
% Lateral Weight Transfer
%
% Created by Timm Von Derau
% Last Modified: 7/1/2017
%%#############################################################################

disp('>>> Lateral Weight Transfer, NO AERO <<<')
corneringForceMaxNA = totalWeight * MU;
lat_load_NA = zeros(2,1);		% 1 is outside, pos, 2 is inside, neg
% Weight Transfer average
latWeightTransferNA = (corneringForceMaxNA * centerMass(2)) / ((trackWidth(1) + ...
                        trackWidth(2)) / 2);
lat_load_NA(1) = totalWeight + latWeightTransferNA;
lat_load_NA(2) = totalWeight - latWeightTransferNA;
% Weight Transfer front and rear
lat_axle_weight_transfer_NA = zeros(2,1);  % 1 is front, 2 is rear
for n = 1:numel(trackWidth)
    lat_axle_weight_transfer_NA(n) = (corneringForceMaxNA * centerMass(2)) / ...
                                    trackWidth(n);
end
% Cornering Speed
car_mass = totalWeight / GRAV;
latCornerSpeedNA = sqrt((corneringForceMaxNA * TEST_CORNER_RADIUS) / car_mass); % ft/s
fprintf('Max Cornering Force = %0.3f lbs\n', corneringForceMaxNA)
fprintf('Average Weight Transfer = +/- %0.3f lbs\n', latWeightTransferNA)
fprintf('    Front = +/- %0.3f lbs\n    Rear = +/- %0.3f lbs\n', ...
            lat_axle_weight_transfer_NA(1), lat_axle_weight_transfer_NA(2))
fprintf('Test Corner Speed = %0.3f ft/s  ->  %0.3f mph\n\n', latCornerSpeedNA, ...
            latCornerSpeedNA * 3600.0 / 5280.0)
end