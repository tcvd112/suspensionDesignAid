function axle_loads = sus_axleLoad(totalWeight, centerMass, wheelBase)
%%#############################################################################
% Washington State University
% 2017-18 Formula SAE
%
% Suspension Design Program
% Axle Load
%
% Created by Timm Von Derau
% Last Modified: 7/1/2017
%%#############################################################################

disp('>>> Front and Rear Axle Loads <<<')
axle_loads = zeros(2,1);		% First entry is front, second is rear
axle_loads(2) = totalWeight * (centerMass(1) / wheelBase);
axle_loads(1) = totalWeight - axle_loads(2);
fprintf('Front = %0.3f lbs\nRear = %0.3f lbs\n', axle_loads(1), axle_loads(2))
fprintf('Percent Front = %0.3f \nPercent Rear = %0.3f \n\n', ... 
			(axle_loads(1)/totalWeight) * 100.0, (axle_loads(2)/totalWeight)...
				* 100.0)
disp('- Individual Wheel Loads -')
fprintf('    Front Left Wheel = %0.3f lbs\n    Front Right Wheel %0.3f lbs\n', ...
			axle_loads(1)/2, axle_loads(1)/2)
fprintf('    Rear Left Wheel = %0.3f lbs\n    Rear Right Wheel %0.3f lbs\n\n', ...
			axle_loads(2)/2, axle_loads(2)/2)
% PLOT: Axle loads by wheel base 
p1_X = 60:0.1:70.0;
p1_Y = totalWeight * (centerMass(1) ./ p1_X);
p1_Y2 = totalWeight - p1_Y;
% figure
% plot(p1_X, p1_Y, p1_X, p1_Y2, 'r', wheelBase, axle_loads(2), 'o', wheelBase, ...
        % axle_loads(1), 'o')
% xlabel('Wheelbase, inches'); ylabel('Axle Load, lbs')
% legend('Rear Axle', 'Front Axle', 'Current Iteration', 'Location', 'southeast')
% grid ON; grid MINOR
% title('Axle Load per Wheelbase Length')
end