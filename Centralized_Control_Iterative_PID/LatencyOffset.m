function [ProjectedTarget] = LatencyOffset(CurrentTarget,Latency)
%Take current target and extrapolate it out to predict/adjust for latency
% LATENCY IN SECONDS
%   Detailed explanation goes here

ProjectedTarget=[CurrentTarget(1) + CurrentTarget(3)*Latency;...
                CurrentTarget(2) + CurrentTarget(4)*Latency;...
                CurrentTarget(3);
                CurrentTarget(4)];

end