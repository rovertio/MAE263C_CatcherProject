function [Statevector] = ThetaDifferentiator(CurrentThetas,PastThetas, TimeBetween)
% Takes current and previous thetas in the form
%
%       T = [Theta 1]   
%           [Theta 2]
%
% and creates state vector of form
%
%           [ Theta 1  ]
%       S = [ Theta 2  ]
%           [Thetadot 1]
%           [Thetadot 2]

Statevector=[CurrentThetas; (CurrentThetas-PastThetas)/TimeBetween];
end