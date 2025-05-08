function [PWMValues, PastState, RunningErrorSums] = DynamixelPID(CurrentState,PastState, TimeSinceLast, RunningErrorSums)
%takes State vectors from current and last known state of 4x1 form 
%       [ Theta1Target ]
%  S  = [ Theta2Target ]
%       [ Theta1Arm    ] 
%       [ Theta2Arm    ]
%  as well as time since last state in seconds, and running sum of errors
%  in 2x1 vector form
%  
%  E  = [RunningErrorSum1]
%       [RunningErrorSum2] 
%
%  Outputs column vector of PWM values(from -884 to 884, corresponding to -100% to 100%),
%  as well as updating the "PastState" vector and "RunningError"
%==========================================================================
%Set K values for P I and D (change these as needed to tune)
K_p=[1 0; 0 1];
K_I=[1 0; 0 1];
K_D=[1 0; 0 1];

%calculate the error of current state
CurrentError= [CurrentState(1)-CurrentState(3);...
               CurrentState(2)-CurrentState(4) ];

%calculate error deriv
ErrorDeriv= [(CurrentState(1)-CurrentState(3))-(PastState(1)-PastState(3));...
             (CurrentState(2)-CurrentState(4))-(PastState(2)-PastState(4))]...
             /TimeSinceLast;

%calculate running error
RunningErrorSums=RunningErrorSums+(CurrentError*TimeSinceLast);

%create PWM initial Values
PWMValues=(K_p*CurrentError)+(K_D*ErrorDeriv)+(K_I*RunningErrorSums);

%make sure PWM values dont exceed -100% to 100%
if abs(PWMValues(1))>1
    PWMValues(1)=PWMValues(1)/abs(PWMValues(1));
end
if abs(PWMValues(2))>1
    PWMValues(2)=PWMValues(2)/abs(PWMValues(2));
end 

%Scale PWM values to the range of Dynamixel and make sure theyre intergers
PWMValues=round(PWMValues*884);

%Update the PastState
PastState=CurrentState;

end
