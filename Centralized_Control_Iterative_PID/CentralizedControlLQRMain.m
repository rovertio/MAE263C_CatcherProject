%%%%%%%initialize stuff %%%%%%%%%%%%%%%
TargetThetasLast = [0; 0];
ArmThetasLast = [-pi/4; pi/2];
BallNotSeenYet = 1;
Latency=0;

L1=.3;  %length of link 1 in m
l1=1; %Distance from pivot to CoM in link 1
M1=1; %mass of link 1
I1=1; %moment of inertia about CoM of link 1

M2=1; %mass of motor 

L3=.3;  %length of link 2 in meters
l3=1; %Distance from pivot to CoM in link 2
M3=1; %mass of link 2
I3=1; %moment of inertia about CoM of link 2

InertiaMatrixConstantPart=[I1+(M1*l1^2)+(M2*L1^2)+I3+(M3*((L1^2)+(l3^2))),     I3+M3*l3; ...
                           I3+M3*l3^2,                                         I3+M3*l3^2];

InertiaMatrixVariablePart=[2*M3*L1*l3, M3*L1*l3;...
                           M3*L1*l3,   0];



A=[0 0 1 0;...
   0 0 0 1;... %A Matrix for LQR
   0 0 0 0;
   0 0 0 0];
B=[0 0;...
   0 0;...  %B matrix for LQR
   1 0;...
   0 1];    
Q=eye(4);   %Punishment for error in LQR
R=eye(2);   %Punishment for effort in LQR

tic %get a timer going to reference as clock
go=1;

while go == 1

    %assume nothing is new to begin with
    NewDataIsHere=0;

    CameraDataThetas=GetCameraDataThetas;%%%%%% Need some way to integrate the camera data with this function!

    %check if camera Data is different from last known
    if CameraDataThetas ~= TargetThetasLast

        if BallNotSeenYet ==0 %%from the 2nd data point onward calculate thetadots and go

            TimeSinceLast=toc-LastTime;
            TargetStateBeforeLatency=ThetaDifferentiator(CameraDataThetas,TargetThetasLast,TimeSinceLast);
            TargetState=LatencyOffset(TargetStateBeforeLatency,Latency);

            NewDataIsHere=1; %we have enough data to tell our controller to go
        end

        TargetThetasLast=CameraDataThetas; %record what we just saw as our last known ball thetas
        LastTime=toc;
        BallNotSeenYet =0; 
    end
    
    
    %if we have data then do controller calculations!
    if NewDataIsHere==1

        %%% generate arm state vector       
        ArmThetasNow=GetArmDataThetas;  %%%%NEED A FUNCTION TO GET ARM THETA DATA!!!
        ArmState=ThetaDifferentiator(ArmThetasNow,ArmThetasLast,TimeSinceLast);
        
        %%% generate error state vector
        ErrorState=TargetState-ArmState;

        %%% use LQR to create gain values
        lqrgain=lqr(A,B,Q,R);

        %%% calculate desired Theta1DDot and Theta2DDot from current error
        %%% and lqr gain values, can be replaced with traditional PID if
        %%% desired. THIS IS "OUTER LOOP" linear control
        DesiredAccelerations=lqrgain*ErrorState;
        
        
        %generate dynamics based on arm state
        InertiaMatrix=InertiaMatrixConstantPart+(InertiaMatrixVariablePart*cos(ArmState(2)));
        CoriolisAffects=[(-2*M3*L1*l3*sin(ArmState(2))*ArmState(3)*ArmState(4))-M3*L1*l3*sin(ArmState(2))*ArmState(4)^2;...
                         M3*L1*l3*sin(ArmState(2))*ArmState(3)^2];

        %compute desired torque from dynamics and target accelerations
        Torques=(InertiaMatrix*DesiredAccelerations)+CoriolisAffects;

        %scale based on motors having max torque of .5N*m
        Torques=Torques*2;

        %Cap out at 100%
        if abs(Torques(1))>1
            Torques(1)=Torques(1)/abs(Torques(1));
            R(1,1)=R(1,1)*1.05; %increase punishment of effort if we're capping out, so we have less gain next time
            disp(strcat('Motor1 saturated, R1 increased to ',num2str(R(1,1))));
        end
        if abs(Torques(2))>1
            Torques(2)=Torques(2)/abs(Torques(2));
            R(2,2)=R(2,2)*1.05; %increase punishment of effort if we're capping out, so we have less gain next time
            disp(strcat('Motor2 saturated, R2 increased to ',num2str(R(2,2))));
        end

        %turn percent value into the PWM value the motors read (-885 to 885 instead of -100% to 100%)
        PWM=round(Torques*884);
    end
       
end
