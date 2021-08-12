% test of EPOS2 class

% create motor
Motor1 = Epos2(5); % nodeの番号を引数に取る．EPOS側のUSBは接続しない

% Check error
if (Motor1.IsInErrorState)
    Motor1.ClearErrorState;
end

% enable motor
Motor1.EnableNode;

% Change to Home Mode
Motor1.SetOperationMode( OperationModes.HommingMode );

% Home to actual position
Motor1.SetHommingMethod( HommingMethods.ActualPosition );

% Perform Home
Motor1.DoHomming;

% Change to Cuurent Mode
Motor1.SetOperationMode( OperationModes.CurrentMode );
for i = 500:1:2000
    Motor1.MotionWithCurrent(i); % unit mA
end

% Wait a maximum of 10 seconds until motion done
Motor1.WaitUntilDone( 10000 );

% Display actual position
msg = sprintf('Actual Motor Current is: %i', Motor1.ActualCurrent );
disp( msg );

% Ok
disp('All done');

% destroy motor
delete(Motor1)

% clear variable
clear Motor1
