%% Simple Hebi Sine Wave Controller
% Version 0.1 - 12/13/2017

% The program commands a simple sine wave to the Hebi X8-3 motors.
% Amplitude, Frequency, and Phase can be changed.

% In order to run, add to path the latest Hebi Matlab API, available at
% http://docs.hebi.us/
% Tested on hebi-matlab-1.0-rev1908

%%
clear;

%% Hebi Stuff

hebiStuff; % Connects to all the Hebi X8-3 motors that are visible on the network.

nbMotors = Group.getNumModules;

%% Main parameters

amplitude = 5.0; % [rad]
frequency = 0.3; % [Hz]
phase = 0.0; % []

%% Go to initial position

% Loop Zeros so not to pull too much current went resetting robot
Cmd.position = ones(1,nbMotors)*NaN;

for j=1:nbMotors
    Cmd.position(j) = 0.0;
    Group.send(Cmd);
    disp(j);
    pause(0.5);
end

clear j;

%% Wait to start walk

disp('Waiting to start. Press any key to continue');
pause;

%% Loop Walk

modules.startLog();
disp('Logging Initiated.');

notQuit = 1;
set(gcf,'CurrentCharacter','@'); % set to a dummy character

globalTime = tic;
displayTime = tic;

h = figure();

while notQuit
    
    % Sine Wave
    for i=1:nbMotors
        cmdMotorPositions(i) = amplitude * sin(2*pi*frequency*toc(globalTime) + phase);
    end
    
    % Send new positions to motors
    Cmd.position = cmdMotorPositions;
    Group.send(Cmd);
    
    

    pause(0.005);
    
    if (toc(displayTime) >= 5.0)       % Display every 5 seconds
        currFbk = Group.getNextFeedback();
        disp(['Battery voltage: ' num2str(currFbk.voltage) 'V. ' 'Elapsed Time: ' num2str(toc(globalTime)) 's.']);
        displayTime = tic;
    end
    

    % check for quit key
    k=get(gcf,'CurrentCharacter');
    if k~='@' % has it changed from the dummy character?
        % now process the key as required
        if k=='q', notQuit=false; end
        
        set(gcf,'CurrentCharacter','@'); % reset the character
    end
end

modules.stopLogFull('LogFormat', 'mat');
disp('Logging Terminated.');
close(h);
close all;
                    
%% Ending sequence. Bring back the robot to the rest position.

disp('Bringing motors back to zero position before quiting.');
for j=1:nbMotors
    Cmd.position(j) = 0.0;
    Group.send(Cmd);
    disp(j);
    pause(0.5);
end

disp('Goodbye!');