%% Simple Hebi Sine Wave Controller
% Version 0.1 - 12/13/2017

% The program commands a simple sine wave to the Hebi X8-3 motors and keeps
% a log of the data. Amplitude, Frequency, and Phase can be changed. 

% The program can be started by inputting 's' in the console, and stopped
% by pressing 'q' while the focus is on the plot window.

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

for jj = 1:nbMotors
    phase(jj) = pi * (jj-1); % [rad]
end

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

globalTime = tic; % Used for the sinusoidal motion
displayTime = tic; % Used to display data at regular intervals

h = figure();

currFbk = Group.getNextFeedback;
disp(['Battery voltage: ' num2str(currFbk.voltage) 'V. ' 'Elapsed Time: ' num2str(toc(globalTime)) 's.']);

% Plot the voltage (from the first motor)
scatter(toc(globalTime), currFbk.voltage(1), 'filled', 'k')
hold on

ylim([0 30]);
xlim([0 toc(globalTime)+ 5]);

xlabel('Time [s]');
ylabel('Voltage [V]');

while notQuit
    
    % Sine Wave
    for i=1:nbMotors
        cmdMotorPositions(i) = amplitude * sin(2*pi*frequency*toc(globalTime) + phase(i));
    end
    
    % Send new positions to motors
    Cmd.position = cmdMotorPositions;
    Group.send(Cmd);
    
    

    pause(0.005);
    
    if (toc(displayTime) >= 5.0)       % Display every 5 seconds
        currFbk = Group.getNextFeedback();
        disp(['Battery voltage: ' num2str(currFbk.voltage) 'V. ' 'Elapsed Time: ' num2str(toc(globalTime)) 's.']);
        displayTime = tic;
        
        % Live plot data - voltage
        scatter(toc(globalTime), currFbk.voltage(1), 'filled', 'k')

        xlim([0 toc(globalTime)+ 5]);

        %drawnow;
    end
    
    

    pause(0.005);
    

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