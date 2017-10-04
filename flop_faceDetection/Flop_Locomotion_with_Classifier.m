%% SUPERball V2 Locomotion
% Version 1.01 - 10/02/2017

% Changelog
% 1.0 Initial release
% 1.01 Swapped definitions triangle 7 and 8.


% SUPERball V2 Locomotion controller
% a.k.a.
% They see me rollin', they hatin'

% The program detects the current base triangle and can move SUPERball v2
% in any of the three directions for each base triangle. Direction of
% motion is selected by the user, via keyboard:
% w = "forward"
% s = "backward"
% t = "turn" - left or right depending on the face number

% These directions depend on the SUPERball orientation...

% In order to run, add to path the latest Hebi Matlab API, available at
% http://docs.hebi.us/
% Tested on hebi-matlab-1.0-rev1908

% Also add the file IMUTrainingData.mat, or run CollectSBdata.m to generate
% a new IMUTrainingData.mat file.

%% Run CollectSBdata.m first to collect training data.

load('IMUTrainingData.mat') % Training data


%% Load definitions

transitionsDef;

%% Hebi Stuff

hebiStuff;

%% Motor positions and gains for basic flop locomotion

motorPosition = 63;
motorOffset = 1.5;

%% Go to initial position

% Loop Zeros so not to pull too much current went resetting robot
Cmd.position = ones(1,24)*NaN;
for j=1:24
    Cmd.position(j) = motorOffset;
    Group.send(Cmd);
    disp(j);
    pause(0.5);
end

clear j;


% BROKEN!!!!!!!!

%% Random definition for initial loop!
% Loop 1 passes through faces 1 to 6. For faces 7:8 we just select loop 4.
currFace = DetectCurrentFace(Group);
if (currFace < 7)
    currLoop = 1;
else
    currLoop = 4;
end

% wait to start walk
disp('Waiting to start walk. Press any key to continue');
pause;

%% Loop Walk

notQuit = 1;
logging = 0;
while notQuit
        
    % Set all the motors to the initial state
    cmdMotorPositions = ones(1,24)*motorOffset;
    
    % Detect current face
    currFace = DetectCurrentFace(Group);
        
    % Select direction of motion
    promptMessage = 'Press w to go forward, s to go backward, 0:5 for new direction, l to start/stop logging, d to display current face, or q to quit!\n';
    
    direction = 0;
    
    inLoop = 1;
    while inLoop
        
        userInput = input(promptMessage, 's');
        switch userInput
            case 'q'
                notQuit = 0;
                display('Goodbye!')
                break % break first user input loop
            case 'w'
                % Forward
                inLoop = 0;
                direction = 0;
            case 's'
                % Backward
                inLoop = 0;
                direction = 3;
            case 'l'
                inLoop = 1;
                if (logging)
                    modules.stopLogFull('LogFormat', 'mat');
                    disp('Logging Terminated.');
                    logging = 0;
                else
                    modules.startLog();
                    disp('Logging Initiated.');
                    logging = 1;
                end
            case '0'
                % Turn
                inLoop = 0;
                direction = 0;
            case '1'
                % Turn
                inLoop = 0;
                direction = 1;
            case '2'
                % Turn
                inLoop = 0;
                direction = 2;
            case '3'
                % Turn
                inLoop = 0;
                direction = 3;
            case '4'
                % Turn
                inLoop = 0;
                direction = 4;
            case '5'
                % Turn
                inLoop = 0;
                direction = 5;
            case 'd'
                DetectCurrentFace(Group);
                
            otherwise
                display('Wrong key pressed!');
        end
    end

    if ~notQuit
        break; % quit program
    end
    
    clear promptMessage inLoop userInput;
    
    % Detect current face
    currFace = DetectCurrentFace(Group);
  
    % Update current loop
    currLoop = CalculateNextLoop(currLoop, currFace, direction);
    display(['We are on loop number: ' num2str(currLoop)]);
    
    % Update next triangle
    nextTriangle = CalculateNextTriangle(currFace, currLoop);
    
    % Now we can select the motor commands to go to the next face
    motorCommand = cell2mat(transition(currFace, nextTriangle));
    
    % We selected these five cables via trial and error
    cmdMotorPositions(motorCommand(1)) = motorPosition + motorOffset;
    cmdMotorPositions(motorCommand(2)) = -motorPosition + motorOffset;
    cmdMotorPositions(motorCommand(3)) = motorPosition + motorOffset;
    cmdMotorPositions(motorCommand(4)) = motorPosition + motorOffset;
    cmdMotorPositions(motorCommand(5)) = -motorPosition + motorOffset;
    
    % Send new positions to motors
    Cmd.position = cmdMotorPositions;
    Group.send(Cmd);
    
    display('Press any key to go to the next step')
    pause
    
    % Bring back the robot to the rest position
    for j=1:24
        Cmd.position(j) = motorOffset;
        Group.send(Cmd);
    end
end


%% Ending sequence. Bring back the robot to the rest position.

disp('Bringing robot back to zero position before quiting.');
for j=1:24
    Cmd.position(j) = motorOffset;
    Group.send(Cmd);
    disp(j);
    pause(0.5);
end