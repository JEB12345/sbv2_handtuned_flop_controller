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

%% Some definitions

% Triangles     motors

% Triangle 1:   5   10  19  % Forward, side, backward
% Triangle 2:   12  23  6
% Triangle 3:   21  2   11
% Triangle 4:   4   15  22
% Triangle 5:   13  18  3
% Triangle 6:   20  7   14
% Triangle 7:   8   16  24
% Triangle 8:   17  1   9


% Actuation scheme

% Initial triangle  Actuated cable  Destination triangle
%       1                   5               2
%       1                   10              7
%       1                   19              6
%       2                   12              3
%       2                   23              8
%       2                   6               1
%       3                   21              4
%       3                   2               7
%       3                   11              2
%       4                   4               5
%       4                   15              8
%       4                   22              3
%       5                   13              6
%       5                   18              7
%       5                   3               4
%       6                   20              1
%       6                   7               8
%       6                   14              5
%       7                   8               6
%       7                   16              4
%       7                   24              2
%       8                   17              5
%       8                   1               3
%       8                   9               1

%% Face transitions

% First value is initial triangle
% Second value is
%   1 = "forward"
%   2 = "turn"
%   3 = "backward"

% Array contains actuation scheme

transition{1,1} = [5    3   11  1   8];
transition{1,2} = [10   16  18  14 	11];
transition{1,3} = [19   21  8   23  18];

transition{2,1} = [12   14  22  16  9];
transition{2,2} = [23   17  7 	19  22];
transition{2,3} = [6    4   9 	2   7];

transition{3,1} = [21   19  3   17  24];
transition{3,2} = [2    8 	10 	6 	3];
transition{3,3} = [11   13  24 	15 	10];

transition{4,1} = [4    6   14  8   1];
transition{4,2} = [15   9 	23 	11 	14];
transition{4,3} = [22   20 	1 	18 	23];

transition{5,1} = [13   11  19  9   16];
transition{5,2} = [18   24 	2 	22 	19];
transition{5,3} = [3    5 	16 	7 	2];

transition{6,1} = [20   22  6   24  17];
transition{6,2} = [7    1 	15 	3 	6];
transition{6,3} = [14   12  17 	10 	15];

transition{7,1} = [8    2 	13 	4 	5];
transition{7,2} = [16   10 	21 	12 	13];
transition{7,3} = [24   18 	5 	20 	21];

transition{8,1} = [17   23 	4 	21 	20];
transition{8,2} = [1    7 	12 	5 	4];
transition{8,3} = [9    15 	20 	13 	12];

%% Hebi Stuff

if ~(exist('Cmd'))
    if ~(exist('HebiLookup'))
        addpath('/usr/local/home/jebruce/Projects/Hebi/MATLAB');
        startup;
        SB_Hebi_start;
    else
        SB_Hebi_start;
    end
    disp('Hebi API setup');
else
    disp('Hebi API already setup');
end

% Make Group, Command Structure, and Feedback
% Setup feedback variable if needed
if ~(exist('tempFbk'))
    tempFbk = Group.getNextFeedback();
end
% Change to set how long commands stay active on the robot (0 == infinite)
Group.setCommandLifetime(0);

% Set Motor Gains
if ~(exist('gains'))
    gains = Group.getGains();
end
% Set control strategy (3 is default)
gains.controlStrategy = ones(1,Group.getNumModules)*3;

Group.send('gains', gains);

%% Motor positions and gains for basic flop locomotion

motorPosition = 63;
motorOffset = 3;

%% Go to initial position

% Loop Zeros so not to pull too much current went resetting robot
Cmd.position = ones(1,24)*NaN;
for j=1:24
    Cmd.position(j) = motorOffset;
    Group.send(Cmd);
    disp(j);
    pause(0.5);
end

% wait to start walk
disp('Waiting to start Walk');
pause;

%% Loop Walk

notQuit = 1;
while notQuit
        
    % Set all the motors to the initial state
    cmdMotorPositions = ones(1,24)*motorOffset;
        
    % Selct direction of motion
    promptMessage = 'Press w to go forward, s to go backward, t to turn, d to display current face, or q to quit!\n';
    
    inLoop = 1;
    while inLoop
        
        userInput = input(promptMessage, 's');
        if userInput == 'q'
            notQuit = 0;
            break % break first user input loop
        elseif userInput == 'w'
            % Forward
            inLoop = 0;
            direction = 1;
        elseif userInput == 's'
            % Backward
            inLoop = 0;
            direction = 3;
        elseif userInput == 't'
            % Turn
            inLoop = 0;
            direction = 2;
        elseif userInput == 'd'
            % FACE!
            % Figure out on which face we are standing, using the classifier
            fbk = Group.getNextFeedback();
            
            % Quick check to verify that all 24 motors are connected!
            if nbMotors ~= 24
                display('Something is wrong with the number of connected motors!')
                pause()
            end
            
            % Create a new point that needs to be classified
            for mm = 1:nbMotors
                newpoint(1, (mm-1)*3 + 1) = fbk.accelX(mm); % Col 1
                newpoint(1, (mm-1)*3 + 2) = fbk.accelY(mm); % Col 2
                newpoint(1, (mm-1)*3 + 3) = fbk.accelZ(mm); % Col 3
            end
            
            [n,d] = knnsearch(trainingData,newpoint,'k',10);
            
            resultTable = tabulate(labs(n));
            
            % Pick the result with the highest percentage
            [~, maxPos] = max(cell2mat(resultTable(:,3)));
            faceLabel = resultTable(maxPos, 1);
            face = str2num(faceLabel{1,1});
            
            display('Detected face: ')
            face
            
            clear newpoint resultTable maxVal maxPos fbk;
        else
            display('Wrong key pressed!');
        end
    end

    if ~notQuit
        break; % quit program
    end
    
    clear promptMessage inLoop userInput;
    
    
    % Figure out on which face we are standing, using the classifier
    fbk = Group.getNextFeedback();
    
    % Quick check to verify that all 24 motors are connected!
    if nbMotors ~= 24
        display('Something is wrong with the number of connected motors!')
        pause()
    end
    
    % Create a new point that needs to be classified
    for mm = 1:nbMotors
        newpoint(1, (mm-1)*3 + 1) = fbk.accelX(mm); % Col 1
        newpoint(1, (mm-1)*3 + 2) = fbk.accelY(mm); % Col 2
        newpoint(1, (mm-1)*3 + 3) = fbk.accelZ(mm); % Col 3
    end
    
    [n,d] = knnsearch(trainingData,newpoint,'k',10);
    
    resultTable = tabulate(labs(n));
    
    % Pick the result with the highest percentage
    [~, maxPos] = max(cell2mat(resultTable(:,3)));
    faceLabel = resultTable(maxPos, 1);
    face = str2num(faceLabel{1,1});
    
    display('Detected face: ')
    face
    
    clear newpoint resultTable maxVal maxPos fbk;
    
    
    % Now we can select the motor commands to go to the next face
    motorCommand = cell2mat(transition(face,direction));
    
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