load('IMUTrainingData.mat') % Training data


% motor loop
face1 = [5 3 11 1 8];
face2 = [12 14 22 16 18]; 
face3 = [21 19 3 17 24]; 
face4 = [4 6 14 8 10]; 
face5 = [13 11 19 9 16]; 
face6 = [20 22 6 24 2]; 
loop = {face1, face2, face3, face4, face5, face6};

% %% Hebi Stuff
if ~(exist('Cmd'))
    if ~(exist('HebiLookup'))
        addpath('/Users/Massimo/Documents/Hebi_motors');
        startup;
        SB_Hebi_start;
    else
        SB_Hebi_start;
    end
    disp('Hebi API setup');
else
    disp('Hebi API already setup');
end


%% Make Group, Command Structure, and Feedback
% Setup feedback variable if needed
if ~(exist('tempFbk'))
    tempFbk = Group.getNextFeedback();
end
% Change to set how long commands stay active on the robot (0 == infinite)
Group.setCommandLifetime(0);

%% Set Motor Gains
if ~(exist('gains'))
    gains = Group.getGains();
end
% Set control strategy (3 is default)
gains.controlStrategy = ones(1,Group.getNumModules)*3;

Group.send('gains', gains);

motorPosition = 60;
motorOffset = 3;
motorSmallPos = 40;

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
for i=1:length(loop)
    cmdMotorPositions = ones(1,24)*motorOffset;
    face = loop{i};
    cmdMotorPositions(face(1)) = motorPosition + motorOffset;
    cmdMotorPositions(face(2)) = -motorPosition + motorOffset;
    cmdMotorPositions(face(3)) = motorPosition + motorOffset;
    cmdMotorPositions(face(4)) = motorPosition + motorOffset;
    cmdMotorPositions(face(5)) = -motorPosition + motorOffset;
        
    Cmd.position = cmdMotorPositions;
    Group.send(Cmd);
    pause
end

for j=1:24
    Cmd.position(j) = motorOffset;
    Group.send(Cmd);
    disp(j);
    pause(0.5);
end

%(-0.6748x[rads] + 104.12)[in cm]
%((y/0.010) - 104.12)/(-0.6748) = x