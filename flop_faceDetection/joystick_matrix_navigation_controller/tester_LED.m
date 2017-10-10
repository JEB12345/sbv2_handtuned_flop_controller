%% Add path of shared face detection functions
addpath('../');

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


%% Initialize matrix with the four rings (one per row)

M = [3    6   7   4   5   8;
     5    2   7   6   1   8;
     7    2   3   8   1   4;
     1    6   3   2   5   4];
 
% Initialize random face where I'm at
COLUMNS = size(M,2); 

% Assume positive direction when starting program
% 0 --
% 1 ++
dir = 1;

%%Initialize matrix with number of motors for each face
MotorsMatrix =[   5   10  19; 
                  4   15  22;
                  3   13  18;
                  6   12  23;
                  2   11  21;
                  7   14  20;
                  8   16  24;
                  1   9   17];
%% Face init
%% Random definition for initial loop!
currFace = DetectCurrentFace(Group);
if (currFace > 2)
    i = 1;
else
    i = 2;
end
% Get's the column of the current face
j = find(M(i,:)' == currFace);

%Color motors of next step forward face
%LedColors = matrix with the RGB combination for each motor
LedColors = zeros(24,3);
%Set all leds to green
LedColors(:,2)=1;
%Figure out which face would be the forward face
if dir== 0
    colorj= matrixStepLeft(j,COLUMNS); %iterate one step left int the matris
elseif dir ==1
    colorj=matrixStepRight(j,COLUMNS); %iterate one step right in the matrix
end
%Set Led color as BLUE for the forward face
LedColors(MotorsMatrix(M(i,colorj),1),:)=[0 0 1];
LedColors(MotorsMatrix(M(i,colorj),2),:)=[0 0 1];
LedColors(MotorsMatrix(M(i,colorj),3),:)=[0 0 1];
%Send LED command to group of motors (TODO- test this to figure out
%if this command format works of need a for loop for each motor
Group.send('led',LedColors);
        