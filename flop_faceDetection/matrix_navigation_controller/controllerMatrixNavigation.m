%% Add path of shared face detection functions
addpath('../');

%% Load training data for the face detection classifier
% If IMUTrainingData.mat does not exist, or there are changes in the
% robot configuration, run CollectSBdata.m to collect new training data.

load('IMUTrainingData.mat') % Training data

%% Load definitions

transitionsDef;

%% Hebi Stuff

hebiStuff;

%% Motor positions and gains for basic flop locomotion

motorPosition = 65; %rad
motorOffset = 1.8; %rad

%% Go to initial position

% Loop Zeros so not to pull too much current went resetting robot
Cmd.position = ones(1,24)*NaN;
for j=1:24
    Cmd.position(j) = motorOffset;
    Group.send(Cmd);
    disp(j);
    pause(0.1);
end

clear j;


%% Initialize matrix with the four rings (one per row)

M = [3    6   7   4   5   8;
     5    2   7   6   1   8;
     7    2   3   8   1   4;
     1    6   3   2   5   4];
 
% Initialize random face where the robot is at
COLUMNS = size(M,2); 

% Assume positive direction when starting program
% 0 --
% 1 ++
dir = 1;

%% Main control loop
% "Random" selection of initial loop!

currFace = DetectCurrentFace(Group, nbMotors, trainingData, labs);

if (currFace > 2)     
    i = 1; % Loop 1 is the only one that contains every face with index > 2
else
    i = 2; % Loop 2 contains faces 1 and 2.
end

% Get the column containing the current face number
j = find(M(i,:)' == currFace);

% wait to start walk
disp('Waiting to start walk. Press any key to continue');
pause;

% Get user input
innerLoop = 1;
quit = 0;
logging = 0;

while (~quit)
    while innerLoop
        disp(['Desired Face: ' num2str(M(i,j))]);
        currFace = DetectCurrentFace(Group, nbMotors, trainingData, labs);
        if (M(i,j) ~= currFace)
            j = find(M(i,:)' == currFace);
            disp('Fixed current face');
        end
        
        cmd = input('Where should I go next?\n F = forward \n B = backwards \n LF = turn left forward\n RF = turn right forward \nLB = turn left backwards\n RB = turn right backwards\nD = display detected face\n','s');
        switch lower(cmd)
            case 'q' % quit program
                if (logging)
                    modules.stopLogFull('LogFormat', 'mat');
                    disp('Logging Terminated.');
                    logging = 0;
                end
                quit = 1;
                innerLoop = 0;
                
            case 'l' % start/stop data log
                if (logging)
                    modules.stopLogFull('LogFormat', 'mat');
                    disp('Logging Terminated.');
                    logging = 0;
                else
                    modules.startLog();
                    disp('Logging Initiated.');
                    logging = 1;
                end
                
                innerLoop = 1;
                
            case 'f' % forward motion
                if dir== 0
                    newDir=dir;
                    newi=i;
                    newj= matrixStepLeft(j,COLUMNS); %iterate one step left int the matris
                    newFace = M(newi,newj);
                elseif dir ==1
                    newDir=dir;
                    newi=i;
                    newj=matrixStepRight(j,COLUMNS); %iterate one step right in the matrix
                    newFace = M(newi,newj);
                end
                innerLoop = 0;
                
            case 'b' % backward motion
                if dir == 0
                    newDir=dir;
                    newi=i;
                    newj=matrixStepRight(j,COLUMNS); %iterate forward
                    newFace = M(newi,newj);
                elseif dir == 1
                    newDir=dir;
                    newi=i;
                    newj= matrixStepLeft(j,COLUMNS); %iterate one step left in the matrix
                    newFace = M(newi,newj);
                end
                innerLoop = 0;
                
            case 'lf'
                % This option allows to change ring so that the robot will tilt
                % to the left forward. To make it navigate forward or
                % backwards in this direction use the FWD or BKW commands
                
                if (mod(M(i,j),2)~=0 && dir==1) || (mod(M(i,j),2)==0 && dir==0)
                    % currentFace is the face where I'm at now
                    currentFace=M(i,j);
                    % find the comparison term that is going to be used to find new ring
                    if dir==0
                        compareTerm = M(i,matrixStepLeft(j,COLUMNS));
                    elseif dir ==1
                        compareTerm = M(i,matrixStepRight(j,COLUMNS));
                    end
                    % Perform a "small turn", next face does not change with
                    % respect to the previous ring but the ring itself changes
                    % to allow the robot to slightly change direction
                    [newi,newj]=smallTurnFWD(M,currentFace,compareTerm,dir);
                    newDir = changeDir(dir);
                    
                elseif (mod(M(i,j),2)~=0 && dir==0) || (mod(M(i,j),2)==0 && dir==1)
                    currentFace=M(i,j);
                    if dir==0
                        compareTerm = M(i,matrixStepRight(j,COLUMNS));
                    elseif dir ==1
                        compareTerm = M(i,matrixStepLeft(j,COLUMNS));
                    end
                    % Perform a "big turn", next face changes with
                    % respect to the previous ring and the ring itself changes
                    % to allow the robot to change direction
                    [newi,newj]=bigTurnFWD(M,currentFace,compareTerm,dir);
                    newDir=changeDir(dir);
                end
                innerLoop = 0;
                
            case 'rf'
                % This option allows to change ring so that the robot will tilt
                % to the right forward. To make it navigate forward or
                % backwards in this direction use the FWD or BKW commands
                
                if (mod(M(i,j),2)~=0 && dir==1) || (mod(M(i,j),2)==0 && dir==0)
                    currentFace=M(i,j);
                    if dir==0
                        compareTerm = M(i,matrixStepRight(j,COLUMNS));
                    elseif dir ==1
                        compareTerm = M(i,matrixStepLeft(j,COLUMNS));
                    end
                    [newi,newj]=bigTurnFWD(M,currentFace,compareTerm,dir);
                    newDir = changeDir(dir);
                elseif (mod(M(i,j),2)~=0 && dir==0) || (mod(M(i,j),2)==0 && dir==1)
                    currentFace=M(i,j);
                    % find term that is going to be used to find new ring
                    if dir==0
                        compareTerm = M(i,matrixStepLeft(j,COLUMNS));
                    elseif dir ==1
                        compareTerm = M(i,matrixStepRight(j,COLUMNS));
                    end
                    [newi,newj]=smallTurnFWD(M,currentFace,compareTerm,dir);
                    newDir = changeDir(dir);
                end
                innerLoop = 0;
                
            case 'lb'
                % This option allows to change ring so that the robot will tilt
                % to the left backwards. To make it navigate forward or
                % backwards in this direction use the FWD or BKW commands.
                % The direction "forward" is still approximately the same as
                % before. To continue navigating in the TLB direction it is
                % necessary to use the command BKW. To understand better,
                % imagine that the robot is a car and you are moving backwards,
                % even if you move slightly to the right or left, the general
                % perception of what is backwards and what is forward does not
                % change.
                
                if (mod(M(i,j),2)~=0 && dir==1) || (mod(M(i,j),2)==0 && dir==0)
                    % currentFace is the face where I'm at now
                    currentFace=M(i,j);
                    % find term that is going to be used to find new ring
                    if dir==1
                        compareTerm = M(i,matrixStepLeft(j,COLUMNS));
                    elseif dir ==0
                        compareTerm = M(i,matrixStepRight(j,COLUMNS));
                    end
                    %reducedMat= matRedux(M,faceMat,i,j);
                    [newi,newj]=smallTurnBKW(M,currentFace,compareTerm,dir);
                    newDir = changeDir(dir);
                    
                elseif (mod(M(i,j),2)~=0 && dir==0) || (mod(M(i,j),2)==0 && dir==1)
                    currentFace=M(i,j);
                    if dir==1
                        compareTerm = M(i,matrixStepRight(j,COLUMNS));
                    elseif dir ==0
                        compareTerm = M(i,matrixStepLeft(j,COLUMNS));
                    end
                    [newi,newj]=bigTurnBKW(M,currentFace,compareTerm,dir);
                    newDir=changeDir(dir);
                end
                innerLoop = 0;
                
            case 'rb'
                % This option allows to change ring so that the robot will tilt
                % to the right backwards.
                
                if (mod(M(i,j),2)~=0 && dir==1) || (mod(M(i,j),2)==0 && dir==0)
                    currentFace=M(i,j);
                    if dir==0
                        compareTerm = M(i,matrixStepLeft(j,COLUMNS));
                    elseif dir ==1
                        compareTerm = M(i,matrixStepRight(j,COLUMNS));
                    end
                    [newi,newj]=bigTurnBKW(M,currentFace,compareTerm,dir);
                    newDir = changeDir(dir);
                elseif (mod(M(i,j),2)~=0 && dir==0) || (mod(M(i,j),2)==0 && dir==1)
                    currentFace=M(i,j);
                    if dir==0
                        compareTerm = M(i,matrixStepRight(j,COLUMNS));
                    elseif dir ==1
                        compareTerm = M(i,matrixStepLeft(j,COLUMNS));
                    end
                    [newi,newj]=smallTurnBKW(M,currentFace,compareTerm,dir);
                    newDir = changeDir(dir);
                end
                innerLoop = 0;
                
            case 'd' % Detect current face
                % This option can be used if the robot rolls for more than
                % one step. It will check if the robot is on the correct
                % face. If not, the user can reset the controller.
                disp(['Desired Face: ' num2str(M(newi,newj))]);
                currFace = DetectCurrentFace(Group, nbMotors, trainingData, labs);
                if (M(newi,newj) ~= currFace)
                    reset = input('Desired and deteced faces do not match. Should we reset direction [1] or quit [0]?');
                    if (reset)
                        currFace = DetectCurrentFace(Group, nbMotors, trainingData, labs);
                        if (currFace > 2)
                            i = 1;
                        else
                            i = 2;
                        end
                        % Get's the column of the current face
                        j = find(M(i,:)' == currFace);
                        dir = 1;
                        innerLoop = 1; 
                    else
                        quit = 1;
                        innerLoop = 0;
                        break;
                    end
                else
                    innerLoop = 1;
                end
            otherwise
                fprintf('Wrong command! Please try again.');
                innerLoop = 1;
        end
    end
    
    if quit
        break; % quit program
    end
    
    % Set all the motors to the initial state
    cmdMotorPositions = ones(1,24)*motorOffset;
    
    disp(['Curr Face: ' num2str(currFace) ' next face: ' num2str(M(newi,newj))]);
    % Now we can select the motor commands to go to the next face
    motorCommand = cell2mat(transition(currFace, M(newi,newj)));
    
    % We selected these five cables via trial and error
    cmdMotorPositions(motorCommand(1)) = motorPosition + motorOffset;
    cmdMotorPositions(motorCommand(2)) = -motorPosition + motorOffset;
    cmdMotorPositions(motorCommand(3)) = motorPosition + motorOffset;
    cmdMotorPositions(motorCommand(4)) = motorPosition + motorOffset;
    cmdMotorPositions(motorCommand(5)) = -motorPosition + motorOffset;
    
    % Send new positions to motors
    Cmd.position = cmdMotorPositions;
    Group.send(Cmd);
    
    disp('Press any key to go to the next step')
    pause
    
    % Bring back the robot to the rest position
    for j=1:24
        Cmd.position(j) = motorOffset;
        Group.send(Cmd);
    end
    innerLoop = 1;

    i = newi;
    j = newj;
    dir = newDir;
end