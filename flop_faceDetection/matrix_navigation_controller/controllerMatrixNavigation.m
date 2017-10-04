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

% wait to start walk
disp('Waiting to start walk. Press any key to continue');
pause;

while (1)
    cmd = input('Where should I go next?\n FWD = forward \n BKW = backwards \n TLF = turn left forward\n TRF = turn right forward \nTLB = turn left backwards\n TRB = turn right backwards\n','s');    
    switch cmd
        case 'FWD'
            if dir== 0
                newDir=dir;
                newi=i;
                newj= backwards(j,COLUMNS); %iterate backwards 
                newFace = M(newi,newj);
            elseif dir ==1
                newDir=dir;
                newi=i;
                newj=forward(j,COLUMNS); %iterate forward
                newFace = M(newi,newj);
            end
        case 'BKW'
            if dir == 0
                newDir=dir;
                newi=i;
                newj=forward(j,COLUMNS); %iterate forward
                newFace = M(newi,newj);
            elseif dir == 1
                newDir=dir;
                newi=i;
                newj= backwards(j,COLUMNS); %iterate backwards 
                newFace = M(newi,newj);
            end
        case 'TLF'
            % This option allows to change ring so that the robot will tilt
            % to the left forward. To make it navigate forward or
            % backwards in this direction use the FWD or BKW commands
            
            if (mod(M(i,j),2)~=0 && dir==1) || (mod(M(i,j),2)==0 && dir==0)
                % currentFace is the face where I'm at now
                currentFace=M(i,j);
                % find the comparison term that is going to be used to find new ring
                 if dir==0 
                     compareTerm = M(i,backwards(j,COLUMNS));
                 elseif dir ==1
                     compareTerm = M(i,forward(j,COLUMNS));
                 end
                % Perform a "small turn", next face does not change with
                % respect to the previous ring but the ring itself changes
                % to allow the robot to slightly change direction
                [newi,newj]=smallTurnFWD(M,currentFace,compareTerm,dir);
                newDir = changeDir(dir);
                
            elseif (mod(M(i,j),2)~=0 && dir==0) || (mod(M(i,j),2)==0 && dir==1)
                currentFace=M(i,j);
                if dir==0 
                     compareTerm = M(i,forward(j,COLUMNS));
                 elseif dir ==1
                     compareTerm = M(i,backwards(j,COLUMNS));
                end
                % Perform a "big turn", next face changes with
                % respect to the previous ring and the ring itself changes
                % to allow the robot to change direction
                [newi,newj]=bigTurnFWD(M,currentFace,compareTerm,dir);
                newDir=changeDir(dir);
            end
        case 'TRF'
            % This option allows to change ring so that the robot will tilt
            % to the right forward. To make it navigate forward or
            % backwards in this direction use the FWD or BKW commands
            
            if (mod(M(i,j),2)~=0 && dir==1) || (mod(M(i,j),2)==0 && dir==0)
                currentFace=M(i,j);
                if dir==0 
                     compareTerm = M(i,forward(j,COLUMNS));
                 elseif dir ==1
                     compareTerm = M(i,backwards(j,COLUMNS));
                 end
                [newi,newj]=bigTurnFWD(M,currentFace,compareTerm,dir);
                newDir = changeDir(dir);
            elseif (mod(M(i,j),2)~=0 && dir==0) || (mod(M(i,j),2)==0 && dir==1)
                currentFace=M(i,j);
                % find term that is going to be used to find new ring
                 if dir==0 
                     compareTerm = M(i,backwards(j,COLUMNS));
                 elseif dir ==1
                     compareTerm = M(i,forward(j,COLUMNS));
                 end
                [newi,newj]=smallTurnFWD(M,currentFace,compareTerm,dir);
                newDir = changeDir(dir);
            end
        case 'TLB'
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
                     compareTerm = M(i,backwards(j,COLUMNS));
                 elseif dir ==0
                     compareTerm = M(i,forward(j,COLUMNS));
                 end
                %reducedMat= matRedux(M,faceMat,i,j);
                [newi,newj]=smallTurnBKW(M,currentFace,compareTerm,dir);
                newDir = changeDir(dir);
                
            elseif (mod(M(i,j),2)~=0 && dir==0) || (mod(M(i,j),2)==0 && dir==1)
                currentFace=M(i,j);
                if dir==1 
                     compareTerm = M(i,forward(j,COLUMNS));
                 elseif dir ==0
                     compareTerm = M(i,backwards(j,COLUMNS));
                 end
                [newi,newj]=bigTurnBKW(M,currentFace,compareTerm,dir);
                newDir=changeDir(dir);
            end
        case 'TRB'
            % This option allows to change ring so that the robot will tilt
            % to the right backwards.
            
            if (mod(M(i,j),2)~=0 && dir==1) || (mod(M(i,j),2)==0 && dir==0)
                currentFace=M(i,j);
                if dir==0 
                     compareTerm = M(i,forward(j,COLUMNS));
                 elseif dir ==1
                     compareTerm = M(i,backwards(j,COLUMNS));
                 end
                [newi,newj]=bigTurnBKW(M,currentFace,compareTerm,dir);
                newDir = changeDir(dir);
            elseif (mod(M(i,j),2)~=0 && dir==1) || (mod(M(i,j),2)==0 && dir==0)
                currentFace=M(i,j);
                 if dir==0 
                     compareTerm = M(i,backwards(j,COLUMNS));
                 elseif dir ==1
                     compareTerm = M(i,forward(j,COLUMNS));
                 end
                [newi,newj]=smallTurnBKW(M,currentFace,compareTerm,dir);
                newDir = changeDir(dir);
            end
        otherwise
            fprintf('Wrong command');
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
    
    display('Press any key to go to the next step')
    pause
    
    disp(['Desired Face: ' num2str(M(newi,newj))]);
    currFace = DetectCurrentFace(Group);
    
    % Bring back the robot to the rest position
    for j=1:24
        Cmd.position(j) = motorOffset;
        Group.send(Cmd);
    end
    pause
    
%     fprintf('command %s\n oldi %d, oldj %d, oldface %d,olddir %d\n newi %d,newj %d, newface %d, newdir %d\n',cmd,i,j,M(i,j),dir,newi,newj,M(newi,newj),newDir);
    i=newi;
    j=newj;
    dir=newDir;
end