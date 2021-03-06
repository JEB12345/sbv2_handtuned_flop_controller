function controllerMatrixNavigation
%% Add path of shared face detection functions
%% NEED TO ADD hebiJoystick to this folder! (https://github.com/HebiRobotics/MatlabInput/releases)
addpath ../ hebiJoystick/ hebiJoystick/lib/

%% Load HebiJoystick Libraries
HebiJoystick.loadLibs();

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
for k=1:24
    Cmd.position(k) = motorOffset;
    Group.send(Cmd);
    disp(k);
    pause(0.5);
end

clear k;


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
global i;
global j;
global currFace;

currFace = DetectCurrentFace(Group);
if (currFace > 2)
    i = 1;
else
    i = 2;
end
% Get's the column of the current face
j = find(M(i,:)' == currFace);

%create joystick controller
id=1;
joy=HebiJoystick(id);

% wait to start walk
disp('Waiting to start walk. Press any key to continue');
pause;

innerLoop = 1;
quit = 0;

delT = 0.5;

mytimer = timer('TimerFcn'      ,{@background_LED,M,MotorsMatrix,dir,COLUMNS,Group},...
                'StartDelay'    ,0              ,...
                'Period'        ,delT           ,...
                'Name'          ,'faceDetectionLED'  ,...
                'ExecutionMode' ,'fixedrate'        );

start(mytimer);
cleanupObj = onCleanup(@closeTimer);

while (1)
    while innerLoop
        %background_LED(M,MotorsMatrix,dir,COLUMNS,Group);
%         disp(['Desired Face: ' num2str(M(i,j))]);
%         currFace = DetectCurrentFace(Group);
%         if (M(i,j) ~= currFace)
%             j = find(M(i,:)' == currFace);
%             disp('Fixed current face');
%         end
%         %Color motors of next step forward face
%         %LedColors = matrix with the RGB combination for each motor
%         LedColors = zeros(24,3);
%         %Set all leds to green
%         LedColors(:,2)=1;
%         %Figure out which face would be the forward face
%         if dir== 0
%             colorj= matrixStepLeft(j,COLUMNS); %iterate one step left int the matris
%         elseif dir ==1
%             colorj=matrixStepRight(j,COLUMNS); %iterate one step right in the matrix
%         end
%         %Set Led color as BLUE for the forward face
%         LedColors(MotorsMatrix(M(i,colorj),1),:)=[0 0 1];
%         LedColors(MotorsMatrix(M(i,colorj),2),:)=[0 0 1];
%         LedColors(MotorsMatrix(M(i,colorj),3),:)=[0 0 1];
%         %Send LED command to group of motors (TODO- test this to figure out
%         %if this command format works of need a for loop for each motor
%         Group.send('led',LedColors);
        
        %%BEGIN CONTROLLER
        disp('Use joystick to move robot');
        joyInput=false;
        %Wait for joystick Input
        while ~joyInput
            if axis(joy,2)==-1 && axis(joy,1)>-0.1 && axis(joy,1)<0.1 
                cmd='f';
                joyInput =true;
            elseif axis(joy,2)==1 && axis(joy,1)>-0.1 && axis(joy,1)<0.1
                cmd='b';
                joyInput =true;
            elseif axis(joy,1)==1 && axis(joy,2)==1
                cmd= 'rb';
                joyInput=true;
            elseif axis(joy,1)==-1 && axis(joy,2)==-1
                cmd= 'lf';
                joyInput=true;
            elseif axis(joy,1)==1 && axis(joy,2)==-1
                cmd= 'rf';
                joyInput=true;
             elseif axis(joy,1)==-1 && axis(joy,2)==1
                cmd= 'lb';
                joyInput=true;
            else
                joyInput=false;
            end
        end
        switch cmd
            case 'f'
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
            case 'b'
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
            case 'd'
                disp(['Desired Face: ' num2str(M(newi,newj))]);
                currFace = DetectCurrentFace(Group);
                if (M(newi,newj) ~= currFace)
                    reset = input('Desired and deteced faces do not match. Should we reset direction [1] or quit [0]?');
                    if (reset)
                        currFace = DetectCurrentFace(Group);
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
    
    
    disp('Press any button to continue.');
    buttonInput=false;
    while ~buttonInput
         if (button(joy,1) == 1 || button(joy,2) == 1 || button(joy,3) == 1 || button(joy,4) == 1)
             buttonInput = true;
         end
         if (M(newi,newj) == currFace && axis(joy,2) ~= 0)
             buttonInput = true;
         end
    end
    
    % Bring back the robot to the rest position
    Cmd.position = ones(1,24)*motorOffset;
    Group.send(Cmd);
    
    innerLoop = 1;
%     fprintf('command %s\n oldi %d, oldj %d, oldface %d,olddir %d\n newi %d,newj %d, newface %d, newdir %d\n',cmd,i,j,M(i,j),dir,newi,newj,M(newi,newj),newDir);   
    
    i=newi;
    j=newj;
    dir=newDir;
end