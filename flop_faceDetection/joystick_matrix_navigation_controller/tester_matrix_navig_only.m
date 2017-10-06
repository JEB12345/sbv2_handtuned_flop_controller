% Initialize matrix with the four rings (one per row)

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

currFace = randi(8);
if (currFace > 2)
    i = 1;
else
    i = 2;
end
% Get's the column of the current face
j = find(M(i,:)' == currFace);

%create joystick controller
id=1;
joy=vrjoystick(id);

% wait to start walk
disp('Waiting to start walk. Press any key to continue');
pause;

innerLoop = 1;
quit = 0;

while (1)
    while innerLoop
        disp(['Current Face: ' num2str(M(i,j))]);
        if (M(i,j) ~= currFace)
            j = find(M(i,:)' == currFace);
            disp('Fixed current face');
        end
        disp('Use joystick to move robot');
        joyInput=false;
        %Wait for joystick Input
        while ~joyInput
            if axis(joy,5)==-1 && axis(joy,4)~=1 && axis(joy,4)~=-1
                cmd='f';
                joyInput =true;
            elseif axis(joy,5)==1 && axis(joy,4)~=1 && axis(joy,4)~=-1
                cmd='b';
                joyInput =true;
            elseif axis(joy,5)==1 && axis(joy,4)==1
                cmd= 'rb';
                joyInput=true;
            elseif axis(joy,5)==-1 && axis(joy,4)==-1
                cmd= 'lf';
                joyInput=true;
            elseif axis(joy,5)==-1 && axis(joy,4)==1
                cmd= 'rf';
                joyInput=true;
             elseif axis(joy,5)==1 && axis(joy,4)==-1
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
                    newj= matrixStepLeft(j,COLUMNS); %iterate one step left int the matrix
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
end