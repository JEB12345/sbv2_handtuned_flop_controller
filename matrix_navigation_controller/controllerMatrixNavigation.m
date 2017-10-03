clear all

% Initialize matrix with the four rings (one per row)

M = [3    6   7   4   5   8;
     5    2   7   6   1   8;
     7    2   3   8   1   4;
     1    6   3   2   5   4];

% Initialize matrix with rows of M where each face appears (OPTIMIZATION POSSIBLE)

faceMat = [2    3   4;
           2    3   4;
           1    3   4;
           1    3   4;
           1    2   4;
           1    2   4;
           1    2   3;
           1    2   3];
     
% Initialize random face where I'm at
ROWS = 4;
COLUMNS = 6;

i = randi(ROWS); %row on the matrix
j = randi(COLUMNS); % column in the matrix, is also the ring I am at
j
% Generate random direction (this direction always refers to the forward
% direction --> moving in this direction will make the robot go forward)
% 0 --
% 1 ++
dir = round(rand);


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
            elseif (mod(M(i,j),2)~=0 && dir==1) || (mod(M(i,j),2)==0 && dir==0)
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
    fprintf('command %s\n oldi %d, oldj %d, oldface %d,olddir %d\n newi %d,newj %d, newface %d, newdir %d\n',cmd,i,j,M(i,j),dir,newi,newj,M(newi,newj),newDir);
    i=newi;
    j=newj;
    dir=newDir;
end;

% Funciton to move backwards on the same ring
function f=backwards(j,COLUMNS)
    f= mod(j+ COLUMNS-2,COLUMNS)+1;
end

% Function to move forward on the same ring
function f=forward(j,COLUMNS)
    f=mod(j,COLUMNS)+1;
end

% Function that can be used for optimization (iterate on a smaller matrix)
function Red = matRedux(M,faceMat,i,j)
    reduxIndex = M(i,j);
    count=1;
    for k=1:3
        oldIndex= faceMat(reduxIndex,k);
        if oldIndex~=i
            Red(count,:)=M(oldIndex,:);
            count =count+1;
        end
    end
end

%more iterations than necessary, can use reduced matrix for the future but
%have to transport back indexes.
function [newi,newj]=smallTurnFWD(M,currentFace,compareTerm,dir)
    for l=1:4
        for k=1:6
            if M(l,k)==currentFace && dir==1 && M(l,backwards(k,6))==compareTerm
                newi=l;
                newj=backwards(k,6);
                return
            elseif M(l,k)==currentFace && dir==0 && M(l,forward(k,6))==compareTerm
                newi=l;
                newj=forward(k,6);
                return
            end
        end
    end
end

function [newi,newj]=bigTurnFWD(M,currentFace,compareTerm,dir)
   for l=1:4
        for k=1:6
            if M(l,k)==currentFace && dir==0 && M(l,backwards(k,6))==compareTerm
                newi=l;
                newj=forward(k,6);
                return
            elseif M(l,k)==currentFace && dir==1 && M(l,forward(k,6))==compareTerm
                newi=l;
                newj=backwards(k,6);
                return
            end
        end
    end 
end

function [newi,newj]=smallTurnBKW(M,currentFace,compareTerm,dir)
    for l=1:4
        for k=1:6
            if M(l,k)==currentFace && dir==0 && M(l,backwards(k,6))==compareTerm
                newi=l;
                newj=backwards(k,6);
                return
            elseif M(l,k)==currentFace && dir==1 && M(l,forward(k,6))==compareTerm
                newi=l;
                newj=forward(k,6);
                return
            end
        end
    end
end

function [newi,newj]=bigTurnBKW(M,currentFace,compareTerm,dir)
   for l=1:4
        for k=1:6
            if M(l,k)==currentFace && dir==1 && M(l,backwards(k,6))==compareTerm
                newi=l;
                newj=forward(k,6);
                return
            elseif M(l,k)==currentFace && dir==0 && M(l,forward(k,6))==compareTerm
                newi=l;
                newj=backwards(k,6);
                return
            end
        end
    end 
end

%Function to change direction
function f=changeDir(dir)
    if dir==0
        f=1;
    elseif dir ==1
        f=0;
    end
end