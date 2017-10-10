
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
%Set Led color as RED for the forward face
LedColors(MotorsMatrix(M(i,colorj),1),:)=[1 0 0];
LedColors(MotorsMatrix(M(i,colorj),2),:)=[1 0 0];
LedColors(MotorsMatrix(M(i,colorj),3),:)=[1 0 0];
%Send LED command to group of motors (TODO- test this to figure out
%if this command format works of need a for loop for each motor
Group.send('led',LedColors);
        