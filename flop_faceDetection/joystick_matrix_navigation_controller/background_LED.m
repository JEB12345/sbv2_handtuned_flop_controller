function background_LED(src,evt,M,MotorsMatrix,dir,COLUMNS,Group)
    
    global currFace;
    global i;
    global j;
    
    disp(['Desired Face: ' num2str(M(i,j))]);
    currFace = DetectCurrentFace(Group);
    
    if (M(i,j) ~= currFace)
        j = find(M(i,:)' == currFace);
        disp('Fixed current face');
    end
    
    %Color motors of next step forward face
    %LedColors = matrix with the RGB combination for each motor
    LedColors = zeros(24,3);
    
    %Set all leds to green
    LedColors(:,2)=1;
    
    %Figure out which face would be the forward face
    if dir== 0
        colorjForward= matrixStepLeft(j,COLUMNS); %iterate one step left int the matris
        colorjBackward=matrixStepRight(j,COLUMNS); %iterate one step right in the matrix
    elseif dir ==1
        colorjForward=matrixStepRight(j,COLUMNS); %iterate one step right in the matrix
        colorjBackward= matrixStepLeft(j,COLUMNS); %iterate one step left int the matris
    end
    
    %Set Led color as BLUE for the forward face
    LedColors(MotorsMatrix(M(i,colorjForward),1),:)=[0 0 1];
    LedColors(MotorsMatrix(M(i,colorjForward),2),:)=[0 0 1];
    LedColors(MotorsMatrix(M(i,colorjForward),3),:)=[0 0 1];
    
    LedColors(MotorsMatrix(M(i,colorjBackward),1),:)=[1 0 1];
    LedColors(MotorsMatrix(M(i,colorjBackward),2),:)=[1 0 1];
    LedColors(MotorsMatrix(M(i,colorjBackward),3),:)=[1 0 1];
    
    %Send LED command to group of motors (TODO- test this to figure out
    %if this command format works of need a for loop for each motor
    Group.send('led',LedColors);
end