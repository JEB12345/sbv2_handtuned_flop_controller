function [ face ] = DetectCurrentFace( Group )

    load('IMUTrainingData.mat') % Training data
    
    % Figure out on which face we are standing, using the classifier
    fbk = Group.getNextFeedback();
    
    % Quick check to verify that all 24 motors are connected!
%     if size(fbk.accelX,2) ~= nbMotors
%         display('Something is wrong with the number of connected motors!')
%         pause()
%     end
    
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
    
    display(['Detected face: ' faceLabel{1,1}]);
        
    %clear newpoint resultTable maxVal maxPos fbk;
    
end

