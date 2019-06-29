%% Collect SUPERball V2 IMU Data

% Script required to collect training data for the classifier.
% Run only if needed. Training data is saved to UMUTrainingData.mat

% In order to run, add to path the latest Hebi Matlab API, available at
% http://docs.hebi.us/
% Tested on hebi-matlab-1.0-rev1908

%% Collect Hebi IMU Data

% Show devices on the network
disp(HebiLookup);

% Select a device 
family = 'X8-3'; % any family
Group = HebiLookup.newGroupFromFamily(family); 
disp(Group);

%% Access feedback
% Feedback is in SI units and can be accessed as below. If a group contains
% more than one module, the returned feedback contains vectors instead of
% scalars.

% Display the most recent feedback
fbk = Group.getNextFeedback();
display(fbk);

nbMotors = Group.getNumModules; % Crappy way to know how many motors are connected
display(nbMotors, ' motor(s) found!');
%% Create Training Data

% The idea is to collect IMU data for each stable triangle. During data
% collection, manually move the endcaps to get a better training set.
% Follow the triangle sequence defined below.

% Some definitions:
% Triangles     motors
% Triangle 1:   5   10  19
% Triangle 2:   12  23  6
% Triangle 3:   21  2   11
% Triangle 4:   4   15  22
% Triangle 5:   13  18  3
% Triangle 6:   20  7   14
% Triangle 7:   8   16  24
% Triangle 8:   17  1   9

Group.setFeedbackFrequency(100); % reduce rate to 5Hz from 100 Hz default

disp('Press a key to start collecting data!');
pause();

% Samples per triangle
nbSamples = 500;
nbTriangles = 8; % 8 different stable triangles

for jj = 1:nbTriangles 
    
    for i=(nbSamples*(jj-1)+1):(nbSamples*jj) % Collect n. samples for each position
        
        fbk = Group.getNextFeedback(); % Read sensor data
        
        for mm = 1:nbMotors
            trainingData(i, (mm-1)*3 + 1) = fbk.accelX(mm); % Col 1
            trainingData(i, (mm-1)*3 + 2) = fbk.accelY(mm); % Col 2
            trainingData(i, (mm-1)*3 + 3) = fbk.accelZ(mm); % Col 3
        end
        
        % Display some data to notify the user that the program is logging
        disp(['Acc_X: ' num2str(fbk.accelX) ' Acc_Y: ' num2str(fbk.accelY) ' Acc_Z: ' num2str(fbk.accelZ)]);
    end
    
    if jj<nbTriangles
        disp('Turn the robot to the next face and press any key to continue...');
        pause();
    end
end 

disp('End acquisition');


%% Create labels
    
% Labels have to be chars or strings for tabulate() to work. 
% In Flop_Locomotion_with_Classifier.m these labels are converted into
% actual numbers.
for ii=1:nbTriangles
    for kk=(nbSamples*(ii-1)+1):(nbSamples*ii)
        if ii == 1
            labs{kk,1} = '1';
        elseif ii == 2
            labs{kk,1} = '2';
        elseif ii == 3
            labs{kk,1} = '3';
        elseif ii == 4
            labs{kk,1} = '4';
        elseif ii == 5
            labs{kk,1} = '5';
        elseif ii == 6
            labs{kk,1} = '6';
        elseif ii == 7
            labs{kk,1} = '7';
        else
            labs{kk,1} = '8';
        end
    end
end

%% Save all data

disp('Saving Workspace data. Press any key to confirm!')
pause()

filename = 'IMUTrainingData.mat';
save(filename, 'trainingData', 'labs', 'nbTriangles', 'nbMotors');


%% Extra: Classification
% NOT NEEDED FOR DATA COLLECTION!
% It's here just to test if the classifier works properly

Group.setFeedbackFrequency(1); % reduce rate to 1Hz from 100 Hz default

while 1
    fbk = Group.getNextFeedback();
    
    for mm = 1:nbMotors
        newpoint(1, (mm-1)*3 + 1) = fbk.accelX(mm); % Col 1
        newpoint(1, (mm-1)*3 + 2) = fbk.accelY(mm); % Col 2
        newpoint(1, (mm-1)*3 + 3) = fbk.accelZ(mm); % Col 3
    end
    
    [n,d] = knnsearch(trainingData,newpoint,'k',10);

    resultTable = tabulate(labs(n));
    [maxVal, maxPos] = max(cell2mat(resultTable(:,3)));
    face = resultTable(maxPos, 1);
    str2num(face{1,1})

end