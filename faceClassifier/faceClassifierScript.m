%% Collect SUPERball V2 IMU Data - Rutgers

% Script required to collect training data for the classifier.
% Run only if needed. Training data is saved to UMUTrainingData.mat

% Modified on 4-9-2018 to include classification of open triangles and
% using Rutgers' face naming convention.

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
disp([num2str(nbMotors) ' motor(s) found!']);
%% Create Training Data

% The idea is to collect IMU data for each stable triangle. During data
% collection, manually move the endcaps to get a better training set.
% Follow the triangle sequence defined below.

% UPDATE:
% End cap convention follows our usual standard, but faces are according to
% the latest Rutgers' convention.

% Some definitions:
% Triangles     motors
% Triangle 0:   2   8   11
% Triangle 1:   2   7   9
% Triangle 2:   1   5   9
% Triangle 3:   3   6   12
% Triangle 4:   3   5   10
% Triangle 5:   1   6   11
% Triangle 6:   4   8   12
% Triangle 7:   4   7   10

% Open triangles:
% Triangle 8:   2   9   11
% Triangle 9:   1   9   11
% Triangle 10:  5   7   9
% Triangle 11:  5   7   10
% Triangle 12:  3   10  12
% Triangle 13:  4   10  12
% Triangle 14:  2   4   7
% Triangle 15:  2   4   8
% Triangle 16:  6   8   12
% Triangle 17:  6   8   11
% Triangle 18:  1   3   5
% Triangle 19:  1   3   6

% Face, end cap 1:3, motor 1:3
triangleDefinitions = [0, 2, 8, 11, 4, 15, 22;
    1, 2, 7, 9, 3 , 13, 18;
    2, 1, 5, 9, 1, 9, 17;
    3, 3, 6, 12, 6, 12, 23;
    4, 3, 5, 10, 5, 10, 19;
    5, 1, 6, 11, 2, 11, 21;
    6, 4, 8, 12, 8, 16, 24;
    7, 4, 7, 10, 7, 14, 20;
    8, 2, 9, 11, 3, 18, 22;
    9, 1, 9, 11, 1, 17, 21;
    10, 5, 7, 9, 9, 13, 17;
    11, 5, 7, 10, 10, 14, 19;
    12, 3, 10, 12, 5, 19, 23;
    13, 4, 10, 12, 7, 20, 24;
    14, 2, 4, 7, 3, 7, 13;
    15, 2, 4, 8, 4, 8, 15;
    16, 6, 8, 12, 12, 16, 23;
    17, 6, 8, 11, 11, 15, 21;
    18, 1, 3, 5, 1, 5, 9;
    19, 1, 3, 6, 2, 6, 11];

Group.setFeedbackFrequency(100); % reduce rate to 5Hz from 100 Hz default

% Samples per triangle
nbSamples = 500;
nbTriangles = size(triangleDefinitions,1); % number of triangles

disp('Press a key to start collecting data!');
pause();

for jj = 1:nbTriangles 
    
    if jj<=nbTriangles
        disp(['Turn the robot on face ' num2str(triangleDefinitions(jj, 1)) ' and press any key to continue...']);
        disp(['The motors on the ground face should be: ' num2str(triangleDefinitions(jj, 5)) ', ' num2str(triangleDefinitions(jj, 6)) ', and ' num2str(triangleDefinitions(jj, 7)) ])
        pause();
    end
    
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
    
end 

disp('End acquisition');


%% Create labels
    
% Labels have to be chars or strings for tabulate() to work. 
% In Flop_Locomotion_with_Classifier.m these labels are converted into
% actual numbers.
for ii=1:nbTriangles
    for kk=(nbSamples*(ii-1)+1):(nbSamples*ii)
        if ii == 1
            labs{kk,1} = '0';
        elseif ii == 2
            labs{kk,1} = '1';
        elseif ii == 3
            labs{kk,1} = '2';
        elseif ii == 4
            labs{kk,1} = '3';
        elseif ii == 5
            labs{kk,1} = '4';
        elseif ii == 6
            labs{kk,1} = '5';
        elseif ii == 7
            labs{kk,1} = '6';
        elseif ii == 8
            labs{kk,1} = '7';
        elseif ii == 9
            labs{kk,1} = '8';
        elseif ii == 10
            labs{kk,1} = '9';
        elseif ii == 11
            labs{kk,1} = '10';
        elseif ii == 12
            labs{kk,1} = '11';
        elseif ii == 13
            labs{kk,1} = '12';
        elseif ii == 14
            labs{kk,1} = '13';
        elseif ii == 15
            labs{kk,1} = '14';
        elseif ii == 16
            labs{kk,1} = '15';
        elseif ii == 17
            labs{kk,1} = '16';
        elseif ii == 18
            labs{kk,1} = '17';
        elseif ii == 19
            labs{kk,1} = '18';
        elseif ii == 20
            labs{kk,1} = '19';
        else
            labs{kk,1} = '-1';
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

Group.setFeedbackFrequency(5); % reduce rate to 1Hz from 100 Hz default

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