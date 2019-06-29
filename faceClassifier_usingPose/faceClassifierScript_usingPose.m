%% Collect SUPERball V2 IMU Data - Rutgers

% Script required to collect training data for the classifier.
% Run only if needed. Training data is saved to UMUTrainingData.mat

% Modified on 11-14-2018 to use pose quaternions from Hebi motors instead
% of raw accelerometer data.

% Modified on 4-9-2018 to include classification of open triangles and
% using Rutgers' face naming convention.

% In order to run, add to path the latest Hebi Matlab API, available at
% http://docs.hebi.us/
% Tested on hebi-matlab-1.0-rev1908

%% Collect Hebi Pose Data

% Enable this variable to move the motors during aquisition
moveMotorsEn = 1;

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

% Change to set how long commands stay active on the robot (0 == infinite)
Group.setCommandLifetime(0);

Cmd = CommandStruct();

if (moveMotorsEn)
    Cmd.position = 0.0*ones(1,24);
    Group.send(Cmd);
end
%% Create Training Data

% The idea is to collect IMU data for each stable triangle. During data
% collection, manually move the endcaps to get a better training set.
% Follow the triangle sequence defined below.

% UPDATE:
% End cap convention follows our usual standard, but faces are according to
% the latest Rutgers' convention.

% Some definitions:
% Face, end cap 1:3, motor 1:3
triangleDefinitions = [0, 3, 5, 10, 5, 10, 19;
    1, 2, 8, 11, 4 , 15, 22;
    2, 2, 7, 9, 3, 13, 18;
    3, 3, 6, 12, 6, 12, 23;
    4, 1, 6, 11, 2, 11, 21;
    5, 4, 7, 10, 7, 14, 20;
    6, 4, 8, 12, 8, 16, 24;
    7, 1, 5, 9, 1, 9, 17;
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

Group.setFeedbackFrequency(20); % reduce rate to 20Hz from 100 Hz default

% Samples per triangle
nbSamples = 500;
nbTriangles = 20; % number of triangles

disp('Press a key to start collecting data!');
pause();

for jj = 1:nbTriangles 
    
    if jj<=nbTriangles
        disp(['Turn the robot on face ' num2str(triangleDefinitions(jj, 1)) ' and press any key to continue...']);
        disp(['The motors on the ground face should be: ' num2str(triangleDefinitions(jj, 5)) ', ' num2str(triangleDefinitions(jj, 6)) ', and ' num2str(triangleDefinitions(jj, 7)) ])
        pause();
    end
    
    
    for i=(nbSamples*(jj-1)+1):(nbSamples*jj) % Collect n. samples for each position
        
        if((i==nbSamples*(jj-1)+1) && moveMotorsEn) % First third of the samples
            Cmd.position = 0.0*ones(1,24);
            Group.send(Cmd);
        elseif((i==nbSamples*(jj-1)+1+floor(nbSamples/3)) && moveMotorsEn) % Second third of the samples
            Cmd.position = -2.5*ones(1,24);
            Group.send(Cmd);
        elseif((i==nbSamples*(jj-1)+1+2*floor(nbSamples/3)) && moveMotorsEn) % Last third of the samples
            Cmd.position = -5.0*ones(1,24);
            Group.send(Cmd);
        elseif((i==nbSamples*jj) && moveMotorsEn) % Last third of the samples
            Cmd.position = 0.0*ones(1,24);
            Group.send(Cmd);
        end
        
        fbk = Group.getNextFeedbackFull(); % Read sensor data
        
        % Save raw accelerometer data
        for mm = 1:nbMotors
            trainingDataAcc(i, (mm-1)*3 + 1) = fbk.accelX(mm); % Col 1
            trainingDataAcc(i, (mm-1)*3 + 2) = fbk.accelY(mm); % Col 2
            trainingDataAcc(i, (mm-1)*3 + 3) = fbk.accelZ(mm); % Col 3
        end

        % Save pose quaternions WXYZ
        for mm = 1:nbMotors
            trainingDataQuat(i, (mm-1)*4 + 1) = fbk.orientationW(mm);
            trainingDataQuat(i, (mm-1)*4 + 2) = fbk.orientationW(mm);
            trainingDataQuat(i, (mm-1)*4 + 3) = fbk.orientationW(mm);
            trainingDataQuat(i, (mm-1)*4 + 4) = fbk.orientationW(mm);
        end

        % Convert quaternions to Euler angles
        for (mm = 1:nbMotors)
            angle(mm, :) = quatern2euler([fbk.orientationW, fbk.orientationX, fbk.orientationY, fbk.orientationZ]);
        end

        % Save Euler Angles (ZYX)
        for mm = 1:nbMotors
            trainingDataQuat2Eul(i, (mm-1)*3 +1) = angle(mm,1);
            trainingDataQuat2Eul(i, (mm-1)*3 +2) = angle(mm,2);
            trainingDataQuat2Eul(i, (mm-1)*3 +3) = angle(mm,3);
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
save(filename, 'trainingDataAcc', 'labs', 'nbTriangles', 'nbMotors');


%% Extra: Classification
% NOT NEEDED FOR DATA COLLECTION!
% It's here just to test if the classifier works properly

Group.setFeedbackFrequency(5); % reduce rate to 1Hz from 100 Hz default

%% Test with Accelerations
clearvars newpoint;

while 1
    fbk = Group.getNextFeedbackFull();
    
    for mm = 1:nbMotors
        newpoint(1, (mm-1)*3 + 1) = fbk.accelX(mm); % Col 1
        newpoint(1, (mm-1)*3 + 2) = fbk.accelY(mm); % Col 2
        newpoint(1, (mm-1)*3 + 3) = fbk.accelZ(mm); % Col 3
    end
    
    [n,d] = knnsearch(trainingDataAcc,newpoint,'k',10);

    resultTable = tabulate(labs(n));
    [maxVal, maxPos] = max(cell2mat(resultTable(:,3)));
    face = resultTable(maxPos, 1);
    str2num(face{1,1})

end

%% Test with Quaternions
clearvars newpoint;

while 1
    fbk = Group.getNextFeedbackFull();
    
    for mm = 1:nbMotors
        newpoint(1, (mm-1)*4 + 1) = fbk.orientationW(mm); % Col 1
        newpoint(1, (mm-1)*4 + 2) = fbk.orientationX(mm); % Col 2
        newpoint(1, (mm-1)*4 + 3) = fbk.orientationY(mm); % Col 3
        newpoint(1, (mm-1)*4 + 4) = fbk.orientationZ(mm); % Col 3
    end
    
    [n,d] = knnsearch(trainingDataQuat,newpoint,'k',10);

    resultTable = tabulate(labs(n));
    [maxVal, maxPos] = max(cell2mat(resultTable(:,3)));
    face = resultTable(maxPos, 1);
    str2num(face{1,1})

end

%% Test with Euler Angles
clearvars newpoint angle;

while 1
    fbk = Group.getNextFeedbackFull();

    % Convert quaternions to Euler angles
    for (mm = 1:nbMotors)
        angle(mm, :) = quatern2euler([fbk.orientationW, fbk.orientationX, fbk.orientationY, fbk.orientationZ]);
    end
    
    for mm = 1:nbMotors
        newpoint(1, (mm-1)*3 + 1) = angle(mm,1); % Col 1
        newpoint(1, (mm-1)*3 + 2) = angle(mm,2); % Col 2
        newpoint(1, (mm-1)*3 + 3) = angle(mm,3); % Col 3
    end

        
    [n,d] = knnsearch(trainingDataQuat2Eul,newpoint,'k',10);

    resultTable = tabulate(labs(n));
    [maxVal, maxPos] = max(cell2mat(resultTable(:,3)));
    face = resultTable(maxPos, 1);
    str2num(face{1,1})

end

