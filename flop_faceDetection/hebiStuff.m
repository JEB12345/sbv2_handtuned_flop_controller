%% HEBI Stuff

if ~(exist('Cmd'))
    if ~(exist('HebiLookup'))
        addpath('/usr/local/home/jebruce/Projects/Hebi/MATLAB');
        startup;
        SB_Hebi_start;
    else
        SB_Hebi_start;
    end
    disp('Hebi API setup');
else
    disp('Hebi API already setup');
end

% Make Group, Command Structure, and Feedback
% Setup feedback variable if needed
if ~(exist('tempFbk'))
    tempFbk = Group.getNextFeedback();
end
% Change to set how long commands stay active on the robot (0 == infinite)
Group.setCommandLifetime(0);

% Set Motor Gains
if ~(exist('gains'))
    gains = Group.getGains();
end
% Set control strategy (3 is default)
gains.controlStrategy = ones(1,Group.getNumModules)*3;

Group.send('gains', gains);