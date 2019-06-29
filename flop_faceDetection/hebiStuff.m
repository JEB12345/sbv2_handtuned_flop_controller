%% HEBI Stuff

% Add to path the HEBI Matlab API, available at
% http://docs.hebi.us/
% Tested on hebi-matlab-1.0-rev1908

if ~(exist('Cmd'))
    if ~(exist('HebiLookup'))
        if (strcmp(char(java.lang.System.getProperty('user.name')),'Massimo'))
            addpath('/Users/Massimo/Documents/Hebi_motors');
        else
            %addpath('/usr/local/home/jebruce/Projects/Hebi/MATLAB');
        end
        
        % This is just adding to path the ./hebi folder contained in the
        % path above. It can probably be replaced with a direct call to
        % addpath(...)
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
