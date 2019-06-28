%% HEBI Stuff

if ~(exist('Cmd'))
    if ~(exist('HebiLookup'))
        if (strcmp(char(java.lang.System.getProperty('user.name')),'Massimo'))
            addpath('/Users/Massimo/Documents/Hebi_motors');
        else
            %addpath('/usr/local/home/jebruce/Projects/Hebi/MATLAB');
        end
        
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

%Group.send('gains', gains);


% %% HEBI Stuff
% 
% if ~(exist('Cmd'))
%     if ~(exist('HebiLookup'))
%         
%         % addpath('/usr/local/home/jebruce/Projects/Hebi/MATLAB');
%         addpath('/Users/Massimo/Documents/Hebi_motors');
%         startup;
%         SB_Hebi_start;
%     else
%         SB_Hebi_start;
%     end
%     disp('Hebi API setup');
% else
%     disp('Hebi API already setup');
% end
% 
% % Make Group, Command Structure, and Feedback
% % Setup feedback variable if needed
% if ~(exist('tempFbk'))
%     tempFbk = Group.getNextFeedback();
% end
% % Change to set how long commands stay active on the robot (0 == infinite)
% Group.setCommandLifetime(0);
% 
% % Set Motor Gains
% if ~(exist('gains'))
%     gains = Group.getGains();
% end
% % Set control strategy (3 is default)
% gains.controlStrategy = ones(1,Group.getNumModules)*3;
% 
% Group.send('gains', gains);