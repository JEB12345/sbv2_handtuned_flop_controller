% addpath('/usr/local/home/jebruce/Projects/Hebi/MATLAB');
addpath('/Users/Massimo/Documents/Hebi_motors');
startup;
SB_Hebi_start;

tempFbk = Group.getNextFeedback();

Group.setCommandLifetime(0);

gains = Group.getGains();

% Set control strategy (3 is default)
gains.controlStrategy = ones(1,Group.getNumModules)*3;

Group.send('gains', gains);
