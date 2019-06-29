tmpModNum = 0;

moduleNames = ['M01'; 'M02'; 'M03'; 'M04'; 'M05'; 'M06'; 'M07'; ...
    'M08'; 'M09'; 'M10'; 'M11'; 'M12'; 'M13'; 'M14'; 'M15'; 'M16'; ...
    'M17'; 'M18'; 'M19'; 'M20'; 'M21'; 'M22'; 'M23'; 'M24']; 

familyName = 'X8-3';

while tmpModNum ~= 24
    HebiLookup.clearModuleList; % To clear stale modules.
    HebiLookup.clearGroups;
    
    Group = HebiLookup.newGroupFromNames( familyName, moduleNames )
    Group.setFeedbackFrequency(200);
    Cmd = CommandStruct();
    tempFbk = Group.getNextFeedback();
    
    tmpModNum = Group.getNumModules;
    disp([int2str(tmpModNum) ' modules found!'])
    
    if (tmpModNum ~=24)
        clearvars HebiLookup Group Cmd tempFbk;
    end
end

clearvars tmpModNum moduleNames familyName ans;