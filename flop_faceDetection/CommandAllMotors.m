function [ ] = CommandAllMotors( Group, Cmd, position, delay )
% Command the same position to all motors

nbModules = Group.getNumModules;

for j=1:nbModules
    Cmd.position(j) = position;
    Group.send(Cmd);
    pause(delay);
end

end

