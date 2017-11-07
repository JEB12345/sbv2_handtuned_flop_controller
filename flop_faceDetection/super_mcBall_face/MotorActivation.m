function [ ] = MotorRandomActivation(Group,cmdMotorPositions,Cmd)
%RANDOMMOTORACTIVATION Summary of this function goes here
%   Detailed explanation goes here
%%
    stop=0;
    resetMotorOffset=0.0;
    while stop==0
        userInput=input('Input number of motor to move, press r for motor reset, press s for stop\n','s');
        randMotorPosition=20.*rand(1000,1)-10
        if any(userInput=='1:24')
  
            % We selected these five cables via trial and error
            cmdMotorPositions(str2num(userInput)) = randMotorPosition ;

            % Send new positions to motors
            Cmd.position = cmdMotorPositions;
            Group.send(Cmd);
            
        elseif userInput=='s'
            stop=1;
        elseif userInput=='r'
            %Reset motors to inital position
            for j=1:24
                Cmd.position(j) = resetMotorOffset;
                Group.send(Cmd);
                disp(j);
                pause(0.5);
            end
        else
            disp('Wrong Input\n');
        end
    end
        
        
%%
end

