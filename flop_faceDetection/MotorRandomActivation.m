function [ ] = MotorRandomActivation(Group,cmdMotorPositions,m_Cmd)
%RANDOMMOTORACTIVATION Summary of this function goes here
%   Detailed explanation goes here
%%
stop=0;
resetMotorOffset=0.0;

cmdMotorPositions = ones(1,Group.getNumModules)*resetMotorOffset;
cmdMotorPositions(1,25) = 0;

for i= 1:24
    A{i} = num2str(i);
end

while ~stop
    userInput=input('Input number of motor to move, press r for motor reset, press s for stop\n','s');
    randMotorPosition=20.*rand(1)-10;
    switch userInput
        case A
            
            % We selected these five cables via trial and error
            cmdMotorPositions(str2num(userInput)) = randMotorPosition ;
            
            display('New position = ');
            display(num2str(randMotorPosition));
            % Send new positions to motors
            m_Cmd.position = cmdMotorPositions;
            Group.send(m_Cmd);
            
        case 's'
            stop=1;
            
        case 'q'
            stop=1;
            
        case 'r'
            %Reset motors to inital position
            for j=1:24
                m_Cmd.position(j) = resetMotorOffset;
                Group.send(m_Cmd);
                disp(j);
                pause(0.5);
            end
        otherwise
            disp('Wrong Input\n');
    end
end


%%
end

