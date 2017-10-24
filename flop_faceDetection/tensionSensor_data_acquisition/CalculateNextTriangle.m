function [ nextFace ] = CalculateNextTriangle( m_currentFace, m_currentLoop )
% Calculate the next base triangle based on current triangle and loop

% Face transitions for each loop. Loops 1:4 are four different loops.
% Loops 5:8 are the first four in reverse
loop = [1   4   5   2   3   6;
    1   4   7   2   3   8;
    1   6   7   2   5   8;
    7   6   3   8   5   4;
    1   6   3   2   5   4;
    1   8   3   2   7   4;
    1   8   5   2   7   6;
    7   4   5   8   3   6;];

currentIndex = find(loop(m_currentLoop,:) == m_currentFace);

if currentIndex
    nextFace = loop(m_currentLoop, mod(currentIndex, size(loop,2)) + 1 );
else
    % Here something is wrong because we are in a face that does not belong
    % to the current loop. 
    display('Something is wrong! The current face does not belong to the current loop.')
    
    % Changing the next face will not change the current loop...
    % This function is anyway called after CalculateNextLoop, so we should
    % never get this error (never say never...).
    
    % Find a loop that contains the current face and select the next
    % triangle on that loop.
    display('Selecting an alternative suitable target face...');
    
    notFound = 1;
    while notFound
        for ii = 1:size(loop,1)
            currentIndex = find(loop(ii,:) == m_currentFace);
            if currentIndex
                nextFace = loop(ii, mod(currentIndex, size(loop,2)) + 1 );
                notFound = 0;
                break;
            end
        end              
    end

    display(['The new target face is ' num2str(nextFace)]);
end

end

