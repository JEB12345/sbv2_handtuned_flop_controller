function [ nextFace ] = CalculateNextTriangle( m_currentFace, m_currentLoop )
% Calculate the next base triangle based on current triangle and loop

% Face transitions for each loop. Loops 1:4 are four different loops.
% Loops 5:8 are the first four in reverse
loop = [    1     2     3     4     5     6;
    1     2     7     4     5     8;
    1     6     7     4     3     8;
    3     2     7     6     5     8;
    1     6     5     4     3     2;
    1     8     5     4     7     2;
    1     8     3     4     7     6;
    3     8     5     6     7     2;];

currentIndex = find(loop(m_currentLoop,:) == m_currentFace);

if currentIndex
    nextFace = loop(m_currentLoop, mod(currentIndex, size(loop,2)) + 1 );
else
    % Here something is wrong because we are in a face that does not belong
    % to the current loop. Possible solutions: stay on the current face, or
    % move randomly to try to get back to the current loop.
    display('Something is wrong!') 
    nextFace = m_currentFace;
end

end

