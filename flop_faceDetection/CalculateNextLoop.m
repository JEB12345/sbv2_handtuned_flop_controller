function [ nextLoop ] = CalculateNextLoop( m_currentLoop, m_currentFace, m_newDirection )
% Calculate a new loop direction based on current loop, face, and new
% direction.

% Direction map:
% 0: straight       (12 o'clock)
% 1: forward-right  (2  o'clock)
% 2: backward-right (4  o'clock)
% 3: backward       (6  o'clock)
% 4: backward-left  (8  o'clock)
% 5: forward-left   (10 o'clock)

% Ordered loop sequence, moving clockwise. Each line corresponds to a
% base triangle. Each entry is the loop number, which is defined on each
% line of the loop variable.
loopSequence = [    1     2     3     5     6     7;
    1     2     4     5     6     8;
    1     7     4     5     3     8;
    1     2     3     5     6     7;
    1     2     4     5     6     8;
    1     7     4     5     3     8;
    4     7     6     8     3     2;
    4     7     6     8     3     2;];


currentIndex = find(loopSequence(m_currentFace,:) == m_currentLoop);

if currentIndex
    nextLoop = loopSequence(m_currentFace, mod(currentIndex + m_newDirection -1, size(loopSequence,2)) + 1);
else
    % Here something is wrong because we are in a face that does not belong
    % to the current loop. Possible solutions: stay on the current face, or
    % move randomly to try to get back to the current loop.
    display('Something went wrong! The loop you selected does not pass through this face');
    nextLoop = currentLoop;
end

end

