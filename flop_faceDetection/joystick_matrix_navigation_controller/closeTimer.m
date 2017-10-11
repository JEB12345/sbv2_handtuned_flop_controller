function closeTimer()
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

% Stop and Delete Timer
mytimer = timerfind('Name', 'faceDetectionLED');

if ~isempty(mytimer)
    stop(mytimer)
    delete(mytimer)
end
end

