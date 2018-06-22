Cmd.position = ones(1,24)*NaN;

for ww=1:24
    Cmd.position(ww) = motorOffset + hillOffset*lowerOffset(currFace, ww) + enableCompliance*compliantOffset(currFace, ww);     
    Group.send(Cmd);
end

clear ww;